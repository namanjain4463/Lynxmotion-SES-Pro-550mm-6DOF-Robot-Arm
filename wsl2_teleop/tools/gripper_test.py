#!/usr/bin/env python3
"""Standalone test for the DH-Robotics CGE-10-10 gripper over Modbus RTU (RS-485).

Plain pyserial, no ROS, so it runs the same on WSL2, native Ubuntu or a Jetson.

Register map (DH CGE-10 short manual, section 2.3; cross-checked against DH-Robotics'
dh_gripper_driver (DH_Modbus_Gripper) and pyDHgripper (PGE); all example frames in the
manual pass the CRC check in `selftest`):
  0x0100 initialise      write 1 = initialise, 0xA5 = full initialise (finds both ends)
  0x0101 force           20-100 %
  0x0103 target position 0-1000 per mille (the fingers move at once)
  0x0104 speed           1-100 %
  0x0200 init state      0 = not initialised, 1 = initialised (2 = initialising on some models)
  0x0201 grip state      0 = moving, 1 = at target (no object), 2 = object caught, 3 = object dropped
  0x0202 actual position 0-1000 per mille
Defaults: slave ID 1, 115200 baud, 8N1. This tool never writes the configuration registers
(0x03xx: slave ID, baud rate, save), so it cannot change how the gripper talks.

  python3 gripper_test.py selftest                 # no hardware: checks the Modbus framing
  python3 gripper_test.py status                   # read-only: is the gripper answering?
  python3 gripper_test.py init [--full]            # initialise (the fingers move!)
  python3 gripper_test.py pos 500                  # move to 500 per mille
  python3 gripper_test.py force 30 | speed 50      # set grip force / speed
  python3 gripper_test.py watch 10                 # print the state every 0.2 s for 10 s
  python3 gripper_test.py test                     # guided test: status, init, moves, grip
Options: --port /dev/ttyUSB0 --baud 115200 --id 1
"""
import argparse
import csv
import os
import sys
import time

REG_INIT, REG_FORCE, REG_POS, REG_SPEED = 0x0100, 0x0101, 0x0103, 0x0104
REG_INIT_STATE, REG_GRIP_STATE, REG_ACTUAL_POS = 0x0200, 0x0201, 0x0202
GRIP_TEXT = {0: "moving", 1: "at target, no object", 2: "OBJECT CAUGHT", 3: "object dropped",
             0xFFFF: "undefined (reads 65535 until initialised)"}
INIT_TEXT = {0: "not initialised", 1: "initialised", 2: "initialising"}


def crc16(data):
    """Modbus RTU CRC-16 (poly 0xA001, init 0xFFFF), returned low byte first."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return bytes([crc & 0xFF, crc >> 8])


def frame(slave, func, reg, value):
    body = bytes([slave, func, reg >> 8, reg & 0xFF, value >> 8, value & 0xFF])
    return body + crc16(body)


class ModbusError(Exception):
    pass


class Gripper:
    def __init__(self, port, baud, slave, timeout=0.3, verbose=False):
        import serial  # imported here so `selftest` works without pyserial
        self.ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=timeout)
        self.slave = slave
        self.verbose = verbose

    def close(self):
        self.ser.close()

    def _transact(self, request, reply_len, retries=3):
        last = None
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write(request)
            self.ser.flush()
            reply = self.ser.read(reply_len)
            if self.verbose:
                print(f"    tx {request.hex(' ')}  rx {reply.hex(' ') or '(nothing)'}")
            # Exception reply: function code with the top bit set, 5 bytes
            if len(reply) >= 5 and reply[1] == (request[1] | 0x80) and crc16(reply[:3]) == reply[3:5]:
                raise ModbusError(f"gripper rejected the request (Modbus exception code {reply[2]})")
            if len(reply) == reply_len and crc16(reply[:-2]) == reply[-2:] and reply[0] == self.slave:
                time.sleep(0.005)  # inter-frame gap
                return reply
            last = reply
            time.sleep(0.05)
        raise ModbusError(f"no valid reply (got {len(last or b'')} bytes: {(last or b'').hex(' ') or 'nothing'})")

    def read(self, reg):
        reply = self._transact(frame(self.slave, 0x03, reg, 1), 7)
        if reply[1] != 0x03 or reply[2] != 2:
            raise ModbusError(f"unexpected reply {reply.hex(' ')}")
        return (reply[3] << 8) | reply[4]

    def write(self, reg, value):
        request = frame(self.slave, 0x06, reg, value)
        reply = self._transact(request, 8)
        if reply != request:  # a single-register write is echoed back unchanged
            raise ModbusError(f"write not confirmed: sent {request.hex(' ')}, got {reply.hex(' ')}")

    def state(self):
        return self.read(REG_INIT_STATE), self.read(REG_GRIP_STATE), self.read(REG_ACTUAL_POS)


def describe(init, grip, pos):
    return (f"init: {INIT_TEXT.get(init, f'unknown ({init})')} | grip: {GRIP_TEXT.get(grip, f'unknown ({grip})')} "
            f"| position: {pos} per mille")


def in_range(name, value, lo, hi):
    if not lo <= value <= hi:
        sys.exit(f"{name} must be {lo}-{hi}, not {value}")
    return value


def wait_settled(g, timeout=5.0, log=None, label=""):
    """Poll until the fingers stop (grip state != 0). Returns the final (init, grip, pos).

    Right after a command the grip state can still show the previous result (e.g. 3 = object
    dropped) for a moment, so a stopped state only counts once the move was seen running
    (state 0) or after 0.3 s (a move to where the fingers already are never shows state 0)."""
    t0 = time.monotonic()
    seen_moving = False
    time.sleep(0.05)
    while True:
        s = g.state()
        elapsed = time.monotonic() - t0
        if log is not None:
            log.append([round(elapsed, 3), label, *s])
        seen_moving |= s[1] == 0
        if (s[1] != 0 and (seen_moving or elapsed > 0.3)) or elapsed > timeout:
            return s, elapsed
        time.sleep(0.05)


def do_init(g, full):
    print(f"Initialising ({'full: finds both ends' if full else 'standard'}). The fingers will move.")
    g.write(REG_INIT, 0xA5 if full else 0x01)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 15.0:
        time.sleep(0.2)
        init, grip, pos = g.state()
        if init == 1:
            print(f"  done in {time.monotonic() - t0:.1f} s. {describe(init, grip, pos)}")
            return True
    print("  NOT initialised after 15 s. " + describe(*g.state()))
    return False


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


def guided_test(g):
    log = []
    print("\n=== 1. Communication (read-only) ===")
    s = g.state()
    print("  " + describe(*s))
    print("\nSafety: keep fingers and cables away from the gripper jaws. Force will be set to the")
    print("minimum (20 %) and speed to 30 %. Nothing should be in the gripper for steps 2-3.")
    if ask("Continue? [y/N] ") != "y":
        return log
    print("\n=== 2. Initialise ===")
    if s[0] != 1 or ask("Already initialised. Initialise again anyway? [y/N] ") == "y":
        if not do_init(g, full=False):
            return log
    g.write(REG_FORCE, 20)
    g.write(REG_SPEED, 30)
    print("  force 20 %, speed 30 % set")

    print("\n=== 3. Moves (nothing in the gripper) ===")
    start = g.read(REG_ACTUAL_POS)
    # First a small move away from where initialisation left the fingers, to learn the direction
    first = 700 if start > 500 else 300
    for target in (first, 1000, 0, 500):
        g.write(REG_POS, target)
        (init, grip, pos), dt = wait_settled(g, log=log, label=f"move to {target}")
        print(f"  target {target:4d} -> {describe(init, grip, pos)}  ({dt:.2f} s)")
        if target == 1000:
            open_answer = ask("  Are the fingers now fully OPEN (wide) or CLOSED? [o/c] ")
    print("  So 1000 per mille = " + ("OPEN, 0 = CLOSED" if open_answer == "o" else
                                     "CLOSED, 0 = OPEN" if open_answer == "c" else "(not answered)"))

    print("\n=== 4. Grip a light object (optional) ===")
    if ask("Put a light object (e.g. a pen, a marker) between the fingers, hands clear, then press y [y/N] ") == "y":
        close_target = 0 if open_answer == "o" else 1000
        open_target = 1000 - close_target
        for force in (20, 50):
            g.write(REG_POS, open_target)
            wait_settled(g, log=log, label="open")
            g.write(REG_FORCE, force)
            g.write(REG_POS, close_target)
            (init, grip, pos), dt = wait_settled(g, log=log, label=f"close force {force}")
            print(f"  close at force {force} % -> {describe(init, grip, pos)}  ({dt:.2f} s)")
        g.write(REG_FORCE, 20)
        g.write(REG_POS, open_target)
        wait_settled(g, log=log, label="release")
        print("  released")
    print("\nDone. The gripper holds its last position.")
    return log


def save_log(log, name):
    if not log:
        return
    folder = os.path.expanduser("~/gripper_logs")
    os.makedirs(folder, exist_ok=True)
    path = os.path.join(folder, time.strftime("%Y%m%d_%H%M%S") + f"_{name}.csv")
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "step", "init_state", "grip_state", "position_permille"])
        w.writerows(log)
    print(f"Log: {path}")


def selftest():
    """No hardware: the tool must produce the manual's example frames byte for byte."""
    manual = {
        (0x06, REG_INIT, 0x01): "01 06 01 00 00 01 49 F6",
        (0x06, REG_INIT, 0xA5): "01 06 01 00 00 A5 48 4D",
        (0x06, REG_FORCE, 30): "01 06 01 01 00 1E 59 FE",
        (0x06, REG_POS, 500): "01 06 01 03 01 F4 78 21",
        (0x03, REG_POS, 1): "01 03 01 03 00 01 75 F6",
        (0x06, REG_SPEED, 50): "01 06 01 04 00 32 48 22",
        (0x03, REG_SPEED, 1): "01 03 01 04 00 01 C4 37",
        (0x03, REG_INIT_STATE, 1): "01 03 02 00 00 01 85 B2",
        (0x03, REG_GRIP_STATE, 1): "01 03 02 01 00 01 D4 72",
        (0x03, REG_ACTUAL_POS, 1): "01 03 02 02 00 01 24 72",
    }
    ok = True
    for (func, reg, val), expected in manual.items():
        got = frame(1, func, reg, val).hex(" ").upper()
        ok &= got == expected
        print(f"  {'OK ' if got == expected else 'BAD'} {got}")
    print("selftest passed" if ok else "selftest FAILED")
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["selftest", "status", "init", "pos", "force", "speed", "watch", "test"])
    ap.add_argument("value", nargs="?", type=float)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--full", action="store_true", help="init: full initialisation (0xA5)")
    ap.add_argument("-v", "--verbose", action="store_true", help="print every Modbus frame")
    a = ap.parse_args()
    if a.cmd == "selftest":
        sys.exit(0 if selftest() else 1)
    if not os.path.exists(a.port):
        sys.exit(f"{a.port} does not exist. Is the RS-485 adapter attached (usbipd on WSL2)? "
                 "Check with: ls /dev/ttyUSB* /dev/ttyACM*")
    g = Gripper(a.port, a.baud, a.id, verbose=a.verbose)
    log = []
    try:
        if a.cmd == "status":
            print(describe(*g.state()))
        elif a.cmd == "init":
            do_init(g, a.full)
        elif a.cmd == "pos":
            g.write(REG_POS, int(in_range("position", int(a.value if a.value is not None else -1), 0, 1000)))
            (init, grip, pos), dt = wait_settled(g, log=log, label=f"pos {int(a.value)}")
            print(f"{describe(init, grip, pos)}  ({dt:.2f} s)")
        elif a.cmd == "force":
            g.write(REG_FORCE, in_range("force", int(a.value if a.value is not None else -1), 20, 100))
            print(f"force set; reads back {g.read(REG_FORCE)} %")
        elif a.cmd == "speed":
            g.write(REG_SPEED, in_range("speed", int(a.value if a.value is not None else -1), 1, 100))
            print(f"speed set; reads back {g.read(REG_SPEED)} %")
        elif a.cmd == "watch":
            end = time.monotonic() + (a.value or 10.0)
            while time.monotonic() < end:
                print(describe(*g.state()))
                time.sleep(0.2)
        else:
            log = guided_test(g)
    except ModbusError as e:
        print(f"\nModbus error: {e}")
        print("If nothing answers: check 24 V power, A/B wires (swap black/blue if unsure; it is safe),\n"
              "GND between adapter and supply, and --baud/--id (defaults 115200 / 1). Use -v to see frames.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        g.close()
        save_log(log, a.cmd)


if __name__ == "__main__":
    main()
