"""DH-Robotics CGE-10-10 gripper over Modbus RTU (RS-485). Plain pyserial, no ROS.

Same protocol code as tools/gripper_test.py (hardware-tested 2026-09-23 on the real gripper
through a CH340 USB-RS485 adapter, slave ID 1, 115200 8N1). Register map from the DH CGE-10
short manual, section 2.3, cross-checked against DH-Robotics' dh_gripper_driver and pyDHgripper.
Only the four control registers are ever written, never the 0x03xx configuration registers.

Measured on the real gripper: 1000 per mille = open, 0 = closed; full stroke ~0.7 s at speed 30 %;
grip state reads 65535 until the gripper is initialised.
"""
import time

import serial

REG_INIT, REG_FORCE, REG_POS, REG_SPEED = 0x0100, 0x0101, 0x0103, 0x0104
REG_INIT_STATE, REG_GRIP_STATE, REG_ACTUAL_POS = 0x0200, 0x0201, 0x0202
GRIP_MOVING, GRIP_AT_TARGET, GRIP_CAUGHT, GRIP_DROPPED = 0, 1, 2, 3
GRIP_TEXT = {GRIP_MOVING: "moving", GRIP_AT_TARGET: "at target, no object", GRIP_CAUGHT: "object caught",
             GRIP_DROPPED: "object dropped", 0xFFFF: "undefined (not initialised)"}


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


class DHGripper:
    def __init__(self, port, baud=115200, slave=1, timeout=0.3):
        self.ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=timeout)
        self.slave = slave

    def close(self):
        self.ser.close()

    def _transact(self, request, reply_len, retries=3):
        last = b""
        for _ in range(retries):
            self.ser.reset_input_buffer()
            self.ser.write(request)
            self.ser.flush()
            reply = self.ser.read(reply_len)
            if len(reply) >= 5 and reply[1] == (request[1] | 0x80) and crc16(reply[:3]) == reply[3:5]:
                raise ModbusError(f"gripper rejected the request (Modbus exception code {reply[2]})")
            if len(reply) == reply_len and crc16(reply[:-2]) == reply[-2:] and reply[0] == self.slave:
                time.sleep(0.005)  # inter-frame gap
                return reply
            last = reply
            time.sleep(0.05)
        raise ModbusError(f"no valid reply (got {len(last)} bytes: {last.hex(' ') or 'nothing'})")

    def read(self, reg):
        reply = self._transact(frame(self.slave, 0x03, reg, 1), 7)
        if reply[1] != 0x03 or reply[2] != 2:
            raise ModbusError(f"unexpected reply {reply.hex(' ')}")
        return (reply[3] << 8) | reply[4]

    def write(self, reg, value):
        request = frame(self.slave, 0x06, reg, value)
        if self._transact(request, 8) != request:  # a single-register write is echoed back
            raise ModbusError("write not confirmed")

    def state(self):
        """(initialised, grip state, actual position per mille)."""
        return self.read(REG_INIT_STATE) == 1, self.read(REG_GRIP_STATE), self.read(REG_ACTUAL_POS)

    def initialise(self, full=False):
        self.write(REG_INIT, 0xA5 if full else 0x01)

    def set_force(self, pct):
        self.write(REG_FORCE, int(min(100, max(20, pct))))

    def set_speed(self, pct):
        self.write(REG_SPEED, int(min(100, max(1, pct))))

    def set_position(self, permille):
        self.write(REG_POS, int(min(1000, max(0, permille))))
