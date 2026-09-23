"""Arm kinematics for teleop, parsed from the URDF: forward kinematics, geometric Jacobian,
and a damped least-squares step that slides along reach and joint limits instead of stopping.

Pure numpy (no ROS), so it can be tested offline against the real URDF.
"""
import math
import xml.etree.ElementTree as ET

import numpy as np


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_rotation(axis, angle):
    """Rotation matrix for `angle` about unit vector `axis`."""
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(angle) * k + (1.0 - math.cos(angle)) * k @ k


def rotation_from_vector(w):
    """Rotation matrix for rotation vector `w` (axis * angle)."""
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    return axis_rotation(np.asarray(w) / angle, angle)


def rotation_error(r_goal, r):
    """Rotation vector (world frame) that turns orientation `r` into `r_goal`."""
    r_err = r_goal @ r.T
    angle = math.acos(max(-1.0, min(1.0, (np.trace(r_err) - 1.0) / 2.0)))
    s = math.sin(angle)
    if angle < 1e-9 or s < 1e-6:  # identical, or ~180 deg away (never happens in small teleop steps)
        return np.zeros(3)
    v = np.array([r_err[2, 1] - r_err[1, 2], r_err[0, 2] - r_err[2, 0], r_err[1, 0] - r_err[0, 1]])
    return angle / (2.0 * s) * v


def smoothstep(s):
    s = min(1.0, max(0.0, s))
    return s * s * (3.0 - 2.0 * s)


class ArmKinematics:
    def __init__(self, urdf_xml, base_link, tip_link, joint_names):
        root = ET.fromstring(urdf_xml)
        by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
        chain, link = [], tip_link
        while link != base_link:
            if link not in by_child:
                raise ValueError(f"no joint chain from {base_link} to {tip_link}")
            joint = by_child[link]
            chain.append(joint)
            link = joint.find("parent").get("link")
        chain.reverse()

        self.steps = []  # (fixed origin transform, joint axis or None, index into joint_names or None)
        for joint in chain:
            origin = joint.find("origin")
            xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0.0] * 3
            rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()] if origin is not None else [0.0] * 3
            t = np.eye(4)
            t[:3, :3] = _rpy_matrix(*rpy)
            t[:3, 3] = xyz
            jtype = joint.get("type")
            if jtype in ("revolute", "continuous"):
                axis_el = joint.find("axis")
                axis = np.array([float(v) for v in (axis_el.get("xyz") if axis_el is not None else "1 0 0").split()])
                name = joint.get("name")
                if name not in joint_names:
                    raise ValueError(f"joint {name} in the chain is not in {joint_names}")
                self.steps.append((t, axis / np.linalg.norm(axis), joint_names.index(name)))
            elif jtype == "fixed":
                self.steps.append((t, None, None))
            else:
                raise ValueError(f"unsupported joint type {jtype}")
        if sum(1 for s in self.steps if s[1] is not None) != len(joint_names):
            raise ValueError("chain does not contain every joint in joint_names")
        self.n = len(joint_names)

    def fk(self, q):
        """Tip pose (4x4) plus, per joint (joint_names order), its world axis and world position."""
        t = np.eye(4)
        axes, points = [None] * self.n, [None] * self.n
        for origin, axis, idx in self.steps:
            t = t @ origin
            if axis is not None:
                axes[idx] = t[:3, :3] @ axis
                points[idx] = t[:3, 3].copy()
                rot = np.eye(4)
                rot[:3, :3] = axis_rotation(axis, q[idx])
                t = t @ rot
        return t, axes, points

    def jacobian(self, q):
        """Tip pose, 6xN geometric Jacobian (linear rows first, world frame), joint positions."""
        t, axes, points = self.fk(q)
        tip = t[:3, 3]
        jac = np.zeros((6, self.n))
        for i in range(self.n):
            jac[:3, i] = np.cross(axes[i], tip - points[i])
            jac[3:, i] = axes[i]
        return t, jac, points


def path_min_height(kin, q_start, q_goal, samples=25):
    """Lowest world z of the joints after the base (elbow, wrist, ...) and the tip along the
    smooth straight joint-space path from q_start to q_goal."""
    q_start, q_goal = np.asarray(q_start, dtype=float), np.asarray(q_goal, dtype=float)
    lowest = float("inf")
    for k in range(samples + 1):
        t, _, points = kin.fk(q_start + smoothstep(k / samples) * (q_goal - q_start))
        lowest = min([lowest, float(t[2, 3])] + [float(pt[2]) for pt in points[1:]])
    return lowest


def dls_step(kin, q, dp, dw, r_hold, limits, max_step, damping, rot_weight, rot_gain, lock=(), pos_weight=1.0):
    """One teleop step from joint vector `q`.

    dp: wanted tip translation this step (m, world); dw: wanted tip rotation this step (rad).
    The orientation is also pulled back toward `r_hold`. Orientation is weighted by
    `rot_weight`, so near the edge of reach the position keeps following and the wrist gives a
    little. Joints that would cross their limits are locked and the others take over.
    If the resulting tip motion would point well away from the pushed direction (the pushed
    direction is blocked and only sideways motion is left), the step is shrunk or dropped, so
    the tip slides along an edge for a slightly-off push but never wanders off sideways.
    `lock`: joint indices that must not move. `pos_weight` 0 lets the tip position go free (used
    for wrist-only rotation, with the shoulder and elbow locked).
    Returns (new q, locked joint indices, predicted tip translation, speed scale applied).
    """
    q = np.asarray(q, dtype=float)
    t, jac, _ = kin.jacobian(q)
    err = np.concatenate([np.asarray(dp, dtype=float),
                          np.asarray(dw, dtype=float) + rot_gain * rotation_error(r_hold, t[:3, :3])])
    w2 = np.diag([pos_weight ** 2] * 3 + [rot_weight ** 2] * 3)
    locked = list(lock)
    dq = np.zeros(kin.n)
    for _ in range(kin.n):
        jl = jac.copy()
        jl[:, locked] = 0.0
        dq = np.linalg.solve(jl.T @ w2 @ jl + damping ** 2 * np.eye(kin.n), jl.T @ w2 @ err)
        q_new = q + dq
        newly = [i for i in range(kin.n) if i not in locked and
                 ((q_new[i] < limits[i][0] and dq[i] < 0) or (q_new[i] > limits[i][1] and dq[i] > 0))]
        if not newly:
            break
        locked += newly
    for i in range(kin.n):  # never push a joint further past its limit
        lo, hi = limits[i]
        if (q[i] + dq[i] < lo and dq[i] < 0) or (q[i] + dq[i] > hi and dq[i] > 0):
            dq[i] = 0.0
    scale = 1.0
    biggest = float(np.max(np.abs(dq)))
    if biggest > max_step:
        scale = max_step / biggest
        dq *= scale
    dx = (jac @ dq)[:3]
    dp = np.asarray(dp, dtype=float)
    dp_norm, dx_norm = float(np.linalg.norm(dp)), float(np.linalg.norm(dx))
    if dp_norm > 1e-9 and dx_norm > 1e-12:
        alignment = float(dx @ dp) / (dx_norm * dp_norm)  # 1 = exactly the pushed direction
        keep = min(1.0, max(0.0, (alignment - 0.5) / 0.3))  # full within ~35 deg, none beyond 60 deg
        dq *= keep
        dx *= keep
    return q + dq, locked, dx, scale
