# ure5_TEST.py
from controller import Supervisor
import numpy as np
import math

TIME_STEP = 32
robot = Supervisor()

# First 4 joints (motors + sensors)
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
]

motors = []
sensors = []
for name in joint_names:
    motors.append(robot.getDevice(name))
    sensors.append(robot.getDevice(name + "_sensor"))

for m in motors:
    m.setPosition(0.0)
    m.setVelocity(2.0)
for s in sensors:
    s.enable(TIME_STEP)

# UR-like geometry (used by forward_kinematics)
d1 = 0.089159
a2 = -0.425
a3 = -0.39225
d4 = 0.10915
tool_offset_x = 0.1


def get_joint_angles():
    """Return current joint sensor values as a length-4 numpy array."""
    return np.array([float(s.getValue()) for s in sensors], dtype=float)


def rot_z(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def forward_kinematics(q):
    """
    Approximate FK for first 4 joints (returns position [x,y,z] in robot base frame).
    q: length-4 array (q1,q2,q3,q4)
    """
    q1, q2, q3, q4 = q
    p = np.array([0.0, 0.0, d1])
    R = rot_z(q1)
    R = R @ rot_y(q2)
    p = p + R @ np.array([a2, 0.0, 0.0])
    R = R @ rot_y(q3)
    p = p + R @ np.array([a3, 0.0, 0.0])
    R = R @ rot_y(q4)
    p = p + R @ np.array([0.0, 0.0, -d4])
    p = p + R @ np.array([tool_offset_x, 0.0, 0.0])
    return p


def axis_angle_to_R(axis, angle):
    """Axis-angle (axis, angle) -> rotation matrix (3x3)."""
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.eye(3)
    axis = axis / norm
    x, y, z = axis
    c = np.cos(angle)
    s = np.sin(angle)
    C = 1 - c
    return np.array([[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
                     [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
                     [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])


def world_to_base(p_world):
    """Convert a world-frame point to the robot base frame."""
    node = robot.getSelf()
    trans = np.array(node.getField("translation").getSFVec3f(), dtype=float)
    rot = np.array(node.getField("rotation").getSFRotation(), dtype=float)
    R_base = axis_angle_to_R(rot[:3], rot[3])
    p_world = np.array(p_world, dtype=float)
    p = R_base.T @ (p_world - trans)
    return np.array([-p[0], p[1], -p[2]])


def read_balls_world_positions():
    """Return a sorted list of world-frame positions for all Ball nodes in the root children."""
    root = robot.getRoot()
    children = root.getField("children")
    balls = []
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node.getTypeName() == "Ball":
            pos = node.getField("translation").getSFVec3f()
            balls.append(list(pos))
    balls.sort(key=lambda p: p[0])
    return balls


# IK state
_last_dq3 = np.zeros(3)
_last_cmd = None


def ik_step_pos_only(target_pos,
                     gain=2.0,
                     damping=0.08,
                     tol=0.008,
                     max_delta=0.08,
                     eps=1e-6,
                     smoothing_alpha=0.6,
                     cmd_blend_alpha=0.2):
    """Single IK iteration for joints 0..2. Returns True if within tol."""
    global _last_dq3, _last_cmd

    q = get_joint_angles()
    if np.any(np.isnan(q)):
        return False

    ee_pos = forward_kinematics(q)
    error = np.array(target_pos, dtype=float) - ee_pos
    err_norm = np.linalg.norm(error)
    if err_norm < tol:
        _last_dq3 = np.zeros(3)
        return True

    # numeric Jacobian (central difference)
    J = np.zeros((3, 3))
    for i in range(3):
        dq = np.zeros_like(q)
        dq[i] = eps
        f_plus = forward_kinematics(q + dq)
        f_minus = forward_kinematics(q - dq)
        J[:, i] = (f_plus - f_minus) / (2.0 * eps)

    JJt = J @ J.T
    lam = (damping**2) * max(1.0, np.linalg.norm(J))
    try:
        inv_term = np.linalg.inv(JJt + lam * np.eye(3))
    except np.linalg.LinAlgError:
        return False
    J_pinv = J.T @ inv_term

    raw_dq3 = J_pinv @ (error * gain)

    # clip big jumps
    raw_nrm = np.linalg.norm(raw_dq3)
    if raw_nrm > max_delta:
        raw_dq3 = raw_dq3 * (max_delta / raw_nrm)

    # low-pass filter
    dq3 = smoothing_alpha * _last_dq3 + (1.0 - smoothing_alpha) * raw_dq3
    _last_dq3 = dq3.copy()

    if np.linalg.norm(dq3) < 1e-5 and err_norm < (tol * 3.0):
        return True

    q_new = q.copy()
    q_new[:3] = q[:3] + dq3

    joint_min = np.array([-3.14, -3.14, -3.14, -3.14])
    joint_max = np.array([3.14, 3.14, 3.14, 3.14])
    q_new = np.minimum(np.maximum(q_new, joint_min), joint_max)

    # initialize _last_cmd if needed
    if _last_cmd is None:
        _last_cmd = q.copy()

    q_cmd = (1.0 - cmd_blend_alpha) * _last_cmd + cmd_blend_alpha * q_new
    _last_cmd = q_cmd.copy()

    if np.any(np.isnan(q_cmd)):
        print("[WARN] q_cmd contains NaN — skipping motor updates")
        return False

    for i, m in enumerate(motors):
        if math.isfinite(q_cmd[i]):
            m.setPosition(float(q_cmd[i]))
        else:
            print(f"[WARN] skipping motor {i} non-finite cmd {q_cmd[i]}")

    return False


def move_to_target(target, wrist_angle, steps):
    """Move to target (target is in robot base frame)"""
    if wrist_angle is not None and math.isfinite(wrist_angle):
        motors[3].setPosition(float(wrist_angle))
        for _ in range(10):
            if robot.step(TIME_STEP) == -1:
                return

    pos_target = target[:3]
    for _ in range(steps):
        if robot.step(TIME_STEP) == -1:
            break
        done = ik_step_pos_only(pos_target, gain=2.0, damping=0.08, tol=0.008, max_delta=0.08)
        if done:
            for _ in range(8):
                if robot.step(TIME_STEP) == -1:
                    break
            break


def move_to_start(joint_targets, steps):
    """Move back to the start position"""
    for step in range(steps):
        if robot.step(TIME_STEP) == -1:
            break
        alpha = (step + 1) / steps
        q_current = get_joint_angles()
        if np.any(np.isnan(q_current)):
            continue
        q_blend = q_current * (1 - alpha) + np.array(joint_targets) * alpha
        for i, m in enumerate(motors):
            if math.isfinite(q_blend[i]):
                m.setPosition(float(q_blend[i]))

    for i, m in enumerate(motors):
        if math.isfinite(joint_targets[i]):
            m.setPosition(float(joint_targets[i]))


def main():
    starting_position = [0.0, -1.14, 2.32, -1.22]

    for i, m in enumerate(motors):
        if math.isfinite(starting_position[i]):
            m.setPosition(starting_position[i])

    for _ in range(3):
        if robot.step(TIME_STEP) == -1:
            break

    balls_world = read_balls_world_positions()
    if len(balls_world) == 0:
        print("No Ball nodes found in scene. Exiting.")
    else:
        print(f"Found {len(balls_world)} balls (world): {balls_world}")

    for _, ball_world in enumerate(balls_world):
        target_base = world_to_base(ball_world)
        approach_offset = np.array([0.0, 0.1, 0.18])
        target_base_adjusted = target_base + approach_offset

        move_to_target(target_base_adjusted, -1.1, 250)
        move_to_start(starting_position, 140)

    print("All targets processed.")


if __name__ == "__main__":
    main()
