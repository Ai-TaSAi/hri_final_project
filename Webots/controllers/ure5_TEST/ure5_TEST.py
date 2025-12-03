from controller import Robot, Keyboard
import time

# -----------------------------
# Webots setup
# -----------------------------
TIME_STEP = 32
robot = Robot()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)

# -----------------------------
# Motors (first 4 joints)
# -----------------------------
joint_names = [
    "shoulder_pan_joint",   # joint 1
    "shoulder_lift_joint",  # joint 2
    "elbow_joint",          # joint 3
    "wrist_1_joint"         # joint 4
]

motors = [robot.getDevice(name) for name in joint_names]

# Initialize motors for position control
for m in motors:
    m.setPosition(0.0)  
    m.setVelocity(0.5)  # slower speed for precision

# -----------------------------
# Joint angles tracking
# -----------------------------
joint_angles = [0.0, 0.0, 0.0, 0.0]  # radians
delta = 0.02  # smaller increment for precise control

# -----------------------------
# Main loop
# -----------------------------
# while robot.step(TIME_STEP) != -1:
    # key = keyboard.getKey()
    
    # while key != -1:
        # if key == ord('Q') or key == ord('q'):
            # joint_angles[0] += delta
        # elif key == ord('W') or key == ord('w'):
            # joint_angles[0] -= delta
        # elif key == ord('A') or key == ord('a'):
            # joint_angles[1] += delta
        # elif key == ord('S') or key == ord('s'):  # changed from D to S
            # joint_angles[1] -= delta
        # elif key == ord('Z') or key == ord('z'):
            # joint_angles[2] += delta
        # elif key == ord('X') or key == ord('x'):
            # joint_angles[2] -= delta
        # elif key == ord('O') or key == ord('o'):  # joint 4 +
            # joint_angles[3] += delta
        # elif key == ord('P') or key == ord('p'):  # joint 4 -
            # joint_angles[3] -= delta
        # elif key == ord('M') or key == ord('m'):
            # rounded_angles = [round(a, 2) for a in joint_angles]
            # print(f"Joint angles (radians): {rounded_angles}")
        
        
        # key = keyboard.getKey()
    
#    # Update motors
    # for i, m in enumerate(motors):
        # m.setPosition(joint_angles[i])
        
starting_position = [0, -1.14, 2.32, -1.22]
        
three_balls = [
    [-0.58, 0.12, 0.42, -1.22],
    [-0.08, 0.14, 0.42, -0.92],
    [0.34, 0.24, 0.14, -1.02]
]

five_balls = [
    [-0.58, 0.12, 0.42, -1.22],
    [-0.34, 0.16, 0.46, -1.1],
    [-0.08, 0.14, 0.42, -0.92],
    [0.1, 0.18, 0.46, -1.22],
    [0.34, 0.24, 0.14, -1.02]
]

seven_balls = [
    [-0.6, 0.2, 0.12, -1.12],
    [-0.40, 0.2, 0.12, -1.12],
    [-0.32, 0.26, 0.24, -0.74],
    [-0.12, 0.26, 0.24, -0.88],
    [0.06, 0.26, 0.24, -0.74],
    [0.22, 0.3, 0.12, -1.12],
    [0.42, 0.3, 0.12, -1.12]
]

selector = 3

# -----------------------------
# Helper function: move to target joint positions
# -----------------------------
def move_to_position(target_angles, delay=3.0):
    """Moves all joints simultaneously to the target angles and waits."""
    # Set all joint target positions at once
    for i, m in enumerate(motors):
        m.setPosition(target_angles[i])

    # Wait while robot moves (they all move together now)
    start_time = time.time()
    while time.time() - start_time < delay:
        robot.step(TIME_STEP)
        
# -----------------------------
# Choose which sequence to use
# -----------------------------
if selector == 3:
    sequence = three_balls
elif selector == 5:
    sequence = five_balls
elif selector == 7:
    sequence = seven_balls
else:
    print("Invalid selector number. Must be 3, 5, or 7.")
    sequence = []

# -----------------------------
# Main sequence loop
# -----------------------------
if sequence:
    print(f"Starting movement sequence for {selector} positions.")
    
    # Move to starting position
    move_to_position(starting_position, delay=6.0)

    # Loop through each target
    for pos in sequence:
        move_to_position(pos, delay=6.0)         # Move to target
        move_to_position(starting_position, 6.0) # Return to start

    print("Sequence complete.")
else:
    print("No valid sequence found. Exiting.")