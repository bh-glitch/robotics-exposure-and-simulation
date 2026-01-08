"""

# 📄 Top-of-file comment (already good, but keep this)

#At the **very top** of `kuka_pick_and_place.py`, keep:

```python"""
"""
KUKA iiwa Pick-and-Place Simulation

This script demonstrates:
- Inverse kinematics control
- Smooth end-effector motion
- Grasping logic in a physics-based simulator

Note:
This is an exploratory learning project.
Instability, oscillation, and failed grasps are expected
and were part of the learning process.
"""


import time
import math
import pybullet as p
import pybullet_data

DT = 1.0 / 240.0


def step(n=1):
    # Guard against the common "Not connected to physics server" crash.
    if not p.isConnected():
        raise RuntimeError(
            "PyBullet is not connected. Close any old PyBullet windows, stop other running main.py processes, and run again."
        )
    for _ in range(n):
        p.stepSimulation()
        time.sleep(DT)


def set_nice_camera():
    # Clean, stable view: table centered, no weird angle.
    p.resetDebugVisualizerCamera(
        cameraDistance=1.05,
        cameraYaw=55,
        cameraPitch=-28,
        cameraTargetPosition=[0.75, 0.0, 0.42],
    )


def arm_joint_indices(robot_id, count=7):
    # KUKA iiwa main arm joints
    joints = []
    for j in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, j)
        jtype = info[2]
        name = info[1].decode("utf-8")
        if jtype == p.JOINT_REVOLUTE and "iiwa_joint" in name:
            joints.append(j)
    # fallback: first 7 revolute joints
    if len(joints) < count:
        joints = []
        for j in range(p.getNumJoints(robot_id)):
            if p.getJointInfo(robot_id, j)[2] == p.JOINT_REVOLUTE:
                joints.append(j)
    return joints[:count]


def finger_joint_indices(robot_id):
    fingers = []
    for j in range(p.getNumJoints(robot_id)):
        name = p.getJointInfo(robot_id, j)[1].decode("utf-8").lower()
        if "finger" in name:
            fingers.append(j)
    # conservative fallback
    if not fingers:
        n = p.getNumJoints(robot_id)
        if n >= 2:
            fingers = [n - 2, n - 1]
    return fingers


def setup_physics():
    # More stable contacts / less jitter (and fewer "explosions")
    p.setPhysicsEngineParameter(
        fixedTimeStep=DT,
        numSubSteps=4,
        numSolverIterations=400,
        solverResidualThreshold=1e-7,
        enableConeFriction=1,
    )


def setup_robot_dynamics(robot_id):
    # Damping reduces twitching
    for link in range(-1, p.getNumJoints(robot_id)):
        p.changeDynamics(robot_id, link, linearDamping=0.06, angularDamping=0.06)


def hold_arm(robot_id, arm_joints, targets, force=55):
    # Softer control to avoid twitching / throwing the cube.
    for i, j in enumerate(arm_joints):
        p.setJointMotorControl2(
            robot_id,
            j,
            p.POSITION_CONTROL,
            targetPosition=targets[i],
            force=force,
            positionGain=0.04,
            velocityGain=0.25,
            maxVelocity=0.8,
        )


def solve_ik(robot_id, ee_link, pos, orn):
    jd = [0.12] * p.getNumJoints(robot_id)
    sol = p.calculateInverseKinematics(
        robot_id,
        ee_link,
        pos,
        orn,
        jointDamping=jd,
        maxNumIterations=150,
        residualThreshold=1e-4,
    )
    return sol


def move_ee_smooth(robot_id, ee_link, start_pos, end_pos, orn, arm_joints, seconds=1.2):
    steps = max(1, int(seconds / DT))
    for k in range(steps):
        t = (k + 1) / steps
        # smoothstep
        t = t * t * (3 - 2 * t)
        pos = [
            start_pos[0] + (end_pos[0] - start_pos[0]) * t,
            start_pos[1] + (end_pos[1] - start_pos[1]) * t,
            start_pos[2] + (end_pos[2] - start_pos[2]) * t,
        ]
        sol = solve_ik(robot_id, ee_link, pos, orn)
        targets = [sol[j] for j in arm_joints]
        hold_arm(robot_id, arm_joints, targets, force=90)
        step(1)


def gripper_open(robot_id, finger_joints, open_pos=0.04, seconds=0.4):
    steps = max(1, int(seconds / DT))
    for _ in range(steps):
        for j in finger_joints:
            p.setJointMotorControl2(
                robot_id,
                j,
                p.POSITION_CONTROL,
                targetPosition=open_pos,
                force=140,
                positionGain=0.2,
                velocityGain=0.6,
            )
        step(1)


def gripper_close(robot_id, finger_joints, close_pos=0.0, seconds=0.7):
    steps = max(1, int(seconds / DT))
    for _ in range(steps):
        for j in finger_joints:
            p.setJointMotorControl2(
                robot_id,
                j,
                p.POSITION_CONTROL,
                targetPosition=close_pos,
                force=120,
                positionGain=0.18,
                velocityGain=0.55,
            )
        step(1)


def maybe_attach(robot_id, ee_link, cube_id, dist_thresh=0.05):
    # Attach when the end-effector is close enough to the cube.
    ee_pos = p.getLinkState(robot_id, ee_link)[4]
    cube_pos, _ = p.getBasePositionAndOrientation(cube_id)
    dx = ee_pos[0] - cube_pos[0]
    dy = ee_pos[1] - cube_pos[1]
    dz = ee_pos[2] - cube_pos[2]
    if (dx * dx + dy * dy + dz * dz) ** 0.5 > dist_thresh:
        return None

    cid = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=ee_link,
        childBodyUniqueId=cube_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        # Hold the cube slightly below the tool to avoid initial interpenetration
        parentFramePosition=[0, 0, 0.05],
        childFramePosition=[0, 0, 0],
    )
    # Too large a force can cause unstable impulses; keep it reasonable.
    p.changeConstraint(cid, maxForce=220)

    # IMPORTANT: once "picked", stop the cube from colliding with the robot.
    # Otherwise tiny penetrations can create huge impulses and the cube gets flung.
    for link in range(-1, p.getNumJoints(robot_id)):
        p.setCollisionFilterPair(robot_id, cube_id, link, -1, enableCollision=0)

    return cid


def main():
    # Connect to the physics server / GUI
    cid = p.connect(p.GUI)
    if cid < 0:
        raise RuntimeError(
            "Could not connect to PyBullet GUI. Close any existing PyBullet windows, then rerun: python main.py"
        )
    p.setRealTimeSimulation(0)
    try:
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        setup_physics()

        # Cleaner UI / nicer look
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        set_nice_camera()

        # World
        p.loadURDF("plane.urdf")
        p.loadURDF("table/table.urdf", basePosition=[0.8, 0.0, 0.0])

        # Object
        cube_start = [0.78, 0.0, 0.66]
        cube_id = p.loadURDF("cube_small.urdf", basePosition=cube_start)
        p.changeDynamics(
            cube_id,
            -1,
            lateralFriction=1.3,
            rollingFriction=0.001,
            spinningFriction=0.001,
            restitution=0.0,
            linearDamping=0.05,
            angularDamping=0.05,
        )

        # Robot (spawn it clearly *in front* of the table and pin the base so it can't fall through)
        robot_id = p.loadSDF("kuka_iiwa/kuka_with_gripper2.sdf")[0]

        # Put the base on the floor, a bit in front of the table
        robot_base_pos = [0.25, 0.0, 0.0]
        p.resetBasePositionAndOrientation(robot_id, robot_base_pos, [0, 0, 0, 1])

        # Pin the base to the world (more reliable than just setting mass=0 in some builds)
        base_cid = p.createConstraint(
            parentBodyUniqueId=robot_id,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=robot_base_pos,
        )
        p.changeConstraint(base_cid, maxForce=1_000_000)

        setup_robot_dynamics(robot_id)

        # Re-center the camera after the robot loads
        set_nice_camera()

        ee_link = 6
        arm_joints = arm_joint_indices(robot_id, count=7)
        finger_joints = finger_joint_indices(robot_id)

        # Safe, stable starting pose (not all zeros)
        home = [0.0, 0.55, 0.0, -1.25, 0.0, 1.05, 0.0]
        for i, j in enumerate(arm_joints):
            p.resetJointState(robot_id, j, home[i])

        # Let the world settle
        step(240)

        # Point tool "down" toward the table
        down_orn = p.getQuaternionFromEuler([math.pi, 0, 0])

        # Plan: detect cube -> smooth approach -> final correction -> grasp -> lift -> (optional) place
        def get_cube_pos():
            pos, _ = p.getBasePositionAndOrientation(cube_id)
            return list(pos)

        def pick_cube():
            # 1) Read cube position (this is the "robot calculates where the cube is" step)
            cube_pos = get_cube_pos()

            # 2) Waypoints based on the current cube pose
            approach = [cube_pos[0] - 0.10, cube_pos[1], cube_pos[2] + 0.25]
            above =    [cube_pos[0],        cube_pos[1], cube_pos[2] + 0.25]
            grasp =    [cube_pos[0],        cube_pos[1], cube_pos[2] + 0.070]
            lift =     [cube_pos[0],        cube_pos[1], cube_pos[2] + 0.35]

            # Start from current EE pose for smoothness
            ee_pos_now = list(p.getLinkState(robot_id, ee_link)[4])

            # Open gripper, then approach
            gripper_open(robot_id, finger_joints, open_pos=0.04, seconds=0.35)
            move_ee_smooth(robot_id, ee_link, ee_pos_now, approach, down_orn, arm_joints, seconds=1.2)
            move_ee_smooth(robot_id, ee_link, approach, above, down_orn, arm_joints, seconds=0.9)

            # 3) Final correction: re-check cube pose in case it drifted slightly
            cube_pos2 = get_cube_pos()
            above2 = [cube_pos2[0], cube_pos2[1], cube_pos2[2] + 0.25]
            grasp2 = [cube_pos2[0], cube_pos2[1], cube_pos2[2] + 0.070]

            move_ee_smooth(robot_id, ee_link, above, above2, down_orn, arm_joints, seconds=0.35)
            move_ee_smooth(robot_id, ee_link, above2, grasp2, down_orn, arm_joints, seconds=1.2)

            # Close and attach
            gripper_close(robot_id, finger_joints, close_pos=0.0, seconds=0.7)
            step(40)
            cube_pos2 = get_cube_pos()  # refresh once more before attaching
            cid = maybe_attach(robot_id, ee_link, cube_id, dist_thresh=0.035)
            print("Attached:", bool(cid))

            # If we failed to attach, back up a bit and return
            if not cid:
                back_up = [cube_pos2[0] - 0.10, cube_pos2[1], cube_pos2[2] + 0.25]
                move_ee_smooth(robot_id, ee_link, grasp2, back_up, down_orn, arm_joints, seconds=0.8)
                return None

            # Lift with the cube
            move_ee_smooth(robot_id, ee_link, grasp2, lift, down_orn, arm_joints, seconds=1.2)
            return cid, lift

        def place_cube(cid, start_pos):
            # Drop spot that is clearly visible on the table
            drop_above = [0.70, -0.20, 0.92]
            drop_down  = [0.70, -0.20, 0.70]

            move_ee_smooth(robot_id, ee_link, start_pos, drop_above, down_orn, arm_joints, seconds=1.1)
            move_ee_smooth(robot_id, ee_link, drop_above, drop_down, down_orn, arm_joints, seconds=0.8)

            # Release
            try:
                p.removeConstraint(cid)
            except Exception:
                pass

            # Re-enable collisions between cube and robot now that we're releasing it
            for link in range(-1, p.getNumJoints(robot_id)):
                p.setCollisionFilterPair(robot_id, cube_id, link, -1, enableCollision=1)

            gripper_open(robot_id, finger_joints, open_pos=0.04, seconds=0.35)

            # Small retreat
            retreat = [drop_above[0] - 0.12, drop_above[1], drop_above[2]]
            move_ee_smooth(robot_id, ee_link, drop_down, retreat, down_orn, arm_joints, seconds=0.9)

        # Do one clean pick-and-place cycle
        result = pick_cube()
        if result:
            cid, lifted_pos = result
            place_cube(cid, lifted_pos)

        # Idle loop
        while True:
            step(1)
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()