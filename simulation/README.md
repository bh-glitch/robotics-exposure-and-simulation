# KUKA iiwa Pick-and-Place Simulation (PyBullet)

This folder contains a physics-based simulation of a robotic arm performing a basic pick-and-place task.

The simulation uses a **KUKA iiwa robotic arm with a parallel gripper**, operating over a table with a small cube.

---

## What This Simulation Demonstrates

- Reading object position directly from the simulator
- Planning a sequence of waypoints above, near, and at the object
- Using inverse kinematics to move the end effector
- Gripper opening, closing, and object attachment
- Stability challenges during contact and lifting

The robot:
1. Calculates the cube’s position
2. Approaches it smoothly from above
3. Attempts to grasp the cube
4. Lifts and places it at a new location

---

## Why Instability Is Expected

During development, the robot may:
- Oscillate near the object
- Miss or throw the cube
- Twitch due to competing joint constraints

These behaviors are **not bugs**, but realistic outcomes of:
- Inaccurate contact modeling
- High control gains
- Tight timing constraints
- Simplified grasp logic

Understanding *why* these failures occur was a major learning outcome.

---

## Key Techniques Used

- Inverse kinematics (PyBullet `calculateInverseKinematics`)
- Quaternion-based orientation control
- Smooth interpolation between waypoints
- Joint damping and force tuning
- Temporary rigid attachment to simulate grasping

---

## How to Run

```bash
conda activate robot311
python kuka_pick_and_place.py
