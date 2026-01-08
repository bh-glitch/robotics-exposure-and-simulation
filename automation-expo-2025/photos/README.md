**# Robot Learning, Perception, and Planning – Hands-on Exploration**

This repository documents my early hands-on exploration of robot manipulation, perception, and planning through both **real-world exposure** and **simulation-based experimentation**.

The goal of this work was not to build a production-ready robot system, but to deeply understand:
- How robots represent the world using coordinates and geometry
- How motion is planned through joint-level control
- Why real-world manipulation is difficult, unstable, and unintuitive
- How perception, planning, and control interact in a manipulation pipeline

This repository contains:
1. Observations and documentation from the **Automation Expo (Mumbai, Summer 2025)**  
2. A **PyBullet-based robotic arm simulation** exploring pick-and-place behavior

Together, these experiences shaped my interest in robot learning and motivated my application to the **Robot Learning, Perception, and Planning** research project.

---

## Repository Sections

### 1. Automation Expo (Mumbai, Summer 2025)
First-hand exposure to industrial robotic systems including:
- High-speed pick-and-place arms
- Autonomous mobile robots (AMRs) carrying heavy loads
- Drones used for warehouse inspection and inventory counting

📁 See: `automation-expo-2025/`

---

### 2. Robotic Arm Simulation (PyBullet)
A virtual manipulation pipeline exploring:
- Inverse kinematics (IK)
- End-effector positioning
- Smooth trajectory execution
- Stability issues such as oscillation and overshoot

📁 See: `simulation/`

---

## Why this matters
These experiences helped me realize that robotic manipulation is not about issuing a “grab” command, but about:
- Precise coordinate reasoning
- Independent joint control
- Timing, forces, and physical constraints
- Debugging subtle failure modes

This repository reflects my learning process and curiosity-driven exploration of robotics.
