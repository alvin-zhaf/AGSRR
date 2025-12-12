# Aerial-Supported Ground Search and Rescue Simulation

This project simulates a cooperative multi-robot system for Search and Rescue operations using **Webots**. It demonstrates how an Unmanned Aerial Vehicle (UAV) can collaborate with an Unmanned Ground Vehicle (UGV) to improve victim localization and rescue response times.

The project compares two approaches:

1.  **Cooperative Mode:** A drone scans the area, locates victims via camera, and transmits coordinates to a ground robot for retrieval.
2.  **Autonomous Ground Mode:** The ground robot explores the area alone using SLAM and obstacle avoidance to find victims.

---

## Key Features

### 1. UAV Controller (Mavic 2 Pro)

- **Path Planning:** Executes a Boustrophedon (lawnmower) flight pattern to ensure 100% scanning coverage of the designated area.
- **Computer Vision:** Uses onboard camera processing to detect victims based on color segmentation (specifically **Green** targets).
- **Communication:** Transmits victim coordinates $(X, Y)$ wirelessly to the ground robot via Webots Emitter (Channel 1).
- **Feedback:** "Tags" victims by changing their shirt color to Red upon detection to prevent duplicate reporting.

### 2. UGV Controller (Surveyor SRV-1)

- **Dual Operation Modes:**
  - **Mode 1 (Drone Guided):** Waits for coordinate packets from the UAV and navigates directly to the target.
  - **Mode 2 (Autonomous SLAM):** Navigates a waypoint list independently while mapping the environment.
- **Obstacle Avoidance:** Uses IR distance sensors to detect obstacles and calculate detours dynamically.
- **Mapping:** Implements a simple Occupancy Grid (SLAM) to map free space, obstacles, and victim locations.
- **State Machine:** Robust architecture handling navigation, obstacle checks, victim approach, and alignment.

---

## Prerequisites

- **Webots Simulator:** Version **R2025a** (or compatible recent version).
- **C Compiler:** (GCC or similar, usually bundled with Webots setup).

---

## How to Run the Simulation

1.  **Open the World:**
    Launch Webots and open the world file located at:
    worlds/Scenario1.wbt
    worlds/Scenario2.wbt

2.  **Compile Controllers:**

    - Webots should ask to build the controllers automatically.
    - If not, go to the menu: `Tools` -> `Clean all`, then `Tools` -> `Build all`.

3.  **Start the Simulation:**
    Press the Play button in Webots.

4.  **Select Ground Robot Mode:**
    The Surveyor robot waits for user input to select its operating mode.

    - Click inside the **3D View** window to focus keyboard input.
    - Press **`1`**: **Autonomous Navigation (Drone Guided)**. The robot waits for coordinates sent by the flying drone.
    - Press **`2`**: **Autonomous Navigation with SLAM**. The robot starts its own search pattern using predefined waypoints and mapping.

> You might need to clean the drone build if you are trying to run the SLAM mode due to the drone changing the colour of the victim shirts.

5.  **View Outputs:**
    - **Console:** Displays mission logs, victim detections, and navigation states.
    - **Map/Victims:** In Mode 2, press **`M`** to print the ASCII map or **`V`** to list found victims in the console.

> Open the world file again to re-run the simulation if the victim’s shirt is red.

---

## Project Structure

```
.
├── controllers/
│   ├── mavic2pro/           # UAV Controller
│   │   ├── mavic2pro.c      # Main flight logic, vision, and comms
│   │   └── Makefile
│   └── surveyor/            # UGV Controller
│       ├── surveyor.c       # Ground navigation, SLAM, and obstacle avoidance
│       └── Makefile
├── protos/                  # Custom Webots Objects
│   ├── RescueVictimFaceUp.proto
│   ├── RescueVictimSitting.proto
│   └── SurveyorSrv1.proto
├── report/                  # Project Documentation
│   └── report.tex           # Project report
└── worlds/
    └── RobotSearch.wbt      # Main simulation environment

```

---

## Project Demo

Demo Video: [here](https://drive.google.com/file/d/1e05G08bL1-pva0Lc204qMuGQwJu-_X1q/view?usp=sharing).

## Team Members

- Edward J. Silvey
- Alfian Fadhlurrahman
- Shailee D. Kampani
- Alvin Zhafif Afilla
