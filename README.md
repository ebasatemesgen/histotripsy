# Histotripsy Robotics Simulation

This repository contains a simulation setup for a histotripsy treatment project using a UR5 robotic arm. The focus is on developing and simulating underwater robotic mechanisms, particularly the manipulation of an end-effector submerged in water.

## Project Overview

Histotripsy is a non-invasive therapeutic technique that uses focused ultrasound waves to treat targeted tissues. In this project:

- **Robot Model:** A UR5 robotic arm is used for end-effector control.
- **Simulation Goals:** Simulate the underwater environment where the robot manipulates an end-effector for precise operations related to histotripsy treatment.
- **Current Challenges:** Water buoyancy forces and resistance are yet to be effectively implemented in the simulation.

## Current Progress

1. **UR5 Robot Integration:** The UR5 robotic arm has been successfully modeled in the simulation.
2. **End-Effector Submersion:** Initial setup includes the motion of the end-effector into a water-like environment.
3. **Challenges:**
   - The water in the simulation behaves as a solid object, preventing realistic submersion of the end-effector.
   - Buoyancy and water resistance forces are not yet implemented.

   See [problem_1.gif](./problem_1.gif) for a demonstration of the current issue.

## Work in Progress

- **Buoyancy Modeling:**
  - Implementing realistic water buoyancy forces to simulate resistance and floatation effects.
  - Exploring tools like UUV Simulator and custom physics models for fine-grained underwater simulation.

## Setup Instructions

### Prerequisites
- Python 3.8 or higher
- ROS 2 Humble
- Gazebo simulator
- UR5 robot model files

### Installation
1. Clone this repository:
   ```bash
   git clone https://github.com/your-username/histotripsy-robotics.git
   cd histotripsy-robotics
   ```

2. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Launch the simulation:
   ```bash
   ros2 launch ur5_simulation simulation.launch.py
   ```

### Usage
- Control the UR5 robot using the provided scripts in the `control` directory.
- Adjust the water environment parameters in the `config/environment.yaml` file.
- Monitor simulation outputs and logs in the `logs/` directory.

## Contributions

Contributions to improve the simulation and address the buoyancy issue are welcome! Please follow these steps to contribute:

1. Fork the repository.
2. Create a new branch for your feature/fix.
3. Submit a pull request describing your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

Special thanks to Professor Maria Gini for guidance and the robotics research group at the University of Minnesota for support.
