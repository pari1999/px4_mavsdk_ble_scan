# PX4 BLE Scan Project

This repository contains the autonomous mission control system for Scanning BLE Tags , built on top of the PX4 flight stack and MAVSDK.

## 📂 Repository Structure

This project uses a split-repository approach:

*   **`PX4-Autopilot/`** (Submodule): Points to the official [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) repository. It contains the firmware and simulation environment.
*   **`mavlink_scripts/`**: Contains the custom Python code for the autonomous mission.
    *   `core/`: Main mission logic (`intelligent_mission.py`).
    *   `localization/`: Particle filter and Gaussian Process mapping.
    *   `simulation/`: Bluetooth tag simulation and environment setup.
    *   `interface/`: Visualization and dashboard tools.
*   **`scan-dashboard/`**: Web-based dashboard for mission monitoring (WIP).

## 🚀 Getting Started

### 1. Clone the Repository
Since this repo uses submodules, clone it recursively:
```bash
git clone --recursive https://github.com/your-username/px4_mavsdk_ble_scan.git
cd px4_mavsdk_ble_scan
```
*If you already cloned it without `--recursive`, run:* `git submodule update --init --recursive`

### 2. Prerequisites
- **PX4 Toolchain**: Follow the [PX4 Dev Guide](https://docs.px4.io/main/en/dev_setup/building_px4.html) to install toolchains for your OS.
- **Python 3.8+**: Required for MAVSDK scripts.
- **MAVSDK**: Install the Python library:
  ```bash
  pip install mavsdk numpy matplotlib websockets pyyaml
  ```

### 3. Running the Simulation

#### Step A: Start PX4 SITL
Open a terminal and launch the PX4 simulation:
```bash
cd PX4-Autopilot
make px4_sitl gazebo
```

#### Step B: Run the Mission
Open a new terminal and run main mission script:
```bash
python3 mavlink_scripts/core/intelligent_mission.py
```

## 🧠 System Overview
The system implements an "Intelligent Mission" controller that:
1.  **Takes off** to a safe scanning altitude.
2.  **Explores** the area using a Gaussian Process (GP) to predict finding new tags.
3.  **Localizes** detected Bluetooth tags using a Particle Filter.
4.  **Returns** to launch once all tags are found or time expires.

## 🛠 Git Management
- **PX4 Updates**: The `PX4-Autopilot` folder is a submodule. It points to a specific commit of the upstream repo. To update it:
  ```bash
  cd PX4-Autopilot
  git checkout main
  git pull
  cd ..
  git add PX4-Autopilot
  git commit -m "Update PX4 submodule"
  ```
