# Introduction to ROS -- Basic Examples

This repository contains **basic ROS (Robot Operating System) code** to help beginners get started with ROS concepts such as nodes, topics, and packages. The code is intended as an introduction for students learning **ROS 2** for the first time, and is used in the online course I taught on **Robotics and Robot Operating Systems** at [TUXSA](https://www.skilllane.com/courses/tuxsa-Robotics-and-Robot-Operating-Systems).  

The examples are tested on **ROS 2 Humble**.  

---

## Docker Setup Instructions

👉 It is recommended to run these examples using Docker:  
[AI-Robot-Book-Humble/docker-ros2-desktop-ai-robot-book-humble](https://github.com/AI-Robot-Book-Humble/docker-ros2-desktop-ai-robot-book-humble)

### Build Image
```bash
git clone https://github.com/AI-Robot-Book-Humble/docker-ros2-desktop-ai-robot-book-humble
cd docker-ros2-desktop-ai-robot-book-humble
./build.bash
```

### Run on Linux/macOS
#### Pull Image
```bash
./arb-pull.bash
```

#### Start Container
```bash
./arb-run.bash
```

### Run on Windows
#### Pull Image
```bash
arb-pull.bat
```

#### Start Container
```bash
arb-run.bat
```
### Using Desktop Environment

#### Using Web Browser
Connect to: http://127.0.0.1:6080
Password: ubuntu

#### Using VNC Viewer
Connect to: http://127.0.0.1:15900
Password: ubuntu

------------------------------------------------------------------------

## 📦 Installation & Setup

### 1. Clone the repository

Clone this repo inside the `src` folder of your ROS 2 workspace (usually
named `ros2_ws`):

``` bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/your-repo-name.git
```

### 2. Build the workspace

Once cloned, build the workspace using `colcon`:

``` bash
cd ~/ros2_ws
colcon build
```

### 3. Source the setup file

After building, don't forget to source your workspace:

``` bash
source install/setup.bash
```

------------------------------------------------------------------------

## 🚀 Usage

Each package in this repository demonstrates a different **basic ROS 2
concept**.\
After sourcing, you can run the nodes using:

``` bash
ros2 run <package_name> <node_name>
```

For example:

``` bash
ros2 run my_first_package talker
ros2 run my_first_package listener
```

------------------------------------------------------------------------

## 📚 What You'll Learn

- How to create and build **ROS 2 packages**  
- Writing simple **publishers** and **subscribers**  
- Using **colcon** to build and manage workspaces  
- Running **ROS 2 nodes**  
- Working with **services** and **actions** in ROS 2  
- Implementing **localization** and **mapping**  
- Developing **path planning** and **navigation** with a custom goal package  

------------------------------------------------------------------------

## 📝 Notes

-   Make sure you have sourced your ROS 2 installation before building:

    ``` bash
    source /opt/ros/<distro>/setup.bash
    ```

-   Replace `<distro>` with your ROS 2 distribution (e.g., `humble`).

------------------------------------------------------------------------

## 🤝 Contributing

Feel free to open issues or submit pull requests if you find bugs or
want to add more beginner-friendly examples.

------------------------------------------------------------------------

## 📖 References

-   [ROS 2 Documentation](https://docs.ros.org/en/rolling/)\
-   [Colcon Documentation](https://colcon.readthedocs.io/)
