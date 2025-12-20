[Chinese Simplified](README.md)|[English]  

**[A9 / TB100 IMU Online Tutorial](https://alidocs.dingtalk.com/i/p/NE0VzgAErrZmJepPRBGvENgrKpvBgmDA)**  
**[B9 / B6 IMU Online Tutorial](https://alidocs.dingtalk.com/i/p/NE0VzgAErrZmJepPRBGvaMMQ9WDBAmDA)**  

# 1. Taobotics IMU User Guide (ROS Environment)  

## 1.1 Installation  
### 1.1.1 Install Dependencies  
Use the following commands to install the serial communication dependency:  

```sh
pip install pyserial numpy matplotlib  # For Python 2  
pip3 install pyserial numpy matplotlib  # For Python 3  
```

### 1.1.2 Build  
The `handsfree_ros_imu` package consists of Python driver scripts, USB-mount `udev` rules, and ROS launch files.  

First, create a new workspace (you may use any name; here we assume `catkin_ws`):  

```bash
cd ~ && mkdir -p catkin_ws/src
```

Download **handsfree_ros_imu.zip** from our online documentation, then extract it into `~/catkin_ws/src`.  

In the extracted `handsfree_ros_imu` folder, open a terminal and run the installation script:  

```bash
cd ~/catkin_ws/src/handsfree_ros_imu/
bash auto_install.sh
```

## 1.2 Usage  

To make it easier to use later, add the workspace environment setup to your `~/.bashrc` (replace `~/catkin_ws/` with your actual workspace path):  

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### 1.2.1 handsfree_imu.launch  

* **Function:**  
  Runs the IMU driver, parses protocol data, and publishes ROS topics.  

* **Serial Port:**  
  `/dev/HFRobotIMU` (can be modified via the `~port` parameter).  

* **Published Topics:**  

  * `/handsfree/imu` ([sensor_msgs/Imu](https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html)) — includes quaternion orientation, angular velocity, and linear acceleration.  
  * `/handsfree/mag` ([sensor_msgs/MagneticField](https://docs.ros.org/en/api/sensor_msgs/html/msg/MagneticField.html)) — publishes magnetometer data.  

* **Launch Command:**  

  ```bash
  roslaunch handsfree_ros_imu handsfree_imu.launch imu_type:=a9 port:=/dev/HFRobotIMU
  ```

  `imu_type` options: `a9`, `tb100`, `b9`, `b6`.  

  * For **A9**, **B9**, and **B6**: use port `/dev/HFRobotIMU`.  
  * For **TB100**: use port `/dev/ttyACM0`.  

### 1.2.2 handsfree_imu_with_rviz.launch  

* **Function:**  
  Launches the IMU driver and opens **RViz** to visualize the IMU’s orientation in real time.  
  Launch parameters are the same as in `handsfree_imu.launch`.  

### 1.2.3 handsfree_imu_with_ui.launch  

* **Function:**  
  Launches the IMU driver with a graphical **UI** interface to visualize IMU attitude.  
  Launch parameters are the same as in `handsfree_imu.launch`.  


# 2. Taobotics IMU Usage (Non-ROS Environment)  

## 2.1 Python Program  
Use the following commands to install the serial communication dependency:  

```sh
pip install pyserial numpy matplotlib  # For Python 2  
pip3 install pyserial numpy matplotlib  # For Python 3  
```

Python examples are located in `./demo/python/`.  
For example, to run the **A9** IMU driver:  

* On **Linux**:  

  ```bash
  python python_driver_a9.py --port /dev/HFRobotIMU --baud 921600
  ```

* On **Windows**:  

  ```bash
  python python_driver_a9.py --port COM3 --baud 921600
  ```

  *(Replace `COM3` with your actual port number.)*  

## 2.2 C++ Program  

c++ examples are located in `./c++/python/`.  
Compilation and execution methods are explained in the comments at the top of each C++ source file.  
