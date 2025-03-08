# **ros2_subscriber_service**  

This ROS 2 package implements:  
- A **subscriber** node that listens to a topic.  
- A **service client** node that sends requests to a service.  

This package is built using **rclpy** (the ROS 2 Python client library) and is structured for easy integration into a ROS 2 workspace.  

---

## **Installation & Setup**  

### **1. Clone the Repository**  
Navigate to your ROS 2 workspace `src` directory and clone the repository:  
```sh
# Linux
cd ~/ros2_ws/src
# Windows PowerShell
cd C:\path\to\ros2\workspace\src  

git clone https://github.com/Vai-Man/ros2_subscriber_service.git
```

### **2. Build the Package**  
Move to your workspace root and build the package:  
```sh
# Linux
cd ~/ros2_ws
# Windows PowerShell
cd C:\path\to\ros2\workspace  

colcon build --packages-select ros2_subscriber_service
```

### **3. Source the Environment**  
Before running the nodes, source the setup file:  
```sh
# Linux
source install/setup.bash  
# Windows PowerShell
source install/setup.ps1  
```

---

## **Setup ROS 2 Environment**  

Before running any ROS 2 commands, you must source both the ROS 2 installation and your workspace setup files. Update the paths accordingly:  

```sh
# Windows Command Prompt / PowerShell
call C:\path\to\ros2\setup.bat  
call C:\path\to\workspace\install\local_setup.bat  
```

🔹 **Replace `C:\path\to\ros2\setup.bat`** with the actual path to your ROS 2 installation.  
🔹 **Replace `C:\path\to\workspace\install\local_setup.bat`** with the path to your workspace setup file.  

---

## **Usage**  

### **1. Run the Subscriber**  
The subscriber listens to a topic (e.g., `/sensor_data`) and processes incoming messages:  
```sh
ros2 run ros2_subscriber_service subscriber
```

### **2. Run the Service Client**  
The client sends requests to a service and receives responses:  
```sh
ros2 run ros2_subscriber_service service_client
```

### **3. Publish a Message**  
You can publish a message to the `/sensor_data` topic to see the nodes in action:  
```sh
ros2 topic pub /sensor_data std_msgs/msg/String "{data: '5'}"
```
The subscriber should log the received message, and the service client should process the data.

---

## **Dependencies**  

These dependencies are installed automatically when you build the package, but if needed, you can manually install them:  
```sh
pip install setuptools rclpy std_msgs
```

---
