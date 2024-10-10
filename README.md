# Turtlesim Gesture Control with ROS2 and OpenCV

## Project Overview

This project aims to develop a system that allows controlling a **Turtlesim** simulation in **ROS2** using **hand gestures** captured by a camera. By leveraging **OpenCV** for hand gesture recognition and translating those gestures into movement commands, we enable intuitive, real-time control of the Turtlesim simulation environment without the need for a keyboard.

### Key Features
- **Gesture Recognition with OpenCV:** Detect hand gestures through real-time camera input and map them to specific movement commands.
- **ROS2 Communication:** Use ROS2 to publish gesture-based commands (`geometry_msgs/Twist`) to control Turtlesim.
- **Modular Design:** Separate components for gesture recognition, command conversion, and Turtlesim control to ensure flexibility and scalability.
  
### Technologies
- **C++** for performance and integration with ROS2.
- **OpenCV** for hand gesture detection.
- **CUDA** for GPU-accelerated processing of image data, improving performance.
- **ROS2 (Robot Operating System 2)** for robot communication and control.
- **Turtlesim** as the primary simulation environment to visualize and test the project.
