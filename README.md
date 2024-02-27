# Baxter-Robotic-Injection-using-IK-solver
## Introduction 
To demonstrate robotic injection by using Baxter robot to scan a surface(2D/3D) and make precise pokes/injections at target points located at different depths from the end effector. This project would require the combination of Computer vision, Visual Servoing and Robot-Camera integration using the ROS platform.

<img src="/Baxter and me.jpg" width="500" height="500">

## Baxter Robot Setup 
1. You need to have Ubuntu ver.16 with ROS Kinetic to interface with Baxter 
2. Quick and dirty way to setup Kinetic, [^1] : <br>
        `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'` <br>
        `sudo apt install curl # if you haven't already installed curl
            curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -` <br>
       `sudo apt update` <br>
       `sudo apt install ros-kinetic-desktop-full` <br>
   
3. Install Baxter SDK files to interface with the robot and source baxter.sh file to your ROS environment, [^2]
4. From command line try to ping ROS MASTER on host PC on Baxter: <br>
` ping <IP Address of Baxter>`
5. Enable the Robot <br>
'rosrun baxter_interface enable_robot.py -e'
6. You could run several example codes that are given in the Rethink robotics Github repository [^3]

## Code to Run Robotic Injection 
1. 








  [^1]: https://wiki.ros.org/noetic/Installation/Ubuntu
  [^2]: https://github.com/RethinkRobotics/baxter?tab=readme-ov-file
  [^3]: https://github.com/RethinkRobotics/baxter_examples
