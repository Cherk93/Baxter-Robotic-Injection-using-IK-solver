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
7. We then need to convert the camera( Inter_real_sense) Frame of Reference to Robot Frame of Reference. For this we used the [Baxter Introduction PDF](BAXTER_Introduction_2_08_2016.pdf)
8. For example - In our case, using this we developed a transformation matrix from the torso of the robot to the camera frame as below: <br>

<img src="https://github.com/Cherk93/Baxter-Robotic-Injection-using-IK-solver/assets/83907972/33fe3eaa-ad0e-4e6d-86e3-1f78b38f30c1" width="200" height="200">


## Code to Run Robotic Injection 
1. You first need to execute the ***IRS_CV_ver2.py*** script in the ROS_MASTER. This has a publish node that will send the 3D pixel information as 'Point' class datatype
2. Then you can execute the ***Baxter_control.py*** script that has a subscriber node that subscribed to this point datatype information
3. A rotation matrix operation is performed to transform the coordinates from camera frame to robot torso frame
4. We then use the forward kinematics and inverse kinematics functions included in the Baxter SDK. [^2]  to move the robot to its initial posture and then move the left arm end effector to the target coordinates. 
5. The Baxter control subscriber node waits until it receives the `move` command from the operator to start publishing the IK solution for joints to the `baxter_interface` node which in turn will move the robot to the final position. 
6. Finally Baxter controller calls the forward kinematics code in the `baxter_interface` function class to make the end effector perform a final forward kinematics maneuver to emulate injection on the human subject. 



### Example Results: 





  [^1]: https://wiki.ros.org/noetic/Installation/Ubuntu
  [^2]: https://github.com/RethinkRobotics/baxter?tab=readme-ov-file
  [^3]: https://github.com/RethinkRobotics/baxter_examples
