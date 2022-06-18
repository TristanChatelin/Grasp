# Grasp_pipeline
<a name="Install the GPD python binding"></a>
This pipeline requires a GPU to be run. Detectron2 needs to be installed. It can be tricky to install it. Firstly make sure that nvcc -V display that the Cuda version is 11.3. If Cuda 11.3 is not installed, intall it. During the installation of Cuda one's needs to make sure that the latest version of Cuda is not installing. For that, it's necessary to add "-11-3" to the last instruction to install cuda. Then, install Pytorch for Cuda 11.3. Then, install Detectron2. Then install ZED_SDK, it's important to install ZED SDK after Detectron2, because it uses Cuda 11.5. Moreover, the "from detectron import *" statements needs to be run in the installation file "detectron2". The latest version of Detectron2 have import issue, so it's better to install the version 0.6.
## 1) Install the GPD python binding
```
git clone https://github.com/patricknaughton01/gpd  
cd gpd
git clone https://github.com/pybind/pybind11
mkdir build
cd build
cmake .. -DBUILD_PYTHON=ON
make -j
```
<a name="Install AprilTag"></a>
## 2) Install AprilTag
In the file pipeline/scripts/AprilTag, run the following instruction:
```
./install.sh
```
## 3) Grasp using the real robot
Firstly, the robot needs to be switched on, to do it run:
1. Make sure that MiR is fully charged, the cable to charge should be near the robot (it has a blue end) and the outlet is in the back right corner of MiR. Usually takes about 2-3 hours to charge fully.
2. To turn on the MiR, you first have to twist the knob on the front left corner. This is hidden so you'll have to get on your knees if you have never done this step before to know exactly where this is. This is not the end of the startup process however!
3. Grab the tablet from the side of the control box. This is for the UR10 but it is neccessary in this step. Turn it on and wait for it to be done (it takes a little while). 
4. Once it is done, you'll see a few popups either talking about initialization not being done or the emergency stop button being pressed. Ignore those for now (i.e. press not now). Go to program robot, the press on the tab that says "I/O". 
5. Once you've reached here, you should see a lot of buttons that can be pressed. Bottom left of the screen is a section called Configurable Output. Press on the button that has the number 0. You should hear a slight click from the inside of the control box
6. You should see a button flashing blue. Press that too. You should hear a louder click now. Once this is done, press on the coloured light on the very top.
7. Here, you might see a few different things, usually the robot has already booted up by now and only needs to be powered on. Simply press on the on button. In case you get a safeguard error, go back to the I/O tab and press on the number 1 in the configuration output section (this resets the safeguard) but don't forget to turn it off again!
8. Go back to where you were before and now you should be able to start the robot. You will hear a few cracking noises, don't worry as those are the brakes being disengaged. 
9. You can now manually move the end effector by using the Move tab! This is, however, not what we're looking for.
10. (This step is only done once!) Now you need to connect your computer/laptop to the robot. Use the ethernet cable and connect it to your computer, then go to your network and click on the settings for the wired connection. Go to the IPv4 tab and click on Manual. This should allow you to add in the IP address to be used: type in 192.168.1.101 and if there is a field for subnet mask type in 255.255.255.0. Save this and now it is initialized so you don't have to do this every time
11. Go back to the tablet and switch to the installation tab and scroll down until you see external control in the list. Click on that and type in the following: for the IP address, type in 192.168.1.101, for the port, use 50002, for the host name, use the name you have for your computer (host name shouldn't matter).
12. In the UR monitor, go to "Installation" > "EtherNet/IP" and press "Disable". "PROFINET" can be disabled as well, but shouldn't make a difference.  
13. Go to the program section and start a new empty program. Switch to Structure on the right and choose URCaps, then external control. There should now be a play button on the bottom left to start the program. Don't do that yet. The robot is now set up (to the best of our knowledge atm).
14. Launch the launch file in mobile_manipulator called testing_real_arm.launch that is very bare-bones and only launches the robot models, controllers and the moveit launch file from the config folder (also copied from original, but that is irrelevant atm).
15. In a separate terminal, launch a launch file called ur10_bringup.launch using the following line:
16. 
### Terminal 1:
```
    roslaunch ur_robot_driver ur10_bringup.launch robot_ip:=192.168.1.56 reverse_port:=50001
```  
This command will also start the ZED2 camera. A blue light should appear on the camera.
50001 should be default, but please make sure that port on the UR tablet is set to 50002 and reverse_port on your computer is set to 50001. (since 50001 is the default, you can actually omit that part in the command).

16. Press the play button in the lower left corner on the UR tablet.  
  
Once this is done, you can begin sending commands to the arm! To simplify that, we can start moveit to control it. To do that, you need two new terminals

### Terminal 2:
Run this command in the second terminal:

```
    rosun pipeline get_pose_robot.py
```

### Terminal 3:
Run this command in the third terminal:

```
    roslaunch ur10_moveit_config move_group.launch
```

### Terminal 4:
Run this command in the fourth terminal to visualize everything:

```
    roslaunch ur10_moveit_config moveit_rviz.launch
```

### Terminal 5:
Run this command to start the moveit controller that checks for the grasp poses:

```
    rosrun mobile_manipulator grasp_pose_subscriber.py
```
### Terminal 6:
Run this command to publish the grasp pose:


```
    rosrun pipeline get_grasp_poses_from_camera_data_publihser.py
```

