 

## Nanotec Stepper motor PD4-C6018L4204-E-08 and PD4-C5918l4204-E-08 position control integration on ROS2 using ros2_canopen library

![image](https://github.com/user-attachments/assets/002a7ba7-7b30-46b9-a33c-a5e4819d9919)



- - - - - - - - - - - - - - - - - -

# NEEDED 

- Ubuntu 22.04.5 LTS 

- [ROS2 Desktop Humble installed](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
      

- [ROS2_CANOPEN library installed in your workspace](https://ros-industrial.github.io/ros2_canopen/manual/rolling/quickstart/installation.html)
   
   Make sure that you use the right branch (humble) otherwise the building won't work :     

      https://github.com/ros-industrial/ros2_canopen/issues/217

  Run these commands to change branch : 

      cd ros2_canopen

      git checkout Humble

- [Stepper Motors PD4-C6018L4204-E-08 Nanotec Datasheet](https://www.nanotec.com/fileadmin/files/Handbuecher/Plug_Drive/PD4-C/fir-v2213/PD4C_CANopen_Technical-Manual_V3.3.0.pdf?1663072012)  


- [PCAN-USB FD Adapter](https://www.peak-system.com/PCAN-USB-FD.365.0.html?&L=1)  

      

 - - - - - - - - - - - - - - - - - -


# Download the GitHub Package and build/launch it 

### Before downloading the package, you should install and build the ros2_canopen library in your workspace

Run the following commands in the /ros2_canopen folder of your new workspace to download the GitHub package :  

    git clone https://github.com/MatHelio314/stepper_motor_ws.git

Build the package :  

    colcon build --symlink-install 
    source install/setup.bash 

You are free to perform two modifications on the ros2_canopen code. 
The first one is to get torque feedback from the motors with [these changes](https://github.com/ros-industrial/ros2_canopen/pull/316/files), I merged my library with this one. 
Be careful, the merging will bring back the branch problem (not humble anymore), my solution was to only copy canopen_402_driver and canopen_ros2_control files from this new merge and keep the previous other files (mainly the core is causing issues with the ros2 version)

The second change is about homing timeout : 
![image](https://github.com/user-attachments/assets/eb373e13-9f3a-4365-ae92-12b9c8056130)
Here I set the timemout to 90 seconds

To know more about why i did these changes, you can read the "Ideas & Improvements" section below


Run the launch files (can0 needs to be UP and RUNNING and connected to at least one motor (can_id 1,2,3 or/and 4)):  

#### Please make sure that you are using the negative limit switch if the parameter are enabling it before launching

 Normal mode :

    ros2 launch stepper_motor_control main.launch.py sim:=false

 Visualization (rviz2) : 

    ros2 launch stepper_motor_control main.launch.py sim:=true

 If you run visualization, make sure that the connected motors are uncommented in this file : 
 ![image](https://github.com/user-attachments/assets/8e554283-75fe-4189-a68b-ea307e0015b2)

- - - - - - - - - - - - - - - - - -

 
# Connect the motors to the computer through a CAN interface :  

See README_WIRING.md for more informations on the wiring of the motors : https://github.com/MatHelio314/stepper_motor_ws/blob/main/README_WIRING.md

 ### I'm using a Peak USB to CAN adapter here but any other adapter with a recognizable driver on your computer would do (see daatsheet above)

   Step 1 : Power up the stepper motors with the corresponding power supplies (24V & 5A) 
   
the Led underneath of the motor should now blink consistently
   
   Step 2 : Connect the CAN output of the adapter to the stepper motors in parallel (CAN_H, CAN_L and GND) 
   
   Step 3 : Plug the USB to CAN adapter to the computer 
   
   Step 4 : Run the Peak_usb driver already installed on Ubuntu 22.04/Linux :
   
    sudo modprobe peak_usb 
    
You should see the led lighting up in green on the adapter 

   Step 5 : Set up the can interface and set it's parameters :  

    sudo ip link set can0 up type can bitrate 1000000 
    sudo ip link set can0 txqueuelen 1000 
    sudo ip link set can0 up 

   Step 6 : Verify that the can interface is up and running  

    ifconfig can0 

OR 

    ip link show can0 
    
And look at the data passing through the can interface (install can_utils first) :  

    sudo apt install can-utils

    candump can0 

You should not see anything in the candump for now and that is normal (my advice is to keep this command running for the futur to visualize the data being transmitted) 

 - - - - - - - - - - - - - - - - - -

# Change the node_id of each stepper motor  

Look for the Hex coding switch for node-ID and baud rate underneath 
The PD4-C is equipped with a hex coding switch. This can be used to set the source for the node-ID and the baud rate.

![image](https://github.com/user-attachments/assets/ec655696-0511-425e-b5d1-e50e5ee60e5f)

 - - - - - - - - - - - - - - - - - -

# Change the parameters of the motors 

You can directly set parameters on the bus_template.yml file (See README_BUS_FILE.md for more details) : https://github.com/MatHelio314/stepper_motor_ws/blob/main/README_BUS_FILE.md



What i recommend doing if clarifications are needed is to get the nanotec USB to CAN adapter ZK-USB-CAN-1 and use it with the Plug & Drive Studio 3 software available on their website (windows only):  

    https://www.nanotec.com/eu/en/products/2256-software-for-brushless-dc-servo-motor-controller 

The software lets you modify the parameters as well as to do some tests, i personnaly used this software to solve many issues with the motors.  
Also check if the firmware is up to date (the software can update it for you), it is also possible to factory reset the motors to avoid any problems. 


 - - - - - - - - - - - - - - - - - -

# Control the motors  

 

After launching the file, you will see a slider gui and buttons to control the motors.

![image](https://github.com/user-attachments/assets/72020cb3-621f-43a7-a27c-7df3aa246ce2)

See PACKAGES_OVERVIEW.md for more informations on the nodes.


The gui allows you to reset the motors, initialize them, launch position control nodes and closes them as well as stopping the motors in case of an emergency


The principle behing the position control relies on these three services that you can manually call yourself either by running a bash command or by using the rqt service caller client


    "Motor's joint name"/init 
    
    "Motor's joint name"/(cyclic position, velocity, torque, …) mode 
    
    "Motor's joint name"/target ( {double} ) 

 

-----------------------------------

# Ideas & Improvements

## Boot up issue (SOLVED)

The ROS2 Canopen program occasionally failed to establish communication with the motors, leading to frequent launching errors. The solution was to set up a specific operating mode within the launch configuration file (bus_template.yml). It does not really matter which operation mode is selected, as long as it is not 'NONE', the device manager will be able to boot up the motor. (in SDO)

    - {index: 0x6060, sub_index: 0, value: 1} # Set profile position mode

## Inconsistent behavior (SOLVED)

One of the motors exhibited inconsistent behavior compared to the others, which created complications in achieving uniform control. I found out that the NanoJ program inside it had been modified previously. This program is stored into an EEPROM and it is executed after the motor powers on. By copying the program from one of the correctly functioning motors and uploading it into the affected motor, the behavior normalized, effectively solving the issue. 
I used [Plug & Drive Studio 3](https://www.nanotec.com/eu/en/products/10492-plug-drive-studio-3) with a [nanotec USB to CAN adapter](https://www.nanotec.com/eu/en/products/9987-zk-usb-can-1) to upload the new NanoJ program

## Perfect synchronization

Right now, the two X axis motor control is not perfect, we can measure at some points a 60 to 100 tenth of degrees offset between the two motors which accounts for about 140 µm offset in the mecanical setup. We still need to measure the actual offset tolerance of the setup when it will be finished but the idea is to minimize this offset as much as possible and achieve a perfect 0 offset if it is possible. 
My current method to achieve synchronization is to send a position target value to both motors at the 'same time' using the async_send_request but it seems like it is causing a slight delay of 0.3ms between the two motors feedback, or maybe it is only the heartbeat of the two motors not beeing completely synchronized. 


## Limit switch security 

If no negative limit switch is wired to the motor and the parameters are set to a homing mode different from 35 (default) then there is a security flaw if the homing mode is launched looking for a limit switch, i should implement a function that checks if a switch is connected before launching the homing mode


## Torque feedback (SOLVED)

For now we can only get position and velocity feedback because the tpdo and rpdo mappings are limiting the feedback to these two units. I need to research how theses mappings work in order to add the torque feedback functionnality.

I modified the ros2_canopen code based on [these changes](https://github.com/ros-industrial/ros2_canopen/pull/316/files), I merged my library with this one.
Be careful, the merging will bring back the branch problem (not humble anymore), my solution was to only copy canopen_402_driver and canopen_ros2_control files from this new merge and keep the previous other files (mainly the core is causing issues with the ros2 version)

## GUI improvements (SOLVED)

Add the possiblity to choose the units in a responsive and intuitive way for the user.
Show position, velocity and torque feedback on screen compared to demanded values.
Change input velocity (in Rev/Min) via the gui.

New gui looks : 
![image](https://github.com/user-attachments/assets/45092ea6-c5f2-4b01-a094-e618cb67eaac)

The only problem is that the velocity change button does not work every time


## Homing mode timeout (SOLVED)

After 10 secs, the homing mode stops and claims that it failed, it is not very convenient when the motor is further than 42mm of the limit switch.

the problem came from the ros2_canopen library, in this particular file : 
![image](https://github.com/user-attachments/assets/eb373e13-9f3a-4365-ae92-12b9c8056130)

Here I set the timemout to 90 seconds

## Limit switch during operation

for now, the limit switch is only used for homing mode and if it is pushed while not being in that mode, it will issue no response on the motor. The behavior i want is that if the limit switch is pressed outside of homing mode, the motor should stop immediatly

## Emergency stop during homing mode

For now, the only way to stop the homing mode is to press the limit switch (or powering off the motor of course). I need to implement a button that stops the homing mode quickly




Concerning the meshes for the visualization launch, if you want to modify them,you have to download a .stl file and convert it into a .stl binary to make it work. 
