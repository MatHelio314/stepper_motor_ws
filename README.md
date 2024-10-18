 

Nanotec Stepper motor PD4-C6018L4204-E-08 ROS2 control integration on ROS2  

Needed  

 

    Ubuntu 22.04.5 LTS 

    ROS2 Desktop Humble installed 

    ROS2_CANOPEN library installed in the workspace 
 

Stepper Motors PD4-C6018L4204-E-08 Nanotec Datasheet :  

    https://www.nanotec.com/fileadmin/files/Handbuecher/Plug_Drive/PD4-C/fir-v2213/PD4C_CANopen_Technical-Manual_V3.3.0.pdf?1663072012 

 

PCAN-USB FD Adapter :  

    https://www.peak-system.com/PCAN-USB-FD.365.0.html?&L=1 

 

ROS2_canopen library :  

    https://github.com/ros-industrial/ros2_canopen 

Make sure that you use the right branch (humble) otherwise the building won't work :     

    https://github.com/ros-industrial/ros2_canopen/issues/217 

 

 

Connect the motors to the computer through a CAN interface :  

 

   Step 1 : Power up the stepper motors with the correspondiing power supplies (24V & 5A) 

 

   Step 2 : Connect the CAN output of the adapter to the stepper motors in parallel (CAN_H, CAN_L and GND) 

 

   Step 3 : Plug the USB to CAN adapter to the computer 

 

   Step 4 : Run the Peak_usb driver already installed on Ubuntu 22.04/Linux 

 

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

 

And look at the data passing through the can interface :  

 

    candump can0 

 

You should not see anything for now and that is normal (my advice is to keep this command running for the futur to visualize the data being transmitted) 

 

 

 

 

Change the node_id of each stepper motor  

 

Look for this Hex coding switch for node-ID and baud rate underneath 

 
Rotary switch The PD4-C is equipped with a hex coding switch — similar to that shown in the following figure. This can be used to set the source for the node-ID and the baud rate.

 

Change the parameters of the motors 

 

You can directly send CAN messages to the motors to modify it's parameters but it is a long process. 

 

What i recommend doing is to get the nanotec USB to CAN adapter ZK-USB-CAN-1 and use it with the Plug & Drive Studio software available on their website :  

    https://www.nanotec.com/eu/en/products/2256-software-for-brushless-dc-servo-motor-controller 

The software lets you modify the parameters as well as to do some tests. I personnaly used this software to solve a friction issue i had with one of the motors. If you have this issue, please check if the following parameters are similar :  

    0x3210:0x01=2048                                // 0x0000 0800 
    
    0x3210:0x02=0                                     // 0x0000 0000 
    
    0x3210:0x03=7000                                // 0x0000 1B58 
    
    0x3210:0x04=4                                     // 0x0000 0004 
    
    0x3210:0x05=8920800                          // 0x0088 1EE0 
    
    0x3210:0x06=509760                            // 0x0007 C740 
    
    0x3210:0x07=8920800                          // 0x0088 1EE0 
    
    0x3210:0x08=509760                            // 0x0007 C740 

 

Also check if the firmware is up to date (the software can update it for you), it is also possible to factory reset the motors to avoid any problems. 

 

 

Download the GitHub Package and build/launch it 

 

Run the following commands in the ros2_canopen/ folder of a new workspace to download the GitHub package :  

 

$ git clone …................ 

 

Build the package :  

 

    colcon build –-symlink-install 

 

    source install/setup.bash 

 

Run the launch file (can0 needs to be UP and RUNNING and connected to 4 motors with different node_ids (1,2,3,4):  

 

    ros2 launch stepper_motor_control nanotec_robot_control.launch.py 

 

 

Control the motors  

 

After launching the file, you will see an rviz2 window spawning. You may use the following services in order to control the motors : 

 

"Motor's joint name"/init 

"Motor's joint name"/(position, velocity, torque, …) mode 

"Motor's joint name"/target ( {double} ) 

 

I recommend using the rqt service caller GUI to make things easier. 

    rqt

 

I also implemented the position_tick_motor.cpp file that automatically runs the above services for you (you may have to change the motors joint name directly in the file) 

    ros2 run stepper_motor_control position_tick_client 

   and

    ros2 run stepper_motor_control slider_button_node

 

 

Concerning the meshes, you have to download a .stl file and convert it into a .stl binary to make it work. 
