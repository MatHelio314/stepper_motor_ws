 

Nanotec Stepper motor PD4-C6018L4204-E-08 ROS2 control integration on ROS2  

- - - - - - - - - - - - - - - - - -

# NEEDED 

- Ubuntu 22.04.5 LTS 

- ROS2 Desktop Humble installed :

      https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

- ROS2_CANOPEN library installed in your workspace :
   
      https://ros-industrial.github.io/ros2_canopen/manual/rolling/quickstart/installation.html

   Make sure that you use the right branch (humble) otherwise the building won't work :     

      https://github.com/ros-industrial/ros2_canopen/issues/217

  Run these commands to change branch : 

      cd ros2_canopen

      git checkout Humble

- Stepper Motors PD4-C6018L4204-E-08 Nanotec Datasheet :  

      https://www.nanotec.com/fileadmin/files/Handbuecher/Plug_Drive/PD4-C/fir-v2213/PD4C_CANopen_Technical-Manual_V3.3.0.pdf?1663072012 

 

- PCAN-USB FD Adapter :  

      https://www.peak-system.com/PCAN-USB-FD.365.0.html?&L=1 

 - - - - - - - - - - - - - - - - - -


# Download the GitHub Package and build/launch it 

Before downloading the package, you should install and build the ros2_canopen library in your workspace

Run the following commands in the /ros2_canopen folder of your new workspace to download the GitHub package :  

    git clone https://github.com/MatHelio314/stepper_motor_ws.git

Build the package :  

    colcon build –-symlink-install 
    source install/setup.bash 


Run the launch files (can0 needs to be UP and RUNNING and connected to at least one motor (can_id 1,2,3 or/and 4)):  


 Normal mode :

    ros2 launch stepper_motor_control main.launch.py sim:=false

 Visualization (rviz2) : 

    ros2 launch stepper_motor_control main.launch.py sim:=true
 

 
# Connect the motors to the computer through a CAN interface :  

 

   Step 1 : Power up the stepper motors with the correspondiing power supplies (24V & 5A) 
   
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

    sudo apt install can_utils

    candump can0 

You should not see anything in the candump for now and that is normal (my advice is to keep this command running for the futur to visualize the data being transmitted) 

 

# Change the node_id of each stepper motor  

 

Look for the Hex coding switch for node-ID and baud rate underneath 
The PD4-C is equipped with a hex coding switch. This can be used to set the source for the node-ID and the baud rate.

 

# Change the parameters of the motors 

You can directly set parameters on the bus.yml file (See README_BUS_FILE.md for more details)


What i recommend doing if clarifications are needed is to get the nanotec USB to CAN adapter ZK-USB-CAN-1 and use it with the Plug & Drive Studio software available on their website (windows only):  

    https://www.nanotec.com/eu/en/products/2256-software-for-brushless-dc-servo-motor-controller 

The software lets you modify the parameters as well as to do some tests, iIpersonnaly used this software to solve many issues with the motors.  
Also check if the firmware is up to date (the software can update it for you), it is also possible to factory reset the motors to avoid any problems. 

 

 



 

 

# Control the motors  

 

After launching the file, you will see a slider gui and buttons to control the motors.





You may also use the following services in order to control the motors : 


"Motor's joint name"/init 

"Motor's joint name"/(cyclic position, velocity, torque, …) mode 

"Motor's joint name"/target ( {double} ) 


 
I recommend using the rqt service caller GUI to make things easier to visualize. 

    rqt

 

 

 

Concerning the meshes, you have to download a .stl file and convert it into a .stl binary to make it work. 
