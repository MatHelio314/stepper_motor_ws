Lets analyse the content of a bus.yml file :  

 

 

Here we are analysing the parameters of the first shaft joint, this joint is related to the motor with a node id of 1 (hex coding switch under each motor). 

    first_shaft_joint:   

 

We define the node_id of the motor, here like expected it is '1', so the device container will know that it will send instructions for the 'first_shaft_joint' to the can channel number 1 and will also get answers from the same channel. You can easily visualize this with a candump of your can interface. (requests and responses)   

    node_id: 1 

 

The dcf parameter contains the .EDS file (Eletronical DataSheet) of the motor, it is really important that it correspond exactly to the real motor, this is how the device container will know what instruction correspond to what behavior. Sometimes these .eds files contain forms that ros2_canopen does not like such as hexadecimal data so you will need to replace them manually if you get building errors. 

    dcf: "PD4-C6018L4204-E-08.eds" 

 

This driver is a content of the ros2_canopen library. Most motors using the canopen protocol use a normalised state machine and object dictionnary and the motors I'm using are controller with the Cia402 state machine.   

    driver: "ros2_canopen::Cia402Driver" 
    
    package: "canopen_402_driver" 

 

In order to get position, velocity and torque feedback, be need to enable the polling mode 

    polling: true 

 

The period here will define the frequency at which the motor sends feedback and executes instructions 

    period: 10 

 

Diagnostic is a great way to get real time feedback of the state of the Cia402 state machine and on the current control mode. This launches a topic that publishes diagnostic data every seconds. It is also useful to see if there are any warning bits enabled and know what are the problems if we get some. 

    diagnostics: 
    
    enable: true 
    
    period: 1000 

 
 

Changing the scale of the units can be useful for a particular case scenario 

    # Change the scale to correspond to the real hardware :  
    
    scale_pos_to_dev : 1.0 
    
    scale_vel_to_dev : 1.0 
    
    scale_pos_from_dev : 1.0 
    
    scale_vel_from_dev : 1.0 
    
    heartbeat_producer: 2000 

 

SDOs are the main parameters of the motor, here we can change the velocity, acceleration, software max values and a lot of other parameters. I Added and removed my parameters by trial and error by reading the datasheet trying to implement the functions and securities i wanted to add. For instance there are parameters to enable the serial input ports's special functions such as the limit switches, useful for the homing mode for example that we can also configure here.  

    sdo: # SDO executed during config 

 

When object 0x3240 subindex 2 is set to one, it inverts the serial input of the negative limit switch, so the pin is set to high only when the switch is activated.  

    - {index: 0x3240, sub_index: 2, value: 1} # Function inverted 

The next params define the velocity, acceleration and deceleration of the motors during certain modes such as the profiled position mode

    - {index: 0x6081, sub_index: 0, value: 100} # Set velocity  
    
    - {index: 0x6083, sub_index: 0, value: 1000} # Set acceleration 
    
    - {index: 0x6084, sub_index: 0, value: 1000} # Set deceleration 
    
    - {index: 0x6085, sub_index: 0, value: 10000} # Set quickstop deceleration 

 

This next parameter is in theory not useful because the ros2_canopen library automatically sets the mode to 'NONE' when booting a motor but it is somehow needed to start booting it up. You can freely change the mode as long as it is not 0, it does not matter. 

    - {index: 0x6060, sub_index: 0, value: 1} # Set profile position mode 

 

The Homing mode is launched once everytime the motor restarts or gets powered on (and when the mode is set to homing , object 0x6060 subindex 0 value 6). It's goal is to search for the position 0 with whatever method is set here. The datasheet gives ample explanations about each method but here the method 17 basically just looks for the negative switch and set the position 0 there. You should set a value of 35 here if you don't want the homing mode to do anything. Keep in mind that calling the /init service of the corresponding motor will start the homing mode everytime so it is an important factor to not get your motor out of control.  

    - {index: 0x6098, sub_index: 0, value: 17} # Set default homing mode to 35 

 

Here, interpolation time is used for the cyclic position mode, in case you use this mode, keep in mind that the motor will move to the target absolute position value in the set interpolation time (don't send a high target value difference or it could damage the motor) 

    - {index: 0x60C2, sub_index: 1, value: 50} # Set interpolation time for cyclic modes to 50 ms 
    
    - {index: 0x60C2, sub_index: 2, value: -3} # Set base 10-3s 

 

The next parameter is needed to enable the interpolated position mode. 

    - {index: 0x60C4, sub_index: 6, value: 1} # Interpolated position mode needed param  
    
    - {index: 0x60C6, sub_index: 0, value: 1000} # Max Acceleration 

 

The two next params are useful to set software position limits. Here i want the motor to move between 0 and 408240 tenths of degrees <-> 113,4 revolutions which correspond to 567mm, the length of the X axis shaft.  

    - {index: 0x607D, sub_index: 1, value: 0} # Min Position 
    
    - {index: 0x607D, sub_index: 2, value: 408240} # Max Position 

 

Enables the limit switches serial port input only for the negative limit switch here. (see datasheet on object 0x3240 for more informations)   

    - {index: 0x3240, sub_index: 1, value: 1} # Special function enabled 

 
 

TPDO and RPDO mappings are important for the software to actually use some functionnalities. For instance, here the mappings allow targets positions values to be recognized for the profile position mode to be used, this mapping also allow position and velocity feedback and i do know that if will need to change the mappings to get torque feedback. Keep in mind that changing the mappings here can disable some functions if you don't be careful, i spent a few weeks before figuring that out. 

    tpdo:  
    
    1: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x6041, sub_index: 0}  
    
    - {index: 0x6041, sub_index: 0}  
    
    2: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x6061, sub_index: 0}  
    
    - {index: 0x6041, sub_index: 0}  
    
    3: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x6064, sub_index: 0}  
    
    - {index: 0x6041, sub_index: 0}  
    
    4: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x606C, sub_index: 0}  
    
     
     
     
    
    rpdo:  
    
    1: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x6040, sub_index: 0}  
    
    - {index: 0x6040, sub_index: 0}  
    
    2: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x6060, sub_index: 0}  
    
    - {index: 0x6040, sub_index: 0}  
    
    3: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x607A, sub_index: 0}  
    
    - {index: 0x6040, sub_index: 0}  
    
    4: 
    
    enabled: true 
    
    cob_id: "auto" 
    
    transmission: 0x01 
    
    mapping: 
    
    - {index: 0x60FF, sub_index: 0}  

 
