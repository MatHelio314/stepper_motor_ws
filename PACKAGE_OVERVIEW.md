Here is a high-level overview of all the files that I created and published on my GitHub, I will go through the explanation folder by folder for simplicity and clarity.

<img width="200" alt="image" src="https://github.com/user-attachments/assets/6ebdca07-546d-4b81-8a3c-6e06f553f116" />


  ### Config folder

The config folder contains configuration files for setting up and managing the robot system.
Here, the bus_template.yml files contain the configuration of each motor over the CAN bus, we can describe the units, velocity, acceleration and much more in the SDO's. The mappings for real time processing of data such as the encoders feedback, the target position values and other exchanged data are defined in the PDO's. This all relate to the CAN Open Protocol. 

<img width="450" alt="image" src="https://github.com/user-attachments/assets/c1e9404a-8f22-43b2-87ff-424f1051be43" />
   ##### Pseudo code for a bus.yml file


The difference between the bus_template.yml and bus.yml file is that the bus.yml file only contains the configuration of the connected motors in the network. The reason for that is that the launch wouldn’t work if we'd try to configure a motor that is not connected (more about it in bus_manager section).
The .eds files are the electronic datasheet of the motors, this is a necessary file to establish a CAN open connection (previously mentioned in the ‘Getting familiar with the hardware’ segment). 

   ### Launch folder

The launch folder contains the launch files coded in python, this is a common procedure in ROS2 to use a launch file, allowing quick and easy launch of all the files, that way we don't have to launch each file by hand through the bash terminal. We always launch the main.launch.py file with one parameter called 'sim' which determine if we want to launch the normal launch file or the visualization.

   ### ros2 launch stepper_motor_control main.launch.py sim:=False

<img width="450" alt="image" src="https://github.com/user-attachments/assets/dbee9cb7-42dd-4f8d-8dbe-8586369ee77b" />
   ##### Pseudo code for main launch file

The difference between the two files is that we decide whether or not to launch a visualization tool called rviz2 that will listen to the robot description, this all takes place in the urdf folder. This feature is nice to have but not particularly useful so we have the choice to not have it to prevent any processing time loss. 

   ### Meshes folder

The meshes folder contain 3D model files (.stl) of the motors, this is an easy way to give the geometry of the motors for the visualization, these files are used in the urdf folder.

<img width="700" alt="image" src="https://github.com/user-attachments/assets/7c27e854-1b53-472f-961e-1c18c897fcb8" />
   ##### Definition of geometry based on mesh files

   ### Src folder

The /src folder contains all the nodes that handle the custom tasks I've developed for the project.

   #### GUI node
    
The main node is the slider_button_node.cpp which creates the GUI, we can launch and terminate almost every other node in this folder through this user interface. I decided to use Qt5 as the tool to create the GUI since it can create dynamical graphical interfaces and it is compatible with ROS2. 

<img width="450" alt="image" src="https://github.com/user-attachments/assets/50b47f27-1bf3-4aba-8e16-98535cbf11d8" />
   ##### Pseudo code for GUI Node

<img width="700" alt="image" src="https://github.com/user-attachments/assets/f910c37b-7d8e-47ab-b72d-3b9de718678b" />
   ##### Screenshot of the GUI 

The node basically takes the user input and publishes the data through a few topics, it also listens to the joint states of each motor to show a real time feedback to the user.

<img width="609" alt="image" src="https://github.com/user-attachments/assets/810fd4c4-9186-4cd5-8d2d-eead552f62f7" />
   ##### Subscribers and Publishers of the GUI

   #### Position control nodes

The GUI creates those position control nodes that listen to the slider values and calls the right services in order to move the motors. An emergency stop button sets the bool value ‘stop_bool’ to 1 and instantly calls a halt service and closes the node.

<img width="450" alt="image" src="https://github.com/user-attachments/assets/a323e39c-d607-44ea-a6ea-fc93c63e3b3d" />
   ##### Pseudo code for position control motor
    
   #### Bus manager file

On the other hand, the bus_manager.cpp node is launched by the main.launch.py file. 
Basically, when we launch the bus manager node, it launches the motor connection checker node. Theses nodes are responsible for checking which motors are connected or not and then write down the parameters of the connected motors into the bus.yml file. 
Here is a representation of how these two nodes interact with each other: 

<img width="609" alt="image" src="https://github.com/user-attachments/assets/b897801f-ed9d-4432-a12f-22c9537d20e8" />

this node creates a .txt file and tests the reaction of each individual motors through the candump. The candump displays all the messages going through the CAN Open interface. The newly created .txt file gets filled up with the candump buffer and the node then analyses this file in search of the potential answers from the motors. 
The idea that it relies on here is that the CAN Open interface uses node_ids to communicate with each connected device, each motor has a specific node_id from 1 to 4. If there is an answer, then that means that the corresponding motor is connected. When the testing is done, the node publishes the result on a topic called /motor_states. The /bus_manager subscribed to this topic looks at the list of connected motors. Let’s take an example and say that it receives : "Connected motors : 1, 2", Then it will automatically assimilate these 1 and 2 to the /first_shaft_joint and /second_shaft_joint. Then it scans through the bus_template.yml file which contains the parameters of the 4 motors and will only copy the parameters under the /first_shaft_joint and /second_shaft_joint into the bus.yml file. This file is the one that gets analysed by the launch file so it will prevent any launch errors if all the motors are not connected. The bus manager finishes by launching a building sequence before terminating itself to save the changes done to the bus.yml file into ros2.

   #### Reset_Home_X & Homing
 
This node does not communicate with any other nodes. Its function is to send a set of instructions to the related motors and then kill itself.
It first sends a specific value to a specific object to start a reset of the motor, precisely ‘1953460066’ to object 0x2800 subindex 1. Then it sets the limit switch behavior to do nothing upon activation expect remembering the position value and then it initializes the motors.
To explain why it sends this particular sets of instructions, I'll start with explaining how the homing mode works for these kinds of motors. The homing mode only starts once, that is when the /init service is called for the first time upon startup. The trick to start it at will is that every time the motor is reset (or unpowered and powered back on), we can start a new homing sequence. We can choose the homing method as well as some other parameters such as the speed of homing or acceleration. Here I chose the homing method number 17: 

<img width="408" alt="image" src="https://github.com/user-attachments/assets/661246f8-1bb5-41e4-b141-9307340a3cad" />
   ##### Visual description of homing method 17

This method is related to the negative limit switch. As you can see, when the homing starts, the motor goes backward until it hits the limit switch and then goes forward until the goes off again. When that happens, the motor knows that it is now in position 0. One thing we can choose is the behavior upon activating the limit switches, such as immediate stopping or do nothing and remember the position. The problem with that is if this is set to immediate stopping, then the homing mode cannot be achieved to the end, that is why we send an instruction to the motor to only remember the position and do nothing else upon activation of the limit switch. After all these instructions are sent, the node terminates itself.

<img width="701" alt="image" src="https://github.com/user-attachments/assets/923b65fc-8232-4699-8e73-ceef80a8bc84" />
   ##### Limit switches on mechanical setup

   #### URDF folder 

The urdf folder defines the robot's structure and kinematics for simulation and control. This only relates to the visualization part.
One way to control the motors is to create a specialized controller in ROS2 using the library called ROS2_control. This library is an amazing tool to implement control solutions for a robot and I had the chance to learn how to use it and practice on a real robot in a ROS2 workshop. This library allows the use of visualization and simulation tools called RVIZ2 and GAZEBO. 
In my library I developed the .xml files in URDF that basically create a robot description usable by the rviz2 tool. These files contain details about links and joints and their interaction in an environment. To explain the writing process simply, I define a joint with specific dimensions, material, color and inertial values and create links that will bound this joint with the others. For instance, I created the motors joints and linked them to their respective shaft joints and each shaft can rotate on a specific axis. Once the robot description is finished, the rivz2 tool will visualize the links and joints based on this description and we would be able to see the shaft’s rotation when the real motors are moving. 

<img width="500" alt="image" src="https://github.com/user-attachments/assets/f145a670-7cf6-4542-ac4f-ca96c4033a95" />

In this screenshot, I described the links and joints for two stepper motors and we can see the axis on the motor’s shafts.
What the ros2_control library does is that it enables us to create controllers that will claim the different control functions. For instance, one controller would claim all the services so we would only need to ask the controller to call the combination of services we want. Another one would claim the position control, we would then only have to send the target position value we need as an input and it would move the motor to the desired location. 
In the end, I only explored the possibility of using this ros2_canopen library to control the motors but decided to keep to the ‘basic’ controls by calling the right services with nodes created by myself. I’m confident that using this library could have been done and would have been a good solution too but my knowledge in ROS2 at that time was not advanced enough to confidently use this method, my coworkers also suggested that simply using the services would be a good solution nonetheless so I didn’t push the research in this field too far. The visualization tool is still useful for debugging and developing new aspects so I kept it aside.

In order to create a visualization, we still create a controller but I'm using the most basic one here (robot controller). The files use the xacro macro in xml language, this is not a required but it allows us to split the files and generate macros such as general variables for convenient file management. It is usually recommended to use this type of language if the robot description becomes a bit complicated.

Others
At last, the CMakeList.txt file is the file that sources all the other files, we reference the different folders as well as include paths to specific files. 
 
The ros2_canopen library is not shown here but most of the files from this package include and use files from the library in order to work. I had to modify some values in the library itself to add torque feedback force instance or modify the feedback frequency for instance.
 
Each package has to be built in order to be launched, this process is crucial and we mainly use the tool called Colcon from ROS2 to build a package. Colcon uses the CmakeLists.txt file as a reference to source everything correctly. This type of package here is based on C++. I also had the choice to create a Python package but C++ is generally much quicker and less demanding on computer resources so it was the preferred choice here. 
