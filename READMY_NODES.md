# slider_button_node : 

This node basically takes the user input and publishes the data through a few topics as you can see here :  

![image](https://github.com/user-attachments/assets/399686e9-c1d9-4478-a478-862ddcf5aa6b)


 

My idea was to publish a value for each slider as you can see with the /slider_value_x,y,z, the position control node of each axis then subscribes to these topics and acts accordingly. The torque slider values are not currently used because I'm first going to implement a torque feedback and then maybe a torque control solution if that is still useful.  

Here for example the /stop_motor_X is set to 1 when i press the stop button for the X axis, then the position control node of the X axis listens to this topic and stops both motors right away. This can also be implemented easily for the other axis and would accelerate greatly the stopping time after pressing the button. 

 

The Slider node is also responsible for launching and stopping the motor control nodes such as the position_tick_motor_X for example. So everything can be controlled from this gui in theory, the next step would be to add position and torque feedback for example and maybe even a graph plotting screen.   

--------------------------------------

# position_tick_motor_X 

This node behaves quite similarly to the Y and Z position_tick_motor nodes, the only big difference is that it controls two motors and not only one. 

Here is a visualization of it's interation with the other nodes :  

![image](https://github.com/user-attachments/assets/aa728ae3-bf99-4cb8-8d32-8e359ce83646)


Basically, the node listens to the X axis slider value and sends the value in real time to the motors. It also listens to the stop_motor_X value, if this value goes to 1, it stops the motors in priority and then terminates the node. 

As you can see, the node also subscribes to the joint states of both first and second motors in order to get feedback from the two motors position in real time. It is useful to prevent any unsync events. 

The node sends instructions to the motors through services, first it initialises the motors and then it selects the profile position mode. Then it goes into a loop that sends the slider value at a fixed frequency to the motors through the target position service. The loop also looks at the difference in value between the two motors position and acts accordingly to the results, if the two motors unsync then it sends the target of the second motor to be the first's shaft position.  

--------------------------------------------


# Bus manager and Motor connection Checker nodes :  

Theses nodes are responsible for checking which motors are connected or not and then write down the parameters of the connected motors into the bus.yml file. This is important because the launch file won't work if it doen't get feedback from every liested motors in the config file.  

Here is a representation of how these two nodes interact with each other :  

 ![image](https://github.com/user-attachments/assets/ebeb577b-f30b-40cc-b7a9-bbdb3a44f6bc)



Basically, when we launch the bus manager node, it launches the motor connection checker node, this node creates a .txt file and pokes each individual motors through the candump, then the newly created .txt file gets filled up with the candump buffer and the node then analyses this file in search of the potential answers from the motors.  

The idea that it relies on here is that the can interface uses nodes_id to communicate with each connected device, each motor has a specific node_id from 1 to 4 that i set on the hardware. So when "poking" a motor, the node just sends a diagnostic message to say node_id 1 and looks for the answer, if there is an answer, then that means that the motor 1 is connected. It does that for each of the 4 node_ids and when it is done, it publishes the result on a topic called /motor_states. That is when the /bus_manager node comes into play, the node subscribes to this topic and looks at the connected motors. Lets take an example and say that it receives : "Connected motors : 1, 2", Then it will automatically assimilate these 1 and 2 to the /first_shaft_joint and /second_shaft_joint. Then it looks into the bus_template.yml file which contains the parameters of the 4 motors and will only copy the parameters under the /first_shaft_joint and /second_shaft_joint into the bus.yml file. This file is the one that gets looked at by the launch file so it will prevent any launch errors of all the motors are not connected. The bus manager launches a building sequence before terminating itself to save the changes done to the bus.yml file into ros2.

----------------------------------------------


# Reset_Home_X 

The node does not communicate with other nodes :  

![image](https://github.com/user-attachments/assets/50e76852-2f3b-4a7d-a5c5-a4892798d950)


It's function is just to send a set of instructions to the related motors and then kill itself. 


It first sends a specific value to a specific object to start a reset of the motor. Then it sets the limit switch behavior to do nothing upon activation expect noting the position value and then it initializes the motors. 

To explain why it sends this particular sets of instructions, I'll start with explaining how the homing mode works for these kind of motors. The homing mode only starts once when the init service is called everytime the motor is reset (or unpowered and powered back on). We can choose the homing method as well as some other prameters such as the speed of homing or acceleration. Here i chose the homing method number 17 :  

![image](https://github.com/user-attachments/assets/72e3ea07-9165-4e48-9e8d-372799fb0b06)


Which is related to the negative limit switch. As you can see, when the homing starts, the motor goes backward until it hits the limit switch and then goes forward until the goes off again. When that happens, the motor knows that it is now in position 0. One thing we can choose is the behavior upon activating the limit switches, such as immediate stopping or do nothing and note the position. The problem with that is if this is set to immediate stopping, then the homing mode cannot be achieved to the end, that is why we send an instruction to the motor to only note the position and do nothing else upon activation of the limit switch. After all these instructions are sent, the node terminates itself. 
