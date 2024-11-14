
# CAN wiring :

In the following picture, you can see the CanOpen pinout on the motor. the wires should connect to CAN_H CAN_L and GND respectively to the same pinouts on the USB to CAN adapter. To add other motors, you can simply connect the second pin in parallel to the other motor's pins.

![image](https://github.com/user-attachments/assets/7dc6f34e-b8a2-47f6-8fcc-7c1aa273ba4f)

If you are using the Peak USB to CAN adapter, you should add a 120 Ohm terminator resistor (between CAN_H and CAN_L) at it's entrance on the canopen bus to protect the signal because it's not already present for this adapter (the nanotec one does have a terminator already built in). The motors also already have a built in terminator.

Here is the pinout of the peak adaptor : 

![image](https://github.com/user-attachments/assets/f72298d2-d2ea-48f6-8a16-27cffc3dd0b6)

So you have to connect to pin 2 (CAN_L) , pin 3 (GND) and pin 7 (CAN_H).

![canmotor](https://github.com/user-attachments/assets/39e562ef-8a41-4222-a80e-1269f351b574)


----------------------------------------

# 
