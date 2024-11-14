
# CAN wiring :

In the following picture, you can see the CanOpen pinout on the motor. the wires should connect to CAN_H CAN_L and GND respectively to the same pinouts on the USB to CAN adapter. To add other motors, you can simply connect the second pin in parallel to the other motor's pins.

![image](https://github.com/user-attachments/assets/7dc6f34e-b8a2-47f6-8fcc-7c1aa273ba4f)

If you are using the Peak USB to CAN adapter, you should add a 120 Ohm terminator resistor (between CAN_H and CAN_L) at it's entrance on the canopen bus to protect the signal because it's not already present for this adapter (the nanotec one does have a terminator already built in). The motors also already have a built in terminator.

Here is the pinout of the peak adaptor : 

![image](https://github.com/user-attachments/assets/f72298d2-d2ea-48f6-8a16-27cffc3dd0b6)

So you have to connect to pin 2 (CAN_L) , pin 3 (GND) and pin 7 (CAN_H).

![canmotor](https://github.com/user-attachments/assets/39e562ef-8a41-4222-a80e-1269f351b574)


----------------------------------------

# Power 

The power supplies should be around 24V and 5A.

![image](https://github.com/user-attachments/assets/2cbb00b0-114e-47fe-80b2-1ef27a7c20f1)


-----------------------------------------

# Digital input (Negative limit switch)

![image](https://github.com/user-attachments/assets/b46a080c-3c53-4626-9e73-5175322c1dce)

Object 0x3240 can enable those special functions in digital inputs 6 to 9 : 

![image](https://github.com/user-attachments/assets/3f98776a-20cc-4dfc-aae1-5f453449adab)

What i enabled in the parameters is only the negative limit switch (pin 6) which draws current from the 12V output and GND. I'm using a 1k Ohms pullup resistor on the cable that leads to the 12V pin. So the signal cable as well as the 12V only are connected to the NO pin and the GND to the COMMON pin : 

![image](https://github.com/user-attachments/assets/995b1662-6e76-4cb8-8b67-8ffd4d419e65)





