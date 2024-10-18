This package is used to control the PD4-C6018L4204-E-08 stepper motor from nanotec via ros2_canopen.

## Usage

### Setup CANController

- Peak CANController
    
    ```bash
    sudo modprobe peak_usb
    sudo ip link set can0 up type can bitrate 1000000
    sudo ip link set can0 txqueuelen 1000
    sudo ip link set up can0
    ```

### Run the launch file
- real_hw_launch.launch.py: 
    
    ```bash
    ros2 launch nanotec_pd4_can real_hw_launch.launch.py
    ```


### Init the device

```bash
ros2 service call /motor_left/init std_srvs/srv/Trigger
```

### Switch mode to Profile Velocity

```bash
ros2 service call /motor_left/velocity_mode std_srvs/srv/Trigger
```

### Set target position

```bash
ros2 service call /motor_left/target canopen_interfaces/srv/COTargetDouble "{ target: 100.0 }"
```

OR

In separate terminals : 
```bash
ros2 run nanotec_pd4_can slider_button_node
```
```bash
ros2 run nanotec_pd4_can position_tick_client
```

