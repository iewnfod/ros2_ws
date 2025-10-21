# m2_joy

This package gathers the nodes that is controller-related (DualShock/DualSense/Xbox), including the config files and code.
Please setup the environment beforehand following the instructions [here](#).

To read controllers as joystick in ROS, `ros-jazzy-joy` is required.
Linux kernel version also affects the joy readings, for details please check from https://wiki.gentoo.org/wiki/Sony_DualShock. The best approach is to follow the setup [here](https://github.com/m2robocon/m2_wiki/wiki/Setting-up-the-skull-canyon-&-network#user-content-linux-version).

Avoid using the `ps` button in other packages; it is reserved for PS4 topic mux.

## Supported Controllers

This package supports the following controller types:
- **PlayStation 4 (DualShock 4)**
- **PlayStation 5 (DualSense)**
- **Xbox One / Xbox Series X|S controllers**

The node automatically detects the controller type based on the number of buttons and axes. You can also manually specify the controller type using the `controller_type` parameter.

## Setting up

- You need to run the setup script (located in `config/setup.sh`) after cloning the project. Otherwise, a cmake error will be raised during `colcon build`.
- To test whether the setup is successful, connect to a controller and run the command `ls /dev | grep ds`. 

## m2_joy_transform_data_node.cpp
- joy -> **this node** -> JoyData
- translates joy message into customised JoyData
- **Automatically detects controller type** (PlayStation or Xbox)

### API Summary
- Parameters
	- `dev` (string, default `/dev/input/js0`), e.g. `/dev/ds5lambda`
	- `controller_type` (string, default `auto`): Controller type detection mode
		- `auto`: Automatically detect controller type based on button/axes count (default)
		- `ps4` or `ps5`: Force PlayStation controller mapping
		- `xbox`: Force Xbox controller mapping
- Subscriptions
	- `param "output_topic"` (default `/joy`): `sensor_msgs/Joy` Output of `joy_node`
- Publications
	- `param "joy_topic"` (default `/input/joy_data`): `JoyData` Transformed joystick controller data
	- Guaranteed to be 100 Hz

### Controller Button Mapping

#### PlayStation Controllers
- Cross, Circle, Triangle, Square buttons
- L1, R1, L2, R2 triggers
- Share and Options buttons
- PS button
- L3, R3 (stick presses)

#### Xbox Controllers
- A → Cross
- B → Circle
- X → Square
- Y → Triangle
- LB → L1
- RB → R1
- LT → L2
- RT → R2
- View/Back → Share
- Menu/Start → Options
- Xbox/Guide → PS
- Left/Right stick presses → L3/R3

## Message types
### JoyData.msg
- Joystick controller button and analog readings

### joy_msg.launch
- launches joy_node from `ros-jazzy-joy`
- take care of device naming using arguments
- have default launch params for `joy_node`