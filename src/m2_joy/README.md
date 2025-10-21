# m2_joy

This package gathers the nodes that is ds(DualShock/DualSense)-related, including the config files and code.
Please setup the environment beforehand following the instructions [here](#).

To read the ds4/ps4 as joystick in ros, `ros-jazzy-joy` is required.
Linux kernel version also affects the joy readings, for details please check from https://wiki.gentoo.org/wiki/Sony_DualShock. The best approach is to follow the setup [here](https://github.com/m2robocon/m2_wiki/wiki/Setting-up-the-skull-canyon-&-network#user-content-linux-version).

Avoid using the `ps` button in other packages; it is reserved for PS4 topic mux.

## Setting up

- You need to run the setup script (located in `config/setup.sh`) after cloning the project. Otherwise, a cmake error will be raised during `colcon build`.
- To test whether the setup is successful, connect to a controller and run the command `ls /dev | grep ds`. 

## m2_joy_transform_data_node.cpp
- joy -> **this node** -> JoyData
- translates joy message into customised JoyData

### API Summary
- Parameters
	- `dev` (string, default `/dev/input/js0`), e.g. `/dev/ds5lambda`
- Subscriptions
	- `param "output_topic"` (default `/joy`): `sensor_msgs/Joy` Output of `joy_node`
- Publications
	- `param "joy_topic"` (default `/input/joy_data`): `JoyData` Transformed joystick controller data
	- Guaranteed to be 100 Hz

## Message types
### JoyData.msg
- Joystick controller button and analog readings

### joy_msg.launch
- launches joy_node from `ros-jazzy-joy`
- take care of device naming using arguments
- have default launch params for `joy_node