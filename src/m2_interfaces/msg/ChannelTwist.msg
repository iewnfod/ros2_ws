### Message type for topic /chan_cmd_vel

# Defined same as geometry_msgs/Twist
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular

# Channel type
# Override order:
# - CHANNEL_EMERGENCY
# - CHANNEL_CONTROLLER
# - CHANNEL_AUTO
#
# If a channel has no messages for at least 0.1 second (configurable in param channel_timeout),
# it is considered a "dead" channel and it would switch to other overridden channels.
#
# If all channels are dead, a timeout warning would be logged every second,
# but this node will still emit a zero velocity command at 100Hz.
uint8 EMERGENCY = 0
uint8 CONTROLLER = 1
uint8 AUTO = 2
uint8 channel
