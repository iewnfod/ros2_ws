# Useful functions

1. `declare_parameter("parameter_name", default_value)`
Used in the *constructor*. Declares that this node will use this launch parameter, and provides a default value if the paramter is not found in the launch file (mandatory)

2. `get_parameter("paramter_name").as_double()`
Used in the *`init()` method*. Retrieves the value of the parameter from the launch file, and converts it to the desired type. There is also `.as_string()`, `.as_int()` and more.

3. `create_wall_timer(10ms, [this]() { /* do stuff */ })`
Used in the *constructor*. Returns a *repeating* timer that calls the function (arg 2) at the set interval of arg 1. `[this]() {}` is a *closure*, which is a way to define functions inline instead of writing it somewhere else. In place of `/* do stuff */` you can write code just like in any other function.

4. `create_subscription<message_type>("topic_name", 1, [this](const message_type& data) { /* do stuff with data */ })`
Used in the *`init()` method*. Returns a Subscription that calls the arg 3 closure whenever a new message is sent on "topic_name". The `1` in arg 2 is the Quality of Service number -- no, I don't know what it does either.

5. `create_publisher<message_type>("topic_name", 1)`
Used in the *`init()` method*. Returns a Publisher that allows you to publish messages to "topic_name". The `1` in arg 2 is the Quality of Service number -- no, I don't know what it does either.

6. `left_wheel_pub_->publish(msg)`
Publishes message `msg` to the topic stored in left_wheel_pub_ when it was created.

7. How to create a new message
E.g. For a string message:
```cpp
std_msgs::msg::String str_message;
str_message.data = "Hello World!"
```