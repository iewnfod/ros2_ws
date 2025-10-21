#!/bin/bash

# clone dependencies
if [ ! -d "../m2b" ]; then git clone git@github.com:m2robocon/m2b.git ../m2b; fi
if [ ! -d "../trolly" ]; then git clone git@github.com:m2robocon/trolly.git ../trolly; fi
if [ ! -d "../m2_interfaces" ]; then git clone git@github.com:m2robocon/m2_interfaces.git ../m2_interfaces; fi
if [ ! -d "../m2_chassis_kinematics" ]; then git clone git@github.com:m2robocon/m2_chassis_kinematics.git ../m2_chassis_kinematics; fi
if [ ! -d "../m2_fsm_cpp" ]; then git clone git@github.com:m2robocon/m2_fsm_cpp.git ../m2_fsm_cpp; fi
if [ ! -d "../m2_joy" ]; then git clone git@github.com:m2robocon/m2_joy.git ../m2_joy; fi
if [ ! -d "../m2_pi_shield" ]; then git clone git@github.com:m2robocon/m2_pi_shield.git ../m2_pi_shield; fi
if [ ! -d "../m2_chassis_utils" ]; then git clone git@github.com:m2robocon/m2_chassis_utils.git ../m2_chassis_utils; fi
if [ ! -d "../m2_localization" ]; then git clone git@github.com:m2robocon/m2_localization.git ../m2_localization; fi
if [ ! -d "../m2_drivers" ]; then git clone git@github.com:m2robocon/m2_drivers.git ../m2_drivers; fi
