#!/bin/bash

# clone dependencies
if [ ! -d "../m2b" ]; then git clone git@github.com:m2robocon/m2b.git ../m2b; fi
if [ ! -d "../trolly" ]; then git clone git@github.com:m2robocon/trolly.git ../trolly; fi
if [ ! -d "../m2_interfaces" ]; then git clone git@github.com:m2robocon/m2_interfaces.git ../m2_interfaces; fi
