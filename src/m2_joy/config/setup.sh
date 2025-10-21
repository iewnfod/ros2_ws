#!/bin/bash

SCRIPT_DIR="$(dirname "$(realpath "$0")")"

sudo apt install -y ros-jazzy-joy udev 
python3 $SCRIPT_DIR/gen_rules.py | sudo tee /etc/udev/rules.d/51-m2-joy-setup.rules > /dev/null
sudo udevadm control --reload
sudo udevadm trigger
echo "m2_joy setup completed successfully"