@default:
	echo "You can use following commands:"
	just list

@list:
	just --list

@help:
	just --help

@build:
	colcon build --symlink-install

@clean:
	rm -rf build/ install/ log/
	echo "Cleaned build/, install/, and log/ directories."

@launch:
	bash install/setup.bash && \
	ros2 launch "$(pwd)/install/m2_sic_robot/share/m2_sic_robot/launch/all_launch.py"

from := "./src"
to := "sic:/home/sic/ros2_ws"

@send2pi:
	scp -r {{from}} {{to}}
	echo "Source files copied to Raspberry Pi."
