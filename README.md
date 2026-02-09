# HERMES-percept
This repository provides the perception nodes that run on the agents.
To build this project, ensure you have openCV (libraries and includes) installed. 
This particular project was implemented on a Raspberry Pi 5 running Ubuntu 24.04.

Assuming colcon is installed, to build the package (s) run `colcon build --symlink-install`.
To launch the publishing node, first source the newly built packages by running `source install/setup.bash`
in the project directory. Then run `ros2 run hermes_percept_facedetection facedetect_node`.
