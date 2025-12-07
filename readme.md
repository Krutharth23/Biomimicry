## Setup

Please put vehicleinfo.py here - ~/ardupilot/Tools/autotest/pysim (or wherever you have installed ardupilot)

Please put iris_runway.sdf here - ~/gz_ws/src/ardupilot_gazebo/worlds

Please extract the contents models package here - ~/gz_ws/src/ardupilot_gazebo/models

make launch.sh an executable

NOTE - ignore any param / mav / logs / terrain files they are autocreated by mavsdk

## To run

1. Open a terminal and run - 

    ``` gz sim -v4 -r iris_runway.sdf ```

2. run launch.sh

3. ``` python3 main.py ```

4. ``` python3 figure8.py ```
