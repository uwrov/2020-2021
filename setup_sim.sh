# Get the required scripts into the ros installation folder
cd ~
mkdir -p ros_catkin_ws/install/share/uuv_assistants/templates
cp -r ros_catkin_ws/src/uuv_simulator/uuv_assistants/templates  ros_catkin_ws/install/share/uuv_assistants/templates

# Add thrusters to python path
export PYTHONPATH="${PYTHONPATH}:/home/uwrov/ros_catkin_ws/install/lib/python3/dist-packages/uuv_thrusters/models"

# make small adjustment to thruster_proportional.py
sed -i '/from thruster import Thruster/c\from .thruster import Thruster' /home/uwrov/ros_catkin_ws/install/lib/python3/dist-packages/uuv_thrusters/models/thruster_proportional.py
