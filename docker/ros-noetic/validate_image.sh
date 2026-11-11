#!/usr/bin/env bash
# Invoked inside a Docker build stage, never by creating a runtime container.
set -eo pipefail
source /opt/ros/noetic/setup.bash
catkin_make -j2 -l2 > /tmp/vip3-catkin.log 2>&1 || { tail -n 80 /tmp/vip3-catkin.log; exit 1; }
source devel/setup.bash
python3 - <<'PY'
import cv2, numpy, scipy, rospy, yaml, ujson
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from drivable_bev.fisheye_svm import FisheyeSvm
from vip3_parking.state import convert_ego
from vip3_parking.evaluation import evaluate_parking
assert 'front_steer' in CtrlCmd.__slots__ and 'steering' not in CtrlCmd.__slots__
assert 'angular_velocity' in EgoVehicleStatus.__slots__
print('VERSIONS: OpenCV', cv2.__version__, 'NumPy', numpy.__version__, 'SciPy', scipy.__version__)
PY
for package in rosbridge_server rqt_image_view rviz compressed_image_transport; do
  rospack find "$package"
done
python3 -m unittest discover -s src/perception/drivable_bev/test -p test_fisheye_svm.py -v
python3 -m unittest discover -s src/data_collection/test -p 'test_bag*.py' -v
python3 -m unittest discover -s src/vip3_bringup/test -p 'test_topic_monitor.py' -v
python3 -m unittest discover -s src/vip3_parking/test -v
python3 tools/check_sensor_preset.py
python3 tools/check_topic_contract.py
python3 tools/check_node_imports.py
python3 tools/check_launch_params.py
printf '\nVIP3_IMAGE_VALIDATION=PASS\n'
