#!/bin/bash
SCRIPT_DIR="$( cd "$(dirname "$0")" ; pwd -P )"

# Packages not used by IST
LABUST_LIST="ident_so labust_diagnostics labust_sim labust_allocation labust_execution labust_navigation labust_control labust_framework"
SENSOR_LIST="labust_imu image_processing diver_net novatel_dgps sensors_util usbl tritech_sdk spatial_ins diver_net labust_kinect navquest_dvl"
VIS_LIST="labust_gearth labust_uwsim rqt_relay usbl_uwsim labust_gui"
DES_LIST="decision_making_examples"

LABUST_PKG=${SCRIPT_DIR}/..
SENSOR_PKG=${SCRIPT_DIR}/../../sensors-ros-pkg
VIS_PKG=${SCRIPT_DIR}/../../labust-visualization-pkg
DES_PKG=${SCRIPT_DIR}/../../decision_making

FOLDER_LIST="${LABUST_PKG} ${SENSOR_PKG} ${VIS_PKG} ${DES_PKG} $*"
PKG_LIST="${LABUST_LIST} ${SENSOR_LIST} ${VIS_LIST} ${DES_LIST}"
CUR_FOLDER=`pwd`
for folder in $FOLDER_LIST
do
  if [[ -d $folder ]];then
    cd $folder
    DIRS=`find * -maxdepth 1 -type d`
    for dir in $DIRS
    do
      for pkg in $PKG_LIST
      do
        if [[ $pkg == $dir ]]; then
          echo "CATKIN_IGNORE directory $dir"
          touch $dir/CATKIN_IGNORE
        fi
      done
    done
    cd $CUR_FOLDER
  fi
done

