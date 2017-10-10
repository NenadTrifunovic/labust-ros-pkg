#!/bin/bash
SCRIPTPATH=$( cd $(dirname $0) ; pwd -P )
sudo ${SCRIPTPATH}/../labust_mission/scripts/install_tinyxml2.sh
sudo ${SCRIPTPATH}/../labust_navigation/scripts/download_geographiclib_models.sh
sudo apt install -y libcgal-qt5-dev

exit 0
