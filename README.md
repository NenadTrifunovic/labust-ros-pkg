Introduction
==============

This repository contains the ROS related code developed at the **LAB**oratory for **U**nderwater **S**ystems and **T**echnologies (**LABUST**), University of Zagreb. Other ROS repositories include:
* **[labust-common-msgs](http://https://github.com/labust/labust-common-msgs)** - the repository contains the custom messages used by *LABUST* code
* **[sensors-ros-pkg](https://github.com/labust/sensors-ros-pkg)** - the repository contains sensor drivers that are used in *LABUST* vehicles and projects. Sensor management, diagnostics and processing code is also located in this repository.
* **[vehicles-ros-pkg](https://github.com/labust/vehicles-ros-pkg)** - vehicle drivers and diagnostics code is located in this repository. The code is organized in per vehicle packages. Dynamic model parameters, allocation data and 3D models for simulations can be found in each vehicle specific package.
* **[labust-visualization-pkg](https://github.com/labust/labust-visualization-pkg)** - the repository contains all visualization related packages including user interfaces, 3D visualization, control and planning interface code. 

Compilation
------------------
The repository depends on the **[labust-common-msgs](http://https://github.com/labust/labust-common-msgs)** packages that contain all required messages. Before proceeding clone this repository into your workspace.

Most dependencies can be installed using `rosdep` with: 

    rosdep install --from-paths <rosws>/src --ignore-src --rosdistro indigo -y

Nodes in the `labust_mission` package require a newer version of the TinyXML2 library which can be installed using
    
    sudo labust-ros-pkg/labust_mission/scripts/install_tinyxml2.sh
    
Navigation nodes require the GeographicLib library which can be installed for the Ubuntu repositories using `sudo apt-get install libgeographiclib-dev`. Geodetic, magnetic and gravity models have to be downloaded for the GeographicLib library. Running the script 

    sudo labust-ros-pkg/labust_navigation/scripts/download_geographiclib_models.sh

will install a minimum subset of available models. If models are not present some capabilities are not enabled, e.g. magnetic declination compensation.