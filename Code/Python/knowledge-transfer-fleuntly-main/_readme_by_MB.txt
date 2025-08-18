MB Comments:

All modified code is marked with "MB"

#################### Install ####################################

All modules of this dir now compile correctly after having installed following:
>conda activate ml_env
>conda install conda-forge::matplotlib==3.8.4

Problem: matplotlib 3.5.0 did not contain pyplot
matplotlib==3.8.4 does contain pyplot
=> You have to find out manually which ver contains pyplot (I asked Vincenzo which ver he uses, and I installed it)

Note: 
Installing matplotlib.pyplot via pycharm did not work
If a version is not supported by conda you get an error, but no advice on which other ver to use (linux does it!)

################### Visualizzatore RViz #####################################

Create a ROS2 workspace

Risk: Paolo does not know if this code works in win10

https://gitlab-core.supsi.ch/dti-isteps/armlab/fluently/ros_driver_installation/-/blob/main/.rosinstall?ref_type=heads

Contiene la lista dei package da installare, usare github clone
https://gitlab-core.supsi.ch/dti-isteps/armlab/fluently/ros_driver_installation/-/blob/main/.rosinstall?ref_type=heads