ethzasl_msf
=====================

## Description
Time delay compensated single and multi sensor fusion framework based on an EKF.
Please see the wiki for more information: https://github.com/ethz-asl/ethzasl_msf/wiki

## Dependencies
Install the following ROS packages from their git repositories.

glog_catkin (https://github.com/ethz-asl/glog_catkin)

catkin_simple (https://github.com/catkin/catkin_simple)

If you are using bare bone ROS version install the following packages
```
 $ sudo apt-get install ros-kinetic-image-transport ros-kinetic-image-geometry
```
## Know issues
If your PC freezes during the compilation, you should check on your swap memory. Instructions how to add swap memory on existing Ubuntu installation can be found online (e.g. https://askubuntu.com/questions/33697/how-do-i-add-a-swap-partition-after-system-installation).

## Documentation
The API is documented here: http://ethz-asl.github.io/ethzasl_msf

## Contributing
You are welcome contributing to the package by opening a pull-request:
Please make yourself familiar with the Google c++ style guide: 
http://google-styleguide.googlecode.com/svn/trunk/cppguide.xml


##License
The source code is released under the Apache 2.0 license
