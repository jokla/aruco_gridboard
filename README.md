# aruco_gridboard
This node detect the ARUCO board that is on the target. It then publishes the corresponding pose in the /vision/pose topic.

## Installation
### OpenCV 3.2
Clone and build [OpenCV](https://github.com/opencv/opencv) from source with the [extra modules](https://github.com/opencv/opencv_contrib) 
```
$ cd <opencv_build_directory>
$ cmake -DOPENCV_EXTRA_MODULES_PATH=<opencv_contrib>/modules <opencv_source_directory>
$ make -j5
```
### vision_opencv
To use aruco with ROS, we need to use opencv3 installed in opt/ros... if we need also `cv_bridge` we have to rebuild [`vision_opencv`](https://github.com/ros-perception/vision_opencv) using the same version of opencv3. Otherwise cv_bridge is built with opencv2. [link](http://stackoverflow.com/questions/37561156/how-to-correctly-link-opencv3-under-ros-indigo-using-cmake)
You can clone  `vision_opencv` in your `catkin_ws/scr` and after build with:
```
$ cd ~/catkin_ws
$ catkin_make -DOpenCV_DIR=/path-to-build-opencv-3.2
```
### aruco_gridboard
Now you can clone aruco_gridboard in your `catkin_ws` and build with catkin_make


## How to use

`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/jokla/Software/opencv-3.2.0/build/lib/`   

`roslaunch aruco_gridboard detection.launch`
