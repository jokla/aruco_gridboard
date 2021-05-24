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
To use aruco with ROS, we need to use opencv 3.2. Since that we need `cv_bridge`, we have to rebuild [`vision_opencv`](https://github.com/ros-perception/vision_opencv) from source. Otherwise `cv_bridge` is built with opencv2. [link](http://stackoverflow.com/questions/37561156/how-to-correctly-link-opencv3-under-ros-indigo-using-cmake)
You can clone  `vision_opencv` in your `catkin_ws/scr` and after build with:
```
$ cd ~/catkin_ws
$ catkin_make -DOpenCV_DIR=/path-to-build-opencv-3.2
```

When catkin_make has finish, you can check if the `cv_bridge` is using the right version of OpenCV with the following commands:

```
$ cd ~/catkin_ws/devel/lib
$ ldd libcv_bridge.so | grep opencv
	libopencv_core.so.3.2 => /home/jokla/Software/opencv-3.2.0/build/lib/libopencv_core.so.3.2 (0x00007f7acb240000)
	libopencv_imgcodecs.so.3.2 => /home/jokla/Software/opencv-3.2.0/build/lib/libopencv_imgcodecs.so.3.2 (0x00007f7acaffe000)
	libopencv_imgproc.so.3.2 => /home/jokla/Software/opencv-3.2.0/build/lib/libopencv_imgproc.so.3.2 (0x00007f7ac97ca000)
```


### aruco_gridboard
Now you can clone aruco_gridboard in your `catkin_ws` and build with catkin_make
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/jokla/aruco_gridboard.git
$ cd ..
$ catkin_make
$ source devel/setup.bash
```

## How to use it
Add the new OpenCV to the path:   
`$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/jokla/Software/opencv-3.2.0/build/lib/`   
Launch the detection node:   
`roslaunch aruco_gridboard detection.launch`

[Here](https://github.com/jokla/aruco_gridboard/blob/master/launch/detection.launch) you can see an example of the launch file:
```xml

<launch>  
  <!-- Launch the detection node -->
  <node pkg="aruco_gridboard" type="aruco_gridboard" name="aruco_gridboard" output="screen" >
    <param name="board_path" value="$(find aruco_gridboard)/data/layout.yaml" />
    <param name="detector_param_path" value="$(find aruco_gridboard)/data/detector_params.yml" />
    <param name="debug_display" value="True" />
   
    <remap from="/aruco_gridboard/camera_info" to="/camera/camera_info"/>
    <remap from="/aruco_gridboard/image_raw" to="/camera/image"/>
  </node>

</launch>


```
The node is subscribing to the image topic `/camera/image` and the topic `/camera/camera_info` containing the camera parameters. Aruco will try to detect the board described by the yaml file (you can set in with the parameter `board_path` and it will publish the board pose on the topic `/vision/pose` and the detection status on the topic `/vision/status`). 



