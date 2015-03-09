hog_haar_person_detection_ros
=============================

A ROS Hydro package implementing the [OpenCV HOG pedestrian detector](http://docs.opencv.org/modules/gpu/doc/object_detection.html)[1] and [HAAR face detector](https://github.com/angusleigh/hog_haar_person_detection_ros.git).
Subscribes to an image being published, detects faces and standing pedestrians in the image,
draws the detections on a copy of the image and publishes results as a rostopic. Also displays
detections on an OpenCV window.

## Usage
[Install ROS](http://wiki.ros.org/ROS/Installation)

* $ cd [your hydro catkin src folder]
* $ git clone https://github.com/angusleigh/hog_haar_person_detection_ros.git
* $ cd ..
* $ catkin_make
* $ roslaunch hog_haar_person_detection_ros hog_haar_person_detection_ros.launch

You will probably also have to edit the "image_topic" parameter in the launch file to reflect the image topic you wish to detect people on.

[1] Link is for GPU documentation. I'm not aware of any documentation for the CPU implementaion (which is the one used for this repo).