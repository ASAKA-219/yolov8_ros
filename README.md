# yolov8_ros
This is ROS-noetic-YOLOv8 package. I couldn't make other nessesarry directries but I'm going to make them.

## Install
install ultralytics
```
pip install ultralytics
```
clone in your workspace
```
cd PATH/TO/WORKSPACE/SRC/
```
```
git clone https://github.com/ASAKA-219/yolov8_ros
```
If you don't install Image msg publish pkg. You can Install
[usb_cam pkg]()
automatically.
```
cd PATH/TO/WORKSPACE/
rosdep install --from-paths src --ignore-src --rosdistro $ROSDISTRO -y
```
build package
```
catkin build
```

## detect.py
 This is yolov8_ros main scripts.


### rosrun
```
rosrun yolov8_ros detect.py image:=<TOPIC>
```
### Topic
- **Subscribe**<br>
    Image
- **Publish**<br>
    /yolov8_ros/detect_img

## Publish

- **yolov8_ros/detect_img**<br>
- **yolov8_ros/bounding_box**<br>
- **yolov8_ros/bounding_boxes**<br>

src :<br>
https://github.com/ultralytics/ultralytics
