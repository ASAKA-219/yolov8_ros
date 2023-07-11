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

src :<br>
https://github.com/ultralytics/ultralytics
