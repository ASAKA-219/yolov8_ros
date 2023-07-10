#! /usr/bin/env python3
from sensor_msgs.msg import Image
from ultralytics import YOLO
import rospy
import cv2
from cv_bridge import CvBridge

rospy.init_node("yolo_ros")

model = YOLO("yolov8m.pt")
bridge=CvBridge()

# from ndarray
#im2 = cv2.imread("bus.jpg")


def img_callback(msg):
    img=bridge.imgmsg_to_cv2(msg)
    img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = model.predict(source=img)  # save predictions as labels
    #print(results)
    class_names=results[0].names
    for result in results:
        # Detection
        result = result.cpu()
        result = result.numpy()
        result.boxes.xyxy   # box with xyxy format, (N, 4)
        print(class_names)
        result.boxes.conf   # confidence score, (N, 1)
        result.boxes.cls    # cls, (N, 1)
        #print(result.boxes.cls)
        
        # Segmentation
        #result.masks.data      # masks, (N, H, W)
        #result.masks.xy        # x,y segments (pixels), List[segment] * N
        #result.masks.xyn       # x,y segments (normalized), List[segment] * N

        # Classification
        #result.probs     # cls prob, (num_class, )
        for i,bbox in enumerate(result.boxes.xyxy):
            x1,y1,x2,y2=bbox
            img=cv2.rectangle(img, (int(x1),int(y1)), (int(x2),int(y2)),color=(255,0,0))

            cls=class_names[result.boxes.cls[i]]
            img=cv2.putText(img,cls, (int(x1),int(y1)),cv2.FONT_HERSHEY_PLAIN,1,color=(255,0,0))
            
    cv2.imshow("image", img)
    cv2.waitKey(1)

#rospy.Publisher()
rospy.Subscriber("image", Image, img_callback)
rospy.spin()
