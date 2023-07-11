#! /usr/bin/env python3
from sensor_msgs.msg import Image
from yolov8_ros_msgs.msg import BoundingBox
from ultralytics import YOLO
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge

if __name__ == "__main__":
    try:
        rospy.init_node("yolo_ros")

        model = YOLO("yolov8m.pt")
        bridge=CvBridge()

        # from ndarray
        #im2 = cv2.imread("bus.jpg")

        img_pub = rospy.Publisher("yolo_v8/detect_img", Image, queue_size=10)
        bb_pub = rospy.Publisher("yolo_v8/boudingbox", BoundingBox, queue_size=10)

        bb = BoundingBox()

        def img_callback(msg):
            if msg is None:
                rospy.loginfo("None data")

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
                #print(class_names)
                result.boxes.conf   # confidence score, (N, 1)
                result.boxes.cls    # cls, (N, 1)
                #print(result.boxes.cls)
                
                # Segmentation
                #result.masks.data      # masks, (N, H, W)
                #result.masks.xy        # x,y segments (pixels), List[segment] * N
                #result.masks.xyn       # x,y segments (normalized), List[segment] * N

                # Classification
                print(result.probs)     # cls prob, (num_class, )
                for i,bbox in enumerate(result.boxes.xyxy):
                    x1,y1,x2,y2=bbox
                    color = list(np.random.random(size=3) * 256)
                    
                    img=cv2.rectangle(img, (int(x1),int(y1)), (int(x2),int(y2)),color=color)

                    cls=class_names[result.boxes.cls[i]]
                    print(cls)
                    img=cv2.putText(img,cls, (int(x1),int(y1)),cv2.FONT_HERSHEY_PLAIN,1,color=color)

                    print(result.boxes.cls[i])

                    bb.xmin = int(x1)
                    bb.ymin = int(y1)
                    bb.xmax = int(x2)
                    bb.ymax = int(y2)
                    bb.Class = cls
                    bb.id = int(result.boxes.cls[i])
                    bb.probability = 0

                    bb_pub.publish(bb)
                    
            img_msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_pub.publish(img_msg)


        #rospy.Publisher()
        rospy.loginfo("yolo_v8 start")
        rospy.Subscriber("image", Image, img_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
