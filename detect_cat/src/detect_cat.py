#! /usr/bin/env python3

import rospy, cv2, time, datetime, sys, traceback, warnings
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Vector2D
from torchvision import transforms
from utils import *
from PIL import Image, ImageDraw, ImageFont

class Model():
    def __init__(self, checkpoint):
        # Load model checkpoint
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.checkpoint = torch.load(checkpoint, map_location=self.device)
        self.start_epoch = self.checkpoint['epoch'] + 1
        print('\nLoaded checkpoint from epoch %d.\n' % self.start_epoch)
        self.model = self.checkpoint['model'].to(self.device).eval()
        #self.model.eval()


class Detection():
    def __init__(self, model):
        self.photo = 255 * np.zeros(shape=[480, 640, 3], dtype=np.uint8)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.bridge = CvBridge()
        self.subCamera = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self.getImage)
        self.pubLocation = rospy.Publisher('/move_bot', Vector2D, queue_size=1)
        self.annot_image = None
        self.coordinates = Vector2D()
                        
        # Model and Comparison Parameters
        self.model = model
        self.min_score = 0.2
        self.max_overlap = 0.1
        self.top_k = 200
        # Suppress everything but cat
        self.suppress = ('aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant', 'sheep', 'sofa', 'train', 'tvmonitor')   
        
        # Transforms
        self.resize = transforms.Resize((300, 300))
        self.to_tensor = transforms.ToTensor()
        self.normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])


    def getImage(self, msg):
        self.photo = self.bridge.compressed_imgmsg_to_cv2(msg)
        
    # Gets the maximum score for a detected cat and returns its index
    def maxScore(self, det_scores, det_labels):
        max_val = 0
        index = -1

        for i in range(det_scores.size(0)):
            if self.suppress is not None:
                if det_labels[i] in self.suppress:
                    continue
                        
            if max_val < det_scores[i]:
                max_val = det_scores[i]
                index = i
        
        return index


    def annotate_Image(self, det_scores, det_labels, det_boxes):
        # Annotate
        draw = ImageDraw.Draw(self.annot_image)
        #font = ImageFont.truetype("./calibril.ttf", 15)
        font = ImageFont.load_default()
        
        # Get the maximum value for a detected cat
        index = self.maxScore(det_scores, det_labels)
                            
        box_location = None
        
        if not index == -1:
            
            # Boxes
            box_location = det_boxes[index].tolist()
            draw.rectangle(xy=box_location, outline=label_color_map[det_labels[index]])
            draw.rectangle(xy=[l + 1. for l in box_location], outline=label_color_map[
                det_labels[index]])  # a second rectangle at an offset of 1 pixel to increase line thickness
            # draw.rectangle(xy=[l + 2. for l in box_location], outline=label_color_map[
            #     det_labels[i]])  # a third rectangle at an offset of 1 pixel to increase line thickness
            # draw.rectangle(xy=[l + 3. for l in box_location], outline=label_color_map[
            #     det_labels[i]])  # a fourth rectangle at an offset of 1 pixel to increase line thickness

            # Text
            text_size = font.getsize(det_labels[index].upper())
            text_location = [box_location[0] + 2., box_location[1] - text_size[1]]
            textbox_location = [box_location[0], box_location[1] - text_size[1], box_location[0] + text_size[0] + 4.,
                                box_location[1]]
            draw.rectangle(xy=textbox_location, fill=label_color_map[det_labels[index]])
            draw.text(xy=text_location, text=det_labels[index].upper(), fill='white',
                    font=font)
        del draw
        
        return box_location
        
    def detect(self): 
        """
        Detect objects in an image with a trained SSD300, and visualize the results.

        :param original_image: image, a PIL Image
        :param min_score: minimum threshold for a detected box to be considered a match for a certain class
        :param max_overlap: maximum overlap two boxes can have so that the one with the lower score is not suppressed via Non-Maximum Suppression (NMS)
        :param top_k: if there are a lot of resulting detection across all classes, keep only the top 'k'
        :param suppress: classes that you know for sure cannot be in the image or you do not want in the image, a list
        :return: annotated image, a PIL Image
        """
        
        original_image = Image.fromarray(self.photo)
        original_image = original_image.convert('RGB')
                
        # Transform
        image = self.normalize(self.to_tensor(self.resize(original_image))).to(self.model.device)

        # Forward prop.
        predicted_locs, predicted_scores = self.model.model(image.unsqueeze(0))

        # Detect objects in SSD output
        det_boxes, det_labels, det_scores = self.model.model.detect_objects(predicted_locs, predicted_scores,       min_score=self.min_score, max_overlap=self.max_overlap, top_k=self.top_k)

        # Move detections to the CPU
        det_boxes = det_boxes[0].to('cpu')

        # Transform to original image dimensions
        original_dims = torch.FloatTensor(
            [original_image.width, original_image.height, original_image.width, original_image.height]).unsqueeze(0)
                
        det_boxes = det_boxes * original_dims

        # Decode class integer labels
        det_labels = [rev_label_map[l] for l in det_labels[0].to('cpu').tolist()]
        
        self.annot_image = original_image

        # If no objects found, the detected labels will be set to ['0.'], i.e. ['background'] in SSD300.detect_objects() in model.py
        if det_labels == ['background']:
            # Provices original image. No box outline for cat since none was found
            return None

        # Annotates the Image and returns the location of the box around the cat
        box_location = self.annotate_Image(det_scores[0], det_labels, det_boxes)
        
        return box_location
        

    def displayImage(self):
        global out 

        while not rospy.is_shutdown():
                        
            box_location = self.detect()
            frame_image = np.array(self.annot_image) 
            
            cv2.line(frame_image,(300,240),(340,240),(128,255,128),1)
            cv2.line(frame_image,(320,220),(320,260),(128,255,128),1)
            timestamp = datetime.datetime.now()

            #cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
            if box_location is not None:
                
                # Find the center of the box
                x = 0.5 * (box_location[2] - box_location[0]) + box_location[0]
                y = 0.5 * (box_location[3] - box_location[1]) + box_location[1]
                
                self.coordinates.x = int(x)
                self.coordinates.y = int(y)

                self.pubLocation.publish(self.coordinates)



            else:
                cv2.putText(frame_image,'Target Detecting',(40,60), self.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                self.coordinates.x = 0
                self.coordinates.y = 0

                self.pubLocation.publish(self.coordinates)
            
            # Uncomment to record video
            #out.write(frame_image)
            cv2.imshow("Frame", frame_image)              # Display image
            key = cv2.waitKey(1) 
            cv2.imwrite('lena_opencv_red.jpg', frame_image)
            

# Uncomment to record video 
#out = cv2.VideoWriter('/home/evan/catkin_ws/src/output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 2, (640,480))


if __name__ == '__main__':
    rospy.init_node('detect_cat')
    # Model location
    checkpoint = '/home/evan/catkin_ws/src/robot_adeept/detect_cat/model/checkpoint_ssd300.pth.tar'
    
    # Suppress Deprecation Warnings
    warnings.filterwarnings("ignore")
    model = Model(checkpoint)    
    

           
    detect = Detection(model)
    
    # Uncomment to manually input image
    #img_path = '/home/evan/catkin_ws/src/robot_adeept/detect_cat/images/matilda.jpg'
    #original_image = Image.open(img_path, mode='r')
    #original_image = original_image.convert('RGB')
    #detect(original_image, min_score=0.2, max_overlap=0.1, top_k=200).show()
    #detect.photo = np.array(original_image)
    
    detect.displayImage()
    ##out.release()
    
    

