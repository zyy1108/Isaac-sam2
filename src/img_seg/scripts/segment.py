#!/usr/bin/env python
import rospy
import numpy as np
import torch
import cv2
import random
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from sam2.sam2_image_predictor import SAM2ImagePredictor
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

class Segment:
    def __init__(self):
        #Initialize SAM2(Adjust according to the download checkpoints)
        model = build_sam2(
            "configs/sam2.1/sam2.1_hiera_l.yaml",#yaml path
            "/home/gpu007-905/isaac_vision/src/sam2/checkpoints/sam2.1_hiera_large.pt" #checkpoints path
        )
        
        #Explained in detail in SAM2/automatic_mask_generator.py
        self.mask_generator = SAM2AutomaticMaskGenerator(
            model=model,
            points_per_side=32,  
            pred_iou_thresh=0.96, 
            stability_score_thresh=0.92,  
            crop_n_layers=1,  
            crop_n_points_downscale_factor=2,  
            min_mask_region_area=100 
        )
        rospy.loginfo("SAM2 masks generate sucess")
        
        self.predictor = SAM2ImagePredictor(model)
        self.bridge = CvBridge()
        
        #Initialization(Adjust based on the local ROS node information)
        self.sub_image = rospy.Subscriber('/camera/color/image_rect_color', Image,self.image)
        # self.sub_depth = rospy.Subscriber('/camera/depth_registered/sw_registered/image_rect_raw', Image, self.depth)
        # self.sub_cam_info = rospy.Subscriber('/camera/depth_registered/sw_registered/camera_info', CameraInfo, self.cam_info)
        self.mask_pub = rospy.Publisher('/sam2/all_masks', Image, queue_size=10)
        self.overlay_pub = rospy.Publisher('/sam2/all_overlays', Image, queue_size=10)
        
        self.current_depth = None
        self.camera_matrix = None           

    #get the cam_info
    def cam_info(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
    
    #get the current depth map
    def depth(self, msg):
        #Convert to 16UC1 format (adjust according to the actual situation)
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        
    #Create an overlay visualization with labels and random color boxes
    def label_overlay(self, image, masks, alpha=0.3):
        """
        param image: (H,W,3)
        param mask: (H,W)
        param alpha: transparency
        """
    
        overlay = image.copy() 
        combined_mask = np.zeros_like(overlay)  #Create a blank image that merges all masks
        
        for i, mask_data in enumerate(masks):
            mask = mask_data["segmentation"]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            combined_mask[mask] = color  #Merge all masks into the same image
            
        #overlay all masks
        overlay = cv2.addWeighted(image, 1, combined_mask, alpha, 0)
        
        for i, mask_data in enumerate(masks):
            mask = mask_data["segmentation"]
            label = mask_data.get("label", f"Object {i}")
            color=(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            
        # Calculate the bounding box of the SAM2 mask
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                cnt = max(contours, key=cv2.contourArea) #Get the bounding box with the largest outline
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(overlay, (x, y), (x+w, y+h),color, 2)# Draw Bounding Box (Line Width 2px)
        
                # Calculate text position (center at the top of the box)
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                text_x = x + (w - text_size[0]) // 2
                text_y = y - 5 if y - 5 > 5 else y + h + 15
            
                # Add a text background box
                cv2.rectangle(overlay, 
                         (text_x - 2, text_y - text_size[1] - 2),
                         (text_x + text_size[0] + 2, text_y + 2),
                          color, -1)
            
                # Add while text
                cv2.putText(overlay, label, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return overlay
    
    #Image preprocess
    def image(self, msg):
        
        #Image preprocessing
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        rospy.loginfo(f"image shape: {cv_image.shape}, type: {cv_image.dtype}, contiguous={cv_image.flags['C_CONTIGUOUS']}")
      
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            try:
                #SAM2 segment
                self.predictor.set_image(cv_image)
                masks = self.mask_generator.generate(cv_image)
                mask_data_list = []
                for i, mask_data in enumerate(masks):
                    mask_data_list.append({
                        "segmentation": mask_data["segmentation"],
                    })
                    
                #Publish labeled visualizations
                overlay = self.label_overlay(cv_image, mask_data_list)
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, "rgb8")
                overlay_msg.header = msg.header
                self.overlay_pub.publish(overlay_msg)
                rospy.loginfo("SAM2 Segement successs")
                
            except Exception as e:
                rospy.logerr(f"SAM2 Segement fail: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('segment')
    rospy.loginfo("[segment start]")
    detector = Segment()
    rospy.spin()
