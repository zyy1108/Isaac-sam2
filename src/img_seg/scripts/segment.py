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
        #初始化SAM2(根据下载的数据集调整)
        model = build_sam2(
            "configs/sam2.1/sam2.1_hiera_l.yaml",#配置文件路径
            "/home/gpu007-905/isaac_vision/src/sam2/checkpoints/sam2.1_hiera_large.pt" #预训练模型路径
        )
        
        #sam2/automatic_mask_generator.py中有各参数的注释
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
        
        #ROS初始化(根据实际的ROS节点信息调整)
        self.sub_image = rospy.Subscriber('/camera/color/image_rect_color', Image,self.image)
        # self.sub_depth = rospy.Subscriber('/camera/depth_registered/sw_registered/image_rect_raw', Image, self.depth)
        # self.sub_cam_info = rospy.Subscriber('/camera/depth_registered/sw_registered/camera_info', CameraInfo, self.cam_info)
        self.mask_pub = rospy.Publisher('/sam2/all_masks', Image, queue_size=10)
        self.overlay_pub = rospy.Publisher('/sam2/all_overlays', Image, queue_size=10)
        
        self.current_depth = None
        self.camera_matrix = None           

    #获取相机内参
    def cam_info(self, msg):
        self.camera_matrix = np.array(msg.K).reshape(3, 3)
    
    #存储当前深度图
    def depth(self, msg):
        # 转换为16UC1格式（根据实际情况调整）
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        
    # 创建带标签和随机颜色框的叠加可视化图像
    def label_overlay(self, image, masks, alpha=0.3):
        """
        param image: 原始RGB图像 (H,W,3)
        param mask: 分割掩码 (H,W)
        param alpha: 叠加透明度
        """
    
        overlay = image.copy() #保持RGB格式
        combined_mask = np.zeros_like(overlay)  # 创建用于合并所有掩码的空白图像
        
        for i, mask_data in enumerate(masks):
            mask = mask_data["segmentation"]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            combined_mask[mask] = color  # 将所有掩码合并到同一图像
            
        #叠加所有掩码
        overlay = cv2.addWeighted(image, 1, combined_mask, alpha, 0)
        
        for i, mask_data in enumerate(masks):
            mask = mask_data["segmentation"]
            label = mask_data.get("label", f"Object {i}")
            color=(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            
            # 计算sam2掩码的边界框
            contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                cnt = max(contours, key=cv2.contourArea) # 获取最大轮廓的边界框
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(overlay, (x, y), (x+w, y+h),color, 2)# 绘制边界框 (线宽2px)
        
                # 计算文本位置(框顶部居中)
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                text_x = x + (w - text_size[0]) // 2
                text_y = y - 5 if y - 5 > 5 else y + h + 15
            
                # 绘制文本背景框
                cv2.rectangle(overlay, 
                         (text_x - 2, text_y - text_size[1] - 2),
                         (text_x + text_size[0] + 2, text_y + 2),
                          color, -1)
            
                # 添加白色文本
                cv2.putText(overlay, label, (text_x, text_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return overlay
    
    #图像处理函数
    def image(self, msg):
        
        #图像预处理
        cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        rospy.loginfo(f"图像尺寸: {cv_image.shape}, 类型: {cv_image.dtype},连续性={cv_image.flags['C_CONTIGUOUS']}")
      
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
            try:
                #SAM2分割
                self.predictor.set_image(cv_image)
                masks = self.mask_generator.generate(cv_image)
                mask_data_list = []
                for i, mask_data in enumerate(masks):
                    mask_data_list.append({
                        "segmentation": mask_data["segmentation"],
                    })
                    
                #发布带标签的可视化结果
                overlay = self.label_overlay(cv_image, mask_data_list)
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, "rgb8")
                overlay_msg.header = msg.header
                self.overlay_pub.publish(overlay_msg)
                rospy.loginfo("SAM2分割成功")
                
            except Exception as e:
                rospy.logerr(f"SAM2分割失败: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('segment')
    rospy.loginfo("[segment start]")
    detector = Segment()
    rospy.spin()
