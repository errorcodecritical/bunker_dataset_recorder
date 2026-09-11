#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torch.nn.functional as F
import numpy as np
from PIL import Image as PILImage
from torchvision import transforms
import yaml
import os
from torch import nn

# === Load config ===
def load_config(config_path):
    with open(config_path, 'r') as file:
        config = yaml.safe_load(file)
    config['height'] = int(config['height'])
    config['width'] = int(config['width'])
    config['original_height'] = int(config['original_height'])
    config['original_width'] = int(config['original_width'])
    return config

# === Model ===
class DINOv2FeatUp(nn.Module):
    def __init__(self, upsampler, seg_channels=384, seg_hidden=64):
        super().__init__()
        self.upsampler = upsampler
        self.seg_head = nn.Sequential(
            nn.Conv2d(seg_channels, seg_hidden, kernel_size=1),
            nn.BatchNorm2d(seg_hidden),
            nn.ReLU(inplace=True),
            nn.Conv2d(seg_hidden, seg_hidden, kernel_size=1),
            nn.BatchNorm2d(seg_hidden),
            nn.ReLU(inplace=True),
            nn.Conv2d(seg_hidden, seg_hidden, kernel_size=1),
            nn.BatchNorm2d(seg_hidden),
            nn.ReLU(inplace=True),
            nn.Conv2d(seg_hidden, 1, kernel_size=1),
            nn.Hardsigmoid()
        )

    def forward(self, x):
        hr_feats = self.upsampler(x)
        seg_out = self.seg_head(hr_feats)
        return seg_out

# === Inference Node ===
class InferenceNode:
    def __init__(self):
        rospy.init_node('dinov2_inference_node')

        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/inference_result", Image, queue_size=1)

        config_path = rospy.get_param("~config_path", "/catkin_ws/src/image_space_path_planning/configs/psteer_config.yaml")
        self.config = load_config(config_path)

        self.device = torch.device(self.config['device'])
        self.model_path = self.config['model_save_path']

        self.transform = transforms.Compose([
            transforms.Resize((self.config['height'], self.config['width'])),
            transforms.ToTensor(),
            lambda x: (x - 0.5) / 0.5  # Manual normalization
        ])

        upsampler = torch.hub.load("mhamilton723/FeatUp", 'dinov2', use_norm=False).to(self.device)
        self.model = DINOv2FeatUp(upsampler).to(self.device)

        if os.path.exists(self.model_path):
            self.model.load_state_dict(torch.load(self.model_path, map_location=self.device))
        else:
            rospy.logwarn("Model checkpoint not found.")
        self.model.eval()

        # Subscribe to image topic
        rospy.Subscriber("/oak/rgb/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            pil_image = PILImage.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            input_tensor = self.transform(pil_image).unsqueeze(0).to(self.device)

            with torch.no_grad():
                pred = self.model(input_tensor)
                pred = F.interpolate(pred, size=(self.config['original_height'], self.config['original_width']),
                                     mode='bilinear', align_corners=False)

            pred_mask = pred.squeeze().cpu().numpy()
            pred_mask = np.clip(pred_mask, 0, 1) * 255
            pred_mask = pred_mask.astype(np.uint8)
            heatmap = cv2.applyColorMap(pred_mask, cv2.COLORMAP_JET)

            heatmap_msg = self.bridge.cv2_to_imgmsg(heatmap, encoding="bgr8")
            self.pub.publish(heatmap_msg)

        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

if __name__ == '__main__':
    try:
        node = InferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
