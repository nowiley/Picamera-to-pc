#!/usr/bin/python3
"""
ROS node that subscribes to the camera/image topic, runs the depth estimation model, and publishes a stream of depth images
along with their original rgb images and imu data. 
"""
import torch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from os.path import dirname, abspath, join
import sys
import time

THIS_DIR = dirname(__file__)
CODE_DIR = abspath(join(THIS_DIR, '..', 'Depth-Anything-V2'))
sys.path.append(CODE_DIR)
CODE_DIR = abspath(join(THIS_DIR, '..', 'Depth-Anything-V2/checkpoints'))
sys.path.append(CODE_DIR)

for path in sys.path:
    print(path)


from depth_anything_v2.dpt import DepthAnythingV2


# Load the model according to https://github.com/DepthAnything/Depth-Anything-V2
DEVICE = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'

model_configs = {
    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
}

encoder = 'vits' # or 'vitl', 'vitb', 'vitg'

model = DepthAnythingV2(**model_configs[encoder])
# model.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
model.load_state_dict(torch.load('/home/lean/Picamera-to-pc/controls_docker/lean_ws-main/src/camera_int_pkg/src/Depth-Anything-V2/checkpoints/depth_anything_v2_vits.pth', map_location='cpu'))
model = model.to(DEVICE).eval()

#######
# ROS #
#######

# Image callback to run the model and publish the depth image
def image_callback(msg):
    print("Starting Frame Ifer")
    cur_time = time.time()
    raw_img = bridge.imgmsg_to_cv2(msg)
    depth_map = model.infer_image(raw_img) # HxW raw depth map in numpy
    # depth_img = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
    # Step 1: Normalize the depth map to the range [0, 255]
    depth_map_normalized = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX)

    # Step 2: Convert to an 8-bit image
    depth_map_8bit = np.uint8(depth_map_normalized)

    # Step 3: Apply a colormap for better visualization (optional)
    depth_img = cv2.applyColorMap(depth_map_8bit, cv2.COLORMAP_JET)

    print("Finished Infer Total time: " + str(time.time()-cur_time))

    cv2.imshow('Depth Img', depth_img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        pass
    depth_img_msg = bridge.cv2_to_imgmsg(depth_img, encoding='bgr8')
    depth_img_msg.header = msg.header
    pub.publish(depth_img_msg)

    

# Initialize the node, bridge, and publishers/subscribers
rospy.init_node('depth_estimator')
bridge = CvBridge()
pub = rospy.Publisher('/camera/depth', Image, queue_size=1)
rospy.Subscriber('/camera/image', Image, image_callback, queue_size=1, buff_size=2**26)

# Keep the node running 
rospy.spin()
