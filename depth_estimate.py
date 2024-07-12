"""
ROS node that subscribes to the camera/image topic, runs the depth estimation model, and publishes a stream of depth images
along with their original rgb images and imu data. 
"""
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
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
model.load_state_dict(torch.load(f'checkpoints/depth_anything_v2_{encoder}.pth', map_location='cpu'))
model = model.to(DEVICE).eval()

#######
# ROS #
#######

# Image callback to run the model and publish the depth image
def image_callback(msg):
    raw_img = bridge.imgmsg_to_cv2(msg)
    depth = model.infer_image(raw_img) # HxW raw depth map in numpy
    depth_img = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
    depth_img_msg = bridge.cv2_to_imgmsg(depth_img, encoding='bgr8')
    depth_img_msg.header = msg.header
    pub.publish(depth_img_msg)

# Initialize the node, bridge, and publishers/subscribers
rospy.init_node('depth_estimator')
bridge = CvBridge()
pub = rospy.Publisher('/camera/depth', Image, queue_size=1)
rospy.Subscriber('/camera/image', Image, image_callback)

# Keep the node running 
rospy.spin()
