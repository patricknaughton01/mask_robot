import argparse
import sys
import time
import klampt
import rospy
import numpy as np
import cv2
sys.path.append('/home/motion/TRINA')
import Motion
import Jarvis

sys.path.append('/home/motion/TRINA/SensorModule')
from sensorModule import Camera_Robot
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def main():
	parser = argparse.ArgumentParser(description="Get imgs")
	parser.add_argument('-w', type=str, 
		default="/home/motion/TRINA/Motion/data/TRINA_world_cholera.xml", 
		help="world file")
	parser.add_argument('-n', type=str, default="cholera", 
		help="codename")
	parser.add_argument('-c', nargs='*', 
		default=['zed_slam_left', 'zed_slam_right', 
			'realsense_slam_l515'], 
		help="cameras to use")
	parser.add_argument('--components', nargs='*', 
		default=['left_limb', 'right_limb', 'base'], help="components to use")
	parser.add_argument('-m', type=str, default='Kinematic',
		help='mode to operate in')
	args = parser.parse_args()
	mc = Motion.MotionClient()
	mc.restartServer(mode=args.m, components=args.components, 
		codename=args.n)
	world = klampt.WorldModel()
	world.loadFile(args.w)
	sensor_module = Camera_Robot(mc, world=world, cameras=args.c)
	time.sleep(1)
	camera_names = ["realsense_slam_l515", "zed_slam_left",
		"zed_slam_right"]
	topic_names = []
	rgb_suf = "_rgb"
	d_suf = "_d"
	for cn in camera_names:
		topic_names.append(cn + rgb_suf)
		topic_names.append(cn + d_suf)
	rate = rospy.Rate(30)
	pubs = {}
	topic_pref = "images/"
	for name in topic_names:
		full_name = topic_pref + name
		pub = rospy.Publisher(full_name, Image, queue_size=10)
		pubs[full_name] = pub
	#rospy.init_node("sim_pipe", anonymous=True)
	bridge = CvBridge()
	max_d = 6.0
	min_d = 0.0
	while not rospy.is_shutdown():
		img_dict = sensor_module.get_rgbd_images()
		for name in camera_names:
			img = img_dict[name]
			if len(img) > 0:
				color = img[0]
				# Normalize depth to be 0 <= d < 1
				depth = (img[1] - min_d) / (max_d - min_d)
				# Fit in maximum range of 16 (unsigned) bits
				depth *= (2**16 - 1)
				depth = depth.astype("uint16")
				depth = bridge.cv2_to_imgmsg(depth, "16UC1")
				pubs[topic_pref + name + rgb_suf].publish(bridge.cv2_to_imgmsg(color, "8UC3"))
				pubs[topic_pref + name + d_suf].publish(depth)
		rate.sleep()


if __name__ == "__main__":
	main()

