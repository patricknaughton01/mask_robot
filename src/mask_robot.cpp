/*
Depends on redis-plus-plus to interact with the redis database
https://github.com/sewenew/redis-plus-plus

and json to parse the outputs
https://github.com/nlohmann/json

ROS Usage:
rosrun mask_robot mask_robot <robot_urdf>
*/
#include <sw/redis++/redis++.h>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <KrisLibrary/math3d/primitives.h>
#include <RenderRobotMask.h>
#include <KrisLibrary/image/ppm.h>
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/math/VectorTemplate.h>
#include <KrisLibrary/image/opencv_convert.h>

#define ROBOT_BUF 0.03
#define F_WIDTH 640
#define F_HEIGHT 480

using namespace sw::redis;
using json = nlohmann::json;


int main(int argc, char **argv){
	if(argc != 2){
		std::cerr << "USAGE: rosrun mask_robot mask_robot <robot_urdf>" 
			<< std::endl;
		return 1;
	}
	auto redis = Redis("tcp://127.0.0.1:6379");
	ROS_INFO("Connected to Redis Database");

	ros::init(argc, argv, "mask_robot");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Publisher mask_pub = it.advertise("robot_mask", 1);

	RenderRobotMask renderer;
	if(!renderer.InitGLContext()){
		return 1;
	}
	if(!renderer.Setup(argv[1], F_WIDTH, F_HEIGHT, ROBOT_BUF)){
		return 1;
	}
	// Manual setting of viewport is just for debugging - eventually
	// select a sensor by name from URDF
	Camera::Viewport vp;
	vp.perspective = true;
	vp.setFOV(0.5);
	vp.x = vp.y = 0;
	vp.w = F_WIDTH;
	vp.h = F_HEIGHT;
	vp.n = 0.1;
	vp.f = 8.0;
	
	// safe is false because we may have other things running on the robot
	bool safe = false;
	ros::Rate rate(1);
	while(n.ok()){
		// Read and parse robot state from redis database
		auto t_val = redis.command<OptionalString>("JSON.GET", "ROBOT_STATE");
		auto j = json::parse(*t_val);
		// Eventually this should be the camera transform
		// / just read directly from the URDF
		Math3D::RigidTransform rt(
			Math3D::Vector3(1, 0, 0),
			Math3D::Vector3(0, 1, 0),
			Math3D::Vector3(0, 0, 1),
			Math3D::Vector3(0, 0, 5)
		);
		vp.xform = rt;
		renderer.SetViewport(vp);

		Image mask;
		auto jc = j["Position"]["Robotq"];
		std::vector<double> tmp;
		for(auto it = jc.begin(); it != jc.end(); it++){
			tmp.push_back(*it);
		}
		Math::VectorTemplate<double> config(tmp);
		renderer.RenderMask(config, mask, safe);
		#ifdef DEBUG
			ExportImagePPM("mask.ppm", mask);
		#endif // DEBUG

		cv::Mat cv_img = toMat(mask);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
			std_msgs::Header(), "bgr8", cv_img).toImageMsg();
		mask_pub.publish(msg);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
