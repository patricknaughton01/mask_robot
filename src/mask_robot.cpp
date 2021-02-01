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
	std::vector<string> sensor_names{"realsense_slam_l515", "zed_slam_left"};
	std::vector<image_transport::Publisher> pubs;
	image_transport::ImageTransport it(n);
	for(size_t i = 0; i < sensor_names.size(); i++){
		image_transport::Publisher mask_pub = it.advertise(
			string("robot_mask/") + sensor_names[i], 1);
		pubs.push_back(mask_pub);
	}

	RenderRobotMask renderer;
	if(!renderer.InitGLContext()){
		return 1;
	}
	if(!renderer.Setup(argv[1], F_WIDTH, F_HEIGHT, ROBOT_BUF)){
		return 1;
	}
	
	// safe is false because we may have other things running on the robot
	bool safe = false;
	ros::Rate rate(1);
	while(n.ok()){
		// Read and parse robot state from redis database
		auto t_val = redis.command<OptionalString>("JSON.GET", "ROBOT_STATE");
		auto j = json::parse(*t_val);
		auto jc = j["Position"]["Robotq"];
		std::vector<double> tmp;
		for(auto it = jc.begin(); it != jc.end(); it++){
			tmp.push_back(*it);
		}
		Math::VectorTemplate<double> config(tmp);

		for(size_t i = 0; i < sensor_names.size(); i++){
			renderer.SetSensor(sensor_names[i].c_str());
			Image mask;
			renderer.RenderMask(config, mask, safe);
			#ifdef DEBUG
				ExportImagePPM((sensor_names[i] + ".ppm").c_str(), mask);
			#endif // DEBUG
			// Flipping the height and width, otherwise image rows and columns
			// are messed up.
			unsigned short tmp = mask.w;
			mask.w = mask.h;
			mask.h = tmp;
			cv::Mat cv_img = toMat(mask);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
				std_msgs::Header(), sensor_msgs::image_encodings::RGB8, cv_img).toImageMsg();
			pubs[i].publish(msg);
		}

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
