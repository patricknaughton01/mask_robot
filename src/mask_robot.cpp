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
#include <memory>
#include <KrisLibrary/math3d/primitives.h>
#include <RenderRobotMask.h>
#include <KrisLibrary/image/ppm.h>
#include <KrisLibrary/camera/viewport.h>
#include <KrisLibrary/math/VectorTemplate.h>
#include <KrisLibrary/image/opencv_convert.h>
#include <Klampt/Modeling/World.h>
#include <Klampt/Sensing/Sensor.h>
#include <Klampt/Sensing/VisualSensors.h>

#define ROBOT_BUF 0.03

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
	std::vector<string> sensor_names{"zed_slam_left", "realsense_slam_l515"};
	std::vector<image_transport::Publisher> pubs;
	image_transport::ImageTransport it(n);
	for(size_t i = 0; i < sensor_names.size(); i++){
		image_transport::Publisher mask_pub = it.advertise(
			string("robot_mask/") + sensor_names[i], 1);
		pubs.push_back(mask_pub);
	}

	std::vector<std::shared_ptr<RenderRobotMask>> renderers;
	Robot robot;
	robot.Load(argv[1]);
	for(size_t i = 0; i < sensor_names.size(); i++){
		std::cout << sensor_names[i] << std::endl;
		RobotSensors sensors;
		sensors.MakeDefault(&robot);
		auto cam = sensors.GetNamedSensor(sensor_names[i]);
		if(!cam) {
			fprintf(stderr,"No sensor named %s\n", sensor_names[i].c_str());
			exit(1);
		}
		CameraSensor* cam2 = dynamic_cast<CameraSensor*>(&*cam);
		Camera::Viewport vp;
		cam2->GetViewport(vp);

		std::shared_ptr<RenderRobotMask> r = std::make_shared<RenderRobotMask>();
		if(!r->InitGLContext()){
			return 1;
		}
		if(!r->Setup(argv[1], vp.w, vp.h, ROBOT_BUF)){
			return 1;
		}
		renderers.push_back(r);
	}
	std::cout << "Loaded renderers" << std::endl;
	
	// safe is true because we may have other things running on the robot
	bool safe = true;
	ros::Rate rate(30);
	std::cout << "Publishing masks" << std::endl;
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
			Image mask;
			renderers[i]->robot.UpdateConfig(config);
			renderers[i]->SetSensor(sensor_names[i].c_str());
			renderers[i]->RenderMask(config, mask, safe);
			// Flipping the height and width, otherwise image rows and columns
			// are messed up.
			unsigned short tmp = mask.w;
			mask.w = mask.h;
			mask.h = tmp;
			cv::Mat cv_img = toMat(mask);
			std::vector<cv::Mat> channels(3);
			cv::split(cv_img, channels);
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
				std_msgs::Header(), "8UC1", channels[0]).toImageMsg();
			pubs[i].publish(msg);
			#ifdef DEBUG
				ExportImagePPM((sensor_names[i] + ".ppm").c_str(), mask);
				std::cout << cv_img.rows << " " << cv_img.cols << std::endl;
			#endif // DEBUG
		}

		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
