
#include <csignal>
#include <cstdio>
#include <math.h>
#include <SickLMS5xx.hh>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
using namespace SickToolbox;
using namespace std;

void publish_scan(ros::Publisher *pub, uint32_t *range_values,
                  uint32_t n_range_values, uint32_t *intensity_values,
                  uint32_t n_intensity_values, double scale, ros::Time start,
                  double scan_time, bool inverted, float angle_min,
                  float angle_max, std::string frame_id)
{
	static int scan_count = 0;
	sensor_msgs::LaserScan scan_msg;
	scan_msg.header.frame_id = frame_id;
	scan_count++;
	if(inverted) {
		scan_msg.angle_min = angle_max;
		scan_msg.angle_max = angle_min;
	} else {
		scan_msg.angle_min = angle_min;
		scan_msg.angle_max = angle_max;
	}
	scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min)
	                / (double)(n_range_values - 1);
	scan_msg.scan_time = scan_time;
	scan_msg.time_increment = scan_time / (2 * M_PI) * scan_msg.angle_increment;
	scan_msg.range_min = 0;
	scan_msg.range_max = 81.;
	scan_msg.ranges.resize(n_range_values);
	scan_msg.header.stamp = start;
	for(size_t i = 0; i < n_range_values; i++) {
		scan_msg.ranges[i] = (float)range_values[i] * (float)scale;
	}
	scan_msg.intensities.resize(n_intensity_values);
	for(size_t i = 0; i < n_intensity_values; i++) {
		scan_msg.intensities[i] = (float)intensity_values[i];
	}

	pub->publish(scan_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sicklms5xx");
	string port;
	int baud;
	bool inverted = false;
	int angle;
	double resolution;
	std::string frame_id;
	std::string ip_add;
	double scan_time = 0;
	double angle_increment = 0;
	float angle_min = 0.0;
	float angle_max = 0.0;

	ros::NodeHandle nh;
	ros::NodeHandle nh_ns("~");
	ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan> ("scan", 1);
	//nh_ns.param("port", port, string("/dev/lms200"));
	//nh_ns.param("baud", baud, 38400);
	//nh_ns.param("inverted", inverted, false);
	nh_ns.param("angle", angle, 0);
	nh_ns.param("resolution", resolution, 0.0);
	nh_ns.param<std::string>("ip_add", ip_add, "192.168.1.8");
	// nh_ns.param<std::string>("ip_add", ip_add, "192.168.137.115");
	nh_ns.param<std::string> ("frame_id", frame_id, "/laser");

	uint32_t range_values[SickLMS5xx::SICK_LMS_5XX_MAX_NUM_MEASUREMENTS] = { 0 };
	uint32_t intensity_values[SickLMS5xx::SICK_LMS_5XX_MAX_NUM_MEASUREMENTS] = { 0 };
	uint32_t n_range_values = 0;
	uint32_t n_intensity_values = 0;
	SickLMS5xx sick_lms(ip_add);
	double scale = 0;
	double angle_offset;
	uint32_t partial_scan_index;

	try {
		sick_lms.Initialize();
		sick_lms.SetSickScanFreqAndRes(SickLMS5xx::SICK_LMS_5XX_SCAN_FREQ_50,
		                                       SickLMS5xx::SICK_LMS_5XX_SCAN_RES_50);
		/*sick_lms.SetSickScanFreqAndRes(SickLMS5xx::SICK_LMS_5XX_SCAN_FREQ_25,
		                               SickLMS5xx::SICK_LMS_5XX_SCAN_RES_25);*/
		//sick_lms.SetSickEchoFilter(SickLMS5xx::SICK_LMS_5XX_ECHO_FILTER_ALL_ECHOES);
		sick_lms.SetSickEchoFilter(SickLMS5xx::SICK_LMS_5XX_ECHO_FILTER_FIRST);

		// Scale is mm with the 5xx driver
		scale = 0.001;

	} catch(...) {
		ROS_ERROR("Initialize failed!");
		return 2;
	}
	try {
		ros::Time last_scan_time = ros::Time::now();
		while(ros::ok()) {
			angle_min = (sick_lms.GetSickStartAngle()-90.) * M_PI / 180.0;
			angle_max = (sick_lms.GetSickStopAngle()-90.)  * M_PI / 180.0;

			sick_lms.GetSickMeasurements(range_values, NULL, NULL, NULL, NULL,
			                             NULL, NULL, NULL, NULL, NULL,
			                             n_range_values);
			n_intensity_values = 0;

			ros::Time end_of_scan = ros::Time::now();
			ros::Time start = end_of_scan; // TODO - ros::Duration(scan_time / 2.0);

			ros::Duration diff = start - last_scan_time;
			last_scan_time = start;

			publish_scan(&scan_pub, range_values, n_range_values,
			             intensity_values, n_intensity_values, scale, start,
			             scan_time, inverted, angle_min, angle_max, frame_id);
			ros::spinOnce();
		}
	} catch(...) {
		ROS_ERROR("woah! error!");
		return 1;
	}
	try {
		sick_lms.Uninitialize();
	} catch(...) {
		ROS_ERROR("error during uninitialize");
		return 1;
	}
	ROS_INFO("success.\n");

	return 0;
}
