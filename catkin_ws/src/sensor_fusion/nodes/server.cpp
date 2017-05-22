#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <sensor_fusion/FusionConfig.h>

void callback(sensor_fusion::FusionConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_fusion");

  dynamic_reconfigure::Server<sensor_fusion::FusionConfig> server;
  dynamic_reconfigure::Server<sensor_fusion::FusionConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
