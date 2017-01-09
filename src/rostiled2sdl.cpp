#include <rostiled/tiled2sdl.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

class RosMapRenderer : public MapRenderer {
public:
  bool create_movable(const std::string & xml_file,
                      double maxspeed = 1,
                      double xortho = 0, double yortho = 0, double angle = 0) {
    MapRenderer::create_movable(xml_file, maxspeed, xortho, yortho, angle);
    ros::NodeHandle nh_public;
    unsigned int idx = _nmovables-1;
    std::ostringstream topic;
    topic << "cmd_vel" << idx;
    _twist_subs.push_back(nh_public.subscribe<geometry_msgs::Twist>
        (topic.str(), 0, boost::bind(&RosMapRenderer::twist_cb, this, _1, idx)));
    return true;
  }

  void twist_cb(const geometry_msgs::Twist::ConstPtr & msg,
                unsigned int idx) {
    printf("twist_cb(idx:%i)\n", idx);
    if (idx >= _nmovables)
      return;
    _movables[idx].set_speeds(msg->linear.x, msg->angular.z);
  }

  std::vector<ros::Subscriber> _twist_subs;
}; // end class RosMapRenderer

int main(int argc, char **argv) {
  ros::init(argc, argv, "rostiled2sdl");
  ros::NodeHandle nh_private("~");
  std::string map_name = "simple_test.tmx";
  nh_private.param("map_name", map_name, map_name);
  std::string longmap_name = ros::package::getPath("rostiled") + "/data/maps/" + map_name;
  std::string cars_path = ros::package::getPath("rostiled") + "/data/cars/";
  RosMapRenderer r;
  if (!r.init(longmap_name, 1350, 600)
      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 200, 250)
      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 400, 250))
    return -1;
  // loop
  ros::Rate rate(20);
  ros::AsyncSpinner spinner(0);
  spinner.start();
  while (ros::ok()) {
    Timer timer;
    if (!r.update())
      break;
    ROS_INFO_THROTTLE(1, "Time for update:%g ms.", timer.getTimeSeconds() * 1000);
    timer.reset();
    if (!r.render())
      break;
    ROS_INFO_THROTTLE(1, "Time for render:%g ms.", timer.getTimeSeconds() * 1000);
    rate.sleep();
  } // end while (true)
  printf("Exiting\n");
  return 0;
}
