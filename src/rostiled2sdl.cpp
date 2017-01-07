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
    ros::NodeHandle nh_private("~");
    unsigned int idx = _nmovables-1;
    _twist_subs.push_back(nh_private.subscribe<geometry_msgs::Twist>
        ("foo", 0, boost::bind(&RosMapRenderer::twist_cb, this, _1, idx)));
    return true;
  }

  void twist_cb(const geometry_msgs::Twist::ConstPtr & msg,
                unsigned int idx) {
    if (idx >= _nmovables)
      return;
    //_movables[idx]._xspeed =
  }

  std::vector<ros::Subscriber> _twist_subs;
}; // end class RosMapRenderer



int main(int argc, char **argv) {
  ros::init(argc, argv, "rostiled2sdl");
  std::string map_name = ros::package::getPath("rostiled") + "/data/maps/simple_test.tmx";
  std::string cars_path = ros::package::getPath("rostiled") + "/data/cars/";
  RosMapRenderer r;
  if (!r.init(map_name, 1350, 600)
      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 200, 250)
      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 400, 250))
    return -1;
  // loop
  ros::Rate rate(20);
  while (true) {
    if (!r.update())
      break;
    if (!r.render())
      break;
    rate.sleep();
  } // end while (true)
  printf("Exiting\n");
  return 0;
}
