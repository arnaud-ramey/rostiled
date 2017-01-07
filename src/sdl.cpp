#include <rostiled/tiled2sdl.h>
#include <ros/package.h>

int main(/*int argc, char **argv*/) {
  std::string map_name = ros::package::getPath("rostiled") + "/data/maps/simple_test.tmx";
  std::string cars_path = ros::package::getPath("rostiled") + "/data/cars/";
  MapRenderer r;
  if (!r.init(map_name, 1350, 600)
      || !r.init_joysticks()
      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 200, 250)
      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 400, 250))
    return -1;

  // loop
  Rate rate(20);
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
