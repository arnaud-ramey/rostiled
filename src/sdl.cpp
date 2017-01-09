#include <rostiled/tiled2sdl.h>
#include <ros/package.h>

int main(/*int argc, char **argv*/) {
  //std::string map_name = "simple_test.tmx";
  std::string map_name = "unailand.tmx";
  std::string longmap_name = ros::package::getPath("rostiled") + "/data/maps/" + map_name;
  std::string cars_path = ros::package::getPath("rostiled") + "/data/cars/";
  MapRenderer r;
  if (!r.init(longmap_name, 1350, 600)
      || !r.init_joysticks()
      //      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 200, 150)
      || !r.create_movable(cars_path + "gimp/gimp.xml", 200, 200, 150)
      || !r.create_movable(cars_path + "adiumy/adiumy.xml", 200, 300, 150)
      || !r.create_movable(cars_path + "spiderman/spiderman.xml", 200, 400, 150))
    return -1;

  // loop
  Rate rate(20);
  while (true) {
    Timer timer;
    if (!r.update())
      break;
    printf("Time for update:%g ms.\n", timer.getTimeSeconds() * 1000);
    timer.reset();
    if (!r.render())
      break;
    printf("Time for render:%g ms.\n", timer.getTimeSeconds() * 1000);
    rate.sleep();
  } // end while (true)
  printf("Exiting\n");
  return 0;
}
