#include <local_occupancy_map/local_occupancy_map.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "local_occupancy_map_node");
  ros::NodeHandle nh("~");

  local_occupancy_map::LocalOccupancyMap map;
  map.init(nh);

  ros::spin();
  return 0;
}
