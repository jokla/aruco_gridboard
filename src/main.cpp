#include "node.h"

int main(int argc,char** argv){
  ros::init(argc, argv, "aruco_gridboard");
  aruco_gridboard::Node().spin();
	return 0;
}
