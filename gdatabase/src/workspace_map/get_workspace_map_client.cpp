#include "ros/ros.h"
#include "gdatabase/GetWorkspaceOnMap.h"
#include <ros/duration.h>
#include <cstdlib>
#include <stdio.h>
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_workspace_map_client");

  ros::NodeHandle wm;
  ros::ServiceClient clientworkspaceonmap = wm.serviceClient<gdatabase::GetWorkspaceOnMap>("GetWorkspaceOnMap");
  gdatabase::GetWorkspaceOnMap srv;
  srv.request.mapID = atoll(argv[1]);
  if (clientworkspaceonmap.call(srv))
  {
	  int num=srv.response.objectID.size();
	  for(int i=0;i<num;i++){
		  cout<<"object ID: "<<srv.response.objectID.at(i)<<", classID: "<<srv.response.classID.at(i)<<endl;
	  }
  }
  else
  {
    ROS_ERROR("Failed to call service GetWorkspaceOnMap");
    return 1;
  }

  return 0;
}


