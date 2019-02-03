//===============================================================================//
// Name			: andorCallback.h
// Author(s)	: Kourosh Darvish
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: The class for handling the ros service and vectors of and/or graphs
//===============================================================================//

#ifndef ANDORCALLBACK_H
#define ANDORCALLBACK_H

#include "andor_aograph.h"
#include "std_msgs/String.h"
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
using namespace std;

class andorCallback{

	vector<shared_ptr<AOgraph>> graphVector;
	double onlineElapse, offlineElapse;
//	ros::Publisher pub_ctrl_cmnd;

public:
	andorCallback(string andor_graph_name, string andor_file_path, string andor_file_name);
	~andorCallback();
	bool updateANDOR(andor_msgs::andorSRV::Request &req, andor_msgs::andorSRV::Response &res);

};


#endif
