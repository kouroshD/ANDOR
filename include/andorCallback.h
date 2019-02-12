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
#include <andor/andorSRV.h>
#include <andor/Hyperarc.h>
#include <andor/Node.h>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
using namespace std;

//! class "andorCallback" for the query of an AND/OR graph among the available ones.
class andorCallback{

	vector<shared_ptr<AOgraph>> graphVector;	//!< the vector of AND/OR graphs
	double onlineElapse;						//!< the online time
	double offlineElapse;						//!<  the offline time
public:
	//! constructor of class andorCallback
	//! @param[in] andor_graph_name	   the name of the AND/OR graph
	//! @param[in] andor_file_path	   the path to the AND/OR graph file
	//! @param[in] andor_file_name	   the name of the file that contains the highest level AND/OR graph
	andorCallback(string andor_graph_name, string andor_file_path, string andor_file_name);

	//! destructor of class andorCallback
	~andorCallback();

	//! destructor of class andorCallback
	//! @param[in] req	   the inquiry msg
	//! @param[in] res	   the response msg
    //! @return result of the operation (true = done, false = not done)
	bool updateANDOR(andor::andorSRV::Request &req, andor::andorSRV::Response &res);

};


#endif
