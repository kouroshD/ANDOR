//===============================================================================//
// Name			   : andor_main.cpp
// Author(s)	 :  Kourosh.darvish@edu.unige.it
// Affiliation : University of Genova, Italy - dept. DIBRIS
// Version		 : 1.0
// Description : Main program using the AND-OR graph library
//===============================================================================//

#include <iostream>
#include <ros/ros.h>
#include "andor_aograph.h"
#include "std_msgs/String.h"
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include <boost/shared_ptr.hpp>

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;
vector<shared_ptr<AOgraph>> graphVector , OfflineGraphVector;


bool updateANDOR(andor_msgs::andorSRV::Request &req, andor_msgs::andorSRV::Response &res){
	cout<<FBLU("------------- andor receive request -------------")<<endl;
	/*! when arrive a request:
	 * 1- check for the graph name, if it is not valid, return and error
	 * 2- if it is empty the solved nodes or hyper-arcs, respond by the feasible nodes and hyper-arcs
	 * 3- if a node or hyper-arc is solved, update the graph, and respond the feasible nodes/hyper-arcs
	 * 4- if the head node is solved, return it is solved
	 */

	// 1- check for the graph name
	int gIndex=-1; //! requested graph index number in graphVector, initialize wit rnd big value
	for (int i=0;i<graphVector.size();i++)
		if(req.graphName==graphVector[i]->gName)
			gIndex=i;

	// if the graph not found in online graphs, search for the graph in offline ones

	if (gIndex==-1){
		cout<<FRED("Graph Name:"<<req.graphName<<" Is False, Not Found in Online or Offline Graph Vectors, Check Your Graph Name You Are Requesting")<<endl;
		return false;
	}
	// graph name is correct
	else
	{
		cout<<FBLU("Requested graph name:")<<req.graphName<<endl;
		cout<<200<<endl;
		if(req.solvedNodes.size()>0)
		{
			cout<<"solved Nodes: ";
			for (int i=0; i<req.solvedNodes.size();i++)
			{
				cout<<req.solvedNodes[i]<<", ";
				graphVector[gIndex]->solveByNameNode(req.solvedNodes[i]);
			}
			cout<<endl;

		}
		if(req.solvedHyperarc.size()>0)
		{
			cout<<"solved Hyper-arcs: ";
			for (int i=0; i<req.solvedHyperarc.size();i++)
			{
				cout<<req.solvedHyperarc[i]<<", ";
				graphVector[gIndex]->solveByNameHyperarc(req.solvedHyperarc[i]);
			}
			cout<<endl;

		}
		if (graphVector[gIndex]->isGraphSolved())
		{
			cout<<FGRN(BOLD("An And/Or graph is solved and deleted from vector of And/Or graphs; Name: "))<<graphVector[gIndex]->gName<<endl;
			vector<shared_ptr<AOgraph>>::iterator it =graphVector.begin()+gIndex;
			graphVector.erase(it);
			res.graphSolved=true;
		}
		else
		{

			res.graphSolved=false;
			vector<andor_msgs::Node> feasileNodeVector;
			vector<andor_msgs::Hyperarc> feasileHyperarcVector;
			graphVector[gIndex]->getFeasibleNode(feasileNodeVector);
			graphVector[gIndex]->getFeasibleHyperarc(feasileHyperarcVector);

			cout<<"feasile Hyper-arcs: ";
			for(int i=0;i<feasileHyperarcVector.size();i++)
				cout<<feasileHyperarcVector[i].hyperarcName<<", ";
			cout<<endl;

			cout<<"feasile Nodes: ";
			for(int i=0;i<feasileNodeVector.size();i++)
				cout<<feasileNodeVector[i].nodeName<<", ";
			cout<<endl;

			for(int i=0;i<feasileHyperarcVector.size();i++)
				res.feasibleHyperarcs.push_back(feasileHyperarcVector[i]);
			for(int i=0;i<feasileNodeVector.size();i++)
				res.feasibleNodes.push_back(feasileNodeVector[i]);
		}

	}
	return true;
}


int main(int argc, char **argv)
{

	string name = "TableAssembly";
	graphVector.emplace_back(make_shared <AOgraph>(name));
	const char* home=getenv("HOME");
	string andor_path(home);
//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssemblyFull3.txt";
	andor_path+="/catkin_ws/src/ANDOR/andor/files/iros2018/Normal_TableAssembly/TableAssembly.txt";
//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly.txt";
	graphVector.back()->loadFromFile(andor_path);

//	name = "Reach_Leg1_Plate_connected";
//	graphVector.emplace_back(make_shared <AOgraph>(name));
//	andor_path=home;
//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg1PlateConnected.txt";
//	graphVector.back()->loadFromFile(andor_path);

//	name = "Reach_Leg2_Plate_connected";
//	graphVector.emplace_back(make_shared <AOgraph>(name));
//	andor_path=home;
//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg2PlateConnected.txt";
//	graphVector.back()->loadFromFile(andor_path);
//
//	name = "Reach_Leg3_Plate_connected";
//	graphVector.emplace_back(make_shared <AOgraph>(name));
//	andor_path=home;
//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg3PlateConnected.txt";
//	graphVector.back()->loadFromFile(andor_path);
//
//	name = "Reach_Leg4_Plate_connected";
//	graphVector.emplace_back(make_shared <AOgraph>(name));
//	andor_path=home;
//	andor_path+="/catkin_ws/src/ANDOR/andor/files/TableAssembly/TableAssembly_Hierarchical/TableAssembly_Leg4PlateConnected.txt";
//	graphVector.back()->loadFromFile(andor_path);


	ros::init(argc, argv, "andor");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("andorService",updateANDOR);

	cout << FGRN(BOLD("*****************")) << endl;
	cout<<FGRN(BOLD("Graphs Name: "));
	for(int i=0;i<graphVector.size();i++)
		cout<<graphVector[i]->gName<<", ";
	cout<<endl;
	cout << FGRN(BOLD("AND/OR graph is alive: ")) << endl;

	ros::spin();
	return 1;

}   

