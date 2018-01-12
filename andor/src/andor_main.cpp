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
vector<shared_ptr<AOgraph>> OnlinegraphVector , OfflineGraphVector;


bool updateANDOR(andor_msgs::andorSRV::Request &req, andor_msgs::andorSRV::Response &res){
	cout<<"------------- andor receive request -------------"<<endl;
	/*! when arrive a request:
	 * 1- check for the graph name, if it is not valid, return and error
	 * 2- if it is empty the solved nodes or hyper-arcs, respond by the feasible nodes and hyper-arcs
	 * 3- if a node or hyper-arc is solved, update the graph, and respond the feasible nodes/hyper-arcs
	 * 4- if the head node is solved, return it is solved
	 */

	// 1- check for the graph name
	int gIndex=-1; //! requested graph index number in graphVector, initialize wit rnd big value
	for (int i=0;i<OnlinegraphVector.size();i++)
		if(req.graphName==OnlinegraphVector[i]->gName)
			gIndex=i;
	// if the graph not found in online graphs, search for the graph in offline ones
	if(gIndex==-1)
	{
		for (int i=0;i<OfflineGraphVector.size();i++)
			if(req.graphName==OfflineGraphVector[i]->gName)
			{
				OnlinegraphVector.emplace_back(OfflineGraphVector[i]);

				gIndex=OnlinegraphVector.size()-1;
				cout<<FGRN(BOLD("A new And/Or graph added to online graph vector; Name: "))<<OnlinegraphVector[gIndex]->gName<<endl;
			}
	}

	if (gIndex==-1){
		cout<<FRED("Graph Name:"<<req.graphName<<" Is False, Not Found in Online or Offline Graph Vectors, Check Your Graph Name You Are Requesting")<<endl;
		return false;
	}
	// graph name is correct
	else
	{
		cout<<200<<endl;
		if(req.solvedNodes.size()>0)
		{
			cout<<201<<endl;
			for (int i=0; i<req.solvedNodes.size();i++)
			{
				cout<<202<<endl;
				OnlinegraphVector[gIndex]->solveByNameNode(req.solvedNodes[i]);
			}

		}
		if(req.solvedHyperarc.size()>0)
		{
			cout<<203<<endl;
			for (int i=0; i<req.solvedHyperarc.size();i++)
			{
				cout<<204<<endl;
				OnlinegraphVector[gIndex]->solveByNameHyperarc(req.solvedHyperarc[i]);
			}

		}
		if (OnlinegraphVector[gIndex]->isGraphSolved())
		{
			cout<<FGRN(BOLD("An And/Or graph is solved and deleted from online vector of And/Or graphs; Name: "))<<OnlinegraphVector[gIndex]->gName<<endl;
			vector<shared_ptr<AOgraph>>::iterator it =OnlinegraphVector.begin()+gIndex;
			OnlinegraphVector.erase(it);
			res.graphSolved=true;
		}
		else
		{
			cout<<206<<endl;
			res.graphSolved=false;
			vector<andor_msgs::Node> feasileNodeVector;
			vector<andor_msgs::Hyperarc> feasileHyperarcVector;
			OnlinegraphVector[gIndex]->getFeasibleNode(feasileNodeVector);
			OnlinegraphVector[gIndex]->getFeasibleHyperarc(feasileHyperarcVector);

			for(int i=0;i<feasileHyperarcVector.size();i++)
				cout<<feasileHyperarcVector[i].hyperarcName<<endl;

			for(int i=0;i<feasileNodeVector.size();i++)
				cout<<feasileNodeVector[i].nodeName<<endl;

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
	// create an empty graph
	string name = "DEFAULT";
	OfflineGraphVector.emplace_back(make_shared <AOgraph>(name));
	const char* home=getenv("HOME");
	string andor_path(home);
	andor_path+="/catkin_ws/src/ANDOR/andor/files/TRANSPORT.txt";
	OfflineGraphVector.back()->loadFromFile(andor_path);


	ros::init(argc, argv, "andor");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("andorService",updateANDOR);

	cout << "*****************" << endl;
	cout << "AND/OR graph is alive: " << endl;

	ros::spin();
	return 1;

}   

