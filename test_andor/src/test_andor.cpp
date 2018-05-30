#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>

using namespace std;
int main(int argc, char **argv)
{
	string andorGraphName;
	bool isGraphSolved;

	ros::init(argc, argv, "test_andor");
	ros::NodeHandle nh;
	ros::ServiceClient andorSRV_client = nh.serviceClient<andor_msgs::andorSRV>("andorService");

	cout<<"- Insert AND/OR graph name: ";
	cin>>andorGraphName;

	andor_msgs::andorSRV andor_srv0 ;
	andor_srv0.request.graphName=andorGraphName;

	if (andorSRV_client.call(andor_srv0))
	{
		isGraphSolved=andor_srv0.response.graphSolved;
		cout<<endl<<"+ is the graph solved:"<<isGraphSolved<<endl;

		cout<<"Feasible Nodes:"<<endl;
		for (int i=0;i<andor_srv0.response.feasibleNodes.size();i++)
		{
			string feasbible_state_name=andor_srv0.response.feasibleNodes[i].nodeName;
			string andorNameHierarchy=andor_srv0.response.feasibleNodes[i].graphName;
			int cost=andor_srv0.response.feasibleNodes[i].nodeCost;

			cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
		}
		cout<<endl<<"Feasible Hyper-arcs:"<<endl;
		for (int i=0;i<andor_srv0.response.feasibleHyperarcs.size();i++)
		{
			string feasbible_state_name=andor_srv0.response.feasibleHyperarcs[i].hyperarcName;
			string andorNameHierarchy=andor_srv0.response.feasibleHyperarcs[i].graphName;
			int cost=andor_srv0.response.feasibleHyperarcs[i].hyperarcCost;

			cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
		}
	}

	cout<<"********************"<<endl;
	while(ros::ok())
	{
		andor_msgs::andorSRV andor_srv;
		andor_srv.request.graphName=andorGraphName;

		string stateName, type, graphName;
		cout<<"insert following info for solved state/state transition: [N(Node)/H(Hyper-arc)] [state name] [graph name]"<<endl;
		cin>>type>>stateName>>graphName;
		bool call_srv=false;
		if(type=="N" || type=="n")
		{
			call_srv=true;
			andor_msgs::Node solvedNode;
			solvedNode.nodeName=stateName;
			solvedNode.graphName=graphName;
			andor_srv.request.solvedNodes.push_back(solvedNode);
		}
		else if(type=="H" || type=="h")
		{
			call_srv=true;
			andor_msgs::Hyperarc solvedHA;
			solvedHA.hyperarcName=stateName;
			solvedHA.graphName=graphName;
			andor_srv.request.solvedHyperarc.push_back(solvedHA);
		}
		else
		{
			cout<<"wrtitten type is wrong, inser again! "<<type<<endl;
		}

		if(call_srv==true)
		{
			if (andorSRV_client.call(andor_srv))
			{
				isGraphSolved=andor_srv.response.graphSolved;
				cout<<endl<<"+ is the graph solved:"<<isGraphSolved<<endl;

				cout<<"Feasible Nodes:"<<endl;
				for (int i=0;i<andor_srv.response.feasibleNodes.size();i++)
				{
					string feasbible_state_name=andor_srv.response.feasibleNodes[i].nodeName;
					string andorNameHierarchy=andor_srv.response.feasibleNodes[i].graphName;
					int cost=andor_srv.response.feasibleNodes[i].nodeCost;

					cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
				}
				cout<<endl<<"Feasible Hyper-arcs:"<<endl;
				for (int i=0;i<andor_srv.response.feasibleHyperarcs.size();i++)
				{
					string feasbible_state_name=andor_srv.response.feasibleHyperarcs[i].hyperarcName;
					string andorNameHierarchy=andor_srv.response.feasibleHyperarcs[i].graphName;
					int cost=andor_srv.response.feasibleHyperarcs[i].hyperarcCost;

					cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
				}
			}
		}


		cout<<"********************"<<endl;
	}


	return 1;
}
