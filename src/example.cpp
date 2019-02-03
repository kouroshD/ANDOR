#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <andor/andorSRV.h>
#include <andor/Hyperarc.h>
#include <andor/Node.h>

using namespace std;

int main(int argc, char **argv)
{
	string andorGraphName;
	bool isGraphSolved;
	bool call_srv=true;
	bool ask_user=false;

	ros::init(argc, argv, "AndOr_example");
	ros::NodeHandle nh;
	ros::ServiceClient andorSRV_client = nh.serviceClient<andor::andorSRV>("andorService");
	ros::Rate loop_rate(40);

	cout<<"Insert AND/OR graph name: ";
	cin>>andorGraphName;
	cout<<endl;

	andor::andorSRV andor_srv;
	// DEL
	andor::andorSRV andor_srv0 ;
	andor_srv0.request.graphName=andorGraphName;

	if (andorSRV_client.call(andor_srv0))
	{
		isGraphSolved=andor_srv0.response.graphSolved;
		cout<<"+ is the graph solved:"<<isGraphSolved<<endl;

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
	//DEL

	while(ros::ok())
	{

		if(ask_user)
		{
			andor_srv.request.graphName=andorGraphName;

			string stateName, type, graphName;
			cout<<"Insert the following info for your intended solved node or hyperarc: [N(Node)/H(Hyper-arc)] [graph name] [state name]"<<endl;
			cin>>type>>graphName>>stateName;
			cout<<endl;
			call_srv=false;
			ask_user=false;
			if(type=="N" || type=="n")
			{
				call_srv=true;
				andor::Node solvedNode;
				solvedNode.nodeName=stateName;
				solvedNode.graphName=graphName;
				andor_srv.request.solvedNodes.push_back(solvedNode);
			}
			else if(type=="H" || type=="h")
			{
				call_srv=true;
				andor::Hyperarc solvedHA;
				solvedHA.hyperarcName=stateName;
				solvedHA.graphName=graphName;
				andor_srv.request.solvedHyperarc.push_back(solvedHA);
			}
			else
			{
				cout<<"Please respond correctly the request, try again! "<<type<<endl;
				ask_user=true;
			}
		}

		if(call_srv)
		{
			if (andorSRV_client.call(andor_srv))
			{
				isGraphSolved=andor_srv.response.graphSolved;
				cout<<"Is the graph solved: "<<isGraphSolved<<endl;

				cout<<"Feasible Nodes: [and/or graph name in hierarchy] [feasible node name] [cost]"<<endl;
				for (int i=0;i<andor_srv.response.feasibleNodes.size();i++)
				{
					string feasbible_state_name=andor_srv.response.feasibleNodes[i].nodeName;
					string andorNameHierarchy=andor_srv.response.feasibleNodes[i].graphName;
					int cost=andor_srv.response.feasibleNodes[i].nodeCost;

					cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
				}
				cout<<endl<<"Feasible Hyper-arcs:: [and/or graph name in hierarchy] [feasible hyperarc name] [cost]"<<endl;
				for (int i=0;i<andor_srv.response.feasibleHyperarcs.size();i++)
				{
					string feasbible_state_name=andor_srv.response.feasibleHyperarcs[i].hyperarcName;
					string andorNameHierarchy=andor_srv.response.feasibleHyperarcs[i].graphName;
					int cost=andor_srv.response.feasibleHyperarcs[i].hyperarcCost;

					cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
				}
				call_srv=false;
				ask_user=true;
				if(isGraphSolved)
					return 1;
			}
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 1;
}
