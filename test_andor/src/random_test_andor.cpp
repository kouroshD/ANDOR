#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include <time.h>

using namespace std;
class feasibleStates
{
public:
	string type;
	string stateName;
	string graphName;
};

int main(int argc, char **argv)
{
	string andorGraphName;
	bool isGraphSolved;
	vector<feasibleStates> FS_vector;
	srand (time(NULL));
	int randomVal, FSV_size;

	ros::init(argc, argv, "random_test_andor");
	ros::NodeHandle nh;
	ros::ServiceClient andorSRV_client = nh.serviceClient<andor_msgs::andorSRV>("andorService");

	cout<<"- Insert AND/OR graph name: ";
//	cin>>andorGraphName;
	andorGraphName="TableAssembly";

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
			feasibleStates temp_fs;
			temp_fs.graphName=andorNameHierarchy;
			temp_fs.stateName=feasbible_state_name;
			temp_fs.type="N";
			FS_vector.push_back(temp_fs);

		}
		cout<<endl<<"Feasible Hyper-arcs:"<<endl;
		for (int i=0;i<andor_srv0.response.feasibleHyperarcs.size();i++)
		{
			string feasbible_state_name=andor_srv0.response.feasibleHyperarcs[i].hyperarcName;
			string andorNameHierarchy=andor_srv0.response.feasibleHyperarcs[i].graphName;
			int cost=andor_srv0.response.feasibleHyperarcs[i].hyperarcCost;

			cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
			feasibleStates temp_fs;
			temp_fs.graphName=andorNameHierarchy;
			temp_fs.stateName=feasbible_state_name;
			temp_fs.type="H";
			FS_vector.push_back(temp_fs);
		}
	}

	cout<<"********************************************************************************"<<endl;
	while(ros::ok())
	{
		andor_msgs::andorSRV andor_srv;
		andor_srv.request.graphName=andorGraphName;

		string stateName, type, graphName;
		cout<<"insert following info for solved state/state transition: [N(Node)/H(Hyper-arc)] [state name] [graph name]"<<endl;
//		cin>>type>>stateName>>graphName;
		bool call_srv=false;
		FSV_size=FS_vector.size();
		randomVal=rand()%FSV_size;

		if(FS_vector[randomVal].type=="N" || FS_vector[randomVal].type=="n")
		{
			call_srv=true;
			andor_msgs::Node solvedNode;
			solvedNode.nodeName=FS_vector[randomVal].stateName;
			solvedNode.graphName=FS_vector[randomVal].graphName;
			andor_srv.request.solvedNodes.push_back(solvedNode);
		}
		else if(FS_vector[randomVal].type=="H" || FS_vector[randomVal].type=="h")
		{
			call_srv=true;
			andor_msgs::Hyperarc solvedHA;
			solvedHA.hyperarcName=FS_vector[randomVal].stateName;
			solvedHA.graphName=FS_vector[randomVal].graphName;
			andor_srv.request.solvedHyperarc.push_back(solvedHA);
		}
		else
		{
			cout<<"written type is wrong, insert again! "<<type<<endl;
		}
		cout<<FS_vector[randomVal].type<<": "<<FS_vector[randomVal].graphName<<" , "<<FS_vector[randomVal].stateName<<endl;

		if(call_srv==true)
		{
			if (andorSRV_client.call(andor_srv))
			{
				isGraphSolved=andor_srv.response.graphSolved;
				cout<<endl<<"+ is the graph solved:"<<isGraphSolved<<endl;
//				if(isGraphSolved)
//					exit(1);
//				else
					FS_vector.clear();
				cout<<"++++++++++++++++++"<<endl;
				cout<<"Feasible Nodes:"<<endl;
				for (int i=0;i<andor_srv.response.feasibleNodes.size();i++)
				{
					string feasbible_state_name=andor_srv.response.feasibleNodes[i].nodeName;
					string andorNameHierarchy=andor_srv.response.feasibleNodes[i].graphName;
					int cost=andor_srv.response.feasibleNodes[i].nodeCost;

					feasibleStates temp_fs;
					temp_fs.graphName=andorNameHierarchy;
					temp_fs.stateName=feasbible_state_name;
					temp_fs.type="N";
					FS_vector.push_back(temp_fs);

					cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
				}
				cout<<endl<<"Feasible Hyper-arcs:"<<endl;
				for (int i=0;i<andor_srv.response.feasibleHyperarcs.size();i++)
				{
					string feasbible_state_name=andor_srv.response.feasibleHyperarcs[i].hyperarcName;
					string andorNameHierarchy=andor_srv.response.feasibleHyperarcs[i].graphName;
					int cost=andor_srv.response.feasibleHyperarcs[i].hyperarcCost;

					feasibleStates temp_fs;
					temp_fs.graphName=andorNameHierarchy;
					temp_fs.stateName=feasbible_state_name;
					temp_fs.type="H";
					FS_vector.push_back(temp_fs);

					cout<<i+1<<") "<<andorNameHierarchy<<" , "<<feasbible_state_name<<" , "<<cost<<endl;
				}
			}
		}


		cout<<"********************************************************************************"<<endl;
	}


	return 1;
}
