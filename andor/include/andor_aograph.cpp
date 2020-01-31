//===============================================================================//
// Name			: aograph.cpp
// Author(s)	: Kourosh Darvish, Barbara Bruno, Yeshasvi Tirupachuri V.S.
// Affiliation	: University of Genova, Italy - dept. DIBRIS
// Description	: AND-OR graph
//===============================================================================//

#include "andor_aograph.h"

//! constructor of class Path
//! @param[in] cost 	initial cost of the path
//! @param[in] index 	unique index of the path
Path::Path(int cost, int index)
{
	pIndex = index;
	pCost = cost;
	pComplete = false;
}

//Path::Path(const Path& new_path){
//	pIndex=new_path.pIndex;
//	pComplete=new_path.pComplete;
//	pCost=new_path.pCost;
//	pathArcs=new_path.pathArcs;
//	checkedNodes=new_path.checkedNodes;
//	checkedHyperarcs=new_path.checkedHyperarcs;
////	vector<AOnode*> pathNodes
//	for(int i=0;i<new_path.pathNodes.size();i++)
//	{
//		AOnode* pathNode=new AOnode(*(new_path.pathNodes[i]));
//		pathNodes.push_back(pathNode);
//	}
//
//}
//Path& Path::operator=(const Path& new_path){
//	pIndex=new_path.pIndex;
//	pComplete=new_path.pComplete;
//	pCost=new_path.pCost;
//	pathArcs=new_path.pathArcs;
//	checkedNodes=new_path.checkedNodes;
//	checkedHyperarcs=new_path.checkedHyperarcs;
//	for(int i=0;i<new_path.pathNodes.size();i++)
//	{
//		AOnode* pathNode=new AOnode(*(new_path.pathNodes[i]));
//		pathNodes.push_back(pathNode);
//	}
//
//	return *this;
//}


//! copy constructor of class Path
//! @param[in] &toBeCopied  path to be copied
//! @param[in] index        unique index of the path
Path::Path(const Path &toBeCopied, int index)
{
	pIndex = index;
	pCost = toBeCopied.pCost;
	pComplete = false;
	pathNodes = toBeCopied.pathNodes;
	pathArcs = toBeCopied.pathArcs;
	checkedNodes = toBeCopied.checkedNodes;
	checkedHyperarcs=toBeCopied.checkedHyperarcs;
}

//! display path information
void Path::printPathInfo()
{
	cout<<"*************************"<<endl;
	cout<<"Info of path: " <<pIndex <<endl;
	//DEBUG: cout<<"Is complete? " <<boolalpha <<pComplete <<endl;
	cout<<"Total cost: " <<pCost <<endl;
	cout<<"Hyperarcs in path: "<<endl;
	for (int i=0; i< (int)pathArcs.size(); i++)
	{
		cout<<pathArcs[i] <<" ";
		if (checkedHyperarcs[i] == true)
			cout<<"- done";
		cout<<", ";
	}
	cout<<endl <<"Nodes in path:" <<endl;
	for (int i=0; i< (int)pathNodes.size(); i++)
	{
		cout<<pathNodes[i]->nName <<" ";
		//DEBUG: cout<<"checked? " <<boolalpha <<checkedNodes[i];
		if (checkedNodes[i] == true)
			cout<<"- done";
		cout<<", ";
	}
	cout<<endl;
}

//! add a node in the path
//! @param[in] node     node to add to the path
void Path::addNode(AOnode* node)
{
	// update the nodes in the path
	pathNodes.push_back(node);
	checkedNodes.push_back(false);

	// N.B. the cost of the path is updated when the node is checked
}

//! update the path information (when a node is solved)
//! @param[in] nameNode     name of the node
//! @param[in] cost         cost to subtract from the path cost
void Path::updatePath(string nameNode, int cost)
{
	// check whether the node is in the path
	for(int i=0; i < (int)pathNodes.size(); i++)
	{
		// keep track of solved nodes
		if (pathNodes[i]->nName == nameNode)
			checkedNodes[i] = true;
	}
	// update the cost of the path
	pCost = pCost - cost;

	cout<<"Path: " <<pIndex <<endl;
	cout<<"Updated path cost: " <<pCost <<endl;
}

//! update the path information (when a node is solved)
//! @param[in] nameNode     name of the node
//! @param[in] cost         cost to subtract from the path cost
void Path::updatePathNode(string nameNode, int cost)
{
	// check whether the node is in the path
	for(int i=0; i < (int)pathNodes.size(); i++)
	{
		// keep track of solved nodes
		if (pathNodes[i]->nName == nameNode)
			checkedNodes[i] = true;
	}
	// update the cost of the path
	pCost = pCost - cost;

	cout<<"Path: " <<pIndex <<endl;
	cout<<"Updated path cost: " <<pCost <<endl;
}

//! update the path information (when a node is solved)
//! @param[in] nameNode     name of the node
//! @param[in] cost         cost to subtract from the path cost

void Path::updatePathHyperarc(string nameHyperarc, int cost)
{
	//   // check whether the hyperarc is in the path
	//    for(int i=0; i < (int)pathNodes.size(); i++)
	//    {
	//        // keep track of solved nodes
	//    	for (int j=0;j<pathNodes[i]->arcs.size();j++)
	//    	{
	//    		for (int k=0;k<path)
	//    		if(pathNodes[i]->arcs[j].hIndex==)
	//            if (pathNodes[i]->arcs[j].hName == nameHyperarc)
	//                checkedHyperarcs[i] = true;
	//    	}
	//
	//    }
	//    // update the cost of the path
	//    pCost = pCost - cost;
	//
	//    cout<<"Path: " <<pIndex <<endl;
	//    cout<<"Updated path cost: " <<pCost <<endl;
}

//! find the feasible node to suggest
//! @return node to suggest
string Path::suggestNode()
{

	//	printPathInfo();

	string selection = "NULL";

	// iterate on the nodes in the path, from the last to the first one
	for (int i = (int)pathNodes.size()-1; i > -1; i--)
	{
		bool harcIsInPath=false;
		// rationale for the suggestion:
		// 1. move along the path from the leaves to the head
		// 2. choose the first feasible & not-solved node
		if (checkedNodes[i] == false)
		{
			if (pathNodes[i]->nFeasible == true)
			{
				selection = pathNodes[i]->nName;
				//				suggestion_node=true;
				break;
			}
			else
			{
				for (int j=0; j< (int)pathNodes[i]->arcs.size();j++)
				{
					int arcNo=0;

					for (int k=0;k<(int)pathArcs.size();k++)
						if(pathArcs[k]==pathNodes[i]->arcs[j].hIndex)
						{
							harcIsInPath=true;
							arcNo=k;
						}
					//					it=find(pathArcs.begin(), pathArcs.end(), pathNodes[i]->arcs[j].hIndex);
					//! if the "*it" hyperarc of the node (i) is in the path
					if ( harcIsInPath==true )
					{
						//! if the hyper-arc is not solved
						if(checkedHyperarcs[arcNo]==false)
						{
							//! if the hyper-arc is feasible
							if(pathNodes[i]->arcs[j].hFeasible==true)
							{
								selection=pathNodes[i]->arcs[j].hName;
								break;
							}
						}
					}

				}
			}
		}
	}

	// raise an error if the suggested node is NULL
	if (selection == "NULL")
		cout<<"[ERROR] No suggestion possible." <<endl;
	return selection;
}

//! add a node in the graph
//! @param[in] nameNode    name of the node
//! @param[in] cost        generic node cost
void AOgraph::addNode(string nameNode, int cost)
{
	// create the node
	AOnode toAdd(nameNode, cost);

	// add it to the set of nodes in the graph
	graph.push_back(toAdd);
}
void AOgraph::addHyperarc(HyperArc* hyperarc)
{
	// create the node
	//    AOnode toAdd(nameNode, cost);

	// add it to the set of nodes in the graph
	graphHA.push_back(hyperarc);
}

//! find a node by name
//! @param[in] nameNode    name of the node
//! @return	               pointer to the node with given name
AOnode* AOgraph::findByName(string nameNode)
{
	AOnode* temp = NULL;

	for (int i=0; i< (int)graph.size(); i++)
	{
		if (graph[i].nName == nameNode)
		{
			temp = &graph[i];
			break;
		}
	}

	// issue a warning if the node has not been found
	if (temp == NULL)
		cout<<"[Warning] Name not found."
		<<"Did you really look for " <<nameNode <<"?" <<endl;
	return temp;
}
HyperArc* AOgraph::findByNameHyperarc(string nameHyperarc)
{
	HyperArc* temp = NULL;
	for (int i=0; i< (int)graph.size(); i++)
	{
		for (int j=0;j<(int)graph[i].arcs.size();j++)
		{
			if (graph[i].arcs[j].hName == nameHyperarc)
			{
				temp = &graph[i].arcs[j];
				break;
			}

		}
	}

	// issue a warning if the hyper-arc has not been found
	if (temp == NULL)
		cout<<"[Warning] Hyper-arc Name not found."
		<<"Did you really look for " <<nameHyperarc<<"?" <<endl;
	return temp;
}


//! update the feasibility status of the nodes in the graph
void AOgraph::updateFeasibility()
{
	for (int i=0; i< (int)graph.size(); i++)
		graph[i].isFeasible(Nodes_solved_infeasible);

	for (int i=0; i< (int)graph.size(); i++)
		for (int j=0; j<(int)graph[i].arcs.size();j++ )
			graph[i].arcs[j].isFeasible();
}


void AOgraph::updateHyperarcFeasibility()
{
	for (int i=0; i< (int)graph.size(); i++)
		graph[i].isFeasible(Nodes_solved_infeasible);
}
//! compute the cost to add to a path
//! @param[in] node     reference to the node to use for cost computation
//! @param[in] hIndex   index of the node's hyperarc to use for cost computation
//! @return             cost to add to a path
int AOgraph::computeAddCost(AOnode &node, int hIndex)
{
	// 1. the cost is to be ADDED to the cost of the path
	// 2. cost = node.nCost + node.arcs[hIndex].hCost
	int cost = 0;

	// raise an error if the hyperarc index is out of bounds
	if (hIndex >= (int)node.arcs.size())
	{
		cout<<"[ERROR] The node has only " <<node.arcs.size() <<" hyperarcs."
				<<"Hyperarc index " <<hIndex <<" does not exist." <<endl;

		node.printNodeInfo();

		return -1;
	}

	// if hIndex == -1, the node is terminal
	if (hIndex == -1)
		cost = node.nCost;
	// otherwise, the cost to add is given by node.cost and hyperarc.cost
	else
		cost = node.nCost + node.arcs[hIndex].hCost;
	//DEBUG:cout<<"Node: " <<node.nName <<" - Cost: " <<cost <<endl;

	return cost;
}

//! generate all possible paths navigating the graph
void AOgraph::generatePaths()
{
	// if the head node is NULL, there are no paths to generate
	if (head == NULL)
	{
		cout<<"[AOgraph::generatePaths] graph name: "<<gName<<" :: [WARNING] There is no graph to navigate (head == NULL)."<<endl;
		return;
	}

	// otherwise, create a path with the head node
	Path* newPath = new Path(0,0);
	newPath->addNode(head);
	paths.push_back(*newPath);

	// iterate through the paths until they're all complete
	AOnode* currentNode = head;
	while(1)
	{
		// find the first not-complete path
		bool allComplete = true;
		int currentPathIndex;
		for (int i=0; i< (int)paths.size(); i++)
		{
			if (paths[i].pComplete == false)
			{
				allComplete = false;
				currentPathIndex = i;
				break;
			}
		}
		// if all paths are complete, the generation is done
		if (allComplete == true)
			return;

		// find the first not-checked node in the open path
		bool allChecked = true;
		int currentNodeIndex;
		for (int i=0; i< (int)paths[currentPathIndex].checkedNodes.size(); i++)
		{
			if (paths[currentPathIndex].checkedNodes[i] == false)
			{
				allChecked = false;
				currentNode = paths[currentPathIndex].pathNodes[i];
				currentNodeIndex = i;
				break;
			}
		}
		// if all nodes are checked, the path is complete
		if (allChecked == true)
			paths[currentPathIndex].pComplete = true;
		else
		{
			// if the current node is terminal:
			// 1. check it
			// 2. update the path cost with the current node cost
			if (currentNode->arcs.size() == 0)
			{
				paths[currentPathIndex].checkedNodes[currentNodeIndex] = true;
				int cost = computeAddCost(*currentNode, -1);
				paths[currentPathIndex].pCost = paths[currentPathIndex].pCost + cost;
			}

			// if the current node has only one hyperarc:
			// 1. check it
			// 2. add the hyperarc index to the path
			// 3. update the path cost with the current node+only_hyperarc cost
			// 4. add its child nodes to the path
			if (currentNode->arcs.size() == 1)
			{
				paths[currentPathIndex].checkedNodes[currentNodeIndex] = true;
				paths[currentPathIndex].pathArcs.push_back(currentNode->arcs[0].hIndex);
				paths[currentPathIndex].checkedHyperarcs.push_back(true);

				int cost = computeAddCost(*currentNode, 0);
				paths[currentPathIndex].pCost = paths[currentPathIndex].pCost + cost;
				for (int i=0; i< (int)currentNode->arcs[0].children.size(); i++)
					paths[currentPathIndex].addNode(currentNode->arcs[0].children[i]);
			}

			// if the current node has more than one hyperarc:
			// 1. create (numArcs-1) copies of the current path
			// 2. check the current node in the copies
			// 3. add the other_hyperarc index to the copies
			// 4. update the path cost with the current node+other_hyperarc cost
			// 5. add the child nodes of the last (numArcs-1) arcs to the copies
			// 6. check the current node in the current path
			// 7. add the first_hyperarc index to the current path
			// 8. update the path cost with the current node+first_hyperarc cost
			// 9. add the child nodes of the first arcs to the current path
			if (currentNode->arcs.size() > 1)
			{
				int numCopies = currentNode->arcs.size()-1;
				for (int i=0; i<numCopies; i++)
				{
					newPath = new Path(paths[currentPathIndex], paths.size());
					newPath->checkedNodes[currentNodeIndex] = true;
					newPath->pathArcs.push_back(currentNode->arcs[i+1].hIndex);
					newPath->checkedHyperarcs.push_back(true);
					int cost = computeAddCost(*currentNode, i+1);
					newPath->pCost = newPath->pCost + cost;
					for (int j=0; j< (int)currentNode->arcs[i+1].children.size(); j++)
						newPath->addNode(currentNode->arcs[i+1].children[j]);
					paths.push_back(*newPath);
				}
				paths[currentPathIndex].checkedNodes[currentNodeIndex] = true;
				paths[currentPathIndex].pathArcs.push_back(currentNode->arcs[0].hIndex);
				paths[currentPathIndex].checkedHyperarcs.push_back(true);
				int cost = computeAddCost(*currentNode, 0);
				paths[currentPathIndex].pCost = paths[currentPathIndex].pCost + cost;
				for (int i=0; i< (int)currentNode->arcs[0].children.size(); i++)
					paths[currentPathIndex].addNode(currentNode->arcs[0].children[i]);
			}
		}
	}
}

//! set up a graph
void AOgraph::setupGraph()
{
	cout<<gName<<"[AOgraph::setupGraph]"<<endl;
//	cout<<1<<endl;
	// update the feasibility status of the nodes in the graph
	updateFeasibility();
	//DEBUG:printGraphInfo();
	// generate all paths navigating the graph
//	cout<<2<<endl;
	generatePaths();
//	cout<<3<<endl;
	// set the "checked" property of the nodes in the paths to false
	// NOTE: during execution, "checked" is used to mark the solved nodes
	for (int i=0; i < (int)paths.size(); i++)
		for (int j=0; j < (int)paths[i].checkedNodes.size(); j++)
			paths[i].checkedNodes[j] = false;

	for (int i=0; i < (int)paths.size(); i++)
		for (int j=0; j < (int)paths[i].checkedHyperarcs.size(); j++)
			paths[i].checkedHyperarcs[j] = false;

//	cout<<"***************************************************************"<<endl;
//	cout<<"************************** PATH INFO **************************"<<endl;
//	for (int i=0; i < (int)paths.size(); i++)
//		paths[i].printPathInfo();

//	cout<<"***************************************************************"<<endl;
	// identify the first suggestion to make (long-sighted strategy chosen BY DEFAULT)
	suggestNext(true);
}

//! find the hyperarc connecting a parent to a child node
//! @param[in] parent   reference to the parent node
//! @param[in] child    reference to the child node
//! @return             index of the hyperarc connecting the parent to the child
HyperArc* AOgraph::findHyperarc(AOnode &parent, AOnode &child, int pathIndex)
{
	HyperArc* temp = NULL;
	for (int j=0; j< (int)parent.arcs.size(); j++)
	{
		// if arc is in the given path
		for (int i=0;i<(int) paths[pathIndex].pathArcs.size();i++)
		{
			if(paths[pathIndex].pathArcs[i]==(int)parent.arcs[j].hIndex) //
			{

				for (int k=0; k< (int)parent.arcs[j].children.size(); k++)
				{
					/* there should be a check for path No, because maybe between
					 * a parent and child node there are more hArcs (related to different paths) and
					 * therefore we should not overwrite*/
					if(parent.arcs[j].children[k]->nName == child.nName)
					{
						temp = &parent.arcs[j];
						//DEBUG:cout<<"Found index: " <<temp->hIndex <<endl;
						break;
					}
				}
			}
		}
	}

	/* DEBUG
    // raise a warning if no hyperarc was found
    if (temp == NULL)
        cout<<"[WARNING] There is no hyperarc connecting " <<parent.nName
            <<" to " <<child.nName <<"." <<endl;
	 */

	return temp;
}

//! compute the overall update cost (intermediate step to update the path cost)
//! @param[in] node     reference to the node to use for cost computation
//! @return             overall update cost (to subtract from the path cost)
int AOgraph::computeOverallUpdate(AOnode &node)
{    
	// 1. the cost is to be SUBTRACTED from the cost of the path
	// 2. the cost is computed as:
	// a. pathsCosts = set of the costs of the hyperarcs TO the current node
	// b. cost = max(pathsCosts)

	// N.B. the cost is to be subtracted from path[i] as:
	// 1. toSubtract = node.nCost + abs(pathsCosts[path_i] - cost);

	vector<int> pathsCosts;

	// for each parent node, find the cost of the hyperarc to the current node
	for (int i=0; i< (int)node.parents.size(); i++)
	{
		// from a node to same parent maybe there are several hyper-arcs, each of them related to a unique path.
		for (int j=0;j<(int)paths.size();j++)
		{
			HyperArc* arc = findHyperarc(*node.parents[i], node, paths[j].pIndex);
			if (arc != NULL)
			{
				int arcCost = arc->hCost;
				pathsCosts.push_back(arcCost);
			}
		}
	}
	/* DEBUG
    cout<<"maxUpdate is the max of: ";
    for (int i=0; i< (int)pathsCosts.size(); i++)
        cout<<pathsCosts[i] <<" ";
    cout<<endl;
	 */

	// find the maximum in pathsCosts
	int cost = pathsCosts[0];
	for (int i=1; i< (int)pathsCosts.size(); i++)
		if (pathsCosts[i] > cost)
			cost = pathsCosts[i];
	//DEBUG:cout<<"maxUpdate = " <<cost <<endl;

	return cost;
}

//! update all paths (update path costs when a node is solved)
//! @param[in] solved       reference to the solved node (to use for paths costs update)
void AOgraph::updatePaths(AOnode &solved)
{
	// update the path information (cost) of EACH path as:
	// toSubtract = solved.nCost + overall_update - path_i_update;
	// path[i].cost = path[i].cost - toSubtract;

	pIndices.clear();
	pUpdate.clear();
	int toSubtract = solved.nCost + computeOverallUpdate(solved);
	//DEBUG:cout<<"solved.nCost = " <<solved.nCost <<endl;
	//DEBUG:cout<<"maxUpdate = " <<computeOverallUpdate(solved) <<endl;
	//    cout<<"--------------- solved.nCost = " <<solved.nCost <<endl;
	//    cout<<"---------------maxUpdate = " <<computeOverallUpdate(solved) <<endl;


	// find all paths which include the solved node
	vector<int> withChild;
	for (int i=0; i < (int)paths.size(); i++)
		for (int j=0; j< (int)paths[i].pathNodes.size(); j++)
			if(paths[i].pathNodes[j]->nName == solved.nName)
				withChild.push_back(i);
	/* DEBUG
    cout<<"---------- Paths with solved node: ";
    for (int i=0; i < (int)withChild.size(); i++)
        cout<<"Path index: " <<withChild[i] <<" ";
    cout<<endl;
	 */

	// find the paths including the solved node AND any of its parents
	vector<int> withBoth;
	int toAdd;
	for (int i=0; i < (int)withChild.size(); i++)
	{
		for (int j=0; j < (int)solved.parents.size(); j++)
			for (int k=0; k < (int)paths[withChild[i]].pathNodes.size(); k++)
				if(paths[withChild[i]].pathNodes[k]->nName == solved.parents[j]->nName)
					toAdd = withChild[i];
		// make sure that paths are added only ONCE
		if ( std::find(withBoth.begin(), withBoth.end(), toAdd) == withBoth.end() )
			withBoth.push_back(toAdd);
	}
	/* DEBUG
    cout<<"------------------- Paths with solved node & any parent: ";
    for (int i=0; i < (int)withBoth.size(); i++)
        cout<<"Path index: " <<withBoth[i] <<" ";
    cout<<endl;
	 */

	// find the paths containing a DIRECT LINK between the solved node and a parent
	for (int i=0; i < (int)withBoth.size(); i++)
	{
		for (int j=0; j < (int)paths[withBoth[i]].pathNodes.size(); j++)
		{
			//        	cout<<"**********************"<<endl;
			HyperArc* arc = findHyperarc(*paths[withBoth[i]].pathNodes[j], solved,paths[withBoth[i]].pIndex);
			//            cout<<"**********************"<<endl;
			// update the path cost (if there is a direct link in THIS path)
			if (arc != NULL)
			{
				//            	cout<<"------------- Found hyperArc index: " <<arc->hIndex <<endl;
				//            	cout<<"------------- Path No: " <<(int)paths[withBoth[i]].pIndex<<endl;

				for (int k=0; k < (int)paths[withBoth[i]].pathArcs.size(); k++)
				{
					if (paths[withBoth[i]].pathArcs[k] == arc->hIndex)
					{
						// compute "path_i_update"
						int pathUpdate = arc->hCost;
						//DEBUG:cout<<"pathUpdate = " <<pathUpdate <<endl;
						//                    	cout<<"------------- Path No updated: " <<(int)paths[withBoth[i]].pIndex<<endl;
						//                        cout<<"------------ pathUpdate = " <<pathUpdate <<endl;
						int thisSubtract = toSubtract - pathUpdate;

						// update the cost of the path
						paths[withBoth[i]].updatePath(solved.nName, thisSubtract);

						// save the index & subtracted cost of the updated path
						pIndices.push_back(withBoth[i]);
						pUpdate.push_back(pathUpdate);

						break;
					}
				}
			}
		}
	}

}

void AOgraph::updatePaths_NodeSolved(AOnode &solved)
{
	pIndices.clear();
	pUpdate.clear();
	int toSubtract = solved.nCost;

	for (int i=0; i < (int)paths.size(); i++)
		for (int j=0; j< (int)paths[i].pathNodes.size(); j++)
			if(paths[i].pathNodes[j]->nName == solved.nName)
			{
				paths[i].pCost=paths[i].pCost-toSubtract;
				paths[i].checkedNodes[j]=true;
				pIndices.push_back(i);
				pUpdate.push_back(toSubtract);
				break;
			}

}

void AOgraph::updatePaths_HyperarcSolved(HyperArc &solved)
{
	pIndices.clear();
	pUpdate.clear();
	int toSubtract = solved.hCost;
	int hIndexSolved=solved.hIndex; //! the index of the hyper-arc solved

	// search for if the solved hyper-arc index is inside the path hyper-arcs indices
	for (int i=0; i < (int)paths.size(); i++)
		for (int j=0; j< (int)paths[i].pathArcs.size(); j++)
			if(paths[i].pathArcs[j] == hIndexSolved)
			{
				paths[i].pCost=paths[i].pCost-toSubtract;
				paths[i].checkedHyperarcs[j]=true;
				pIndices.push_back(i);
				pUpdate.push_back(toSubtract);
				break;
			}

}

//! find the optimal path (long-sighted strategy)
//! @return index of the optimal path (minimum cost)
int AOgraph::findOptimalPath()
{
	// raise an error if there are no paths
	if (paths.size() == 0)
	{
		cout<<"[ERROR] There are no paths navigating the graph. "
				<<"Did you run generatePaths()?" <<endl;
		return -1;
	}

	int index = 0;
	int cost = paths[0].pCost;
	for (int i=0; i< (int)paths.size(); i++)
	{
		// raise an error if there are not-complete paths
		if (paths[i].pComplete == false)
		{
			cout<<"[ERROR] The paths navigating the graph are not complete. "
					<<"Did you run generatePaths()?" <<endl;
			return -1;
		}

		if (paths[i].pCost < cost)
		{
			cost = paths[i].pCost;
			index = i;
		}
	}
//	cout<<"The optimal path is: " <<index <<endl;
//	paths[index].printPathInfo();

	return index;
}

int AOgraph::findNextOptimalPath(int previousOptimalPathIndex){

	// raise an error if there are no paths
	if (paths.size() == 0)
	{
		cout<<"[ERROR] There are no paths navigating the graph. "
				<<"Did you run generatePaths()?" <<endl;
		return -1;
	}

	int index = 0;
	int cost = 999999; //! a big number, so that the path values are less than this number.
	int minCost=paths[previousOptimalPathIndex].pCost;
	for (int i=0; i< (int)paths.size(); i++)
	{
		if (i!=previousOptimalPathIndex){
			// raise an error if there are not-complete paths
			if (paths[i].pComplete == false)
			{
				cout<<"[ERROR] The paths navigating the graph are not complete. "
						<<"Did you run generatePaths()?" <<endl;
				return -1;
			}

			if (paths[i].pCost < cost && paths[i].pCost>=minCost)
			{
				cost = paths[i].pCost;
				index = i;
			}
		}
	}
	cout<<"The optimal path is: " <<index <<endl;
	paths[index].printPathInfo();

	return index;
}

//! constructor of class AOgraph
//! @param[in] name 	name of the graph
AOgraph::AOgraph(string name)
{
	gName = name;
	head = NULL;
	upperLevelHyperarc=NULL;

	//DEBUG:printGraphInfo();
}
//AOgraph::AOgraph(const AOgraph& new_graph){
//	gName=new_graph.gName;
//	paths=new_graph.paths;
//	pIndices=new_graph.pIndices;
//	pUpdate=new_graph.pUpdate;
//	Nodes_solved_infeasible=new_graph.Nodes_solved_infeasible;
//
////	graph=new_graph.graph;
//	for(int i=0;i<new_graph.graph.size();i++)
//	{
//		AOnode nodeToAdd(new_graph.graph[i]);
//		graph.push_back(nodeToAdd);
//
//	}
//
//
//	head=new AOnode(*(new_graph.head));
//
//	for(int i=0;i<new_graph.graphHA.size();i++)
//	{
//		HyperArc* HA=new HyperArc(*(new_graph.graphHA[i]));
//		graphHA.push_back(HA);
//	}
//}

//AOgraph& AOgraph::operator=(const AOgraph& new_graph){
//	gName=new_graph.gName;
//	graph=new_graph.graph;
//	head=new AOnode(*(new_graph.head));
//	paths=new_graph.paths;
//	pIndices=new_graph.pIndices;
//	pUpdate=new_graph.pUpdate;
//	Nodes_solved_infeasible=new_graph.Nodes_solved_infeasible;
//
//	for(int i=0;i<new_graph.graphHA.size();i++)
//	{
//		HyperArc* HA=new HyperArc(*(new_graph.graphHA[i]));
//		graphHA.push_back(HA);
//	}
//
//	return *this;
//}

//! load the graph description from a file
//! @param[in] fileName    name of the file with the graph description
void AOgraph::loadFromFile(string filePath, string fileName)
{
	cout<<"[AOgraph::loadFromFile] "<<filePath<<" , "<<fileName<<endl;
	// raise an error if the graph is not empty
	if (graph.size() != 0)
	{
		cout<<"[ERROR] The graph is not empty."
				<<"Do you really want to overwrite the current graph?" <<endl;
		return;
	}

	fileName=filePath+fileName+".txt";
	ifstream graphFile(fileName.c_str());
	cout <<"Loading graph description from file: " <<fileName <<endl;
//	cout<<1<<endl;
	while (!graphFile.eof())
	{
		// the first line contains:
		// 1. the name of the graph
		// 2. the number N=numNodes of nodes
		// 3. the name of the head node (corresponding to the final assembly)
		string name;
		int numNodes;
		string headName;

		graphFile >>name >>numNodes >>headName;
		if (!graphFile)
			break;
		gName += name;

//		cout<<gName <<endl;
		// the next N lines contain the name and cost of all the nodes in the graph
		string nameNode;
		int cost;
		for (int i=0; i < numNodes; i++)
		{
			graphFile >>nameNode >> cost;
			if (!graphFile)
				break;
			addNode(nameNode, cost);
		}


		// identify the head node in the graph
		cout<<"head name: "<<headName<<endl;
		head = findByName(headName);
		cout<<"head: "<<(head==NULL) <<endl;


		// the next ?? lines contain the descriptions of the hyperarcs in the graph
		int hyperarcIndex = 0;
		while (!graphFile.eof())
		{
			AOnode* father;
			string nameFather;
			int numChildren;
			int hyperarcCost;
			string hyperarcName;
			string lowerGraphName;// lower level graph of the hyper-arc

			vector<AOnode*> childNodes;

//			std::stringstream buffer;
//			buffer << graphFile;
//			cout<<"graphfile: "<<buffer.str()<<endl;
			graphFile >>hyperarcName>>numChildren>>nameFather>>hyperarcCost>>lowerGraphName;
			if (!graphFile)
				break;
			father = findByName(nameFather);
//			DEBUG:cout<<"nameFather = " <<nameFather <<endl;

			// the next numChildren lines contain the names of the child nodes
			for (int i=0; i < numChildren; i++)
			{
				AOnode* temp;
				string nameChild;
				graphFile >>nameChild;
				if (!graphFile)
					break;
				temp = findByName(nameChild);
				childNodes.push_back(temp);
			}
//			cout<<"loadFromFile:: "<<hyperarcName<<", "<< hyperarcIndex<<", "<<childNodes.size()<<", " <<hyperarcCost<<", "<< nameFather<<", "<<lowerGraphName<<endl;

//			for(int k=0;k<childNodes.size();k++)
//				childNodes[k]->printNodeInfo();



			HyperArc& newHA=father->addArc(hyperarcName, hyperarcIndex, childNodes, hyperarcCost, nameFather);
			if(lowerGraphName!="-")
			{
//				cout<<FGRN(BOLD("***** Hierarchical Graph: started ************* "))<<endl;
//				cout<<"ha cost: "<<newHA.hCost<<endl;

				AOgraph* newGraph=new AOgraph(lowerGraphName);
				newGraph->gName=this->gName+":"+hyperarcName+":";
				newGraph->upperLevelHyperarc=&newHA;

				newGraph->loadFromFile(filePath,lowerGraphName);

				int newHACost=INT_MAX; //! if a hyper-arc owns a lower level graph(g), the hyper-arc cost equals to min path cost of the g.
				for(int i=0;i<newGraph->paths.size();i++ )
					if(newGraph->paths[i].pCost<newHACost)
						newHACost=newGraph->paths[i].pCost;
				newHA.hCost=newHACost;

				newHA.SetGraphs(newGraph, this);

//				cout<<"ha cost: "<<newHA.hCost<<endl;
//				cout<<FGRN(BOLD("********** Hierarchical Graph: Ended *********** "))<<endl;
			}


			hyperarcIndex = hyperarcIndex+1;
		}


	}
	graphFile.close();

//	for(int i=0;i<graph.size();i++)
//		graph[i].printNodeInfo();

	// set up the graph (nodes feasibility, paths costs)
	setupGraph();
	for(int i=0;i<(int)graph.size();i++)
	{
		for (int j=0;j<(int)graph[i].arcs.size();j++)
		{
			HyperArc* tempHA;
			tempHA=&(graph[i].arcs[j]);
			graphHA.push_back(tempHA);
		}
	}
//	printGraphInfo();

}


//! display graph information
void AOgraph::printGraphInfo()
{
	cout<<endl;
	cout<<"Info of graph: " <<gName <<endl;
	cout<<"Number of nodes: " <<graph.size() <<endl;
	cout<<"Head node: " <<head->nName <<endl <<endl;
	for (int i=0; i< (int)graph.size(); i++)
		graph[i].printNodeInfo();
	cout<<endl;

	for(int i=0;i<paths.size();i++){
		paths[i].printPathInfo();
	}
}

//! suggest the node to solve
//! @param[in] strategy     "0" = short-sighted, "1" = long-sighted
//! @return                 name of the suggested node
string AOgraph::suggestNext(bool strategy)
{
	// issue a warning if the graph has been solved already
	if (head->nSolved == true)
	{
		cout<<"[WARNING] The graph is solved. No suggestion possible." <<endl;
		return "end";
	}
	int optimalPathIndex = 0;
	int previousOptimalPathIndex=0;
	// short-sighted strategy:
	// pick the path which received the highest benefit from the last action
	if (strategy == false)
	{
		// find the path with highest benefit from last action
		for (int i=1; i< (int)pUpdate.size(); i++)
			if (pUpdate[i] > pUpdate[optimalPathIndex])
				optimalPathIndex = pIndices[i];
	}
	// long-sighted strategy:
	// pick the path which minimizes the cost to completion
	string suggestion ="NULL";
	if (strategy == true){
		for(int i=0; i<(int)paths.size();i++)
		{
			if (i==0)
			{
				optimalPathIndex = findOptimalPath();
			} else
			{
				optimalPathIndex=findNextOptimalPath(previousOptimalPathIndex);
			}
			suggestion = paths[optimalPathIndex].suggestNode();
			if(suggestion !="NULL")
				break;
			else
				previousOptimalPathIndex=optimalPathIndex;
		}
	}
	if (suggestion =="NULL")
		cout<<"Error: There Is Not an Active Path"<<endl;

//	cout<<"ENDOR suggestion: " <<endl
//			<<"Suggested path = " <<optimalPathIndex <<endl
//			<<"Suggested = " <<suggestion<<endl;

	return suggestion;
}

//! solve a node, finding it by name
//! @param[in] nameNode    name of the node
void AOgraph::solveByNameNode(string graphName,string nameNode)
{

	AOgraph * newGraph=findGraph(graphName);
	AOnode* solved = newGraph->findByName(nameNode);
	bool result = solved->setSolved();
	newGraph->updateFeasibility();
//	printGraphInfo();

	// report that the graph has been solved if the solved node is the head node
	if (newGraph->head->nSolved == true)
	{
		if(newGraph==this)
		{
		cout<<"[REPORT] The graph is solved (head node solved)." <<endl;
		}
		else
		{
			newGraph->upperLevelHyperarc->includingGraph->solveByNameHyperarc(newGraph->upperLevelHyperarc->includingGraph->gName,newGraph->upperLevelHyperarc->hName);
		}
		return;
	}

	// update the path information (cost) of all paths
	if (result == true)
		newGraph->updatePaths_NodeSolved(*solved);
//	cout<<endl <<"Updated paths: " <<endl;
//	for(int i=0; i< (int)pUpdate.size(); i++)
//		cout<<"Path index: " <<pIndices[i] <<" - Benefit: " <<pUpdate[i] <<endl;

	//    cout<<"*************"<<endl;
	//    for (int i=0;i<(int)paths.size();i++)
	//    	paths[i].printPathInfo();
}
void AOgraph::solveByNameHyperarc(string graphName,string nameHyperarc)
{
	AOgraph * newGraph=findGraph(graphName);
	HyperArc* solved = newGraph->findByNameHyperarc(nameHyperarc);
	bool result = solved->setSolved(newGraph->Nodes_solved_infeasible);
	newGraph->updateFeasibility();
//	printGraphInfo();
	// report that the graph has been solved if the solved node is the head node
	if (newGraph->head->nSolved == true)
	{
		cout<<"[REPORT] The graph is solved (head node solved)." <<endl;
		cout<<"This condition can not b true, because the we are in the hyper-arc solved member function!"<<endl;
		return;
	}

	// update the path information (cost) of all paths
	if (result == true)
		newGraph->updatePaths_HyperarcSolved(*solved); // -----
//	cout<<45<<endl;
//	cout<<endl <<"Updated paths: " <<endl;
//	for(int i=0; i< (int)pUpdate.size(); i++)
//		cout<<"Path index: " <<pIndices[i] <<" - Benefit: " <<pUpdate[i] <<endl;
}

void AOgraph::getFeasibleNode(vector<andor_msgs::Node> &feasileNodeVector)
{
//	cout<<"AOgraph::getFeasibleNode"<<endl;
	//! return the nodes that are feasible but not solved
	// the cost is the cost of the node

	// the ha cost we return is the min path cost pass through the node of a path,
	// that's how we can say which node is better to solve


	for(int i=0;i<(int)graph.size();i++)
	{
//		cout<<graph[i].nName<<"; feasibility:"<<graph[i].nFeasible<<", Solved: "<<graph[i].nSolved<<endl;
		if(graph[i].nFeasible==true && graph[i].nSolved==false)
		{

			int min_cost=INT_MAX; // a high number
			// if the node exits in several graph paths, we find the min cost from there
			for(int j=0; j<(int)paths.size();j++)
			{
				for(int k=0; k<(int)paths[j].pathNodes.size();k++)
				{

					if(graph[i].nName==paths[j].pathNodes[k]->nName)
					{
						if (paths[j].pCost<min_cost)
						{
							min_cost=paths[j].pCost;
						}
					}
				}

			}


			andor_msgs::Node temp_node_msg;
			temp_node_msg.nodeCost=min_cost;
			temp_node_msg.nodeName=graph[i].nName;
			temp_node_msg.graphName=gName;
			feasileNodeVector.push_back(temp_node_msg);
		}
	}
}
void AOgraph::getFeasibleHyperarc(vector<andor_msgs::Hyperarc> &feasileHyperarcVector, vector<andor_msgs::Node> &feasileNodeVector)
{
//	cout<<"AOgraph::getFeasibleHyperarc"<<endl;
	//! return the hyperarcs that are feasible but not solved
	// the ha cost we return is the min path cost pass through the ha of a path,
	// that's how we can say which hyperarc is better to solve
//	cout<<(int)graphHA.size()<<endl;
	for(int i=0;i<(int)graphHA.size();i++)
	{
//		cout<<graphHA[i]->hName<<"; feasibility:"<<graphHA[i]->hFeasible<<", Solved: "<<graphHA[i]->hSolved<<endl;
		if(graphHA[i]->hFeasible==true && graphHA[i]->hSolved==false)
		{

//			cout<<graphHA[i]->hName<<endl;
			int min_cost=INT_MAX; // rnd number
			// if the ha exits in several graph path, we find the min cost from there
			for(int j=0; j<(int)paths.size();j++)
			{
				for(int k=0; k<(int)paths[j].pathArcs.size();k++)
				{

					if(graphHA[i]->hIndex==paths[j].pathArcs[k])
					{
						if (paths[j].pCost<min_cost)
						{
							min_cost=paths[j].pCost;
						}
					}
				}
			}

			// check if there is a graph hierarchy inside this feasible hyper-arc
			vector<andor_msgs::Node> NEWfeasileNodeVector;			// feasible nodes from lower level
			vector<andor_msgs::Hyperarc> NEWfeasileHyperarcVector;	// feasible hyper-arcs from lower level

			if(graphHA[i]->lowerGraph!=NULL)
			{
				graphHA[i]->lowerGraph->getFeasibleNode(NEWfeasileNodeVector);
				graphHA[i]->lowerGraph->getFeasibleHyperarc(NEWfeasileHyperarcVector, NEWfeasileNodeVector);

				// fill the output msg
				for(int j=0;j<NEWfeasileNodeVector.size();j++)
				{
//					int cost1=NEWfeasileNodeVector[j].nodeCost;
//					cout<<"test: "<<NEWfeasileNodeVector.size()<<", "<<NEWfeasileNodeVector[j].nodeName<<", "<<cost1<<endl;

//					cout<<"lower level graph node: "<<min_cost<<" , "<<graphHA[i]->hCost<<" , "<<NEWfeasileNodeVector[j].nodeCost<<" ."<<endl;
					NEWfeasileNodeVector[j].nodeCost=min_cost - graphHA[i]->hCost + NEWfeasileNodeVector[j].nodeCost;
					feasileNodeVector.push_back(NEWfeasileNodeVector[j]);
				}
				for(int j=0;j<NEWfeasileHyperarcVector.size();j++)
				{
//					cout<<"test: "<<NEWfeasileHyperarcVector.size()<<", "<<NEWfeasileHyperarcVector[j].hyperarcName<<", "<<NEWfeasileHyperarcVector[j].hyperarcCost<<endl;
//					cout<<"lower level graph hyper-arc: "<<min_cost<<" , "<<graphHA[i]->hCost<<" , "<<NEWfeasileHyperarcVector[j].hyperarcCost<<" ."<<endl;
					NEWfeasileHyperarcVector[j].hyperarcCost=min_cost - graphHA[i]->hCost + NEWfeasileHyperarcVector[j].hyperarcCost;
					feasileHyperarcVector.push_back(NEWfeasileHyperarcVector[j]);
				}
			}
			else
			{
				// fill the output msg
				andor_msgs::Hyperarc temp_hyperarc_msg;
				temp_hyperarc_msg.hyperarcName=graphHA[i]->hName;
				temp_hyperarc_msg.hyperarcCost=min_cost;
				temp_hyperarc_msg.parentNode=graphHA[i]->hfatherName;
				for (int l=0;l<(int)graphHA[i]->children.size();l++)
				{
					temp_hyperarc_msg.childNodes.push_back(graphHA[i]->children[l]->nName);
				}
				temp_hyperarc_msg.graphName=gName;
				feasileHyperarcVector.push_back(temp_hyperarc_msg);

			}
		}
	}
}

bool AOgraph::isGraphSolved(){

	return head->nSolved;
}

AOgraph* AOgraph::findGraph(string graphName){
// the graph name in hierarchical and/or graph are like this:
	// graphName:hyper-arcName:newGraph:newHyper-arc:newGraph ....
// if there is a hierarchy, we write the hyper-arc name before the graph name.
	// ":" is a key character here

	AOgraph* tempGraph=NULL;
	string delimType=":";
	vector<string> graphList;

	boost::split(graphList, graphName, boost::is_any_of(delimType));

	if(graphList.size()==0)
	{
		cout<<"ERROR: in findGraph"<<endl;
	}
	else if(graphList.size()==1)
	{
		tempGraph=this;
	}
	else
	{
//		for(int i=0;i<graphList.size();i++)
//			cout<<graphList[i]<<" , ";
//		cout<<endl;
		string thisLevelGraphName=graphList[0];
		string nameHyperarc=graphList[1];
//		HyperArc* tempHA=this->findByNameHyperarc(nameHyperarc);
//		tempGraph=tempHA->lowerGraph->findGraph(graphName);

		for(int i=1;i<=graphList.size();)
		{

//			cout<<"********* "<<gName<<",1 , "<<i<<"): "<<thisLevelGraphName<<" , "<<gName<<endl;

			if(thisLevelGraphName==gName)
			{
//				cout<<gName<<",2 ,"<<endl;
				if(i==graphList.size())
				{
//					cout<<gName<<",3 ,"<<endl;
					tempGraph=this;
				}
				else
				{
					// the graph we are looking for is in lower level than the current level
//					cout<<gName<<",4 ,"<<endl;
					nameHyperarc=graphList[i];
					HyperArc* tempHA=this->findByNameHyperarc(nameHyperarc);
//					cout<<gName<<",5 ,"<<endl;
					tempGraph=tempHA->lowerGraph->findGraph(graphName);
//					cout<<gName<<",6 ,"<<endl;
				}
//				cout<<gName<<",7 ,"<<endl;
				break;
			}
			thisLevelGraphName+=":"+graphList[i]+":"+graphList[i+1];
			int z;
//			cout<<"++ to kill the process enter 1: "<<endl;
//			cin>>z;
//			if(z==1)
//				exit(0);

			i=i+2;
		}


//		if(tempHA->lowerGraph->gName==graphList[1])
//		{
//			string newGraphName;
//			for(int i=2;i<graphList.size();i++)
//				newGraphName+=graphList[i];
//
//		}
//		else
//		{
////			cout<<tempHA->lowerGraph->gName<<" == "<<
//			cout<< "the graph name given in the query and the hyper-arc graph name are not matched!"<<endl;
//			exit(1);
//		}
	}
	return tempGraph;
};

