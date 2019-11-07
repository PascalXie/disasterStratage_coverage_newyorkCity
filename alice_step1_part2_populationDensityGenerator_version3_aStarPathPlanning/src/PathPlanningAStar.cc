#include "PathPlanningAStar.hh"

//-------------------------
// Constructor
//-------------------------
PathPlanningAStar::PathPlanningAStar(string name, Histogram2D *hCostFunction)
:	name_(name),
	hObstacles_(hCostFunction)
{
	Open_.clear();
	Closed_.clear();

	obstacleValue_ = 0;

	IsEndNodeReached_ = false;
}

//-------------------------
// Destructor
//-------------------------
PathPlanningAStar::~PathPlanningAStar()
{}

//-------------------------
// Public
//-------------------------
void PathPlanningAStar::InitiatPathPlanning()
{
	//
	// https://www.bilibili.com/video/av7830414?from=search&seid=14105517394617518701
	//

	//
	// step 0 : add the start node to Open
	//
	Open_.push_back(StartNodes_[0]);
	StartNodes_[0]->SetStatus("Open");
	//cout<<"Step 0 : Add start node to Open, Open size "<<Open_.size()<<endl;

	//
	// step 1 : loop
	//
	ItartionID_ = 0;
	while(Open_.size()!=0)
	{
		// step 1 - 1 : get node with lowest F cost
		double Lowest_F_Cost = 10000;
		AStarNode* bestNode;
		int bestNodeID = 0;
		for(int i=0;i<Open_.size();i++)
		{
			if(i==0) {
				Lowest_F_Cost = Open_[i]->F_cost_;
				bestNode = Open_[i];
				bestNodeID = i;
				continue;
			}

			if(Open_[i]->F_cost_<Lowest_F_Cost)
			{
				Lowest_F_Cost = Open_[i]->F_cost_;
				bestNode = Open_[i];
				bestNodeID = i;
			}
		}

		// step 1 - 1 - 2 : recording
		if(bestNode->FatherNode_->name_ == StartNodes_[0]->name_)
		{
			Path_.clear();
			Path_.push_back(StartNodes_[0]);
		}
		Path_.push_back(bestNode);

		// step 1 - 2 : remove the best node from open 
		Open_.erase(Open_.begin()+bestNodeID);

		// step 1 - 3 : add best node to closed
		Closed_.push_back(bestNode);
		bestNode->Status_ = "Closed";

		// step 1 - 4 : if the best node the target, i.e. end node, search done
		if(bestNode->xID_==EndNodes_[0]->xID_ && bestNode->yID_==EndNodes_[0]->yID_)
		{
			IsEndNodeReached_ = true;

			EndNodes_[0]->FatherNode_ = bestNode->FatherNode_;
			bestNode->name_ = EndNodes_[0]->name_;
			//cout<<"Path found!"<<endl;

			//cout<<"ItartionID_ "<<ItartionID_<<", bestNode "<<bestNode->name_<<", x y "<<bestNode->xID_<<" "<<bestNode->yID_<<", FatherNode_ "<<bestNode->FatherNode_->xID_<<" "<<bestNode->FatherNode_->yID_<<endl;

			break;
		}

		// step 1 - 5 : 8 neighbour nodes
		GeneratePerimeter8Nodes("Up",		bestNode, bestNode->xID_  , bestNode->yID_+1); 
		GeneratePerimeter8Nodes("UpRight",	bestNode, bestNode->xID_+1, bestNode->yID_+1); 
		GeneratePerimeter8Nodes("Right",	bestNode, bestNode->xID_+1, bestNode->yID_  );
		GeneratePerimeter8Nodes("DownRight",bestNode, bestNode->xID_+1, bestNode->yID_-1);
		GeneratePerimeter8Nodes("Down",		bestNode, bestNode->xID_  , bestNode->yID_-1);
		GeneratePerimeter8Nodes("DownLeft",	bestNode, bestNode->xID_-1, bestNode->yID_-1);
		GeneratePerimeter8Nodes("Left",		bestNode, bestNode->xID_-1, bestNode->yID_  );
		GeneratePerimeter8Nodes("UpLeft",	bestNode, bestNode->xID_-1, bestNode->yID_+1);


		// tag iteration
		ItartionID_++;


		/*
		// debug
		cout<<"----------------------------------------------------------"<<endl;
		cout<<"------"<<endl;
		cout<<"ItartionID_ "<<ItartionID_<<", bestNode "<<bestNode->name_<<", x y "<<bestNode->xID_<<" "<<bestNode->yID_<<", FatherNode_ "<<bestNode->FatherNode_->xID_<<" "<<bestNode->FatherNode_->yID_<<endl;
		cout<<"Open_   Size "<<Open_.size()<<endl;
		cout<<"Closed_ Size "<<Closed_.size()<<endl;
		cout<<"------"<<endl;
		cout<<"----------------------------------------------------------"<<endl;

		break;
		// !debug
		*/
	}

	// 
	PathAnalysis();

}

//-------------------------
// Public
//-------------------------
void PathPlanningAStar::PathAnalysis()
{
	AStarNode* node = EndNodes_[0];

	if(IsEndNodeReached_==true)
	{
		PathSuccess_.clear();

		int pathID = 0;
		while(1)
		{
			PathSuccess_.insert(PathSuccess_.begin(), node);

			//cout<<"pathID "<<pathID<<", name "<<node->name_<<", x y "<<node->xID_<<" "<<node->yID_<<", father node "<<node->FatherNode_->name_<<endl;

			if(node->name_==StartNodes_[0]->name_) {
				break;
			}

			node = node->FatherNode_;
			pathID++;
		}

		Path_ = PathSuccess_;
	}

	/*
	// debug
	for(int i=0;i<Path_.size();i++)
	{
		int pathID = i;
		cout<<"pathID "<<pathID<<", name "<<Path_[i]->name_<<", x y "<<Path_[i]->xID_<<" "<<Path_[i]->yID_<<", father node "<<Path_[i]->FatherNode_->name_<<endl;
	}
	*/

}

bool PathPlanningAStar::GeneratePerimeter8Nodes(string name, AStarNode* centerNode, int xID, int yID)
{
	// debug
	//cout<<"name "<<name<<", xID yID "<<xID<<" "<<yID<<endl;

	AStarNode * periNode = new AStarNode(name,xID,yID);

	double x, y;
	bool isNInsideTheMap = hObstacles_->GetBinCentor2D(xID,yID, x, y);
	bool isNObstacle = hObstacles_->GetBinContent(xID,yID);

	// check : if neighbour is not traversable 
	if(isNInsideTheMap==false||isNObstacle==true) return false;

	// check : if neighnour is in closed
	bool isClosedNode = false;
	for(int i=0;i<Closed_.size();i++)
	{
		if(Closed_[i]->xID_==xID&&Closed_[i]->yID_==yID) isClosedNode = true;
	}
	if(isClosedNode==true) return false;

	//
	// compute costs
	//
	// G
	double G = 10000;
	if(name=="Up"||name=="Right"||name=="Down"||name=="Left")
	{
		G = centerNode->G_cost_ + 10;
	}
	else if(name=="UpRight"||name=="DownRight"||name=="DownLeft"||name=="UpLeft")
	{
		G = centerNode->G_cost_ + 14;
	}

	// H
	double xDelta = abs(EndNodes_[0]->xID_ - xID);
	double yDelta = abs(EndNodes_[0]->yID_ - yID);

	double shortDelta = xDelta;
	double longDelta  = yDelta;
	if(xDelta>yDelta) 
	{
		shortDelta = yDelta;
		longDelta  = xDelta;
	}

	double H = shortDelta*14 + (longDelta-shortDelta)*10;

	// F
	double F = G + H;

	// if this perimeter is in the open, compare the F cost 
	bool isOpenNode = false;
	int OpenNodeID = 0;
	for(int i=0;i<Open_.size();i++)
	{
		if(Open_[i]->xID_==xID&&Open_[i]->yID_==yID) {
			isOpenNode = true;
			OpenNodeID = i;
		}
	}

	if(isOpenNode == true)
	{
		if(F<Open_[OpenNodeID]->F_cost_)
		{
			Open_[OpenNodeID]->F_cost_ = F;
			Open_[OpenNodeID]->FatherNode_ = centerNode;
		}
	}

	if(isOpenNode == false)
	{
		periNode->F_cost_ = F;
		periNode->FatherNode_ = centerNode;
		periNode->Status_ = "Open";
		Open_.push_back(periNode);
		//cout<<"Added, Open_ size "<<Open_.size()<<endl;
	}

	/*
	// debug
	cout<<"----------------------------------------------------------"<<endl;
	cout<<"------"<<endl;
	cout<<"debug PathPlanningAStar::GeneratePerimeter8Nodes "<<endl;
	cout<<"Direction "<<name<<"; x y "<<x<<", "<<y<<"; xID yID "<<xID<<", "<<yID<<"; isNInsideTheMap "<<isNInsideTheMap<<", isNObstacle "<<isNObstacle<<", isOpenNode "<<isOpenNode<<endl;
	cout<<"Cost F "<<F<<", G(from start) "<<G<<", H(to end) "<<H<<endl;
	cout<<"Open_ size "<<Open_.size()<<endl;
	cout<<"------"<<endl;
	cout<<"----------------------------------------------------------"<<endl;
	*/


	return true;
}

//-------------------------
// Public
//-------------------------
void PathPlanningAStar::SetStartNode(AStarNode *Node)
{
	StartNodes_.clear();
	StartNodes_.push_back(Node);
	StartNodes_[0]->FatherNode_ = StartNodes_[0];

	/*
	// debug
	cout<<"Start Node size "<<StartNodes_.size()<<endl;
	*/
}

void PathPlanningAStar::SetEndNode(AStarNode *EndNode)
{
	EndNodes_.clear();
	EndNodes_.push_back(EndNode);
}

void PathPlanningAStar::SetObstacleValue(double obstacleValue)
{
	obstacleValue_ = obstacleValue;

	/*
	// debug
	cout<<"debug PathPlanningAStar::SetObstacleValue "<<endl;
	cout<<"obstacleValue_ "<<obstacleValue_<<endl;
	*/
}
