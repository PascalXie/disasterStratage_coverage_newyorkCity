#include "AStarNode.hh"

//-------------------------
// Constructor
//-------------------------
AStarNode::AStarNode(string name, int xID, int yID)
:	name_(name),
	xID_(xID),
	yID_(yID)
{
	F_cost_ = 0;
	G_cost_ = 0;
	H_cost_ = 0;

	Status_ = "None";

	if(xID<0) 
	{
		xID_ = 0;
		//cout<<"An error occured in Class AStarNode::AStarNode : xID is less than 0"<<endl;
	}
	if(yID<0) 
	{
		yID_ = 0;
		//cout<<"An error occured in Class AStarNode::AStarNode : yID is less than 0"<<endl;
	}
}

//-------------------------
// Destructor
//-------------------------
AStarNode::~AStarNode()
{}

//-------------------------
// Public
//-------------------------
void AStarNode::ComputeHCost()
{
}

void AStarNode::SetStatus(string Status)
{
	Status_ = Status;
}

