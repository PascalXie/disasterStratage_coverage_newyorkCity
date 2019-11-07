#ifndef PATHPLANNINGASTAR_HH
#define PATHPLANNINGASTAR_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "AStarNode.hh"
#include "Histogram2D.hh"

using namespace std;

class PathPlanningAStar
{
    public:
		PathPlanningAStar(string name, Histogram2D *hCostFunction);
		~PathPlanningAStar();

    public:
		void SetStartNode(AStarNode *Node);
		void SetEndNode(AStarNode *EndNode);
		void SetObstacleValue(double obstacleValue);

    public:
		void InitiatPathPlanning();
		bool GeneratePerimeter8Nodes(string name, AStarNode* centerNode, int xID, int yID);
		void PathAnalysis();

	private:
		string name_;
		Histogram2D *hObstacles_;

		int ItartionID_;

		double obstacleValue_;

		vector<AStarNode*> Open_;
		vector<AStarNode*> Closed_;

		vector<AStarNode*> StartNodes_;
		vector<AStarNode*> EndNodes_;

		// results analysis
		bool IsEndNodeReached_;
		//string PathFileName_;

	public:
		// path, for successfully and unsuccessfully searching
		vector<AStarNode*> Path_;
	
	private:
		// path, for successfully searching
		vector<AStarNode*> PathSuccess_;





};
#endif
