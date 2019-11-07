#ifndef ASTARNODE_HH
#define ASTARNODE_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Histogram2D.hh"

using namespace std;

class AStarNode
{
    public:
		AStarNode(string name, int xID, int yID);
		~AStarNode();

    public:
		void ComputeHCost();
		void SetStatus(string Status);

    public:
		string name_;
		int xID_;
		int yID_;

		// F = G+H
		double F_cost_;
		double G_cost_;
		double H_cost_; // Heuristic

		// status referes to the three status: Open, Closed, None
		string Status_;

		// father node
		AStarNode * FatherNode_;

};
#endif
