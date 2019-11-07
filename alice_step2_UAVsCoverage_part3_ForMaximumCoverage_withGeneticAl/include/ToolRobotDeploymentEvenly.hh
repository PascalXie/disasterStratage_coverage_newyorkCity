#ifndef TOOLROBOTDEPLOYMENTEVENLY_HH
#define TOOLROBOTDEPLOYMENTEVENLY_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Units.hh"
#include "Histogram.hh"
#include "Histogram2D.hh"
#include "AStarNode.hh"
#include "PathPlanningAStar.hh"
#include "MapsForTime.hh"

using namespace std;

class ToolRobotDeploymentEvenly
{
	public:
		ToolRobotDeploymentEvenly(string name, int CoverageMode);
		~ToolRobotDeploymentEvenly();

    private:
		// 
		// step 1 - 0 : import histogram parameters
		// 
		bool Tool_ReadHistogramsParameters_ForBuildings(string filename);

		// step 1 - 1 : import histogram data for buildings, which are obstacles
		bool Tool_ReadDistributionObstacles(string filename);
		bool Tool_ReadDistributionObstacles_binary(string filename);

		// step 1 - 2 : import data for crowds
		bool Tool_ReadPathForCrowds(string filename);

		// step 1 - 3 : import Robots Parameters
		bool Tool_ReadRobotParameters(string filename);

		// step 2 - 1 : deploy the Robots (UAVs) evenly
		bool Deploy_step1_evenly();
		bool ComputeCoverage_DeployStep1();

	public:
		// Tools
		bool Tool_IsRobotPositoinGood(double x, double y);
		bool Tool_WriteHistogram2D(Histogram2D *h, string filename);

    private:
		string name_;

	public:
		//  step 1 - 1 : import histogram data for buildings, which are obstacles
		int		binSizes[2];
		double	mins[2];
		double 	maxs[2];
		double	binWidths[2];

		Histogram2D *hObstacles_, *hObstacles_binary_;

		// step 1 - 2 : import data for crowds
		vector<Histogram2D *> Paths_;
		
		// step 1 - 3 : import Robots Parameters
		int		RobotSize_;
		double	UAV_CommunicationRange_metre_;
		
	public:
		// step 2 - 1 : deploy the Robots (UAVs) evenly
		vector<double> robotCoor_x_1_all_, robotCoor_y_1_all_;
		vector<double> robotCoor_x_2_, robotCoor_y_2_; // selected, inside the map, outside the buildings
		Histogram2D *Coverage_step1_;

		// Coverage Mode
		// mode 1 : for localization
		// mode 2 : for coverage with minimum UAVs 
		int CoverageMode_;
};
#endif
