#ifndef DYNAMICCONTROLCENTOR_HH
#define DYNAMICCONTROLCENTOR_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Units.hh"
#include "Histogram2D.hh"

// path planning : A* algorithm
#include "PathPlanningAStar.hh"

#include "DynamicRobotUnit.hh"

using namespace std;

class DynamicControlCentor
{
    public:
		DynamicControlCentor(string name, string filename_BuildingHis2D, string FileName_obstacle, string FileName_obstacleBinary);
		~DynamicControlCentor();

    public:
		// step 1 - 0 : import histogram parameters
		bool Tool_ReadHistogramsParameters_ForBuildings(string filename);

		// step 1 - 1 : import histogram data for buildings, which are obstacles
		bool Tool_ReadDistributionObstacles(string filename);
		bool Tool_ReadDistributionObstacles_binary(string filename);

		// step 1 - 4 : import robot deployment
		bool Tool_ReadRobotDeployment(string filename);

		// step 1 - 5 : set SafeZones
		bool Tool_SetSafeZones(vector<double> SafeZone_x, vector<double> SafeZone_y);

		// step 2 - 1 : time interval
		bool Tool_SetMaximumTimeInterval(double MaximumTimeInterval);

    public:
		// step 2 : initiate
		bool InitiateDynamicControl();

		// step 3 : output
		bool WriteRobotPath();

    private:
		// step 2 - 1 - 2 - 1 : checking robots
		bool CheckingRobot(int robotID);
		// step 2 - 1 - 2 - 2 : checking deployment
		bool CheckingDeployment(double dep_x, double dep_y);

		// step 2 : tools
		int FindTheNearestSafeZone(double x, double y);
		bool FindAndSentARobotHere(double dep_x, double dep_y);
		bool SentTheRobotBack(DynamicRobotUnit *rob, double sf_x, double sf_y);

    public:
		string name_;

		// speed for crowds and robots
		double speed_crowd_;
		double speed_robot_;

    public:
		// step 1 - 0 : import histogram parameters
		bool IsStep1_0Good_;
		//  step 1 - 1 : import histogram data for buildings, which are obstacles
		bool IsStep1_11Good_, IsStep1_12Good_;
		// step 1 - 4 : import robot deployment
		bool IsStep1_4Good_;
		// step 1 - 5 : set SafeZones
		bool IsStep1_5Good_;
		// step 2 - 1 : time interval
		bool IsStep2_1Good_;

    public:
		//  step 1 - 1 : import histogram data for buildings, which are obstacles
		int		binSizes[2];
		double	mins[2];
		double 	maxs[2];
		double	binWidths[2];

		Histogram2D *hObstacles_, *hObstacles_binary_; 
		Histogram2D *hObstacles_NoObstacle_; 
		// step 1 - 3 : path planning
		//PathPlanningAStar *pp_; // do not use

		// step 1 - 4 : import robot deployment
		vector<double> dep_x_, dep_y_;

		// step 1 - 5 : set SafeZones
		vector<double> SafeZone_x_, SafeZone_y_;

		// step 2 - 1 : time interval
		double MaximumTimeInterval_;
		double CurrentTime_;
		double TimeInterval_;

		// step 2 - 1 - 1 - 1
		vector<DynamicRobotUnit *> robots_;
};
#endif
