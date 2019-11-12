#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <random>

#include "Units.hh"
#include "Histogram.hh"
#include "Histogram2D.hh"

// A star
#include "AStarNode.hh"
#include "PathPlanningAStar.hh"

// DynamicControlCentor
#include "DynamicRobotUnit.hh"
#include "DynamicControlCentor.hh"

using namespace std;

int		binSizes[2] = {10,10};
double	binWidths[2];
double	mins[2] = {-1000, -1000};
double 	maxs[2] = {1000, 1000};

vector<string> Zones_IDs;
vector<double> Zones_xs, Zones_ys, Zones_rs; // for SafeZonesAndDangerZones
vector<double> SafeZones_xs, SafeZones_ys, SafeZones_rs; // for SafeZonesAndDangerZones
bool ReadData_SafeZonesAndDangerZones(string filename1);

// test
Histogram2D *hObstacles_, *hObstacles_binary_;
bool Tool_ReadDistributionObstacles(string filename);
bool test();

int main()
{
	cout<<"Hello "<<endl;

	// test
	//test();
	//return 1;
	// !test

	// step 0 
	string filename0 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/data_histogram2d_parameters.txt";

	// step 0 : read obstacle data
	string obFileName1 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/data_step1_part2_tool_obstacles.txt";
	string obFileName2 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/data_step1_part2_tool_obstacles_binary.txt";

	// step 1 : DynamicControlCentor
	DynamicControlCentor * dcc = new DynamicControlCentor("DynamicControlCentor", filename0, obFileName1, obFileName2);

	//DynamicRobotUnit * rob = new DynamicRobotUnit("robottest", 0 , 0 , 0, 15);


	// step 0 : safe zones and danger zones
	// ../data_safeZonesAndDangerZones.txt
	string filename0_1 = "../../data_safeZonesAndDangerZones.txt";
	ReadData_SafeZonesAndDangerZones(filename0_1);
	dcc->Tool_SetSafeZones(SafeZones_xs, SafeZones_ys);

	// set time 
	//double maximumTimeInterval = 0.5*minute; // short time
	//double maximumTimeInterval = 50*minute;  // long time
	double maximumTimeInterval = 3*hour;  // long time
	dcc->Tool_SetMaximumTimeInterval(maximumTimeInterval);
	// initiate
	dcc->InitiateDynamicControl();

	// write
	dcc->WriteRobotPath();

	return 1;
}

bool test()
{
	// step 0 
	string filename0 = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/data_histogram2d_parameters.txt";

	// step 0 : ../data_histogram2d_parameters.txt
	ifstream file(filename0.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename0<<" \""<<endl;
		return 0;
	}

	string temp;
	file>>temp>>binSizes[0]>>mins[0]>>maxs[0];
	cout<<temp<<", bin "<<binSizes[0]<<", min "<<mins[0]<<", max "<<maxs[0]<<endl;

	file>>temp>>binSizes[1]>>mins[1]>>maxs[1];
	cout<<temp<<", bin "<<binSizes[1]<<", min "<<mins[1]<<", max "<<maxs[1]<<endl;

	string filename_ob = "../../alice_step1_part2_populationDensityGenerator_version3_aStarPathPlanning/build/data_step1_part2_tool_obstacles_binary.txt";
	Tool_ReadDistributionObstacles(filename_ob);

	// pp
	PathPlanningAStar *pp_ = new PathPlanningAStar("PathPlanning",hObstacles_binary_);
	//PathPlanningAStar *pp = new PathPlanningAStar("PathPlanning",hObstacles_); // test
	int xSID = hObstacles_binary_->GetBinIDX(23670.9);
	int ySID = hObstacles_binary_->GetBinIDY(7007.27);
	int xEID = hObstacles_binary_->GetBinIDX(21700);
	int yEID = hObstacles_binary_->GetBinIDY(20600);

	cout<<"Start ID "<<xSID<<" "<<ySID<<endl;
	cout<<"End   ID "<<xEID<<" "<<yEID<<endl;

	pp_->SetStartNode(new AStarNode("start",xSID,ySID));

	AStarNode *n_end = new AStarNode("end1",xEID,yEID);
	pp_->SetEndNode(n_end);
	pp_->InitiatPathPlanning();

	cout<<"Is End node reached "<<pp_->IsEndNodeReached_<<endl;
	cout<<"Path size "<<pp_->Path_.size()<<endl;
	for(int i=0;i<pp_->Path_.size();i++)
	{
		double x,y;
		hObstacles_->GetBinCentor2D(pp_->Path_[i]->xID_,pp_->Path_[i]->yID_,x,y);
		cout<<"Path ID "<<i<<endl;
	}

}

bool ReadData_SafeZonesAndDangerZones(string filename1)
{
	ifstream file1(filename1.c_str());

	if(file1.fail())
	{
		cout<<"Can not find the file \" "<<filename1<<" \""<<endl;
		return 0;
	}

	string ID;
	double x, y, r;
	while(!file1.eof())
	{
		file1>>ID>>x>>y>>r;

		if(file1.eof()) break;

		Zones_IDs.push_back(ID);
		Zones_xs.push_back(x);
		Zones_ys.push_back(y);
		Zones_rs.push_back(r);

		// for safe zones
		if(ID[4]=='S') 
		{
			//cout<<"Safe Zone : "<<ID<<endl;
			SafeZones_xs.push_back(x);
			SafeZones_ys.push_back(y);
			SafeZones_rs.push_back(r);
		}

		//cout<<ID<<" "<<x<<" "<<y<<" "<<r<<endl;
	}

	file1.close();

	/*
	// debug
	for(int i=0;i<SafeZones_xs.size();i++)
	{
		cout<<"SafeZones x y "<<SafeZones_xs[i]<<" "<<SafeZones_ys[i]<<endl;
	}
	*/

	return true;
}

bool Tool_ReadDistributionObstacles(string filename)
{
	hObstacles_			= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);
	hObstacles_binary_	= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	double x,y;
	double content;
	while(!file.eof())
	{
		file>>x>>y>>content;

		if(file.eof()) break;

		//hObstacles_->Fill(x,y,content);
		hObstacles_binary_->Fill(x,y,content);

	}

	file.close();
}
