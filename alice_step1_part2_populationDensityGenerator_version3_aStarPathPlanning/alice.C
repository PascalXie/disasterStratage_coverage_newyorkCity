#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <random>

#include "UserCostFunction.hh"
#include "SteepestCostFunction.hh"
#include "NewtonsCostFunction.hh"

#include "UserResidualBlockFunction.hh"
#include "PolyResidualBlockFunction.hh"

#include "UserOptimizationManager.hh"
#include "SteepestOptimizationManager.hh"
#include "NewtonsOptimizationManager.hh"

#include "Units.hh"
#include "Histogram.hh"
#include "Histogram2D.hh"
#include "AStarNode.hh"
#include "PathPlanningAStar.hh"
#include "MapsForTime.hh"

using namespace std;

default_random_engine generator(time(0));

vector<string> IDs;
vector<double> xs, ys, rs; // for buildings

vector<string> Zones_IDs;
vector<double> Zones_xs, Zones_ys, Zones_rs; // for SafeZonesAndDangerZones

vector<double> SafeZones_xs, SafeZones_ys, SafeZones_rs; // for SafeZonesAndDangerZones

// maps for times
MapsForTime *maps;

int		bins[2] = {10,10};
double	mins[2] = {-1000, -1000};
double 	maxs[2] = {1000, 1000};

double sigmaWeight_ = 1.;
double PopulationDensity_ = 27627./1e6; // person/m2

double Velocity_ = 10000; // m.min

//double ObstacleValue_ = 0.0015;
double ObstacleValue_ = 0.5; // fake


Histogram2D *h_;
Histogram2D *hCostFunction_;
Histogram2D *hObstacles_, *hObstacles_binary_;
vector<Histogram2D *> hFx_, hFy_;
vector<PathPlanningAStar*> pps_;

void ComputePath(string computeName, double xStart, double yStart, double xEnd, double yEnd);
bool Tool_DistributionCostFunction();
bool Tool_DistributionObstacles();
bool Tool_ReadDistributionObstacles(string filename);

bool ReadData_SafeZonesAndDangerZones(string filename1);

int main()
{
	cout<<"Hello "<<endl;

	/*
	// test
	normal_distribution<double> distributionTest(0.0, 2.0);
	uniform_real_distribution<double> uniTest(0.0, 1.0);

	for(int i=0;i<100;i++)
	{
		cout<<i<<" "<<distributionTest(generator)<<" "<<uniTest(generator)<<endl;
	}
	*/

	// step 0 
	string filename0 = "../data_histogram2d_parameters.txt";
	double timeWidth = 100; // second
	maps = new MapsForTime("Flood",timeWidth,filename0);

	// step 0 : ../data_histogram2d_parameters.txt
	ifstream file(filename0.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename0<<" \""<<endl;
		return 0;
	}

	string temp;
	file>>temp>>bins[0]>>mins[0]>>maxs[0];
	cout<<temp<<", bin "<<bins[0]<<", min "<<mins[0]<<", max "<<maxs[0]<<endl;

	file>>temp>>bins[1]>>mins[1]>>maxs[1];
	cout<<temp<<", bin "<<bins[1]<<", min "<<mins[1]<<", max "<<maxs[1]<<endl;

	// step 0 : histogram
	h_ = new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]);


	// step 0 : safe zones and danger zones
	// ../data_safeZonesAndDangerZones.txt
	string filename0_1 = "../../data_safeZonesAndDangerZones.txt";
	ReadData_SafeZonesAndDangerZones(filename0_1);


	// step 1 : buildings
	string filename1 = "../../data_buildings.txt";
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

		if(r>10000) continue; // ground object in Blender

		IDs.push_back(ID);
		xs.push_back(x);
		ys.push_back(y);
		rs.push_back(r);

		//cout<<ID<<" "<<x<<" "<<y<<" "<<r<<endl;
	}

	file1.close();

	cout<<"IDname test : name "<<IDs[0]<<", name[0] "<<IDs[0][0]<<endl;

	// step 2 - 1 : compute DistributionCostFunction 
	//bool isBuildGood =Tool_DistributionCostFunction();

	// step 2 - 2 : compute DistributionObstacles
	bool isFakeDataUsed = false;
	if(!isFakeDataUsed)
	{
		bool isObstaclesGood = Tool_DistributionObstacles();
	}
	if(isFakeDataUsed)
	{
		cout<<"Attention! Fake Obstacle Data was used!"<<endl;
		bool isObstaclesGood = Tool_ReadDistributionObstacles("data_step1_part2_tool_obstacles.txt");

	}

	// step 2 - 3 : path generating
	// test
	//ComputePath("test",24700,12000,17000,27000);

	// path for all bins
	int binSizes[2] = {bins[0], bins[1]}; // the same as bins
	double binWidths[2];
	binWidths[0] = (maxs[0]-mins[0])/double(binSizes[0]);
	binWidths[1] = (maxs[1]-mins[1])/double(binSizes[1]);
	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;

		double x = mins[0] + binWidths[0]*(double(xID)+0.5);
		double y = mins[1] + binWidths[1]*(double(yID)+0.5);

		// find the nearest safe zones
		double nearest_d = 100000;
		int    nearest_ID = 0;
		for(int k=0;k<SafeZones_xs.size();k++)
		{
			double zx = SafeZones_xs[k];
			double zy = SafeZones_ys[k];
			double zr = SafeZones_rs[k];
			double d2 = (x-zx)*(x-zx) + (y-zy)*(y-zy);
			double d  = sqrt(d2);

			if(k==0)
			{
				nearest_d = d;
				nearest_ID = 0;
			}
			else
			{
				if(d<nearest_d)
				{
					nearest_d = d;
					nearest_ID = k;
				}
			}
		}

		// compute path by A*
		string IDName = to_string(xID) + "_" + to_string(yID);
		ComputePath(IDName,x, y, SafeZones_xs[nearest_ID], SafeZones_ys[nearest_ID]);


		// debug
		//break;
		// !debug
	}

	// step 3 : WriteMaps
	// step 3 - 1 : compute the largest round size
	int largestRoundSize = 1;
	for(int k=0;k<pps_.size();k++)
	{
		PathPlanningAStar *pp = pps_[k];

		if(pp->Path_.size()>largestRoundSize)
		{
			largestRoundSize = pp->Path_.size();
		}
	}
	cout<<"largestRoundSize "<<largestRoundSize<<endl;

	// add path to maps of time
	//double binWidths[2];
	//binWidths[0] = (maxs[0]-mins[0])/double(binSizes[0]);
	//binWidths[1] = (maxs[1]-mins[1])/double(binSizes[1]);
	double populationPerBin = PopulationDensity_*binWidths[0]*binWidths[1];

	for(int k=0;k<pps_.size();k++)
	{
		PathPlanningAStar *pp = pps_[k];

		for(int i=0;i<pp->Path_.size();i++)
		{
			double x,y;
			hObstacles_->GetBinCentor2D(pp->Path_[i]->xID_,pp->Path_[i]->yID_,x,y);

			int RoundID = i;
			maps->AddABinAtATimeStamp(x,y,populationPerBin,RoundID);

			// debug
			//cout<<"PathID "<<i<<", name "<<pp.Path_[i]->name_<<": ID "<<pp.Path_[i]->xID_<<" "<<pp.Path_[i]->yID_<<", x y "<<x<<" "<<y<<endl;
		}

		int lastxID = pp->Path_[pp->Path_.size()-1]->xID_;
		int lastyID = pp->Path_[pp->Path_.size()-1]->yID_;
		double lastx,lasty;
		hObstacles_->GetBinCentor2D(lastxID,lastyID,lastx,lasty);
		//cout<<"lastxID lastyID"<<lastxID<<" "<<lastyID<<endl;
		for(int i=pp->Path_.size();i<largestRoundSize;i++)
		{
			int RoundID = i;
			//cout<<"RoundID "<<RoundID<<endl;
			maps->AddABinAtATimeStamp(lastx,lasty,populationPerBin,RoundID);
		}

	}

	//  WriteMaps
	maps->WriteMaps("data_step1_part2_populationMaps");
	



	return 1;
}

// path planning
void ComputePath(string computeName, double xStart, double yStart, double xEnd, double yEnd)
{
	cout<<yEnd<<" "<<maxs[1]<<endl;

	// range check 
	if(xStart<mins[0]||xStart>=maxs[0]) 
	{
		cout<<"An error occured in Main::ComputePath, xStart is out of range"<<endl;
		return;
	}
	if(xEnd<mins[0]||xEnd>=maxs[0]) 
	{
		cout<<"An error occured in Main::ComputePath, xEnd is out of range"<<endl;
		return;
	}
	if(yStart<mins[1]||yStart>=maxs[1]) 
	{
		cout<<"An error occured in Main::ComputePath, yStart is out of range"<<endl;
		return;
	}
	if(yEnd<mins[1]||yEnd>=maxs[1]) 
	{
		cout<<"An error occured in Main::ComputePath, yEnd is out of range"<<endl;
		return;
	}

	int xSID = hObstacles_->GetBinIDX(xStart);
	int ySID = hObstacles_->GetBinIDY(yStart);
	int xEID = hObstacles_->GetBinIDX(xEnd);
	int yEID = hObstacles_->GetBinIDY(yEnd);
	cout<<"computeName "<<computeName<<endl;
	cout<<"Start : xID yID "<<xSID<<" "<<ySID<<", x y "<<xStart<<" "<<yStart<<endl;
	cout<<"End   : xID yID "<<xEID<<" "<<yEID<<", x y "<<xEnd  <<" "<<yEnd  <<endl;


	//PathPlanningAStar pp("PathPlanning",hObstacles_);
	PathPlanningAStar *pp = new PathPlanningAStar("PathPlanning",hObstacles_);

	pp->SetStartNode(new AStarNode("start",xSID,ySID));

	AStarNode *n_end = new AStarNode("end1",xEID,yEID);
	pp->SetEndNode(n_end);

	// set ObstacleValue
	pp->SetObstacleValue(ObstacleValue_);

	pp->InitiatPathPlanning();

	pps_.push_back(pp);

	/*
	// add path to maps of time
	int binSizes[2] = {bins[0], bins[1]}; // the same as bins
	double binWidths[2];
	binWidths[0] = (maxs[0]-mins[0])/double(binSizes[0]);
	binWidths[1] = (maxs[1]-mins[1])/double(binSizes[1]);

	for(int i=0;i<pp->Path_.size();i++)
	{
		double x,y;
		hObstacles_->GetBinCentor2D(pp->Path_[i]->xID_,pp->Path_[i]->yID_,x,y);

		int RoundID = i;
		double populationPerBin = PopulationDensity_*binWidths[0]*binWidths[1];
		maps->AddABinAtATimeStamp(x,y,populationPerBin,RoundID);

		// debug
		//cout<<"PathID "<<i<<", name "<<pp.Path_[i]->name_<<": ID "<<pp.Path_[i]->xID_<<" "<<pp.Path_[i]->yID_<<", x y "<<x<<" "<<y<<endl;
	}
	*/

}

bool Tool_DistributionCostFunction()
{
	hCostFunction_ = new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]);

	//
	// Optimization 
	//
	int observationsSize	= 4;
	int residualSize		= 1;
	int varialbeSize 		= 2;

	UserOptimizationManager * manager = new NewtonsOptimizationManager("NewtonsMethod",observationsSize,varialbeSize,residualSize);

	// set cost function
	UserCostFunction* costFunction = new NewtonsCostFunction("costFunction",observationsSize,varialbeSize,residualSize);

	// get observations
	// buildings
	for(int i=0;i<IDs.size();i++)
	{
		int buildingID = i;
		string IDname = IDs[buildingID];
		double bx = xs[buildingID];
		double by = ys[buildingID];
		double br = rs[buildingID];

		double IDname_d = 0; // 0 for building

		vector<double> observation_current;
		observation_current.push_back(bx);
		observation_current.push_back(by);
		observation_current.push_back(br);
		observation_current.push_back(IDname_d);
		costFunction->AddResidualBlock(observation_current);
	}

	// zones
	for(int i=0;i<Zones_IDs.size();i++)
	{
		int buildingID = i;
		string IDname = Zones_IDs[buildingID];
		double bx = Zones_xs[buildingID];
		double by = Zones_ys[buildingID];
		double br = Zones_rs[buildingID];

		double IDname_d = 1; // 1 for Danger Zone

		// for safe zones
		if(IDname[4]=='S') 
		{
			//cout<<"Safe : "<<IDname<<endl;
			br *= -1.; 
			IDname_d = 2; // 2 for Safe Zone
		}

		vector<double> observation_current;
		observation_current.push_back(bx);
		observation_current.push_back(by);
		observation_current.push_back(br);
		observation_current.push_back(IDname_d);
		costFunction->AddResidualBlock(observation_current);
	}

	//
	// histogram of cost function
	//
	string filename = "data_step1_part2_tool_costFunction.txt";
	ofstream write(filename);

	int binSizes[2] = {bins[0], bins[1]}; // the same as bins
	double binWidths[2];
	binWidths[0] = (maxs[0]-mins[0])/double(binSizes[0]);
	binWidths[1] = (maxs[1]-mins[1])/double(binSizes[1]);

	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;

		double x = mins[0] + binWidths[0]*(double(xID)+0.5);
		double y = mins[1] + binWidths[1]*(double(yID)+0.5);

		// debug
		//cout<<"ID_ "<<ID_<<", x "<<x<<", y "<<y<<endl;

		vector<double> variables, costFunctionValue;
		variables.push_back(x);
		variables.push_back(y);

		costFunction->CostFunction(variables, costFunctionValue);

		hCostFunction_->Fill(x,y,costFunctionValue[0]);

		//write<<x<<" "<<y<<" "<<costFunctionValue[0]<<endl;

	}

	// write
	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;

		double x,y;
		hCostFunction_->GetBinCentor2D(xID, yID, x, y);

		double content = hCostFunction_->GetBinContent(xID, yID);

		write<<x<<" "<<y<<" "<<content<<endl;
	}

	write.close();

	return true;
}

bool Tool_DistributionObstacles()
{
	hObstacles_				= new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]);
	hObstacles_binary_		= new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]);

	//
	// Optimization 
	//
	int observationsSize	= 4;
	int residualSize		= 1;
	int varialbeSize 		= 2;

	UserOptimizationManager * manager = new NewtonsOptimizationManager("NewtonsMethod",observationsSize,varialbeSize,residualSize);

	// set cost function
	UserCostFunction* costFunction = new NewtonsCostFunction("costFunction",observationsSize,varialbeSize,residualSize);

	// get observations
	// buildings
	for(int i=0;i<IDs.size();i++)
	{
		int buildingID = i;
		string IDname = IDs[buildingID];
		double bx = xs[buildingID];
		double by = ys[buildingID];
		double br = rs[buildingID]/1.;

		double IDname_d = 0; // 0 for building

		vector<double> observation_current;
		observation_current.push_back(bx);
		observation_current.push_back(by);
		observation_current.push_back(br);
		observation_current.push_back(IDname_d);
		costFunction->AddResidualBlock(observation_current);
	}


	//
	// histogram of cost function
	//
	string filename2 = "data_step1_part2_tool_obstacles.txt";
	ofstream write2(filename2);

	string filename3 = "data_step1_part2_tool_obstacles_binary.txt";
	ofstream write3(filename3);

	int binSizes[2] = {bins[0], bins[1]}; // the same as bins
	double binWidths[2];
	binWidths[0] = (maxs[0]-mins[0])/double(binSizes[0]);
	binWidths[1] = (maxs[1]-mins[1])/double(binSizes[1]);

	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;

		double x = mins[0] + binWidths[0]*(double(xID)+0.5);
		double y = mins[1] + binWidths[1]*(double(yID)+0.5);

		// debug
		//cout<<"ID_ "<<ID_<<", x "<<x<<", y "<<y<<endl;

		vector<double> variables, costFunctionValue;
		variables.push_back(x);
		variables.push_back(y);

		costFunction->CostFunction(variables, costFunctionValue);

		int isObstacle = 1;
		if(costFunctionValue[0]<ObstacleValue_) isObstacle = 0;

		hObstacles_->Fill(x,y,costFunctionValue[0]); // continuos

		hObstacles_binary_->Fill(x,y,isObstacle); // true, i.e. binary 
	}

	// write
	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;

		double x,y;
		hObstacles_->GetBinCentor2D(xID, yID, x, y);

		double content = hObstacles_->GetBinContent(xID, yID);
		write2<<x<<" "<<y<<" "<<content<<endl;

		double content3 = hObstacles_binary_->GetBinContent(xID, yID);
		write3<<x<<" "<<y<<" "<<content3<<endl;

	}

	write2.close();
	write3.close();

	return true;
}

bool Tool_ReadDistributionObstacles(string filename)
{
	hObstacles_			= new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]);
	hObstacles_binary_	= new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]);

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

		hObstacles_->Fill(x,y,content);

		int isObstacle = 1;
		if(content<ObstacleValue_) isObstacle = 0;

		hObstacles_binary_->Fill(x,y,isObstacle); // true, i.e. binary 
	}

	file.close();
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

	return true;
}
