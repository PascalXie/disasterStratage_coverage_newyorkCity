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


using namespace std;

default_random_engine generator(time(0));

vector<string> IDs;
vector<double> xs, ys, rs; // for buildings

vector<string> Zones_IDs;
vector<double> Zones_xs, Zones_ys, Zones_rs; // for SafeZonesAndDangerZones

int		bins[2] = {10,10};
double	mins[2] = {-1000, -1000};
double 	maxs[2] = {1000, 1000};

double sigmaWeight_ = 1.;
double PopulationDensity_ = 27627./1e6;

double Velocity_ = 10000; // m.min


Histogram2D *h_;
vector<Histogram2D *> hpopu_;
vector<Histogram2D *> hFx_, hFy_;

void ComputePath(string computeName, double x, double y);
bool Tool_DistributionCostFunction();

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

	// step 0 : ../data_histogram2d_parameters.txt
	string filename0 = "../data_histogram2d_parameters.txt";
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
	hpopu_.push_back(new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]));


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

	// Tool_DistributionCostFunction 
	bool isBuildGood =Tool_DistributionCostFunction();

	// path generating
	// test
	ComputePath("test",24700,12000);

	/*
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

		string computeName = to_string(xID) + "_" + to_string(yID);
		ComputePath(computeName,x,y);
	}
	*/


	return 1;
}

void ComputePath(string computeName, double x, double y)
{
	cout<<"computeName "<<computeName<<endl;
	//
	// Optimization 
	//
	int observationsSize	= 4;
	int residualSize		= 1;
	int varialbeSize 		= 2;

	UserOptimizationManager * manager = new NewtonsOptimizationManager(computeName,observationsSize,varialbeSize,residualSize);

	// set variables
	vector<double> variables;
	for(int i=0;i<varialbeSize;i++)
	{
		variables.push_back(0.);
	}
	variables[0]=x;
	variables[1]=y;
	manager->SetUserInitialization(variables);

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
		double br = Zones_rs[buildingID] * 10.;

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
	//cout<<"alice SetUserInitialization"<<endl;
	manager->SetUserInitialization(costFunction);

	//
	double UserReferencedLength = 100.; // m
	manager->SetUserReferencedLength(UserReferencedLength);

	// 
	double UserReferencedEpsilon = 1e-3;
	manager->SetUserEpsilonForTerminating(UserReferencedEpsilon);

	// initialize
	//cout<<"Initialize "<<endl;
	manager->Initialize();

	// get results
	vector<double> tarLocation;
	manager->GetVariables(tarLocation);
}

bool Tool_DistributionCostFunction()
{
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
		double br = Zones_rs[buildingID]*10;

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

//int		bins[2] = {10,10};
//double	mins[2] = {-1000, -1000};
//double 	maxs[2] = {1000, 1000};

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

		write<<x<<" "<<y<<" "<<costFunctionValue[0]<<endl;

	}

	write.close();

	return true;
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

		//cout<<ID<<" "<<x<<" "<<y<<" "<<r<<endl;
	}

	file1.close();

	return true;
}
