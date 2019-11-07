#include "ToolFitnessFunction.hh"

//-------------------------
// Constructor
//-------------------------
ToolFitnessFunction::ToolFitnessFunction(string name, ToolRobotDeploymentEvenly *toolRobotDeploymentEvenly)
:	name_(name),
	toolRobotDeploymentEvenly_(toolRobotDeploymentEvenly)
{
	/*
	// debug
	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug ToolFitnessFunction::ToolFitnessFunction"<<endl;
	cout<<"name "<<name_<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;
	*/
}

//-------------------------
// Destructor
//-------------------------
ToolFitnessFunction::~ToolFitnessFunction()
{}

//-------------------------
// Public
//-------------------------
double ToolFitnessFunction::ComputeFitness(GAIndividual* ind)
{
	//
	// compute fitness
	//

	//
	// aspect 1 : inside map and outside buildings
	//
	double fitness_1 = 0;

	for(int i=0;i<ind->x_.size();i++)
	{
		int gID = i;
		bool IsGeneGood_insideMap_outsideBuildings 
			= toolRobotDeploymentEvenly_->Tool_IsRobotPositoinGood(ind->x_[gID],ind->y_[gID]);

		if(IsGeneGood_insideMap_outsideBuildings==true) {
			fitness_1 ++;
		}
		//else {
		//	fitness_1 --;
		//}
	}

	// Normalization 
	fitness_1 = fitness_1/double(ind->x_.size());

	//
	// 3-coverage or more
	//
	double fitness_2 = 0;

	// histogram for fitness
	int		binSizes[2];
	double	mins[2];
	double 	maxs[2];
	double	binWidths[2];

	for(int i=0;i<2;i++)
	{
		binSizes[i]		= toolRobotDeploymentEvenly_->binSizes[i];
		mins[i]			= toolRobotDeploymentEvenly_->mins[i];
		maxs[i]			= toolRobotDeploymentEvenly_->maxs[i];
		binWidths[i]	= toolRobotDeploymentEvenly_->binWidths[i];
	}

	Histogram2D *hFitness = new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	// compute fitness
	double comRange = toolRobotDeploymentEvenly_->UAV_CommunicationRange_metre_;
	for(int k=0;k<ind->x_.size();k++)
	{
		int rID = k; // robot ID
		double rx = ind->x_[rID];
		double ry = ind->x_[rID];

		int xIDMin = hFitness->GetBinIDX(rx-comRange);
		if(xIDMin==-1) xIDMin=0;

		int xIDMax = hFitness->GetBinIDX(rx+comRange);
		if(xIDMax==-1) xIDMax=binSizes[0]-1;

		int yIDMin = hFitness->GetBinIDY(ry-comRange);
		if(yIDMin==-1) yIDMin=0;

		int yIDMax = hFitness->GetBinIDY(ry+comRange);
		if(yIDMax==-1) yIDMax=binSizes[1]-1;

		for(int i=xIDMin-1;i<xIDMax+2;i++)
		for(int j=yIDMin-1;j<yIDMax+2;j++)
		{
			int xID = i;
			int yID = j;
			double x, y;
			hFitness->GetBinCentor2D(xID, yID, x, y);

			double d2 = (x-rx)*(x-rx) + (y-ry)*(y-ry);
			double d = sqrt(d2);
			if(d<=1.*comRange)
			{
				hFitness->Fill(x,y,1);
			}
		}
	}

	// 3 coverage
	for(int i=0;i<binSizes[0];i++)
	for(int j=0;j<binSizes[1];j++)
	{
		int xID = i;
		int yID = j;
		double k_coverage = hFitness->GetBinContent(xID, yID);
		if(k_coverage>=3)
		{
			fitness_2 ++;
		}
	}

	// Normalization
	fitness_2 = fitness_2/double(binSizes[0]);
	fitness_2 = fitness_2/double(binSizes[1]);

	// Total fitness value
	double fitness = 0.5*fitness_1 + 0.5*fitness_2;

	return fitness;
}

double ToolFitnessFunction::ComputeFitness_Old(GAIndividual* ind)
{
	//
	// compute fitness
	//

	//
	// aspect 1 : inside map and outside buildings
	//
	double fitness_1 = 0;

	for(int i=0;i<ind->x_.size();i++)
	{
		int gID = i;
		bool IsGeneGood_insideMap_outsideBuildings 
			= toolRobotDeploymentEvenly_->Tool_IsRobotPositoinGood(ind->x_[gID],ind->y_[gID]);

		if(IsGeneGood_insideMap_outsideBuildings==true) {
			fitness_1 ++;
		}
		//else {
		//	fitness_1 --;
		//}
	}

	// Normalization 
	fitness_1 = fitness_1/double(ind->x_.size());

	//
	// 3-coverage or more
	//
	double fitness_2 = 0;

	for(int i=0;i<toolRobotDeploymentEvenly_->binSizes[0];i++)
	for(int j=0;j<toolRobotDeploymentEvenly_->binSizes[1];j++)
	{
		int xID = i;
		int yID = j;
		double x, y;
		toolRobotDeploymentEvenly_->Coverage_step1_->GetBinCentor2D(xID, yID, x, y);

		int NumberCovered = 0;
		for(int k=0;k<ind->x_.size();k++)
		{
			int rID = k; // robot ID
			double rx = ind->x_[rID];
			double ry = ind->x_[rID];

			double d2 = (x-rx)*(x-rx) + (y-ry)*(y-ry);
			double d = sqrt(d2);
			if(d<=1.*toolRobotDeploymentEvenly_->UAV_CommunicationRange_metre_)
			{
				NumberCovered ++;
			}
		}

		// 3 coverage
		if(NumberCovered>=3)
		{
			fitness_2 ++;
		}
	}

	// Normalization
	fitness_2 = fitness_2/double(toolRobotDeploymentEvenly_->binSizes[0]);
	fitness_2 = fitness_2/double(toolRobotDeploymentEvenly_->binSizes[1]);


	// Total fitness value
	double fitness = 0.5*fitness_1 + 0.5*fitness_2;

	return fitness;
}
