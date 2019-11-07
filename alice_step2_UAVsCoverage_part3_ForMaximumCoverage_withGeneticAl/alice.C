#include <iostream>
#include <string>
#include <fstream>
#include <random>
#include <vector>

#include "Units.hh"

#include "Histogram.hh"
#include "Histogram2D.hh"
#include "AStarNode.hh"
#include "PathPlanningAStar.hh"
#include "MapsForTime.hh"

#include "ToolRobotDeploymentEvenly.hh"

// Global Variables
default_random_engine GLOBAL_RANDOM_GENERATOR(time(0));

// Genetic Algorithm
#include "GAIndividual.hh"
#include "GAGeneration.hh"
#include "GAGeneticAlgorithm.hh"

using namespace std;

void test();

int main()
{
	cout<<"Hello "<<endl;

	// 
	// step 1 : GA
	// 
	int generationSize = 20;
	int individualSize = 100;
	GAGeneticAlgorithm GA("GA", generationSize, individualSize);
	double MutationPr	= 0.4;
	double CrossOverPr	= 0.5;
	double ElitePr		= 0.1;
	double SurvivedPr	= 0.7;
	GA.SetProbabilities(MutationPr, CrossOverPr, ElitePr, SurvivedPr);
	bool IsGAGood = GA.InitiateGeneticAlgorithm();

	return 1;
}

void test()
{

	//
	// step 0 : sort test
	//
	vector<double> test;
	test.push_back(4);
	test.push_back(1);
	test.push_back(0);

	for(int i=0;i<test.size();i++)
	{
		cout<<"ID "<<i<<", value "<<test[i]<<endl;
	}

	cout<<"after insert"<<endl;

	double v = 4;
	for(int i=0;i<test.size();i++)
	{
		if(v>test[i])
		{
			test.insert(test.begin()+i, v);
			break;
		}
	}

	for(int i=0;i<test.size();i++)
	{
		cout<<"ID "<<i<<", value "<<test[i]<<endl;
	}
}
