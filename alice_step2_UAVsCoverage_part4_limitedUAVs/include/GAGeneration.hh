#ifndef GAGENERATION_HH
#define GAGENERATION_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <random>

// Genetic Algorithm
#include "GAIndividual.hh"

// tool for deployment
#include "ToolRobotDeploymentEvenly.hh"

// tool for fitness computation
#include "ToolFitnessFunction.hh"

using namespace std;

class GAGeneration
{
    public:
		GAGeneration(string name, int IndividualSize, ToolRobotDeploymentEvenly *toolRobotDeploymentEvenly);
		~GAGeneration();

    public:
		// for Genetic Algorithm
		// step 1 : Generate the first generation
		bool GenerateFirstGeneration_DeployingEvenly_UnLimitedRobots();
		bool GenerateFirstGeneration_DeployingEvenly_LimitedRobots(int GeneSize);

		// step 2 : Generate the other generations
		bool GenerateAGeneration(GAGeneration *ParentGeneration, double MutationSize, double CrossOverSize, double EliteSize, double SurvivedSize);

    public:
		// Tool 
		void ToolWrite();

    public:
		string name_;

		// individuals
		int IndividualSize_; // individual size for the first generation
		vector<GAIndividual *> individuals_;

		// for mutation
		int MutatedGeneSize_;

		// Tool
		ToolRobotDeploymentEvenly * toolRobotDeploymentEvenly_;

		// tool for fitness computation
		ToolFitnessFunction * fitnessFunction_;
};
#endif
