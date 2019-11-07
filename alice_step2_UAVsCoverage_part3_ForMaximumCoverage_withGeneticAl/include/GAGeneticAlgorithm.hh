#ifndef GAGENETICALGORITHM_HH
#define GAGENETICALGORITHM_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "GAGeneration.hh"

// tool
#include "ToolRobotDeploymentEvenly.hh"

using namespace std;

class GAGeneticAlgorithm
{
    public:
		GAGeneticAlgorithm(string name, int GenerationSize, int IndividualSize);
		~GAGeneticAlgorithm();

    public:
		bool InitiateGeneticAlgorithm();
		void SetProbabilities(double MutationPr, double CrossOverPr, double ElitePr, double SurvivedPr);

    public:
		string name_;

		// Genetic Algorithm
		int GenerationSize_;

		// Generations
		vector<GAGeneration *> generations_;
		int IndividualSize_;

		// Probabilities
		double MutationPr_;
		double CrossOverPr_; 
		double ElitePr_;
		double SurvivedPr_;

		// Tool 
		ToolRobotDeploymentEvenly * toolRobotDeploymentEvenly_;

};
#endif

