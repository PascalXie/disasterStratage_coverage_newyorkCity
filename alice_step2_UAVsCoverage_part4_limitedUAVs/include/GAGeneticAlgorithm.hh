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
		GAGeneticAlgorithm(string name, int GenerationSize, int IndividualSize, int CrowdPathTimeID);
		~GAGeneticAlgorithm();

    public:
		bool InitiateGeneticAlgorithm();
		void SetProbabilities(double MutationPr, double CrossOverPr, double ElitePr, double SurvivedPr);

    public:
		// Tool : for the limited-robot situation
		void SetGeneSize(int GeneSize);

    public:
		string name_;

		// Genetic Algorithm
		int GenerationSize_;

		// Generations
		vector<GAGeneration *> generations_;
		int IndividualSize_;

		// Individual
		bool IsGeneSizeSet_;
		int  GeneSize_; // Number of Robots for each individual

		// Probabilities
		double MutationPr_;
		double CrossOverPr_; 
		double ElitePr_;
		double SurvivedPr_;

		// Tool 
		ToolRobotDeploymentEvenly * toolRobotDeploymentEvenly_;
};
#endif

