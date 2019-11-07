#ifndef TOOLFITNESSFUNCTION_HH
#define TOOLFITNESSFUNCTION_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "GAIndividual.hh"
#include "ToolRobotDeploymentEvenly.hh"

using namespace std;

class ToolFitnessFunction
{
    public:
		ToolFitnessFunction(string name, ToolRobotDeploymentEvenly *toolRobotDeploymentEvenly);
		~ToolFitnessFunction();

    public:
		double ComputeFitness(GAIndividual* ind);
		double ComputeFitness_Old(GAIndividual* ind);

    public:
		string name_;

		ToolRobotDeploymentEvenly * toolRobotDeploymentEvenly_;
};
#endif
