#ifndef GAINDIVIDUAL_HH
#define GAINDIVIDUAL_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

class GAIndividual
{
    public:
		GAIndividual(string name);
		~GAIndividual();

    public:
		string name_;

		// Genes
		vector<double> x_;
		vector<double> y_;

		// Fitness
		double Fitness_;
};
#endif
