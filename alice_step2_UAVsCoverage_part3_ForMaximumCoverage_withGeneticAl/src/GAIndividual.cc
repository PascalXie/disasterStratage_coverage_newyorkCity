#include "GAIndividual.hh"

//-------------------------
// Constructor
//-------------------------
GAIndividual::GAIndividual(string name)
:	name_(name)
{
	// Fitness
	Fitness_ = 0;

	/*
	// debug
	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug GAIndividual::GAIndividual"<<endl;
	cout<<"name "<<name_<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;
	*/
}

//-------------------------
// Destructor
//-------------------------
GAIndividual::~GAIndividual()
{}

//-------------------------
// Public
//-------------------------
