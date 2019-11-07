#include "GAGeneration.hh"

//-------------------------
// Constructor
//-------------------------
GAGeneration::GAGeneration(string name, int IndividualSize, ToolRobotDeploymentEvenly *toolRobotDeploymentEvenly)
:	name_(name),
	IndividualSize_(IndividualSize),
	toolRobotDeploymentEvenly_(toolRobotDeploymentEvenly)
{
	// Set Fitness function
	fitnessFunction_ = new ToolFitnessFunction("Fitness Function",toolRobotDeploymentEvenly_);

	/*
	// debug
	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug GAGeneration::GAGeneration"<<endl;
	cout<<"name "<<name_<<endl;
	cout<<"IndividualSize_ "<<IndividualSize_<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;
	*/
}

//-------------------------
// Destructor
//-------------------------
GAGeneration::~GAGeneration()
{}

//-------------------------
// Public
// step 1 for Genetic Algorithm
//-------------------------
bool GAGeneration::GenerateFirstGeneration_DeployingEvenly()
{
	// Generate the first generation
	for(int i=0; i<IndividualSize_; i++)
	{
		int iID = i; // individual id
		GAIndividual * ind = new GAIndividual("Individual_FirstGen"+to_string(iID));

		// set genes
		for(int j=0;j<toolRobotDeploymentEvenly_->robotCoor_x_2_.size();j++)
		{
			int gID = j; // gene id
			ind->x_.push_back(toolRobotDeploymentEvenly_->robotCoor_x_2_[gID]);
			ind->y_.push_back(toolRobotDeploymentEvenly_->robotCoor_y_2_[gID]);
		}

		individuals_.push_back(ind);
	}

	/*
	// debug
	for(int i=0;i<individuals_[1]->x_.size();i++)
	{
		cout<<"debug GAGeneration::GenerateFirstGeneration_DeployingEvenly"<<endl;
		cout<<"Gene ID "<<i<<": x y "<<individuals_[1]->x_[i]/m<<" "<<individuals_[1]->y_[i]/m<<endl;
	}
	// !debug
	*/


	return true;
}

//-------------------------
// Public
// step 2 : Generate the other generations
//-------------------------
bool GAGeneration::GenerateAGeneration(GAGeneration *ParentGeneration, double MutationSize, double CrossOverSize, double EliteSize, double SurvivedSize)
{
	// random generator
	default_random_engine RANDOM_GENERATOR(time(0));

	// for mutation
	MutatedGeneSize_ = 1;

	//
	// step 1 : generate individuals for THIS generation
	//

	//
	// step 1 - 1 : generate  Elites
	//
	for(int i=0; i<EliteSize; i++)
	{
		int eID = i; // elite id
		GAIndividual * ind = new GAIndividual("Elite_"+to_string(eID));

		// set genes
		for(int j=0;j<ParentGeneration->individuals_[eID]->x_.size();j++)
		{
			int gID = j;
			ind->x_.push_back(ParentGeneration->individuals_[eID]->x_[gID]);
			ind->y_.push_back(ParentGeneration->individuals_[eID]->y_[gID]);
		}
		
		// compute fitness 
		ind->Fitness_ = fitnessFunction_->ComputeFitness(ind);

		// Add individual
		individuals_.push_back(ind);

		/*
		// debug
		cout<<"Elite "<<eID<<" Fitness "<<ind->Fitness_<<endl;
		cout<<"Size "<<individuals_.size()<<endl;
		*/
	}

	//
	// step 1 - 2 : generate Mutations
	//
	for(int i=0; i<MutationSize; i++)
	{
		int mID = i; // mutation id
		uniform_int_distribution<int> IDgen(0,SurvivedSize-1);
		int parent_iID = IDgen(RANDOM_GENERATOR);

		GAIndividual * ind = new GAIndividual("Mutation_"+to_string(mID));
		// set genes
		for(int j=0;j<ParentGeneration->individuals_[parent_iID]->x_.size();j++)
		{
			int gID = j;
			ind->x_.push_back(ParentGeneration->individuals_[parent_iID]->x_[gID]);
			ind->y_.push_back(ParentGeneration->individuals_[parent_iID]->y_[gID]);
		}

		// mutation
		for(int j=0;j<MutatedGeneSize_;j++)
		{
			uniform_int_distribution<int> gIDGen(0,ind->x_.size()-1);
			int gID = gIDGen(RANDOM_GENERATOR);

			double px = ind->x_[gID];
			double py = ind->y_[gID];
			normal_distribution<double> Gaussx(px, 8.*2.355*toolRobotDeploymentEvenly_->binWidths[0]);
			normal_distribution<double> Gaussy(py, 8.*2.355*toolRobotDeploymentEvenly_->binWidths[1]);
			ind->x_[gID] = Gaussx(RANDOM_GENERATOR);
			ind->y_[gID] = Gaussy(RANDOM_GENERATOR);
		}
		

		// compute fitness 
		ind->Fitness_ = fitnessFunction_->ComputeFitness(ind);
		//ind->Fitness_ = fitnessFunction_->ComputeFitness_Old(ind);

		// Add individual
		bool IsTheSmallest = true;
		for(int j=0;j<individuals_.size();j++)
		{
			int iID = j;
			if(ind->Fitness_>individuals_[iID]->Fitness_)
			{
				individuals_.insert(individuals_.begin()+j, ind);
				IsTheSmallest = false;
				break;
			}
		}
		if(IsTheSmallest)
		{
			individuals_.push_back(ind);
		}

		/*
		// debug
		cout<<"Mutation "<<mID<<" Fitness "<<ind->Fitness_<<endl;
		cout<<"Size "<<individuals_.size()<<endl;
		*/
	}

	//
	// step 1 - 3 : generate Cross Over
	//
	for(int i=0; i<CrossOverSize; i++)
	{
		int coID = i; // crossover id
		uniform_int_distribution<int> IDgen(0,SurvivedSize-1);
		int ps_iID[2] = {0,0};
		ps_iID[0] = IDgen(RANDOM_GENERATOR);
		ps_iID[1] = IDgen(RANDOM_GENERATOR);

		// new generation
		GAIndividual * ind = new GAIndividual("CrossOver_"+to_string(coID));

		// set genes
		uniform_int_distribution<int> pIDGen(0,1);
		for(int j=0;j<ParentGeneration->individuals_[ps_iID[0]]->x_.size();j++)
		{
			int gID = j;
			int whichParent = pIDGen(RANDOM_GENERATOR);
			double x = ParentGeneration->individuals_[ps_iID[whichParent]]->x_[gID];
			double y = ParentGeneration->individuals_[ps_iID[whichParent]]->y_[gID];
			ind->x_.push_back(x);
			ind->y_.push_back(y);
		}

		// compute fitness 
		ind->Fitness_ = fitnessFunction_->ComputeFitness(ind);
		//ind->Fitness_ = fitnessFunction_->ComputeFitness_Old(ind);

		// Add individual
		bool IsTheSmallest = true;
		for(int j=0;j<individuals_.size();j++)
		{
			int iID = j;
			if(ind->Fitness_>individuals_[iID]->Fitness_)
			{
				individuals_.insert(individuals_.begin()+j, ind);
				IsTheSmallest = false;
				break;
			}
		}
		if(IsTheSmallest)
		{
			individuals_.push_back(ind);
		}

		/*
		// debug
		cout<<"Cross over "<<coID<<" Fitness "<<ind->Fitness_<<endl;
		cout<<"Size "<<individuals_.size()<<endl;
		// !debug
		*/
	}

	/*
	// debug
	cout<<"Debug GAGeneration::GenerateAGeneration"<<endl;
	cout<<"MutationSize "<<MutationSize<<endl;
	cout<<"CrossOverSize "<<CrossOverSize<<endl;
	cout<<"EliteSize "<<EliteSize<<endl;
	cout<<"SurvivedSize "<<SurvivedSize<<endl;
	for(int i=0;i<individuals_.size();i++)
	{
		cout<<"iID "<<i<<": Fitness "<<individuals_[i]->Fitness_<<endl;
	}
	// !debug
	*/

	return true;
}

//-------------------------
// Public
// Tool
//-------------------------
void GAGeneration::ToolWrite()
{
	ofstream write("data_"+name_+".txt");

	for(int k=0;k<individuals_.size();k++)
	{
		int iID = k;
		for(int i=0;i<individuals_[iID]->x_.size();i++)
		{
			int gID = i;

			write<<iID<<" "<<gID<<" "<<individuals_[iID]->Fitness_<<" "<<individuals_[iID]->x_[gID]<<" "<<individuals_[iID]->y_[gID]<<endl;
		}
	}

	write.close();
}
