#include "GAGeneticAlgorithm.hh"

//-------------------------
// Constructor
//-------------------------
GAGeneticAlgorithm::GAGeneticAlgorithm(string name, int GenerationSize, int IndividualSize)
:	name_(name),
	GenerationSize_(GenerationSize),
	IndividualSize_(IndividualSize)
{
	// Tool preparing
	int CoverageMode_ = 1; // mode 1 : coverage for localization
	toolRobotDeploymentEvenly_ = new ToolRobotDeploymentEvenly("DeployingForGA",CoverageMode_);

	// debug
	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug GAGeneticAlgorithm::GAGeneticAlgorithm"<<endl;
	cout<<"name "<<name_<<endl;
	cout<<"GenerationSize_ "<<GenerationSize_<<endl;
	cout<<"IndividualSize_ "<<IndividualSize_<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;
	// !debug
}

//-------------------------
// Destructor
//-------------------------
GAGeneticAlgorithm::~GAGeneticAlgorithm()
{}

//-------------------------
// Public
//-------------------------
bool GAGeneticAlgorithm::InitiateGeneticAlgorithm()
{
	//
	// step 1 : generate the first Generation
	//			in which all individuals are the same,
	//			and every individual owns genes refers to 
	//			the UAVs evenly deployed. 
	//
	GAGeneration * generation_0 = new GAGeneration("Generation_0",IndividualSize_,toolRobotDeploymentEvenly_);
	bool IsFristGenGood = generation_0->GenerateFirstGeneration_DeployingEvenly();

	generations_.push_back(generation_0);

	// debug
	if(!IsFristGenGood)
	{
		cout<<"An error occured in GAGeneticAlgorithm::InitiateGeneticAlgorithm"<<endl;
		cout<<"IsFristGenGood "<<IsFristGenGood<<endl;
		return false;
	}


	//
	// step 2 : generate other Generations
	//
	int EliteSize		= IndividualSize_ * ElitePr_;
	int MutationSize	= IndividualSize_ * MutationPr_;
	int CrossOverSize	= IndividualSize_ - EliteSize - MutationSize; 
	if(CrossOverSize<0) CrossOverSize=0;

	int SurvivedSize	= IndividualSize_ * SurvivedPr_;
	if(SurvivedSize==0||SurvivedSize<EliteSize)
	{
		cout<<"An error occured in GAGeneticAlgorithm::InitiateGeneticAlgorithm"<<endl;
		cout<<"SurvivedSize is wrong, Too small or smaller than Elite size : "<<SurvivedSize<<endl;
		return false;
	}

	/*
	// debug
	cout<<"EliteSize "<<EliteSize<<endl;
	cout<<"MutationSize "<<MutationSize<<endl;
	cout<<"CrossOverSize "<<CrossOverSize<<endl;
	// !debug
	*/

	for(int i=0;i<GenerationSize_-1;i++)
	{
		int gID = i + 1;
		cout<<"debug GAGeneticAlgorithm::InitiateGeneticAlgorithm : Generation "<<gID<<" Begin..."<<endl;

		GAGeneration *generation_current = new GAGeneration("Generation_"+to_string(gID),IndividualSize_,toolRobotDeploymentEvenly_);
		bool IsGenerationGood = generation_current->GenerateAGeneration(
				generations_[gID-1],
				MutationSize,
				CrossOverSize,
				EliteSize,
				SurvivedSize);

		// write
		generation_current->ToolWrite();

		// record
		generations_.push_back(generation_current);

		cout<<"debug GAGeneticAlgorithm::InitiateGeneticAlgorithm : Generation "<<gID<<" End."<<endl<<endl;
	}

	return true;
}

void GAGeneticAlgorithm::SetProbabilities(double MutationPr, double CrossOverPr, double ElitePr, double SurvivedPr)
{
	MutationPr_ = MutationPr;
	CrossOverPr_ = CrossOverPr;
	ElitePr_ = ElitePr;
	SurvivedPr_ = SurvivedPr;

	// normalization
	double total = MutationPr_ + CrossOverPr_ + ElitePr_;
	if(total<=0) total =1;

	MutationPr_		/= total;
	CrossOverPr_	/= total;
	ElitePr_		/= total;

	// debug
	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug GAGeneticAlgorithm::SetProbabilities"<<endl;
	cout<<"MutationPr_  "<<MutationPr_	<<endl;
	cout<<"CrossOverPr_ "<<CrossOverPr_ <<endl;
	cout<<"ElitePr_     "<<ElitePr_		<<endl;
	cout<<"SurvivedPr_  "<<SurvivedPr_<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;
	// !debug
}
