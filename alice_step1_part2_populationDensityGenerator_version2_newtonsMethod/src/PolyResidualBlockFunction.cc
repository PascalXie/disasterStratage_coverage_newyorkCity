#include "PolyResidualBlockFunction.hh"

//-------------------------
// Constructor
//-------------------------
PolyResidualBlockFunction::PolyResidualBlockFunction(string name, vector<double> observations, int SizeObservations, int SizeVariables, int SizeResiduals)
:	UserResidualBlockFunction(name+"_User"),
	name_(name),
	SizeObservations_(SizeObservations),
	SizeVariables_(SizeVariables), SizeResiduals_(SizeResiduals)
{
	observations_.clear();
	observations_ = observations;
}

//-------------------------
// Destructor
//-------------------------
PolyResidualBlockFunction::~PolyResidualBlockFunction()
{}

//-------------------------
// Public
//-------------------------
bool PolyResidualBlockFunction::ResidualFunction(vector<double> variables, vector<double> &residuals)
{
	//
	// step 0
	//
	if(variables.size()!=SizeVariables_)
	{
		cout<<"An error happend in PolyResidualBlockFunction::ResidualFunction"<<endl;
		return false;
	}
	if(observations_.size()!=SizeObservations_)
	{
		cout<<"An error happend in PolyResidualBlockFunction::ResidualFunction"<<endl;
		return false;
	}

	// observations_[0] : x, x coordinate of the building, safe zone for danger zone
	// observations_[1] : y, y coordinate of the building, safe zone for danger zone
	// observations_[2] : r, radius of the building, safe zone for danger zone
	// observations_[3] : n, name of the building(0), safe zone(2) for danger zone(1), 

	// varialbles[0] : tx, x coordinate of the target, which refers to the crowd
	// varialbles[1] : ty, y coordinate of the target, which refers to the crowd

	// residual = Gaussian(d, r)

	// informations about the building, safe zone for danger zone
	double x = observations_[0];
	double y = observations_[1];
	double r = observations_[2];
	double n = observations_[3];

	// target position
	double tx = variables[0];
	double ty = variables[1];

	// distance
	double d2 = (x-tx)*(x-tx)+(y-ty)*(y-ty);
	double d  = sqrt(d2);

	// residual
	double sigma = r;
	double weight_for_Zone = 1;

	if(n==0)		weight_for_Zone = 1e2; // building
	else if(n==2)	weight_for_Zone = 1e4; // safe zone
	else if(n==1) 	weight_for_Zone = 1e4; // danger zone

	double Gauss_part1 = 1./(sqrt(2.*PI)*sigma);
	double Gauss_part2 = exp(-1.*d2/(2.*sigma*sigma));
	double residual_0 = weight_for_Zone*Gauss_part1*Gauss_part2;


		/*
		// debug
		cout<<"Debug class PolyResidualBlockFunction::ResidualFunction"<<endl;
		cout<<"NodeID "<<NodeID<<endl;
		cout<<"xx "<<xx<<", xy "<<xy<<endl;
		cout<<"Anchor : "<<A_Anchor[0]<<", "<<A_Anchor[1]<<", "<<A_Anchor[2]<<endl;
		cout<<"Node : "<<A_Node[0]<<", "<<A_Node[1]<<", "<<A_Node[2]<<"; "<<S_node<<endl;
		cout<<"Power Observed : "<<S_ao<<", computed "<<S_a<<endl;
		*/

	residuals.clear();
	residuals.push_back(residual_0);

	/*
	// debug
	cout<<"Debug class PolyResidualBlockFunction::ResidualFunction"<<endl;
	cout<<"Debug Residual : Observation "<<distanceSquared<<" "<<ax<<" "<<ay<<endl;
	cout<<"Debug Residual : varialbles "<<xx<<" "<<xy<<", residual "<<residual_0<<endl;
	*/

	return true;
}

int PolyResidualBlockFunction::GetObervationsSize()
{
	return SizeObservations_;
}

void PolyResidualBlockFunction::ShowResidualFunction()
{
	// debug
	cout<<"debug Class PolyResidualBlockFunction : "<<name_<<endl;
	cout<<"Observations : "<<endl;
	for(int i=0; i<observations_.size(); i++)
	{
		cout<<"    ID "<<i<<"; Observation "<<observations_[i]<<endl;
	}
}


void PolyResidualBlockFunction::SetToolSignalPowerGenerator(ToolSignalPowerGenerator *spg)
{
	spg_ = spg;
}
