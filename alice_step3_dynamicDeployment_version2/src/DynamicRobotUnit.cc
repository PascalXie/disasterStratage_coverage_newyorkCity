#include "DynamicRobotUnit.hh"

//-------------------------
// Constructor
//-------------------------
DynamicRobotUnit::DynamicRobotUnit(string name, double x, double y, double time, double Speed)
:	name_(name)
{
	Status_ = "Free";
	Speed_ = Speed;

	// 
	MaximumBattery_ = 1.*hour;
	Battery_ = MaximumBattery_;
	//Battery_ = 100.*second;

	ThePassedPath_x_.clear();
	ThePassedPath_y_.clear();
	ThePassedPath_timeStamp_.clear();
	ThePassedPath_status_.clear();

	ThePassedPath_x_.push_back(x);
	ThePassedPath_y_.push_back(y);
	ThePassedPath_timeStamp_.push_back(time);
	ThePassedPath_status_.push_back(Status_);

}

//-------------------------
// Destructor
//------------------------- 
DynamicRobotUnit::~DynamicRobotUnit()
{}

//-------------------------
// Public
//-------------------------
void DynamicRobotUnit::AddPlannedPath_coorAndTime(double x, double y, double CurrentTime_)
{
	// set time
	double curentTime = 0; // previous time
	double cx = 0;
	double cy = 0;

	if(ThePlannedPath_x_.size()>=1)
	{
		curentTime = ThePlannedPath_timeStamp_[ThePlannedPath_timeStamp_.size()-1];
		cx = ThePlannedPath_x_[ThePlannedPath_timeStamp_.size()-1];
		cy = ThePlannedPath_y_[ThePlannedPath_timeStamp_.size()-1];
	}
	else 
	{
		curentTime = CurrentTime_;
		cx = ThePassedPath_x_[ThePassedPath_timeStamp_.size()-1];
		cy = ThePassedPath_y_[ThePassedPath_timeStamp_.size()-1];
	}

	// distance
	double d2 = (cx-x)*(cx-x) + (cy-y)*(cy-y);
	double d = sqrt(d2);

	double thisTime = curentTime + d/Speed_;
	ThePlannedPath_timeStamp_.push_back(thisTime);

	// add a point from  planned path
	ThePlannedPath_x_.push_back(x);
	ThePlannedPath_y_.push_back(y);

}

//-------------------------
// Public
//-------------------------
void DynamicRobotUnit::MoveATimeInterval(double currentTimeStamp, double timeInterval)
{
	// part 1 : battery consumption
	if(Status_ == "Recharging")
	{
		Battery_ = Battery_ + timeInterval/(1.*hour)*MaximumBattery_; // 1 hour to get full energy
		if(Battery_ >= MaximumBattery_) {
			Battery_ = MaximumBattery_;
			Status_ = "Free";
		}
	}
	else
	{
		Battery_ -= timeInterval;
		if(Battery_<=0) 
		{
			Status_ == "Dead";
		}
	}

	// part 2 : checking status

	//  "Flying_Work" and "Flying_Recharge"
	// set path
	if(Status_=="Flying_Work"||Status_=="Flying_Recharge")
	{
		vector<double> ThePlannedPath_x;
		vector<double> ThePlannedPath_y;
		vector<double> ThePlannedPath_timeStamp; 
		ThePlannedPath_x.clear();
		ThePlannedPath_y.clear();
		ThePlannedPath_timeStamp.clear();

		for(int i=0;i<ThePlannedPath_timeStamp_.size();i++)
		{
			int pID = i;
			double x = ThePlannedPath_x_[pID];
			double y = ThePlannedPath_y_[pID];
			double t = ThePlannedPath_timeStamp_[pID];
			if(t>currentTimeStamp)
			{
				ThePlannedPath_x.push_back(x);
				ThePlannedPath_y.push_back(y);
				ThePlannedPath_timeStamp.push_back(t);
			}
			else
			{
				ThePassedPath_x_.push_back(x);
				ThePassedPath_y_.push_back(y);
				ThePassedPath_timeStamp_.push_back(t);
				ThePassedPath_status_.push_back(Status_);
			}
		}

		ThePlannedPath_x_.clear();
		ThePlannedPath_y_.clear();
		ThePlannedPath_timeStamp_.clear();
		ThePlannedPath_x_			= ThePlannedPath_x;
		ThePlannedPath_y_ 			= ThePlannedPath_y;
		ThePlannedPath_timeStamp_	= ThePlannedPath_timeStamp;
	}

	//if(Status_=="Working")
	//{
	//	if(ThePassedPath_status_[ThePassedPath_status_.size()-1]=="Flying_Work")
	//	{
	//		double x = ThePassedPath_x_[ThePassedPath_x_.size()-1];
	//		double y = ThePassedPath_y_[ThePassedPath_x_.size()-1];
	//		ThePassedPath_x_.push_back(x);
	//		ThePassedPath_y_.push_back(y);
	//		ThePassedPath_timeStamp_.push_back(currentTimeStamp);
	//		ThePassedPath_status_.push_back(Status_);
	//	}
	//}

	//if(Status_=="Recharging")
	//{
	//	if(ThePassedPath_status_[ThePassedPath_status_.size()-1]=="Flying_Recharge")
	//	{
	//		double x = ThePassedPath_x_[ThePassedPath_x_.size()-1];
	//		double y = ThePassedPath_y_[ThePassedPath_x_.size()-1];
	//		ThePassedPath_x_.push_back(x);
	//		ThePassedPath_y_.push_back(y);
	//		ThePassedPath_timeStamp_.push_back(currentTimeStamp);
	//		ThePassedPath_status_.push_back(Status_);
	//	}
	//}

	if(ThePassedPath_status_[ThePassedPath_status_.size()-1]!=Status_)
	{
		double x = ThePassedPath_x_[ThePassedPath_x_.size()-1];
		double y = ThePassedPath_y_[ThePassedPath_x_.size()-1];
		ThePassedPath_x_.push_back(x);
		ThePassedPath_y_.push_back(y);
		ThePassedPath_timeStamp_.push_back(currentTimeStamp);
		ThePassedPath_status_.push_back(Status_);
	}

}
