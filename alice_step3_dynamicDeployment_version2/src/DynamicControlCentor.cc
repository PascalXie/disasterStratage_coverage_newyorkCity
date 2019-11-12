#include "DynamicControlCentor.hh"

//-------------------------
// Constructor
//-------------------------
DynamicControlCentor::DynamicControlCentor(string name, string filename_BuildingHis2D, string FileName_obstacle, string FileName_obstacleBinary)
:	name_(name)
{
	IsStep1_0Good_	= false;
	IsStep1_11Good_ = false;
	IsStep1_12Good_ = false;
	IsStep1_4Good_	= false;
	IsStep1_5Good_	= false;
	IsStep2_1Good_	= false;

	// step 1 - 0 : import histogram parameters
	IsStep1_0Good_ = Tool_ReadHistogramsParameters_ForBuildings(filename_BuildingHis2D);

	// step 1 - 1 : import histogram data for buildings, which are obstacles
	IsStep1_11Good_ = Tool_ReadDistributionObstacles(FileName_obstacle);
	IsStep1_12Good_ = Tool_ReadDistributionObstacles_binary(FileName_obstacleBinary);

	// Prepare a hist 2d with no obstacle, which means all the bins are 0, referes to the UAV flying in the air
	hObstacles_NoObstacle_	= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	// step 1 -2 : set speed by default
	speed_crowd_ = 0.3*m/second;
	speed_robot_ = 15.*m/second;

	// step 1 - 3 : set path planning
	//pp_ = new PathPlanningAStar("PathPlanning_DyncamicDeploying",hObstacles_binary_);

	// step 1 - 4 : import robot deployment
	string filename_1_4 = "../../alice_step2_UAVsCoverage_part4_limitedUAVs/build/data_Generation_0.txt";
	IsStep1_4Good_ = Tool_ReadRobotDeployment(filename_1_4);

	// debug
	cout<<"----------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"debug DynamicControlCentor::DynamicControlCentor"<<endl;
	cout<<"speed_crowd_ "<<speed_crowd_/(m/second)<<" (m/second); time lapse per bin "<<binWidths[0]/speed_crowd_/minute<<" minutes"<<endl;
	cout<<"speed_robot_ "<<speed_robot_/(m/second)<<" (m/second); time lapse per bin "<<binWidths[0]/speed_robot_/minute<<" minutes"<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------"<<endl;
}

//-------------------------
// Destructor
//-------------------------
DynamicControlCentor::~DynamicControlCentor()
{}

//-------------------------
// Public
// step 2 : initiate
//-------------------------
bool DynamicControlCentor::InitiateDynamicControl()
{
	// step 0 : check
	if(!IsStep1_0Good_||!IsStep1_11Good_||!IsStep1_12Good_||!IsStep1_4Good_||!IsStep1_5Good_)
	{
		cout<<"An error occured in DynamicControlCentor::InitiateDynamicControl"<<endl;
		cout<<"Step 0 : check, Not All Data have been imported"<<endl;
		return false;
	}
	if(!IsStep2_1Good_)
	{
		cout<<"An error occured in DynamicControlCentor::InitiateDynamicControl"<<endl;
		cout<<"Step 0 : check, Time intervals had not been set"<<endl;
		return false;
	}

	// debug
	cout<<"debug DynamicControlCentor::InitiateDynamicControl"<<endl;
	for(int i=0;i<SafeZone_x_.size();i++)
	{
		int szID = i;
		cout<<"SafeZone ID "<<szID<<", x y "<<SafeZone_x_[szID]/m<<" m, "<<SafeZone_y_[szID]/m<<" m"<<endl;
	}
	for(int i=0;i<dep_x_.size();i++)
	{
		int dID = i;
		cout<<"Deployed Point ID "<<dID<<", x y "<<dep_x_[dID]/m<<" m, "<<dep_y_[dID]/m<<" m"<<endl;
	}
	cout<<"!debug DynamicControlCentor::InitiateDynamicControl"<<endl<<endl;
	// !debug

	/*
	// testing : !!!!!!!!!!!!!
	DynamicRobotUnit *rob = new DynamicRobotUnit("test",SafeZone_x_[0],SafeZone_y_[0],0,speed_robot_);
	robots_.push_back(rob);
	// !testing : !!!!!!!!!!!!!
	*/

	/*
	// testing : !!!!!!!!!!!!!
	DynamicRobotUnit *rob = new DynamicRobotUnit("test",dep_x_[0],dep_y_[0],0,speed_robot_);
	rob->Status_ = "Flying_Work";
	rob->ThePassedPath_status_[0] = rob->Status_;
	rob->Destination_x_ = dep_x_[0];
	rob->Destination_y_ = dep_y_[0];
	robots_.push_back(rob);
	// !testing : !!!!!!!!!!!!!
	*/

	/*
	// testing : !!!!!!!!!!!!!
	DynamicRobotUnit *rob = new DynamicRobotUnit("test",SafeZone_x_[0],SafeZone_y_[0],0,speed_robot_);
	rob->Status_ = "Flying_Recharge";
	rob->ThePassedPath_status_[0] = rob->Status_;
	rob->Destination_x_ = SafeZone_x_[0];
	rob->Destination_y_ = SafeZone_y_[0];
	robots_.push_back(rob);
	// !testing : !!!!!!!!!!!!!
	*/


	// step 1 : loop
	// step 1 - 1 : set time interval : the time lapse a robot penetrate a map bin
	//				x axis only, assume binWidths[0] == binWidths[1]
	TimeInterval_ = binWidths[0]/speed_robot_; 
	int LoopSize = MaximumTimeInterval_/TimeInterval_;
	cout<<"LoopSize "<<LoopSize/second<<" s"<<endl;
	cout<<"TimeInterval_ "<<TimeInterval_/second<<" s"<<endl;

	// step 1 - 2 : loop
	for(int i=0;i<LoopSize;i++)
	{
		int loopID = i;
		CurrentTime_ = 0. + (double(loopID)+0.5)*TimeInterval_;
		cout<<"loop ID : "<<i<<", CurrentTime_ "<<CurrentTime_/second<<" s"<<endl;

		// step 1 - 2 - 1 : loop deployed points
		for(int j=0;j<dep_x_.size();j++)
		{
			CheckingDeployment(dep_x_[j], dep_y_[j]);
		}

		// step 1 - 2 - 2 : loop for robots, robot selfchecking
		for(int j=0;j<robots_.size();j++)
		{
			int rID = j;
			CheckingRobot(rID);
		}

		// step 1 - 2 - 3 : loop for robots, excuting by the status
		for(int j=0;j<robots_.size();j++)
		{
			int rID = j;
			robots_[rID]->MoveATimeInterval(CurrentTime_, TimeInterval_);
		}
	}

	// debug
	for(int i=0;i<robots_.size();i++)
	{
		DynamicRobotUnit * rob = robots_[i];

		cout<<"- - - - "<<endl;
		cout<<"robot "<<i<<", name "<<rob->name_<<", status "<<rob->Status_<<", battery "<<rob->Battery_/second<<" s"<<endl;

		for(int j=0;j<rob->ThePassedPath_x_.size();j++)
		{
			cout<<rob->name_;
			cout<<" ThePassedPath "<<rob->ThePassedPath_x_[j]/m<<" m, ";
			cout<<rob->ThePassedPath_y_[j]/m<<" m, time ";
			cout<<rob->ThePassedPath_timeStamp_[j]/second<<" s, status ";
			cout<<rob->ThePassedPath_status_[j]<<endl;
		}
		for(int j=0;j<rob->ThePlannedPath_x_.size();j++)
		{
			cout<<rob->name_;
			cout<<" ThePlannedPath "<<rob->ThePlannedPath_x_[j]/m<<" m, ";
			cout<<rob->ThePlannedPath_y_[j]/m<<" m, time ";
			cout<<rob->ThePlannedPath_timeStamp_[j]/second<<" s"<<endl;
		}
		cout<<"- - - - "<<endl;
	}
}

//-------------------------
// Private
// step 2 - 1 - 2 - 1 : checking robots
//-------------------------
bool DynamicControlCentor::CheckingRobot(int robotID)
{
	// check robot 
	DynamicRobotUnit *rob = robots_[robotID];
	double rx = rob->ThePassedPath_x_[rob->ThePassedPath_x_.size()-1];
	double ry = rob->ThePassedPath_y_[rob->ThePassedPath_x_.size()-1];

	//
	// part 1 : checking battery
	//
	double Battery = rob->Battery_;

	// find a safe zone to go in case of the low-battery situation
	int szID = FindTheNearestSafeZone(rx, ry);

	// compute Manhhatan distance
	double ManhhDis = abs(SafeZone_x_[szID]-rx) + abs(SafeZone_y_[szID]-ry);

	// compute tiem
	double ManhhTime = ManhhDis/rob->Speed_;

	// make decision
	if(rob->Status_!="Recharging"&&rob->Status_!="Flying_Recharge"&&rob->Battery_<=ManhhTime)
	{
		rob->Status_ = "Flying_Recharge";

		rob->ThePlannedPath_x_.clear();
		rob->ThePlannedPath_y_.clear();
		rob->ThePlannedPath_timeStamp_.clear();

		// Path Planning 
		double xStart = rob->ThePassedPath_x_[rob->ThePassedPath_x_.size()-1];
		double yStart = rob->ThePassedPath_y_[rob->ThePassedPath_x_.size()-1];
		int xSID = hObstacles_->GetBinIDX(xStart);
		int ySID = hObstacles_->GetBinIDY(yStart);
		int xEID = hObstacles_->GetBinIDX(SafeZone_x_[szID]);
		int yEID = hObstacles_->GetBinIDY(SafeZone_y_[szID]);

		PathPlanningAStar pp("PathPlanning_DyncamicDeploying",hObstacles_NoObstacle_); // for UAV
		pp.SetStartNode(new AStarNode("start",xSID,ySID));
		pp.SetEndNode  (new AStarNode("end1", xEID,yEID));
		pp.SetObstacleValue(0.5); // useless
		pp.InitiatPathPlanning();

		// get path : part1 claer
		rob->ThePlannedPath_x_.clear();
		rob->ThePlannedPath_y_.clear();
		rob->ThePlannedPath_timeStamp_.clear();
		// get path : part2 add path
		for(int i=0;i<pp.Path_.size();i++)
		{
			double x,y;
			hObstacles_->GetBinCentor2D(pp.Path_[i]->xID_,pp.Path_[i]->yID_,x,y);
			rob->AddPlannedPath_coorAndTime(x,y,CurrentTime_);
		}
		// get path : part3 add destination to path
		rob->AddPlannedPath_coorAndTime(SafeZone_x_[szID],SafeZone_y_[szID], CurrentTime_);
		// get path : part4 set destination
		rob->Destination_x_ = SafeZone_x_[szID];
		rob->Destination_y_ = SafeZone_y_[szID];

		return true;
	}

	//
	// part 2 : checking status
	//
	if(rob->Battery_ > ManhhTime)
	{
		// part 2 - 1 : "Flying_Work"
		if(rob->Status_ == "Flying_Work")
		{
			if(rob->Destination_x_==rx&&rob->Destination_y_==ry)
			{
				cout<<"flying work"<<endl;
				rob->Status_ = "Working";

				rob->ThePlannedPath_x_.clear();
				rob->ThePlannedPath_y_.clear();
				rob->ThePlannedPath_timeStamp_.clear();
			}
		}
	}

	// part 2 - 2 : "Flying_Recharge"
	if(rob->Status_ == "Flying_Recharge")
	{
		if(rob->Destination_x_==rx&&rob->Destination_y_==ry)
		{
			cout<<"flying recharge"<<endl;
			rob->Status_ = "Recharging";

			rob->ThePlannedPath_x_.clear();
			rob->ThePlannedPath_y_.clear();
			rob->ThePlannedPath_timeStamp_.clear();
		}
	}

	return true;
}

//-------------------------
// Private
// step 2 - 1 - 2 - 1 : checking deployment, deployed points
//-------------------------
bool DynamicControlCentor::CheckingDeployment(double dep_x, double dep_y)
{
	/*
	// debug
	// check is the deployed point inside the obstacle
	int dep_x_id = hObstacles_binary_->GetBinIDX(dep_x);
	int dep_y_id = hObstacles_binary_->GetBinIDX(dep_y);
	bool isNObstacle = hObstacles_binary_->GetBinContent(dep_x_id, dep_y_id);
	cout<<"isNObstacle "<<isNObstacle<<endl;
	// !debug
	*/

	//
	// part 1 : check all deployed points
	//
	// has the current deployed point been dispatched with robots
	int DispatchedRobotSize = 0;
	vector<int> DispatchedRobotsID;

	vector<int> WorkingRobotsID, FlyingWorkRobotsID;

	for(int i=0;i<robots_.size();i++)
	{
		int rID = i;

		if(robots_[rID]->Status_=="Working"||robots_[rID]->Status_=="Flying_Work")
		{
			if(robots_[rID]->Destination_x_==dep_x&&robots_[rID]->Destination_y_==dep_y)
			{
				DispatchedRobotSize ++;
				DispatchedRobotsID.push_back(rID);

				// for situatin 2
				if(robots_[rID]->Status_=="Working")
				{
					WorkingRobotsID.push_back(rID);
				}
				if(robots_[rID]->Status_=="Flying_Work")
				{
					FlyingWorkRobotsID.push_back(rID);
				}
				// !for situatin 2
			}
		}
	}

	//
	// situation 1 : No robot that has been dispatched, send one
	//
	if(DispatchedRobotSize==0) 
	{
		// find a robot with the minimum distance to this deployed point
		FindAndSentARobotHere(dep_x, dep_y);
	}
	//
	// end situation 1 : No robot that has been dispatched, send one
	//

	//
	// situation 2 : there are robots working, but they are going to fly back
	//
	if(DispatchedRobotSize>0&&WorkingRobotsID.size()>0&&FlyingWorkRobotsID.size()==0) 
	{
		bool situ3_areWorkingRobsBatteryEnough = false; 

		for(int i=0;i<WorkingRobotsID.size();i++)
		{
			DynamicRobotUnit *rob = robots_[WorkingRobotsID[i]];
			double rx = rob->ThePassedPath_x_[rob->ThePassedPath_x_.size()-1];
			double ry = rob->ThePassedPath_y_[rob->ThePassedPath_x_.size()-1];
			// part 1 : checking battery
			double Battery = rob->Battery_;
			// find a safe zone to go in case of the low-battery situation
			int szID = FindTheNearestSafeZone(rx, ry);
			// compute Manhhatan distance
			double ManhhDis = abs(SafeZone_x_[szID]-rx) + abs(SafeZone_y_[szID]-ry);
			// compute tiem
			double ManhhTime = ManhhDis/rob->Speed_;

			if(Battery > 2.2*ManhhTime)
			{
				situ3_areWorkingRobsBatteryEnough = true;
			}
		}

		// make decision
		if(!situ3_areWorkingRobsBatteryEnough)
		{
			// find a robot with the minimum distance to this deployed point
			FindAndSentARobotHere(dep_x, dep_y);
		}
	}

	//
	// end situation 2 : there are robots working, but they are going to fly back
	//

	return true;
}

//-------------------------
// Public
// step 2 
//-------------------------
int DynamicControlCentor::FindTheNearestSafeZone(double x, double y)
{
	// find the nearest safe zone and return its ID
	int nearestID = 0;
	double smallestDis = 0;

	for(int i=0;i<SafeZone_x_.size();i++)
	{
		int szID = i;
		double szx = SafeZone_x_[szID];
		double szy = SafeZone_y_[szID];

		double d2 = (szx-x)*(szx-x) + (szy-y)*(szy-y);
		double d = sqrt(d2);

		if(szID==0)
		{
			nearestID = 0;
			smallestDis = d;
		}
		else
		{
			if(d<smallestDis)
			{
				nearestID = szID;
				smallestDis = d;
			}
		}
	}

	return nearestID;
}

//-------------------------
// Public
// step 2
//-------------------------
bool DynamicControlCentor::FindAndSentARobotHere(double dep_x, double dep_y)
{
	// find a robot with the minimum distance to this deployed point
	int rID_toBeUsed = -1;
	double minDis = 1e10;

	for(int i=0;i<robots_.size();i++)
	{
		int rID = i;
		if(robots_[rID]->Status_=="Free")
		{
			double x = robots_[rID]->ThePassedPath_x_[robots_[rID]->ThePassedPath_x_.size()-1];
			double y = robots_[rID]->ThePassedPath_y_[robots_[rID]->ThePassedPath_x_.size()-1];
			double d2 = (x-dep_x)*(x-dep_x) + (y-dep_y)*(y-dep_y);
			double d  = sqrt(d2);
			if(d<minDis)
			{
				minDis = d;
				rID_toBeUsed = rID;
			}
		}
	}

	/*
	// debug
	if(rID_toBeUsed>=0)
	{
		cout<<"robot ID that is to Be Used "<<rID_toBeUsed<<endl;
	}
	else if(rID_toBeUsed==-1)
	{
		cout<<"robot ID means that NO robot found that could be used "<<rID_toBeUsed<<endl;
	}
	// !debug
	*/

	// find a robot
	if(rID_toBeUsed>=0)
	{
		robots_[rID_toBeUsed]->Status_ = "Flying_Work";
		robots_[rID_toBeUsed]->Destination_x_ = dep_x;
		robots_[rID_toBeUsed]->Destination_y_ = dep_y;

		// Path Planning 
		double xStart = robots_[rID_toBeUsed]->ThePassedPath_x_[robots_[rID_toBeUsed]->ThePassedPath_x_.size()-1];
		double yStart = robots_[rID_toBeUsed]->ThePassedPath_y_[robots_[rID_toBeUsed]->ThePassedPath_x_.size()-1];
		int xSID = hObstacles_->GetBinIDX(xStart);
		int ySID = hObstacles_->GetBinIDY(yStart);
		int xEID = hObstacles_->GetBinIDX(dep_x);
		int yEID = hObstacles_->GetBinIDY(dep_y);

		//PathPlanningAStar pp("PathPlanning_DyncamicDeploying",hObstacles_binary_); // for vihecle
		PathPlanningAStar pp("PathPlanning_DyncamicDeploying",hObstacles_NoObstacle_); // for UAV
		pp.SetStartNode(new AStarNode("start",xSID,ySID));
		pp.SetEndNode  (new AStarNode("end1", xEID,yEID));
		pp.SetObstacleValue(0.5); // useless
		pp.InitiatPathPlanning();

		// get path : part1 claer
		robots_[rID_toBeUsed]->ThePlannedPath_x_.clear();
		robots_[rID_toBeUsed]->ThePlannedPath_y_.clear();
		robots_[rID_toBeUsed]->ThePlannedPath_timeStamp_.clear();
		// get path : part2 add path
		for(int i=0;i<pp.Path_.size();i++)
		{
			double x,y;
			hObstacles_->GetBinCentor2D(pp.Path_[i]->xID_,pp.Path_[i]->yID_,x,y);
			robots_[rID_toBeUsed]->AddPlannedPath_coorAndTime(x,y, CurrentTime_);
		}
		// get path : part3 add destination to path
		robots_[rID_toBeUsed]->AddPlannedPath_coorAndTime(dep_x,dep_y, CurrentTime_);
		// get path : part4 set destination
		robots_[rID_toBeUsed]->Destination_x_ = dep_x;
		robots_[rID_toBeUsed]->Destination_y_ = dep_y;

	}
	else if(rID_toBeUsed==-1)
	{
		// no more robot, generate one
		int nearestSZID = FindTheNearestSafeZone(dep_x,dep_y);

		int robID = robots_.size();
		DynamicRobotUnit *rob = new DynamicRobotUnit("Robot_"+to_string(robID), SafeZone_x_[nearestSZID], SafeZone_y_[nearestSZID], CurrentTime_, speed_robot_);

		rob->Status_ = "Flying_Work";
		rob->Destination_x_ = dep_x;
		rob->Destination_y_ = dep_y;

		/*
		// debug
		cout<<"a robot has been generated, name "<<rob->name_<<", status "<<rob->Status_;
		cout<<", destination "<<rob->Destination_x_<<" "<<rob->Destination_y_;
		cout<<", position "<<rob->ThePassedPath_x_[rob->ThePassedPath_x_.size()-1]<<" "<<rob->ThePassedPath_y_[rob->ThePassedPath_x_.size()-1]<<endl;
		// !debug
		*/

		// Path Planning 
		double xStart = rob->ThePassedPath_x_[rob->ThePassedPath_x_.size()-1];
		double yStart = rob->ThePassedPath_y_[rob->ThePassedPath_x_.size()-1];
		int xSID = hObstacles_->GetBinIDX(xStart);
		int ySID = hObstacles_->GetBinIDY(yStart);
		int xEID = hObstacles_->GetBinIDX(dep_x);
		int yEID = hObstacles_->GetBinIDY(dep_y);

		PathPlanningAStar pp("PathPlanning_DyncamicDeploying",hObstacles_NoObstacle_); // for UAV
		pp.SetStartNode(new AStarNode("start",xSID,ySID));
		pp.SetEndNode  (new AStarNode("end1", xEID,yEID));
		pp.SetObstacleValue(0.5); // useless
		pp.InitiatPathPlanning();

		/*
		// debug
		cout<<"path size "<<pp.Path_.size()<<endl;
		// !debug
		*/

		// get path : part1 claer
		rob->ThePlannedPath_x_.clear();
		rob->ThePlannedPath_y_.clear();
		rob->ThePlannedPath_timeStamp_.clear();
		// get path : part2 add path
		for(int i=0;i<pp.Path_.size();i++)
		{
			double x,y;
			hObstacles_->GetBinCentor2D(pp.Path_[i]->xID_,pp.Path_[i]->yID_,x,y);
			rob->AddPlannedPath_coorAndTime(x,y,CurrentTime_);
		}
		// get path : part3 add destination to path
		rob->AddPlannedPath_coorAndTime(dep_x,dep_y, CurrentTime_);
		// get path : part4 set destination
		rob->Destination_x_ = dep_x;
		rob->Destination_y_ = dep_y;

		robots_.push_back(rob);
	}

	return true;
}

//-------------------------
// Public
// step 2
//-------------------------
bool DynamicControlCentor::SentTheRobotBack(DynamicRobotUnit *rob, double sf_x, double sf_y)
{
	//// set status
	//rob->Status_ = "Flying";

	//// set destination 
	//rob->Destination_x_ = sf_x;
	//rob->Destination_y_ = sf_y;

	//// Path Planning 
	//double xStart = rob->ThePassedPath_x_[rob->ThePassedPath_x_.size()-1];
	//double yStart = rob->ThePassedPath_y_[rob->ThePassedPath_x_.size()-1];
	//int xSID = hObstacles_->GetBinIDX(xStart);
	//int ySID = hObstacles_->GetBinIDY(yStart);
	//int xEID = hObstacles_->GetBinIDX(sf_x);
	//int yEID = hObstacles_->GetBinIDY(sf_y);

	//PathPlanningAStar pp("PathPlanning_DyncamicDeploying",hObstacles_NoObstacle_); // for UAV
	//pp.SetStartNode(new AStarNode("start",xSID,ySID));
	//pp.SetEndNode  (new AStarNode("end1", xEID,yEID));
	//pp.SetObstacleValue(0.5); // useless
	//pp.InitiatPathPlanning();

	//// get path
	//rob->ThePlannedPath_x_.clear();
	//rob->ThePlannedPath_y_.clear();
	//rob->ThePlannedPath_timeStamp_.clear();
	//for(int i=0;i<pp.Path_.size();i++)
	//{
	//	double x,y;
	//	hObstacles_->GetBinCentor2D(pp.Path_[i]->xID_,pp.Path_[i]->yID_,x,y);
	//	rob->AddPlannedPath_coorAndTime(x,y);
	//}

	return true;
}

//-------------------------
// Public
// step 3 : output
//-------------------------
bool DynamicControlCentor::WriteRobotPath()
{
	ofstream write("data_robots_path_"+name_+".txt");

	for(int i=0;i<robots_.size();i++)
	{
		int rID = i;
		for(int j=0;j<robots_[rID]->ThePassedPath_x_.size();j++)
		{
			int pID = j;
			write<<robots_[rID]->name_<<" ";
			write<<robots_[rID]->ThePassedPath_timeStamp_[pID]/second<<" ";
			write<<robots_[rID]->ThePassedPath_x_[pID]/m<<" ";
			write<<robots_[rID]->ThePassedPath_y_[pID]/m<<" ";
			write<<20<<" ";
			write<<robots_[rID]->ThePassedPath_status_[pID]<<endl;
		}

	}

	write.close();
	return true;
}

//-------------------------
// Public
// step 1 - 0 : import histogram parameters
//-------------------------
bool DynamicControlCentor::Tool_ReadHistogramsParameters_ForBuildings(string filename)
{
	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	string temp;
	file>>temp>>binSizes[0]>>mins[0]>>maxs[0];
	file>>temp>>binSizes[1]>>mins[1]>>maxs[1];

	mins[0] *= m;
    mins[1] *= m;
	maxs[0] *= m;
    maxs[1] *= m;

	binWidths[0] = (maxs[0]-mins[0])/double(binSizes[0]);
	binWidths[1] = (maxs[1]-mins[1])/double(binSizes[1]);

	cout<<"----------------------------------------------------------------"<<endl;
	cout<<"----"<<endl;
	cout<<"DynamicControlCentor : Tool_ReadHistogramsParameters_ForBuildings"<<endl;
	cout<<"x , bin "<<binSizes[0]<<", min "<<mins[0]/m<<" m, max "<<maxs[0]/m<<" m, binWidths "<<binWidths[0]/m<<" m"<<endl;
	cout<<"y , bin "<<binSizes[1]<<", min "<<mins[1]/m<<" m, max "<<maxs[1]/m<<" m, binWidths "<<binWidths[1]/m<<" m"<<endl;
	cout<<"----"<<endl;
	cout<<"----------------------------------------------------------------"<<endl;

	return 1;
}

//-------------------------
// Public
// step 1 - 1 : import histogram data for buildings, which are obstacles
//-------------------------
bool DynamicControlCentor::Tool_ReadDistributionObstacles(string filename)
{
	hObstacles_			= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	double x,y;
	double content;
	while(!file.eof())
	{
		file>>x>>y>>content;

		if(file.eof()) break;

		hObstacles_->Fill(x,y,content);

		//cout<<"x y "<<x<<" "<<y<<" building "<<content<<endl;
	}

	file.close();

	return 1;
}

//-------------------------
// Public
// step 1 - 1 : import histogram data for buildings, which are obstacles
//-------------------------
bool DynamicControlCentor::Tool_ReadDistributionObstacles_binary(string filename)
{
	hObstacles_binary_	= new Histogram2D("h",binSizes[0],mins[0],maxs[0],binSizes[1],mins[1],maxs[1]);

	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	double x,y;
	double content;
	while(!file.eof())
	{
		file>>x>>y>>content;

		if(file.eof()) break;

		hObstacles_binary_->Fill(x,y,content);

		//cout<<"x y "<<x<<" "<<y<<" building (binary) "<<content<<endl;
	}

	file.close();

	return 1;
}

//-------------------------
// Public
// step 1 - 4 : import robot deployment
//-------------------------
bool DynamicControlCentor::Tool_ReadRobotDeployment(string filename)
{
	ifstream file(filename.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename<<" \""<<endl;
		return 0;
	}

	double indID, geneID, fitness, x, y;
	string name;
	while(!file.eof())
	{
		file>>indID>>geneID>>fitness>>x>>y>>name;

		if(file.eof()) break;

		x *= m;
		y *= m;

		if(indID==0)
		{
			dep_x_.push_back(x);
			dep_y_.push_back(y);
		}
	}

	file.close();

	/*
	// debug
	for(int i=0;i<dep_x_.size();i++)
	{
		cout<<"Robot deployment x y "<<dep_x_[i]/m<<" m, "<<dep_y_[i]/m<<" m"<<endl;
	}
	*/

	return true;
}

//-------------------------
// Public
// step 1 - 5 : set SafeZones
//-------------------------
bool DynamicControlCentor::Tool_SetSafeZones(vector<double> SafeZone_x, vector<double> SafeZone_y)
{
	SafeZone_x_ = SafeZone_x;
	SafeZone_y_ = SafeZone_y;

	if(SafeZone_x_.size()==0)
		return false;

	IsStep1_5Good_ = true;

	/*
	// debug
	for(int i=0;i<SafeZone_x_.size();i++)
	{
		cout<<"SafeZone ID "<<i<<": x, y "<<SafeZone_x_[i]/m<<" m, "<<SafeZone_y_[i]/m<<" m"<<endl;
	}
	*/

	return true;
}

//-------------------------
// Public
// step 2 - 1 : time interval
//-------------------------
bool DynamicControlCentor::Tool_SetMaximumTimeInterval(double MaximumTimeInterval)
{
	MaximumTimeInterval_ = MaximumTimeInterval;

	IsStep2_1Good_ = true;

	cout<<"debug DynamicControlCentor::Tool_SetMaximumTimeInterval - max time : "<<MaximumTimeInterval_/hour<<" hours"<<endl;
	return true;
}
