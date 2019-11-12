#ifndef DYNAMICROBOTUNIT_HH
#define DYNAMICROBOTUNIT_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Units.hh"

#include "Histogram2D.hh"

using namespace std;

class DynamicRobotUnit
{
    public:
		DynamicRobotUnit(string name, double x, double y, double time, double Speed);
		~DynamicRobotUnit();

    public:
		void AddPlannedPath_coorAndTime(double x, double y, double CurrentTime_);
		void MoveATimeInterval(double currentTimeStamp, double timeInterval);

    public:
		string name_;

		// status : "Flying_Work", "Flying_Recharge", "Working", "Recharging", "Free", "Dead" 
		string Status_;

		// speed
		double Speed_;

		// battery
		double Battery_; // time
		double MaximumBattery_; // time

		// Destination , coordinates of the deployed point
		double Destination_x_;
		double Destination_y_;

		// the planned path
		vector<double> ThePlannedPath_x_;
		vector<double> ThePlannedPath_y_;
		vector<double> ThePlannedPath_timeStamp_;

		// the passed path
		vector<double> ThePassedPath_x_;
		vector<double> ThePassedPath_y_;
		vector<double> ThePassedPath_timeStamp_;
		vector<string> ThePassedPath_status_;
};
#endif

