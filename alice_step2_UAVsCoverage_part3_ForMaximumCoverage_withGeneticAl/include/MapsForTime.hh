#ifndef MAPSFORTIME_HH
#define MAPSFORTIME_HH

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "Histogram2D.hh"

using namespace std;

class MapsForTime
{
    public:
		MapsForTime(string name, double TimeWidth, string hist2dParaFilePath);
		~MapsForTime();

    public:
		void AddABinAtATimeStamp(double x, double y, double content, int RoundID);
		void WriteMaps(string filename);

	private:
		void WriteAMap(string filename, int RoundID);

	private:
		string name_;

		double TimeWidth_;
		vector<double> TimeStamps_;
		vector<Histogram2D *> Maps_;


		// histogram parameters, 0 for x, 1 for y
		int     binSizes_[2];
		double	binWidths_[2];
		double  mins_[2];
		double  maxs_[2];
};
#endif
