#include "MapsForTime.hh"

//-------------------------
// Constructor
//-------------------------
MapsForTime::MapsForTime(string name, double TimeWidth, string hist2dParaFilePath)
:	name_(name),
	TimeWidth_(TimeWidth)
{
	// import parameters for histograms
	// step 0 : ../data_histogram2d_parameters.txt
	ifstream file(hist2dParaFilePath.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<hist2dParaFilePath<<" \""<<endl;
	}

	cout<<"Debug class MapsForTime::MapsForTime"<<endl;
	string temp;
	file>>temp>>binSizes_[0]>>mins_[0]>>maxs_[0];
	cout<<temp<<", bin "<<binSizes_[0]<<", min "<<mins_[0]<<", max "<<maxs_[0]<<endl;

	file>>temp>>binSizes_[1]>>mins_[1]>>maxs_[1];
	cout<<temp<<", bin "<<binSizes_[1]<<", min "<<mins_[1]<<", max "<<maxs_[1]<<endl;

	binWidths_[0] = (maxs_[0]-mins_[0])/double(binSizes_[0]);
	binWidths_[1] = (maxs_[1]-mins_[1])/double(binSizes_[1]);
}

//-------------------------
// Destructor
//-------------------------
MapsForTime::~MapsForTime()
{}

//-------------------------
// Public
//-------------------------
void MapsForTime::AddABinAtATimeStamp(double x, double y, double content, int RoundID)
{
	if(RoundID<0) 
	{
		cout<<"An error occured in class MapsForTime::AddABinAtATimeStamp"<<endl;
		return;
	}

	// check the size for maps
	if(RoundID>=Maps_.size())
	{
		for(int i=0;i<RoundID-Maps_.size()+1;i++)
		{
			Maps_.push_back(new Histogram2D(to_string(RoundID),binSizes_[0],mins_[0],maxs_[0],binSizes_[1],mins_[1],maxs_[1]));
		}
	}

	// add a bin
	Maps_[RoundID]->Fill(x,y,content);

}

void MapsForTime::WriteMaps(string filename)
{
	for(int i=0;i<Maps_.size();i++)
	{
		int RoundID = i;
		WriteAMap(filename, RoundID);
	}
}


//-------------------------
// Private
//-------------------------
void MapsForTime::WriteAMap(string filename, int RoundID)
{
	ofstream write(filename+"_roundID_"+to_string(RoundID)+".txt");

	for(int i=0;i<binSizes_[0];i++)
	for(int j=0;j<binSizes_[1];j++)
	{
		int xID = i;
		int yID = j;

		double x = mins_[0] + binWidths_[0]*(double(xID)+0.5);
		double y = mins_[1] + binWidths_[1]*(double(yID)+0.5);

		double content = Maps_[RoundID]->GetBinContent(xID, yID);

		write<<x<<" "<<y<<" "<<content<<endl;
	}

	write.close();
}
