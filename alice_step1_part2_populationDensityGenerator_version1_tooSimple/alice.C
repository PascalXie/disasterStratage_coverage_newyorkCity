#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <random>

#include "Units.hh"
#include "Histogram.hh"
#include "Histogram2D.hh"


using namespace std;

default_random_engine generator(time(0));

vector<string> IDs;
vector<double> xs, ys, rs; // for buildings

vector<string> Zones_IDs;
vector<double> Zones_xs, Zones_ys, Zones_rs; // for SafeZonesAndDangerZones

int		bins[2] = {10,10};
double	mins[2] = {-1000, -1000};
double 	maxs[2] = {1000, 1000};

double sigmaWeight_ = 1.;
double PopulationDensity_ = 27627./1e6;

double Velocity_ = 10000; // m.min


Histogram2D *h_;
vector<Histogram2D *> hpopu_;
vector<Histogram2D *> hFx_, hFy_;

bool ReadData_SafeZonesAndDangerZones(string filename1);
double computeWeight_population(double locx, double locy, double sigmaWeight);
double computeWeight_forceField(double locx, double locy, double sigmaWeight_building, double sigmaWeight_zones);
void GenerateForceField(double locx, double locy, double &Fx, double &Fy);

int main()
{
	cout<<"Hello "<<endl;

	/*
	// test
	normal_distribution<double> distributionTest(0.0, 2.0);
	uniform_real_distribution<double> uniTest(0.0, 1.0);

	for(int i=0;i<100;i++)
	{
		cout<<i<<" "<<distributionTest(generator)<<" "<<uniTest(generator)<<endl;
	}
	*/

	// step 0 : ../data_histogram2d_parameters.txt
	string filename0 = "../data_histogram2d_parameters.txt";
	ifstream file(filename0.c_str());

	if(file.fail())
	{
		cout<<"Can not find the file \" "<<filename0<<" \""<<endl;
		return 0;
	}

	string temp;
	file>>temp>>bins[0]>>mins[0]>>maxs[0];
	cout<<temp<<", bin "<<bins[0]<<", min "<<mins[0]<<", max "<<maxs[0]<<endl;

	file>>temp>>bins[1]>>mins[1]>>maxs[1];
	cout<<temp<<", bin "<<bins[1]<<", min "<<mins[1]<<", max "<<maxs[1]<<endl;

	// step 0 : histogram
	h_ = new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]);
	hpopu_.push_back(new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]));


	// step 0 : safe zones and danger zones
	// ../data_safeZonesAndDangerZones.txt
	string filename0_1 = "../../data_safeZonesAndDangerZones.txt";
	ReadData_SafeZonesAndDangerZones(filename0_1);


	// step 1 : buildings
	string filename1 = "../../data_buildings.txt";
	ifstream file1(filename1.c_str());

	if(file1.fail())
	{
		cout<<"Can not find the file \" "<<filename1<<" \""<<endl;
		return 0;
	}

	string ID;
	double x, y, r;
	while(!file1.eof())
	{
		file1>>ID>>x>>y>>r;

		if(file1.eof()) break;

		if(r>10000) continue; // ground object in Blender

		IDs.push_back(ID);
		xs.push_back(x);
		ys.push_back(y);
		rs.push_back(r);

		//cout<<ID<<" "<<x<<" "<<y<<" "<<r<<endl;
	}

	file1.close();

	cout<<"IDname test : name "<<IDs[0]<<", name[0] "<<IDs[0][0]<<endl;


	//
	// step 2 : compute population
	//
	// density 
	double TotalDensity = 0.;
	for(int i=0;i<bins[0];i++)
	for(int j=0;j<bins[1];j++)
	{
		int xid = i;
		int yid = j;

		double x = 0;
		double y = 0;
		bool isCenterGot = h_->GetBinCentor2D(xid, yid, x, y);

		double content = computeWeight_population(x,y,sigmaWeight_);
		h_->Fill(x,y,content);

		TotalDensity += content;
	}

	// normalization
	double BinWidth_area = (maxs[0]-mins[0])/double(bins[0])*(maxs[1]-mins[1])/double(bins[1]);
	double Total_area = (maxs[0]-mins[0])*(maxs[1]-mins[1]);
	cout<<"BinWidth_area "<<BinWidth_area<<endl;
	cout<<"Population per BinWidth_area "<<PopulationDensity_*BinWidth_area<<endl;

	//ofstream write("data_step1_part2_population_time0.txt");
	for(int i=0;i<bins[0];i++)
	for(int j=0;j<bins[1];j++)
	{
		int xid = i;
		int yid = j;

		double x = 0;
		double y = 0;
		bool isCenterGot = h_->GetBinCentor2D(xid, yid, x, y);

		double content = h_->GetBinContent(xid, yid)/TotalDensity*PopulationDensity_*Total_area/BinWidth_area;

		hpopu_[0]->Fill(x,y,content);

		// kill the area in buildings
		//if(content<0.33)  content = 0;

		//write<<xid<<" "<<yid<<" "<<x<<" "<<y<<" "<<content<<endl;

	}

	//write.close();


	//
	//  move
	//
	double time_begin_ = 0; // min
	double time_end_ = 120;  // min
	double time_step_ = 3;  // min
	int time_step_size_ = (time_end_-time_begin_)/time_step_;

	for(int k=0;k<time_step_size_;k++)
	{
		int tID = k;
		cout<<"Time step : "<<tID<<", time "<<double(tID)*time_step_<<endl;

		// next time step
		hpopu_.push_back(new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]));

		hFx_.push_back(new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]));
		hFy_.push_back(new Histogram2D("h",bins[0],mins[0],maxs[0],bins[1],mins[1],maxs[1]));

		for(int i=0;i<bins[0];i++)
		for(int j=0;j<bins[1];j++)
		{
			int xid = i;
			int yid = j;

			double x = 0;
			double y = 0;
			bool isCenterGot = hpopu_[tID]->GetBinCentor2D(xid, yid, x, y);
			double population = hpopu_[tID]->GetBinContent(xid, yid);

			double Fx, Fy;
			GenerateForceField(x, y, Fx, Fy);

			// moving
			double dx = 0.5*Fx*time_step_*time_step_;
			double dy = 0.5*Fy*time_step_*time_step_;
			//cout<<"dx "<<dx<<", dy "<<dy<<endl;

			double x_new = x + dx*Velocity_*time_step_;
			double y_new = y + dy*Velocity_*time_step_;
			//cout<<"dx "<<x_new-x<<", dy "<<y_new-y<<endl;

			hpopu_[tID+1]->Fill(x_new, y_new, population);

			// debug
			hFx_[tID]->Fill(x, y, Fx);
			hFy_[tID]->Fill(x, y, Fy);

		}
	}

	// write
	for(int k=0;k<time_step_size_;k++)
	{
		int tID = k;
		string writefilename = "data_step1_part2_population_time_"+ to_string(tID)+".txt";
		ofstream write(writefilename);

		for(int i=0;i<bins[0];i++)
		for(int j=0;j<bins[1];j++)
		{
			int xid = i;
			int yid = j;

			double x = 0;
			double y = 0;
			bool isCenterGot = hpopu_[tID]->GetBinCentor2D(xid, yid, x, y);
			double population = hpopu_[tID]->GetBinContent(xid, yid);

			write<<xid<<" "<<yid<<" "<<x<<" "<<y<<" "<<population<<endl;
		}
		write.close();
	}

	// write forces
	for(int k=0;k<time_step_size_;k++)
	{
		int tID = k;
		string writefilename2 = "data_step1_part2_forcex_time_"+ to_string(tID)+".txt";
		string writefilename3 = "data_step1_part2_forcey_time_"+ to_string(tID)+".txt";
		ofstream write2(writefilename2);
		ofstream write3(writefilename3);

		for(int i=0;i<bins[0];i++)
		for(int j=0;j<bins[1];j++)
		{
			int xid = i;
			int yid = j;

			double x = 0;
			double y = 0;
			bool isCenterGot = hpopu_[tID]->GetBinCentor2D(xid, yid, x, y);
			double xf = hFx_[tID]->GetBinContent(xid, yid);
			double yf = hFy_[tID]->GetBinContent(xid, yid);

			write2<<xid<<" "<<yid<<" "<<x<<" "<<y<<" "<<xf<<endl;
			write3<<xid<<" "<<yid<<" "<<x<<" "<<y<<" "<<yf<<endl;
		}
		write2.close();
		write3.close();
	}


	return 1;
}

void GenerateForceField(double locx, double locy, double &Fx, double &Fy)
{
	Fx = 0;
	Fy = 0;

	double sigmaWeight_building = 1.;
	double sigmaWeight_zones	= 1e4;
	double forceWeight_zones	= 1e2;

	// buildings
	for(int i=0;i<IDs.size();i++)
	{
		int buildingID = i;
		string IDname = IDs[buildingID];
		double bx = xs[buildingID];
		double by = ys[buildingID];
		double br = rs[buildingID];

		// distance
		double d2 = (bx-locx)*(bx-locx) + (by-locy)*(by-locy);
		double d = sqrt(d2);

		double sigma = br*sigmaWeight_building;
		double Gauss_part1 = 1./(sqrt(2.*PI)*sigma);
		double Gauss_part2 = exp(-1.*d2/(2.*sigma*sigma));
		double Gauss = Gauss_part1*Gauss_part2; // pull

		// forces
		Fx = Fx + Gauss*((locx-bx))/d;
		Fy = Fy + Gauss*((locy-by))/d;
	}

	// zones
	for(int i=0;i<Zones_IDs.size();i++)
	{
		int buildingID = i;
		string IDname = Zones_IDs[buildingID];
		double bx = Zones_xs[buildingID];
		double by = Zones_ys[buildingID];
		double br = Zones_rs[buildingID];

		// distance
		double d2 = (bx-locx)*(bx-locx) + (by-locy)*(by-locy);
		double d = sqrt(d2);

		double sigma = br*sigmaWeight_zones;
		double Gauss_part1 = 1./(sqrt(2.*PI)*sigma);
		double Gauss_part2 = exp(-1.*d2/(2.*sigma*sigma));
		double Gauss = forceWeight_zones*Gauss_part1*Gauss_part2; // pull

		if(IDname[4]='S') // for safe zones
		{
			//cout<<"Safe : "<<IDname<<endl;
			Gauss *= -1.; 
		}

		// forces
		Fx = Fx + Gauss*((locx-bx))/d;
		Fy = Fy + Gauss*((locy-by))/d;
	}

}

double computeWeight_population(double locx, double locy, double sigmaWeight)
{
	// Gaussian 
	double Content = 0;
	for(int i=0;i<IDs.size();i++)
	{
		int buildingID = i;
		string IDname = IDs[buildingID];
		double bx = xs[buildingID];
		double by = ys[buildingID];
		double br = rs[buildingID];

		// distance
		double d2 = (bx-locx)*(bx-locx) + (by-locy)*(by-locy);

		double sigma = br*sigmaWeight;
		double Gauss_part1 = 1./(sqrt(2.*PI)*sigma);
		double Gauss_part2 = exp(-1.*d2/(2.*sigma*sigma));
		double Gauss = Gauss_part1*Gauss_part2;

		Content += Gauss;
	}

	//if(Content<1e-10) Content=1e10;

	//Content = -1. * log(Content);

	//Content = 1. / Content;

	return Content;
}

double computeWeight_forceField(double locx, double locy, double sigmaWeight_building, double sigmaWeight_zones)
{
	// Gaussian 
	double Content = 0;

	// buildings
	for(int i=0;i<IDs.size();i++)
	{
		int buildingID = i;
		string IDname = IDs[buildingID];
		double bx = xs[buildingID];
		double by = ys[buildingID];
		double br = rs[buildingID];

		// distance
		double d2 = (bx-locx)*(bx-locx) + (by-locy)*(by-locy);

		double sigma = br*sigmaWeight_building;
		double Gauss_part1 = 1./(sqrt(2.*PI)*sigma);
		double Gauss_part2 = exp(-1.*d2/(2.*sigma*sigma));
		double Gauss = -1.*Gauss_part1*Gauss_part2; // pull

		Content += Gauss;
	}

	// zones
	for(int i=0;i<IDs.size();i++)
	{
		int buildingID = i;
		string IDname = IDs[buildingID];
		double bx = xs[buildingID];
		double by = ys[buildingID];
		double br = rs[buildingID];

		// distance
		double d2 = (bx-locx)*(bx-locx) + (by-locy)*(by-locy);

		double sigma = br*sigmaWeight_zones;
		double Gauss_part1 = 1./(sqrt(2.*PI)*sigma);
		double Gauss_part2 = exp(-1.*d2/(2.*sigma*sigma));
		double Gauss = -1.*Gauss_part1*Gauss_part2; // pull

		if(IDname[4]='S') // for safe zones
		{
			cout<<"Safe : "<<IDname<<endl;
			Gauss *= -1.; 
		}

		Content += Gauss;
	}


	return Content;
}

bool ReadData_SafeZonesAndDangerZones(string filename1)
{
	ifstream file1(filename1.c_str());

	if(file1.fail())
	{
		cout<<"Can not find the file \" "<<filename1<<" \""<<endl;
		return 0;
	}

	string ID;
	double x, y, r;
	while(!file1.eof())
	{
		file1>>ID>>x>>y>>r;

		if(file1.eof()) break;

		Zones_IDs.push_back(ID);
		Zones_xs.push_back(x);
		Zones_ys.push_back(y);
		Zones_rs.push_back(r);

		//cout<<ID<<" "<<x<<" "<<y<<" "<<r<<endl;
	}

	file1.close();

	return true;
}
