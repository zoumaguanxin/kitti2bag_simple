#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;
typedef pcl::PointXYZ Point3d;
typedef pcl::PointXYZI PointI3d;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudIPtr;

 bool readlaserPointCloudFromKITTI( string path, int number, string suffix, PointCloudPtr& pout )
{
	char fullname[1000];
	sprintf(fullname,"%s00%.4i%s",path.c_str(),number,suffix.c_str());
	std::cout<<"file name: "<<fullname<<std::endl;
	float data[4];
	ifstream filecplus;
	filecplus.open(fullname,ios::binary|ios::in);
	PointCloudPtr source_pcd(new PointCloud); 
	if(filecplus.good())
	{
		while(filecplus.peek()!=EOF)
		{
			//char is equal to one byte. so the streamsize _n means the number of the bytes that you want to read. 
			filecplus.read((char*)data,4*sizeof(float));
			if(!filecplus.fail())
			{
				pcl::PointXYZ tempPt;
				tempPt.x=data[0];
				tempPt.y=data[1];
				tempPt.z=data[2];
				//std::cout<<tempPt<<std::endl;
				source_pcd->push_back(tempPt);
			}
			else
			{
				throw std::runtime_error("file format error");
			}
		}
	}
	else
	{
		return false;
	}
	pout=source_pcd;
	return  true;
}


bool readlaserPointCloudIFromKITTI( string path, int number, string suffix, PointCloudIPtr& pout )
{
	char fullname[1000];
	sprintf(fullname,"%s00%.4i%s",path.c_str(),number,suffix.c_str());
	std::cout<<"file name: "<<fullname<<std::endl;
	float data[4];
	ifstream filecplus;
	filecplus.open(fullname,ios::binary|ios::in);
	PointCloudIPtr source_pcd(new PointCloudI); 
	if(filecplus.good())
	{
		while(filecplus.peek()!=EOF)
		{
			//char is equal to one byte. so the streamsize _n means the number of the bytes that you want to read. 
			filecplus.read((char*)data,4*sizeof(float));
			if(!filecplus.fail())
			{
				pcl::PointXYZI tempPt;
				tempPt.x=data[0];
				tempPt.y=data[1];
				tempPt.z=data[2];
				tempPt.intensity=data[3];
				//std::cout<<tempPt<<std::endl;
				source_pcd->push_back(tempPt);
			}
			else
			{
				throw std::runtime_error("file format error");
			}
		}
	}
	else
	{
		return false;
	}
	pout=source_pcd;
	return  true;
}


std::vector<double> readtimesfromKitti(std::string path, std::string suffix)
{
	ifstream file;
	std::vector<double> times;
	string fullname=path+"timestamps"+suffix;	
	file.open(fullname.c_str(),std::ios::in);
	if(file.is_open())
	{
		while(file.peek()!=EOF)
		{
			stringstream xx;
			//char x[10000];
			//file.getline(x,10000,'\n');
			std::string xxx;
			std::getline(file,xxx);
			string timestamp_string;
			std::stringstream timestream(xxx);
			timestamp_string=timestream.str();
			std::tm t={};
			t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
			t.tm_mon = std::stoi(timestamp_string.substr(5, 2)) - 1;
			t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
			t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
			t.tm_min = std::stoi(timestamp_string.substr(14, 2));
			t.tm_sec = std::stoi(timestamp_string.substr(17, 2));
			t.tm_isdst = -1;
			static const uint64_t kSecondsToNanoSeconds = 1e9;
			time_t time_since_epoch = mktime(&t);

			uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
						std::stoi(timestamp_string.substr(20, 9));
			//xx<<xxx;
			std::cout<<"time:" <<xxx<<std::endl;
			string date;
			double time=timestamp*1e-9;
			std::cout<<"time (sec): "<<time<<std::endl; 
			times.push_back(time);
		}
		
	}
	else
	{
		string tm="can not open the file from ";
		string tems=fullname;
		tm+=tems;
		throw std::runtime_error(tm);
	}
	return times;
}

bool readOdomTimes(string path, std::vector<double>& times)
{
	times.clear();
	ifstream file;
	file.open(path.c_str(),std::ios::in);
	if(file.is_open())
	{
		while(file.peek()!=EOF)
		{
			double time;
			file>>time;
			times.emplace_back(time);
		}
	}
	else
	{
		return false;
	}
	return true;
}


int main(int argc, char **argv) {


	ros::init(argc,argv,"kitti2bag");
	ros::NodeHandle nh;
	rosbag::Bag bag;
	string bagDir=argv[3];
	bag.open(bagDir, rosbag::bagmode::Write);
	bool fileok=true;
	int i=0;
	string basepath=argv[1];
		
	string topic="/velodyne_points";
	
	string odomtime_dir=argv[2];
	
	std::vector<double> timessec;
	ros::Time begin_time=ros::Time::now() ;
	double db_time=begin_time.toSec();
	if(readOdomTimes(odomtime_dir,timessec))
	{
		std::cout<<"read times successly"<<std::endl;
	}
	else
	{
		throw runtime_error("can not read times");
	}
	while(1)
	{
		PointCloudIPtr pcd;
		fileok=readlaserPointCloudIFromKITTI(basepath,i,".bin",pcd);
		if(!fileok)
		{
			break;
		}
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg(*pcd,cloud);
		cloud.header.frame_id="velo_link";
//
		ros::Time lidar_time;
		double current_time=db_time+timessec[i];
		lidar_time.fromSec(current_time);
		i++;
		cloud.header.stamp=lidar_time;
		bag.write(topic,lidar_time,cloud);
	} 
	bag.close();
    return 0;
}
