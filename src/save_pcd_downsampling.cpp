#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
/* #include <pcl/filters/passthrough.h> */
#include <pcl/filters/voxel_grid.h>

class SavePCDDownsampling{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscribe*/
		/*publish*/
		ros::Publisher pub_pc_origin;
		ros::Publisher pub_pc_filtered;
		/*cloud*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_origin {new pcl::PointCloud<pcl::PointXYZ>};
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_filtered {new pcl::PointCloud<pcl::PointXYZ>};
		/*parameters*/
		double leafsize;
		std::string filename_load;
		std::string filename_save;
		std::string frame_id_name;
	public:
		SavePCDDownsampling();
		void Execution(void);
		void Load(void);
		void Downsampling(void);
		void Publication(void);
		void Save(void);
};

SavePCDDownsampling::SavePCDDownsampling()
	:nhPrivate("~")
{
	pub_pc_origin = nh.advertise<sensor_msgs::PointCloud2>("/cloud/origin", 1);
	pub_pc_filtered = nh.advertise<sensor_msgs::PointCloud2>("/cloud/filtered", 1);

	nhPrivate.param("leafsize", leafsize, 0.1);
	std::cout << "leafsize = " << leafsize << std::endl;
	nhPrivate.param("filename_load", filename_load, std::string("hoge.pcd"));
	std::cout << "filename_load = " << filename_load << std::endl;
	nhPrivate.param("filename_save", filename_save, std::string("hoge.pcd"));
	std::cout << "filename_save = " << filename_save << std::endl;
	nhPrivate.param("frame_id_name", frame_id_name, std::string("/map"));
	std::cout << "frame_id_name = " << frame_id_name << std::endl;
}

void SavePCDDownsampling::Execution(void)
{
	Load();
	Downsampling();
	Publication();
	Save();
}

void SavePCDDownsampling::Load(void)
{
	ros::Time t_start = ros::Time::now();
	std::cout << "Loading ..." << std::endl;
	if(pcl::io::loadPCDFile<pcl::PointXYZ>(filename_load, *pc_origin) == -1){
		std::cout << "Loading error: " << filename_load << std::endl;
		exit(1);
	}
	std::cout << "Done ( " << (ros::Time::now() - t_start).toSec() << " [s])" << std::endl;
	/* std::cout << "pc_origin->header.frame_id = " << pc_origin->header.frame_id << std::endl; */
}

void SavePCDDownsampling::Downsampling(void)
{
	std::cout << "pc_origin->points.size() = " << pc_origin->points.size() << std::endl;
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setInputCloud(pc_origin);
	vg.setLeafSize((float)leafsize, (float)leafsize, (float)leafsize);
	vg.filter(*pc_filtered);
	std::cout << "pc_filtered->points.size() = " << pc_filtered->points.size() << std::endl;
}

void SavePCDDownsampling::Publication(void)
{
	/*pc_origin*/
	// pc_origin->header.stamp = ros::Time::now();
	pc_origin->header.frame_id = frame_id_name;
	sensor_msgs::PointCloud2 ros_pc_origin;
	pcl::toROSMsg(*pc_origin, ros_pc_origin);
	pub_pc_origin.publish(ros_pc_origin);	
	/*pc_filtered*/
	/* pc_filtered->header.stamp = ros::Time::now(); */
	pc_filtered->header.frame_id = frame_id_name;
	sensor_msgs::PointCloud2 ros_pc_filtered;
	pcl::toROSMsg(*pc_filtered, ros_pc_filtered);
	pub_pc_filtered.publish(ros_pc_filtered);	
}

void SavePCDDownsampling::Save(void)
{
	std::string answer;
	while(answer != "y" && answer != "n"){
		std::cout << "Do you want to save the point cloud? (y or n)" << std::endl;
		std::cin >> answer;
		if(answer == "y"){
			pcl::io::savePCDFileASCII(filename_save, *pc_filtered);
			std::cout << "The point cloud was saved as " << filename_save << std::endl;
		}
		else if(answer == "n")	std::cout << "Dumped" << std::endl;
		else	std::cout << "Type y or n" << std::endl;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_pcd_downsampling");
	
	SavePCDDownsampling save_pcd_downsampling;
	save_pcd_downsampling.Execution();
}
