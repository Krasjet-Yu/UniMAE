#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
 
int main(int argc, char** argv)
{
	pcl::PCLPointCloud2 cloud;
	//加载ply文件
	pcl::PLYReader reader;
	reader.read("/home/krasjet/Documents/ros1/UniMAE/src/simulator/simulator/odom_visualization/meshes/house_1.ply", cloud);
	//将ply文件保存为pcd文件
	pcl::PCDWriter writer;
	writer.write("/home/krasjet/Documents/ros1/UniMAE/src/simulator/simulator/odom_visualization/meshes/house_1.pcd", cloud);
 
	return 0;
}