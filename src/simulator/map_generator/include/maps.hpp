#ifndef MAPS_HPP
#define MAPS_HPP 

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

namespace mocka {

  class Maps {
  public:
    typedef struct BasicInfo
    {
      ros::NodeHandle *nh_private;
      int sizeX;
      int sizeY;
      int sizeZ;
      int seed;
      double scale;
      sensor_msgs::PointCloud2 *output;
      pcl::PointCloud<pcl::PointXYZ> *cloud;
    } BasicInfo;
    
    BasicInfo getInfo() const;
    void setInfo(const BasicInfo &value);
  

  public:
    Maps();

  public:
    void generate(int type);
    void addMap(const geometry_msgs::PoseStamped::ConstPtr& msg);

  private:
    BasicInfo info;

  private:
    void pcl2ros();
    void addGround();

    /* Map Generate */
    // Random Map
    void randomMapGenerate();
    // Rugged Map
    void ruggedMapGenerate();

  };

  class RuggedMap {
    private: 
      typedef struct node
      {
        int x_l, x_h;
        int y_l, y_h;
        double s;    // smoothness
        node(int _x_l, int _x_h, int _y_l, int _y_h, double _s): x_l(_x_l), x_h(_x_h), y_l(_y_l), y_h(_y_h) {}
      } node;
      
      inline double sq(double x);
      inline double rock(double d, double h, double x, double y);
      inline double crater(double d, double h, double x, double y);
      // crater: input: distance, output: number
      inline int c_d_n(double d);
      // crater: input: number, output: distance
      inline double c_n_d(int n);
      // rock: input:distance, output:number
      inline int r_d_n(double d);
      inline double r_n_d(int n);
      void fill_rock(double d, double h, int xl, int xh, int yl, int yh);
      void fill_crater(double d, double h1, double h2,
                       int xl, int xh, int yl, int yh, int xm, int ym);

    private:
      int sizeX;
      int sizeY;
      int sizeZ;
      int seed;
      double scale;
      const double min_rock_d = 1.5;
      const double min_crater_d = 10;
      const int smoothness = 25;
      int buffer_size;
      std::vector<unsigned char> vis;
      std::vector<double> map_buffer;
      const double h1_d[3][2] = {0.23,  0.17,  0.11,  0.25, 0.19,  0.13};
      const double h2_d[3][2] = {0.022, 0.016, 0.008, 0.06, 0.045, 0.03};
      
    public:
      RuggedMap(int _x, int _y, int _z, int _seed, double _scale);
      void diamond_square_algorithm();
      void generate_crater();
      void generate_rock();
  };
}
#endif