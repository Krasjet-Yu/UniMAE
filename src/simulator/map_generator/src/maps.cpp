#include "maps.hpp"

#include <vector>
#include <queue>
#include <algorithm>
#include <iostream>
#include <random>

#include <Eigen/Core>

using namespace mocka;

void Maps::pcl2ros() {
  pcl::toROSMsg(*info.cloud, *info.output);
  info.output->header.frame_id = "world";
  ROS_INFO("finish: infill %lf%%",
           info.cloud->width / (1.0 * info.sizeX * info.sizeY * info.sizeZ));
}

// @GaaiLam
void Maps::addGround() {
  int add_ground;
  info.nh_private->param("ground", add_ground, 0);
  if (add_ground) {
    double _x_low = -info.sizeX / (2 * info.scale);
    double _y_low = -info.sizeY / (2 * info.scale);
    for (int i = 0; i < info.sizeX; ++i) {
      for (int j = 0; j < info.sizeY; ++j) {
        // push_back()需要先构造临时对象，再将这个对象拷贝到容器的末尾，
        // 而emplace_back()
        info.cloud->points.emplace_back(
          _x_low + i * 1.0 / info.scale, _y_low + j * 1.0 / info.scale, 0);
      }
    }
  }
}

void Maps::addMap(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  double _w_low, _w_high;
  double _h_low, _h_high;
  info.nh_private->param("width_min", _w_low, 0.6);
  info.nh_private->param("width_max", _w_high, 1.5);
  info.nh_private->param("height_min", _h_low, 1.5);
  info.nh_private->param("height_max", _h_high, 1.5);

  std::default_random_engine eng(info.seed);
  std::uniform_real_distribution<double> rand_w;
  std::uniform_real_distribution<double> rand_h;

  rand_h = std::uniform_real_distribution<double>(_h_low, _h_high);
  rand_w = std::uniform_real_distribution<double>(_w_low, _w_high);

  double _resolution = 1 / info.scale;

  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double w = rand_w(eng);
  double h;

  pcl::PointXYZ pt_random;

  x = floor(x / _resolution) * _resolution + _resolution / 2.0;
  y = floor(y / _resolution) * _resolution + _resolution / 2.0;

  int widNum = ceil(w / _resolution);

  for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
    for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
      h = rand_h(eng);
      int heiNum = ceil(h / _resolution);
      for (int t = -1; t < heiNum; t++) {
        pt_random.x = x + (r + 0.5) * _resolution + 1e-2;
        pt_random.y = y + (s + 0.5) * _resolution + 1e-2;
        pt_random.z = (t + 0.5) * _resolution + 1e-2;
        info.cloud->points.emplace_back(pt_random);
      }
    }
  return;
}


void Maps::randomMapGenerate() {
  std::default_random_engine eng(info.seed); // 固定随机种子

  double _resolution = 1 / info.scale;

  double _x_low  = -info.sizeX / (2 * info.scale); // restore origin map x
  double _x_high =  info.sizeX / (2 * info.scale);
  double _y_low  = -info.sizeY / (2 * info.scale);
  double _y_high =  info.sizeY / (2 * info.scale);

  double _w_low, _w_high, _h_low, _h_high;
  int _obs_num;
  info.nh_private->param("width_min", _w_low, 0.6);
  info.nh_private->param("width_max", _w_high, 1.5);
  info.nh_private->param("height_min", _h_low, 1.5);
  info.nh_private->param("height_max", _h_high, 1.5);
  info.nh_private->param("obstacle_number", _obs_num, 10);

  _h_low  = _h_low >= 0 ? _h_low : 0;
  _h_high = _h_high <= info.sizeZ / info.scale ? _h_high : info.sizeZ / info.scale;

  std::uniform_real_distribution<double> rand_x;
  std::uniform_real_distribution<double> rand_y;
  std::uniform_real_distribution<double> rand_w;
  std::uniform_real_distribution<double> rand_h;

  pcl::PointXYZ pt_random;

  rand_x = std::uniform_real_distribution<double>(_x_low, _x_high);
  rand_y = std::uniform_real_distribution<double>(_y_low, _y_high);
  rand_w = std::uniform_real_distribution<double>(_w_low, _w_high);
  rand_h = std::uniform_real_distribution<double>(_h_low, _h_high);


  for (int i = 0; i < _obs_num; ++i) {
    double x, y;
    x = rand_x(eng);
    y = rand_y(eng);

    double w, h;
    w = rand_w(eng);
    h = rand_h(eng);

    // ceil: 它的作用是取一个浮点数的最小整数，也就是返回大于或者等于该浮点数的最小整数
    int widNum = ceil(w / _resolution);
    int heiNum = ceil(h / _resolution);

    int rl, rh, sl, sh;
    rl = -widNum / 2;
    rh = widNum / 2;
    sl = -widNum / 2;
    sh = widNum / 2;

    // 只填充障碍物的边界，填充点云。
    for (int r = rl; r < rh; r++)
      for (int s = sl; s < sh; s++)
      {
        for (int t = 0; t < heiNum; t++)
        {
          if ((r - rl) * (r - rh + 1) * (s - sl) * (s - sh + 1) * t *
                (t - heiNum + 1) ==
              0)
          {
            pt_random.x = x + r * _resolution;
            pt_random.y = y + s * _resolution;
            pt_random.z = t * _resolution;
            info.cloud->points.push_back(pt_random);
          }
        }
      }
  }
  addGround();
  info.cloud->width    = info.cloud->points.size();
  info.cloud->height   = 1;
  info.cloud->is_dense = true;
  pcl2ros();
}

void Maps::ruggedMapGenerate() {
  RuggedMap rugmap(info.sizeX, info.sizeY, info.sizeZ, info.seed, info.scale);
  rugmap.diamond_square_algorithm();
  rugmap.generate_crater();
  rugmap.generate_rock();
}

Maps::Maps() {}

Maps::BasicInfo Maps::getInfo() const{
  return info;
}

void Maps::setInfo(const BasicInfo& value) {
  info = value;
}

void Maps::generate(int type) {
  switch(type) {
    default:
    case 1:
      // TODO: 
      break;
    case 2:
      randomMapGenerate();
      break;
    case 3:
      std::srand(info.seed);
      break;
    case 4:
      std::srand(info.seed);
      break;
  }
}

RuggedMap::RuggedMap(int _x, int _y, int _z, int _seed, double _scale) {
  sizeX = _x;
  sizeY = _y;
  sizeZ = _z;
  seed = _seed;
  scale = _scale;
  buffer_size = sizeX * sizeY * sizeY;
  map_buffer.resize(buffer_size);
  vis.resize(buffer_size);
}

void RuggedMap::diamond_square_algorithm() {
  std::queue<node> q;
  int x_l = -sizeX / (2 * scale);
  int x_h =  sizeX / (2 * scale);
  int y_l = -sizeY / (2 * scale);
  int y_h =  sizeY / (2 * scale);
  q.push(node(x_l, x_h, y_l, y_h, smoothness));


}
void RuggedMap::generate_crater() {

}
void RuggedMap::generate_rock() {

}