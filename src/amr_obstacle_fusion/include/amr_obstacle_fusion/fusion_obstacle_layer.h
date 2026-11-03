#pragma once

#include <mutex>
#include <string>
#include <vector>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace amr_obstacle_fusion {

// 自研 costmap 层：订阅融合后的点云，在 master 代价地图上标致命障碍。
// 清障靠“过期不再绘制”。静态地图由前面的 StaticLayer 每周期重画，不会被本层擦掉。
class FusionObstacleLayer : public costmap_2d::CostmapLayer {
public:
  FusionObstacleLayer();
  ~FusionObstacleLayer() override;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw,
                    double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override;
  void reset() override;
  void activate() override;
  void deactivate() override;

private:
  struct Observation {
    ros::Time stamp;
    std::vector<geometry_msgs::Point> points;
  };

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void subscribe();

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::mutex mutex_;
  std::vector<Observation> observations_;

  std::string cloud_topic_;
  std::string global_frame_;
  double obstacle_range_;
  double min_obstacle_height_;
  double max_obstacle_height_;
  double keep_time_;
  double transform_tolerance_;
  double stale_timeout_;
  bool rolling_window_;
  bool ever_received_;
  ros::Time last_stamp_;
  double robot_x_;
  double robot_y_;
};

}  // namespace amr_obstacle_fusion
