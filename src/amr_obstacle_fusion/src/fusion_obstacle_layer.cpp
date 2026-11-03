#include <amr_obstacle_fusion/fusion_obstacle_layer.h>

#include <algorithm>
#include <cmath>

#include <costmap_2d/cost_values.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/exceptions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

PLUGINLIB_EXPORT_CLASS(amr_obstacle_fusion::FusionObstacleLayer, costmap_2d::Layer)

namespace amr_obstacle_fusion {

FusionObstacleLayer::FusionObstacleLayer()
    : obstacle_range_(8.0),
      min_obstacle_height_(0.05),
      max_obstacle_height_(1.6),
      keep_time_(0.6),
      transform_tolerance_(0.3),
      stale_timeout_(1.0),
      rolling_window_(false),
      ever_received_(false),
      robot_x_(0.0),
      robot_y_(0.0) {}

FusionObstacleLayer::~FusionObstacleLayer() {}

void FusionObstacleLayer::onInitialize() {
  nh_ = ros::NodeHandle("~/" + name_);
  // Layer 构造时 enabled_ 默认是 false，不打开就不会被 LayeredCostmap 调用。
  nh_.param("enabled", enabled_, true);
  nh_.param("cloud_topic", cloud_topic_, std::string("/amr/fusion/cloud"));
  nh_.param("obstacle_range", obstacle_range_, 8.0);
  nh_.param("min_obstacle_height", min_obstacle_height_, 0.05);
  nh_.param("max_obstacle_height", max_obstacle_height_, 1.6);
  nh_.param("keep_time", keep_time_, 0.6);
  nh_.param("transform_tolerance", transform_tolerance_, 0.3);
  nh_.param("stale_timeout", stale_timeout_, 1.0);

  rolling_window_ = layered_costmap_->isRolling();
  global_frame_ = layered_costmap_->getGlobalFrameID();
  default_value_ = costmap_2d::NO_INFORMATION;
  matchSize();
  current_ = true;
  subscribe();
  ROS_INFO("FusionObstacleLayer[%s] 订阅 %s，全局坐标系 %s",
           name_.c_str(), cloud_topic_.c_str(), global_frame_.c_str());
}

void FusionObstacleLayer::subscribe() {
  if (cloud_topic_.empty()) {
    return;
  }
  sub_ = nh_.subscribe(cloud_topic_, 1, &FusionObstacleLayer::cloudCallback, this);
}

void FusionObstacleLayer::activate() {
  subscribe();
  current_ = true;
}

void FusionObstacleLayer::deactivate() {
  sub_.shutdown();
}

void FusionObstacleLayer::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  observations_.clear();
  ever_received_ = false;
  resetMaps();
  current_ = true;
}

void FusionObstacleLayer::matchSize() {
  CostmapLayer::matchSize();
}

void FusionObstacleLayer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  if (!enabled_ || tf_ == nullptr) {
    return;
  }
  global_frame_ = layered_costmap_->getGlobalFrameID();

  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(global_frame_, msg->header.frame_id, msg->header.stamp,
                                     ros::Duration(transform_tolerance_));
  } catch (const tf2::TransformException&) {
    try {
      transform = tf_->lookupTransform(global_frame_, msg->header.frame_id, ros::Time(0),
                                       ros::Duration(0.05));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "FusionObstacleLayer 变换失败 %s -> %s: %s",
                        msg->header.frame_id.c_str(), global_frame_.c_str(), ex.what());
      return;
    }
  }

  sensor_msgs::PointCloud2 cloud;
  try {
    tf2::doTransform(*msg, cloud, transform);
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(2.0, "FusionObstacleLayer 点云变换失败: %s", ex.what());
    return;
  }

  Observation obs;
  obs.stamp = ros::Time::now();
  try {
    sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix;
      const float y = *iy;
      const float z = *iz;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      if (z < min_obstacle_height_ || z > max_obstacle_height_) {
        continue;
      }
      geometry_msgs::Point point;
      point.x = x;
      point.y = y;
      point.z = z;
      obs.points.push_back(point);
    }
  } catch (const std::runtime_error& ex) {
    ROS_WARN_THROTTLE(2.0, "融合点云缺少 xyz 字段: %s", ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  observations_.push_back(obs);
  ever_received_ = true;
  last_stamp_ = obs.stamp;
}

void FusionObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                       double* min_x, double* min_y, double* max_x, double* max_y) {
  (void)robot_yaw;
  if (!enabled_) {
    return;
  }
  robot_x_ = robot_x;
  robot_y_ = robot_y;
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }

  const ros::Time now = ros::Time::now();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const ros::Duration keep(keep_time_);
    observations_.erase(
        std::remove_if(observations_.begin(), observations_.end(),
                       [&](const Observation& obs) { return now - obs.stamp > keep; }),
        observations_.end());
    if (ever_received_ && !last_stamp_.isZero() && (now - last_stamp_).toSec() > stale_timeout_) {
      current_ = false;
    } else {
      current_ = true;
    }
  }

  // 把机器人周围整块交给 updateCosts，过期障碍才会从 master 上消失。
  const double reach = std::min(obstacle_range_, std::max(getSizeInMetersX(), getSizeInMetersY()) * 0.5);
  useExtraBounds(min_x, min_y, max_x, max_y);
  touch(robot_x - reach, robot_y - reach, min_x, min_y, max_x, max_y);
  touch(robot_x + reach, robot_y + reach, min_x, min_y, max_x, max_y);
}

void FusionObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
                                      int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_ || getSizeInCellsX() == 0 || getSizeInCellsY() == 0) {
    return;
  }
  resetMaps();

  std::vector<Observation> snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot = observations_;
  }

  const double range_sq = obstacle_range_ * obstacle_range_;

  for (const Observation& obs : snapshot) {
    for (const geometry_msgs::Point& point : obs.points) {
      const double dx = point.x - robot_x_;
      const double dy = point.y - robot_y_;
      if (dx * dx + dy * dy > range_sq) {
        continue;
      }
      unsigned int mx = 0;
      unsigned int my = 0;
      if (worldToMap(point.x, point.y, mx, my)) {
        setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace amr_obstacle_fusion
