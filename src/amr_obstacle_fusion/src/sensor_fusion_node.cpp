#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include <boost/bind.hpp>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <amr_obstacle_fusion/scan_fusion.hpp>

// 自研节点：把 /lidar1/scan_filtered 与两路深度点云投到 base_link，
// 发布一圈融合扫描和一朵障碍点云，供代价地图使用。
// 建图不要订阅这两个输出，地图只应由雷达和里程计生成。

namespace amr_obstacle_fusion {

struct VoxelKey {
  int x;
  int y;
  int z;
  bool operator==(const VoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash {
  std::size_t operator()(const VoxelKey& key) const {
    return (static_cast<std::size_t>(key.x) * 73856093u) ^
           (static_cast<std::size_t>(key.y) * 19349663u) ^
           (static_cast<std::size_t>(key.z) * 83492791u);
  }
};

class SensorFusionNode {
public:
  SensorFusionNode() : pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param("scan_topic", scan_topic_, std::string("/lidar1/scan_filtered"));
    pnh_.param("output_scan_topic", output_scan_topic_, std::string("/amr/fusion/scan"));
    pnh_.param("output_cloud_topic", output_cloud_topic_, std::string("/amr/fusion/cloud"));
    pnh_.param("target_frame", target_frame_, std::string("base_link"));
    pnh_.param("min_height", min_height_, 0.08);
    pnh_.param("max_height", max_height_, 1.50);
    pnh_.param("min_range", min_range_, 0.20);
    pnh_.param("max_range", max_range_, 20.0);
    pnh_.param("voxel_size", voxel_size_, 0.08);
    pnh_.param("point_stride", point_stride_, 3);
    pnh_.param("angle_increment_deg", angle_increment_deg_, 0.5);
    if (angle_increment_deg_ < 0.05) {
      angle_increment_deg_ = 0.5;
    }
    pnh_.param("publish_rate", publish_rate_, 10.0);
    pnh_.param("tf_timeout", tf_timeout_, 0.1);
    pnh_.param("footprint_half_x", footprint_half_x_, 0.42);
    pnh_.param("footprint_half_y", footprint_half_y_, 0.27);
    pnh_.param("max_cloud_points", max_cloud_points_, 20000);

    XmlRpc::XmlRpcValue listed;
    if (pnh_.getParam("cloud_topics", listed) && listed.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      for (int i = 0; i < listed.size(); ++i) {
        cloud_topics_.push_back(static_cast<std::string>(listed[i]));
      }
    }
    if (cloud_topics_.empty()) {
      cloud_topics_.push_back("/camera/depth/points_cam");
      cloud_topics_.push_back("/camera2/depth/points_cam");
    }

    scan_sub_ = nh_.subscribe(scan_topic_, 1, &SensorFusionNode::scanCallback, this,
                              ros::TransportHints().tcpNoDelay());
    latest_clouds_.resize(cloud_topics_.size());
    for (std::size_t i = 0; i < cloud_topics_.size(); ++i) {
      cloud_subs_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(
          cloud_topics_[i], 1,
          boost::bind(&SensorFusionNode::cloudCallback, this, i, boost::placeholders::_1)));
      ROS_INFO("sensor_fusion 订阅点云 %s", cloud_topics_[i].c_str());
    }

    scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(output_scan_topic_, 1);
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(1.0, publish_rate_)),
                             &SensorFusionNode::publish, this);
    ROS_INFO("sensor_fusion: %s + %zu 路点云 -> %s , %s",
             scan_topic_.c_str(), cloud_topics_.size(),
             output_scan_topic_.c_str(), output_cloud_topic_.c_str());
  }

private:
  void scanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_scan_ = msg;
  }

  void cloudCallback(std::size_t index, const sensor_msgs::PointCloud2ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_clouds_[index] = msg;
  }

  bool lookup(const std::string& source, const ros::Time& stamp, tf2::Transform* out) {
    geometry_msgs::TransformStamped stamped;
    try {
      stamped = tf_buffer_.lookupTransform(target_frame_, source, stamp, ros::Duration(tf_timeout_));
    } catch (const tf2::TransformException&) {
      try {
        stamped = tf_buffer_.lookupTransform(target_frame_, source, ros::Time(0), ros::Duration(0.02));
      } catch (const tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(2.0, "融合 TF %s -> %s 失败: %s", source.c_str(), target_frame_.c_str(), ex.what());
        return false;
      }
    }
    tf2::fromMsg(stamped.transform, *out);
    return true;
  }

  void addPoint(double x, double y, double z, bool apply_height_filter,
                ScanBins* bins, std::vector<std::array<float, 3>>* cloud,
                std::unordered_set<VoxelKey, VoxelHash>* voxels) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return;
    }
    if (apply_height_filter && (z < min_height_ || z > max_height_)) {
      return;
    }
    const double range = std::hypot(x, y);
    if (range < min_range_ || range > max_range_) {
      return;
    }
    if (insideAxisAlignedFootprint(x, y, footprint_half_x_, footprint_half_y_)) {
      return;
    }
    // 扫描先按角度取最小距离，不能被体素去重丢掉，否则空 bin 会被代价地图当成自由空间清掉。
    markMinRange(*bins, std::atan2(y, x), static_cast<float>(range));
    if (voxel_size_ > 1e-4) {
      VoxelKey key{
          static_cast<int>(std::floor(x / voxel_size_)),
          static_cast<int>(std::floor(y / voxel_size_)),
          static_cast<int>(std::floor(z / voxel_size_))};
      if (!voxels->insert(key).second) {
        return;
      }
    }
    if (static_cast<int>(cloud->size()) < max_cloud_points_) {
      cloud->push_back({static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
    }
  }

  void consumeScan(const sensor_msgs::LaserScan& scan, ScanBins* bins,
                   std::vector<std::array<float, 3>>* cloud,
                   std::unordered_set<VoxelKey, VoxelHash>* voxels) {
    tf2::Transform tf_scan;
    if (!lookup(scan.header.frame_id, scan.header.stamp, &tf_scan)) {
      return;
    }
    const int count = static_cast<int>(scan.ranges.size());
    for (int i = 0; i < count; ++i) {
      const float range = scan.ranges[static_cast<std::size_t>(i)];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const tf2::Vector3 point = tf_scan * tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0.0);
      // 二维雷达在扫描平面上，不再按高度丢掉，否则水平安装的雷达会被滤光。
      addPoint(point.x(), point.y(), point.z(), false, bins, cloud, voxels);
    }
  }

  void consumeCloud(const sensor_msgs::PointCloud2& input, ScanBins* bins,
                    std::vector<std::array<float, 3>>* cloud,
                    std::unordered_set<VoxelKey, VoxelHash>* voxels) {
    tf2::Transform tf_cloud;
    if (!lookup(input.header.frame_id, input.header.stamp, &tf_cloud)) {
      return;
    }
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(input, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(input, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(input, "z");
      int index = 0;
      const int stride = std::max(1, point_stride_);
      for (; ix != ix.end(); ++ix, ++iy, ++iz, ++index) {
        if (index % stride != 0) {
          continue;
        }
        const tf2::Vector3 point = tf_cloud * tf2::Vector3(*ix, *iy, *iz);
        addPoint(point.x(), point.y(), point.z(), true, bins, cloud, voxels);
      }
    } catch (const std::runtime_error& ex) {
      ROS_WARN_THROTTLE(2.0, "点云 %s 没有 xyz: %s", input.header.frame_id.c_str(), ex.what());
    }
  }

  void publish(const ros::TimerEvent&) {
    sensor_msgs::LaserScanConstPtr scan;
    std::vector<sensor_msgs::PointCloud2ConstPtr> clouds;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      scan = latest_scan_;
      clouds = latest_clouds_;
    }

    const double increment = angle_increment_deg_ * std::acos(-1.0) / 180.0;
    const int bins_n = std::max(8, static_cast<int>(std::lround((2.0 * std::acos(-1.0)) / increment)));
    ScanBins bins = makeScanBins(-std::acos(-1.0), increment, bins_n,
                                 std::numeric_limits<float>::infinity());
    std::vector<std::array<float, 3>> points;
    std::unordered_set<VoxelKey, VoxelHash> voxels;
    points.reserve(4096);

    if (scan) {
      consumeScan(*scan, &bins, &points, &voxels);
    }
    for (const auto& cloud : clouds) {
      if (cloud) {
        consumeCloud(*cloud, &bins, &points, &voxels);
      }
    }

    const ros::Time stamp = ros::Time::now();
    sensor_msgs::LaserScan out_scan;
    out_scan.header.stamp = stamp;
    out_scan.header.frame_id = target_frame_;
    out_scan.angle_min = static_cast<float>(bins.angle_min);
    out_scan.angle_increment = static_cast<float>(bins.angle_increment);
    out_scan.angle_max = static_cast<float>(bins.angle_min + bins.angle_increment * (bins.size - 1));
    out_scan.range_min = static_cast<float>(min_range_);
    out_scan.range_max = static_cast<float>(max_range_);
    out_scan.scan_time = static_cast<float>(1.0 / std::max(1.0, publish_rate_));
    out_scan.ranges.swap(bins.ranges);
    scan_pub_.publish(out_scan);

    sensor_msgs::PointCloud2 out_cloud;
    out_cloud.header.stamp = stamp;
    out_cloud.header.frame_id = target_frame_;
    sensor_msgs::PointCloud2Modifier modifier(out_cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> ox(out_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(out_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(out_cloud, "z");
    for (const auto& point : points) {
      *ox = point[0];
      *oy = point[1];
      *oz = point[2];
      ++ox;
      ++oy;
      ++oz;
    }
    cloud_pub_.publish(out_cloud);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber scan_sub_;
  std::vector<ros::Subscriber> cloud_subs_;
  ros::Publisher scan_pub_;
  ros::Publisher cloud_pub_;
  ros::Timer timer_;
  std::mutex mutex_;

  std::string scan_topic_;
  std::string output_scan_topic_;
  std::string output_cloud_topic_;
  std::string target_frame_;
  std::vector<std::string> cloud_topics_;
  double min_height_;
  double max_height_;
  double min_range_;
  double max_range_;
  double voxel_size_;
  int point_stride_;
  double angle_increment_deg_;
  double publish_rate_;
  double tf_timeout_;
  double footprint_half_x_;
  double footprint_half_y_;
  int max_cloud_points_;

  sensor_msgs::LaserScanConstPtr latest_scan_;
  std::vector<sensor_msgs::PointCloud2ConstPtr> latest_clouds_;
};

}  // namespace amr_obstacle_fusion

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_fusion");
  amr_obstacle_fusion::SensorFusionNode node;
  ros::spin();
  return 0;
}
