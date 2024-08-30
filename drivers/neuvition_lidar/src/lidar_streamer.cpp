#define PCL_NO_PRECOMPILE
#include "neuv_defs.hpp"
#include "neuvition_lidar/handler_allocator.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/timer.hpp>
#include <boost/version.hpp>
#include <chrono>
#include <cstdio>
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include <stdlib.h>
#include <string>

// Define PointType
struct PointXYZRGBATI //定义点类型结构
{
  PCL_ADD_POINT4D; // This adds the members x,y,z which can also be accessed
                   // using the point (which is float[4])
  PCL_ADD_UNION_RGB;
  uint32_t time_sec;
  uint32_t time_usec;
  uint32_t intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符对齐操作
} EIGEN_ALIGN16;                  // 强制SSE对齐

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBATI, // 注册点类型宏
    (float, x, x)(float, y, y)(float, z, z)(uint32_t, rgba,
                                            rgba)(uint32_t, time_sec, time_sec)(
        uint32_t, time_usec, time_usec)(uint32_t, intensity, intensity))

typedef PointXYZRGBATI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class LidarStreamer : public ::rclcpp::Node {
public:
  explicit LidarStreamer(rclcpp::NodeOptions node_options);
  ~LidarStreamer(void);
  void neuProcessImage(sensor_msgs::msg::Image &);
  void neuProcessPoint(PointCloudT &);
  void neuProcessIMU(sensor_msgs::msg::Imu &);

  int m_param_color_mode;

private:
  void AcquireLidarPC(void);

  void neuConnect();
  void neuStartScan();
  void neuStartData();
  void neuStopScan();
  void neuStopData();
  void neuDisconnect();
  void neuSetLaserInterval(int);
  void neuSetFrameRate(int);
  void neuSetLaserPower(int);
  void neuSetColorMode(int);
  void neuVideoFusion(bool);
  void neuSet_g_filter_enabled(bool);
  void neuSet_g_gaus_fit_enabled(bool);
  void neuSet_g_linear_fit_enabled(bool);
  int neuInit();
  void neuProcessCameraLadar(std::vector<long int>);

  rclcpp::Publisher<::sensor_msgs::msg::PointCloud2>::SharedPtr
      m_lpc_publisher{};
  rclcpp::Publisher<::sensor_msgs::msg::Image>::SharedPtr m_img_publisher{};
  rclcpp::Publisher<::sensor_msgs::msg::Imu>::SharedPtr m_imu_publisher{};

  std::unique_ptr<std::thread> m_p_lpc_producer;

  std::string m_param_lidar_ns;
  std::string m_param_frame_id_cloud;
  std::string m_param_frame_id_image;
  std::string m_param_frame_id_imu;
  int m_param_laser_power;
  int m_param_laser_interval;
  int m_param_frame_rate;
  int m_param_upstream_port;
  std::string m_param_upstream_host;
  bool m_param_video_fusion;
  int m_param_isImageRotate;
  int m_param_ifilterSt;
  int m_param_isTimemode;
  bool m_param_use_sys_default_qos;
  bool m_param_use_ptp_time;
  bool m_param_viz;
};

double get_timestamp(void) {
  struct timeval now;
  gettimeofday(&now, 0);

  return (double)(now.tv_sec) + (double)(now.tv_usec) / 1000000.0;
}

void showretval(int ret) {
  if (ret == 0)
    return;
  std::cout << "ret:" << ret << std::endl;
}

class gEventHandler : public neuvition::INeuvEvent {
private:
  LidarStreamer *neudrv;

  neuvition::NeuvUnits predata;

  uint32_t iFrontFrameTime = 0;
  uint32_t iNowFrameTime = 0;
  uint32_t iMsecTime = 0;

  /* YELLOW ff/ff/00 RED ff/00/00 MAGENTA ff/00/ff BLUE 00/00/ff CYAN 00/ff/ff
   * GREEN 00/ff/00 */
  const uint32_t coloredge[6] = {0xffff00, 0xff0000, 0xff00ff,
                                 0x0000ff, 0x00ffff, 0x00ff00};

  uint32_t tof_cycle = 65000; // orig:9000 2m:13000 10m:65000 30m:200000

public:
  gEventHandler(LidarStreamer *_drv) : neudrv(_drv) {
    std::cout << "The callback instance of driver initialized" << std::endl;
  }

  virtual void on_connect(int code, const char *msg) {
    std::cout << std::fixed << std::setprecision(6) << get_timestamp()
              << "neuvtion: on_connect: " << code << "(" << msg << ")"
              << std::endl;
    if (code == 0) {
      int ret = neuvition::set_reconnect_params(false, 5);
      showretval(ret);
    }
  }

  virtual void on_disconnect(int code) {
    std::cout << std::fixed << std::setprecision(6) << get_timestamp()
              << "neuvition: on_disconnect: " << code << std::endl;
  }

  virtual void on_lidar_info_status(neuvition::LidarInfoStatus *) {
    // TODO: nothing
  }

  virtual void on_response(int code, enum neuvition::neuv_cmd_code cmd) {
    // std::cout << std::fixed << std::setprecision(6) << get_timestamp() <<
    // "neuvition: on_response[ " << cmd << " ]: " << code << std::endl;
    if (cmd == neuvition::NEUV_CMD_START_SCAN && code == 0) {
      std::cout << std::fixed << std::setprecision(6) << get_timestamp()
                << "neuvition: scanning started..." << std::endl;
    }
    if (cmd == neuvition::NEUV_CMD_STOP_SCAN && code == 0) {
      std::cout << std::fixed << std::setprecision(6) << get_timestamp()
                << "neuvition: scanning stopped..." << std::endl;
    }
    if (cmd == neuvition::NEUV_CMD_START_STREAM && code == 0) {
      std::cout << std::fixed << std::setprecision(6) << get_timestamp()
                << "neuvition: streaming started..." << std::endl;
    }
    if (cmd == neuvition::NEUV_CMD_STOP_STREAM && code == 0) {
      std::cout << std::fixed << std::setprecision(6) << get_timestamp()
                << "neuvition: streaming stopped..." << std::endl;
    }
    if (cmd == neuvition::NEUV_CMD_GET_PARAMS && code == 0) {
      std::cout << std::fixed << std::setprecision(6) << get_timestamp()
                << "neuvition: device parameter is  synced..." << std::endl;
    }
    if (cmd == neuvition::NEUV_CMD_GPS_UPDATED && code == 0) {
      std::cout << std::fixed << std::setprecision(6) << get_timestamp()
                << "neuvition: gps info message..." << std::endl;
    }
  }

  virtual void on_framedata(int, int64_t, const neuvition::NeuvUnits &,
                            const neuvition::nvid_t &,
                            const neuvition::NeuvWireDatas &) {}

  virtual void on_framedata(int, int64_t, const neuvition::NeuvUnits &,
                            const neuvition::nvid_t &,
                            const neuvition::NeuvCt &) {}

  virtual void on_framedata(int code, int64_t microsec,
                            const neuvition::NeuvUnits &data,
                            const neuvition::nvid_t &frame_id) {

    PointCloudT cloud_;
    cloud_.reserve(data.size());

    predata = data;

    int i = 0;
    for (neuvition::NeuvUnits::const_iterator iter = data.begin();
         iter != data.end(); iter++) {
      const neuvition::NeuvUnit &np = (*iter);
      PointT point;

      // PointXYZRGBATI
      point.x = np.x * 0.001;
      point.y = np.y * 0.001;
      point.z = np.z * 0.001;

      if (np.z == 0)
        continue;

      point.a = 255;
      point.intensity = np.intensity;

      point.time_sec = np.time_sec;
      point.time_usec = np.time_usec;

      if (i == 0) {
        iNowFrameTime = point.time_sec;
      }

      i++;
#if 1

      if (neudrv->m_param_color_mode == 0) { /* 0:polychrome */
        uint32_t tofcolor = np.tof % tof_cycle;
        uint32_t tofclass = tofcolor / (tof_cycle / 6);
        uint32_t tofreman = tofcolor % (tof_cycle / 6);
        uint32_t cbase = coloredge[(tofclass + 0) % 6];
        uint32_t cnext = coloredge[(tofclass + 1) % 6];
        uint32_t ccand = cnext & cbase; // UNUSED(ccand);
        uint32_t ccxor = cnext ^ cbase;
        int shift = __builtin_ffs(ccxor) - 1;
        int xincr = (cnext > cbase) ? 1 : -1;
        uint32_t crender =
            cbase +
            xincr * ((int)(tofreman * (256.0 / (tof_cycle / 6))) << shift);
        point.r = (crender & 0xff0000) >> 16;
        point.g = (crender & 0x00ff00) >> 8;
        point.b = (crender & 0x0000ff) >> 0;
      } else if (neudrv->m_param_color_mode == 1) { /* 1:camera */
        point.r = np.r;
        point.g = np.g;
        point.b = np.b;
      } else if (neudrv->m_param_color_mode == 2) { /* 2:intensity */
        // std::cout << "intensity" << endl;
        uint32_t tofcolor = np.intensity;
        uint32_t tofclass = tofcolor / (256 / 6);
        uint32_t tofreman = tofcolor % (256 / 6);
        uint32_t cbase = coloredge[(tofclass + 0) % 6];
        uint32_t cnext = coloredge[(tofclass + 1) % 6];
        uint32_t ccand = cnext & cbase; // UNUSED(ccand);
        uint32_t ccxor = cnext ^ cbase;
        int shift = __builtin_ffs(ccxor) - 1;
        int xincr = (cnext > cbase) ? 1 : -1;
        uint32_t crender =
            cbase + xincr * ((int)(tofreman * (256.0 / (256 / 6))) << shift);
        point.r = (crender & 0xff0000) >> 16;
        point.g = (crender & 0x00ff00) >> 8;
        point.b = (crender & 0x0000ff) >> 0;
      }
#endif

      cloud_.push_back(point);
    }
    if (iFrontFrameTime == 0) {
      iMsecTime = 0;
    } else {
      iMsecTime += ((iNowFrameTime - iFrontFrameTime) / 1000);
    }

    if (iMsecTime > 1000) {
      iMsecTime -= 1000;
    }

    /*
    Rotating point cloud
    */

    iFrontFrameTime = iNowFrameTime;
    neudrv->neuProcessPoint(cloud_);
  }

  virtual void on_imudata(int code, int64_t microsec,
                          const neuvition::NeuvUnits &data,
                          const neuvition::ImuData &imu) {
    sensor_msgs::msg::Imu imu_data;

#define QP(n) (1.0f / (1 << n))

    imu_data.orientation.x = QP(14) * imu.quat_i;
    imu_data.orientation.y = QP(14) * imu.quat_j;
    imu_data.orientation.z = QP(14) * imu.quat_k;
    imu_data.orientation.w = QP(14) * imu.quat_r;

    imu_data.linear_acceleration.x = 0.0f;
    imu_data.linear_acceleration.y = 0.0f;
    imu_data.linear_acceleration.z = 0.0f;

    imu_data.angular_velocity.x = 0.0f;
    imu_data.angular_velocity.y = 0.0f;
    imu_data.angular_velocity.z = 0.0f;

    neudrv->neuProcessIMU(imu_data);
  }

  virtual void on_pczdata(bool status) {}
  virtual void on_Ladar_Camera(
      neuvition::NeuvCameraLadarDatas
          *neuvcameraladarpos) { // neuvition::NeuvCameraLadarDatas
                                 // 数据容器，通过循环遍历取出里面的单个元素
  }
  virtual void on_framestart1(int nCode) {}
  virtual void on_framestart2(int nCode) {}

  virtual void on_mjpgdata(int code, int64_t microsec, cv::Mat Mat) {
    if (Mat.empty())
      return;
    if (0) {
      transpose(Mat, Mat);
      flip(Mat, Mat, 1);
      transpose(Mat, Mat);
      flip(Mat, Mat, 1);
    } else {
    }
    sensor_msgs::msg::Image img_msg;
    std_msgs::msg::Header header;

    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, "bgr8", Mat);
    img_bridge.toImageMsg(img_msg);

    neudrv->neuProcessImage(img_msg);
  }
};

void LidarStreamer::AcquireLidarPC(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Neuvition lidar driver starts");

  // start the driver
  while (rclcpp::ok()) {
    if (neuInit()) {
      rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(5000)));
      continue;
    }
    break;
  }

  // loop until shut down or end of file
  while (rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
  }

  neuStopData();
  neuStopScan();
  neuDisconnect();

  RCLCPP_INFO_STREAM(this->get_logger(), "Neuvition lidar driver exits");
  return;
}

LidarStreamer::LidarStreamer(rclcpp::NodeOptions node_options)
    : rclcpp::Node("neuvition_lidar_streamer_node",
                   node_options.allow_undeclared_parameters(true)) {
  declare_parameter("lidar_ns", "neuvition");
  declare_parameter("cloud_frame_id", "neuvition_lidar");
  declare_parameter("image_frame_id", "neuvition_camera");
  declare_parameter("imu_frame_id", "neuvition_imu");
  declare_parameter("laser_power",
                    10); // 50 percents is recommended according to the SDK
                         // manual, default to 10 to make it safe
  declare_parameter("laser_interval",
                    1); // 1:300K is recommended for M1-R according to the SDK
                        // manual, 3:500KHz recommended for other models
  declare_parameter("frame_rate", 10);
  declare_parameter("color_mode", 0);
  declare_parameter("upstream_port", 6668);
  declare_parameter("upstream_host", "192.168.1.101");
  declare_parameter("video_fusion", false);
  declare_parameter("laser_image", 0);
  declare_parameter("laser_filters", 0);
  declare_parameter("laser_time", 0);
  declare_parameter("use_system_default_qos", true);
  declare_parameter("use_ptp_time", true);
  declare_parameter("viz", false);
  m_param_lidar_ns = get_parameter("lidar_ns").as_string();
  m_param_frame_id_cloud = get_parameter("cloud_frame_id").as_string();
  m_param_frame_id_image = get_parameter("image_frame_id").as_string();
  m_param_frame_id_imu = get_parameter("imu_frame_id").as_string();
  m_param_laser_power = get_parameter("laser_power").as_int();
  m_param_laser_interval = get_parameter("laser_interval").as_int();
  m_param_frame_rate = get_parameter("frame_rate").as_int();
  m_param_color_mode = get_parameter("color_mode").as_int();
  m_param_upstream_port = get_parameter("upstream_port").as_int();
  m_param_upstream_host = get_parameter("upstream_host").as_string();
  m_param_video_fusion = get_parameter("video_fusion").as_bool();
  m_param_isImageRotate = get_parameter("laser_image").as_int();
  m_param_ifilterSt = get_parameter("laser_filters").as_int();
  m_param_isTimemode = get_parameter("laser_time").as_int();
  m_param_use_sys_default_qos =
      get_parameter("use_system_default_qos").as_bool();
  m_param_use_ptp_time =
      get_parameter("use_ptp_time").as_bool();  // currently not in use
  m_param_viz = get_parameter("viz").as_bool(); // currently not in use

  rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
  system_default_qos.keep_last(
      10); // to allow intra-process comm in a composition container
  system_default_qos.durability_volatile(); // to allow intra-process comm in a
                                            // composition container
  rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
  auto selected_qos =
      m_param_use_sys_default_qos ? system_default_qos : sensor_data_qos;
  m_lpc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/" + m_param_lidar_ns + "/points", selected_qos);
  m_img_publisher = this->create_publisher<sensor_msgs::msg::Image>(
      "/" + m_param_lidar_ns + "/image", selected_qos);
  m_imu_publisher = this->create_publisher<sensor_msgs::msg::Imu>(
      "/" + m_param_lidar_ns + "/imu", selected_qos);

  RCLCPP_INFO_STREAM(this->get_logger(), "Spinning up processing thread");
  m_p_lpc_producer = std::unique_ptr<std::thread>(
      new std::thread(&LidarStreamer::AcquireLidarPC, this));
}

LidarStreamer::~LidarStreamer() {
  if (m_p_lpc_producer != nullptr)
    m_p_lpc_producer->join();

  // clean up
  if (m_param_viz)
    cv::destroyAllWindows();
}

void LidarStreamer::neuConnect() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Start connecting");
  neuvition::INeuvEvent *phandler = new gEventHandler(this);

  int ret = neuvition::setup_client(m_param_upstream_host.c_str(),
                                    m_param_upstream_port, phandler, false);

  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuStartScan() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Start scan");
  int ret = neuvition::start_scan();
  showretval(ret); // ask device to start scanning
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuStartData() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Start stream");
  int ret = neuvition::start_stream();
  showretval(ret); // ask device to start streaming
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuStopScan() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Stop scan");
  int ret = neuvition::stop_scan();
  showretval(ret); // ask device to stop scanning
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuStopData() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Stop stream");
  int ret = neuvition::stop_stream();
  showretval(ret); // ask device to stop streaming
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuDisconnect() {
  RCLCPP_INFO_STREAM(this->get_logger(), "Disconnecting");
  int ret = neuvition::teardown_client();
  showretval(ret);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuSetLaserInterval(int value) {
  if (value < 0 or value > 6)
    return;
  // accepted values: 0:200KHz, 1:300K, 2:400K, 3:500K, 4:750K, 5:1M, 6:1.5M
  // 500K is recommended
  RCLCPP_INFO_STREAM(this->get_logger(), "Setting Laser Period to: " << value);
  int ret = neuvition::set_laser_interval(value);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(2000)));
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Confirming laser interval set to: " << neuvition::get_laser_interval());
}

void LidarStreamer::neuSetFrameRate(int fps) {
  if (fps < 1 or fps > 30)
    return;
  // accepted fps values: 30 20 15 10 6 5 3 1
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Setting Frame Rate to: " << fps << " fps");
  int ret = neuvition::set_frame_frequency(fps);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(2000)));
  RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Confirming frame rate set to: " << neuvition::get_frame_frequency());
}

void LidarStreamer::neuSetLaserPower(int value) {
  if (value < 1 or value > 75)
    return;
  // max accepted value 75, recommended 50
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Setting Laser Power to: " << value << "%");
  int ret = neuvition::set_laser_power(value);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(2000)));
  RCLCPP_INFO_STREAM(this->get_logger(), "Confirming laser power set to: "
                                             << neuvition::get_laser_power());
}

void LidarStreamer::neuSetColorMode(int value) {
  if (value < 0 or value > 2)
    return;
  // 0:PolyChrome  1:Camera  2:Intensity
  m_param_color_mode = value;
  std::string mode;
  switch (value) {
  case 0: {
    mode = "PolyChrome";
    break;
  }
  case 1: {
    mode = "Camera";
    break;
  } // only valid when Video Fusion mode is on
  case 2: {
    mode = "Intensity";
    break;
  }
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Color Mode set to: " << mode.c_str());
}

void LidarStreamer::neuVideoFusion(bool value) {
  std::string status = value ? "Open" : "Close";
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Setting Video Fusion mode to: " << status);
  int ret = neuvition::set_camera_status(value);
  ret = neuvition::set_mjpg_curl(value);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuSet_g_filter_enabled(bool value) {
  std::string status = value ? "True" : "False";
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "neuSet_g_filter_enabled set to: " << status);
  int ret = neuvition::set_g_filter_enabled(value);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuSet_g_gaus_fit_enabled(bool value) {
  std::string status = value ? "True" : "False";
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "set_g_gaus_fit_enabled set to: " << status);
  int ret = neuvition::set_g_gaus_fit_enabled(value);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

void LidarStreamer::neuSet_g_linear_fit_enabled(bool value) {
  std::string status = value ? "True" : "False";
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "set_g_linear_fit_enabled set to: " << status);
  int ret = neuvition::set_g_linear_fit_enabled(value);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));
}

int LidarStreamer::neuInit() {
  neuConnect();

  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));

  if (neuvition::is_connected()) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Connected");

    double hfov = neuvition::get_hfov();
    double vfov = neuvition::get_vfov();
    int device_type = neuvition::get_device_type();

    neuvition::set_flip_axis(false, true); //  x y

    neuvition::set_npvt_value(1); // first

    neuStartScan();
    if (neuvition::is_scanning())
      RCLCPP_INFO_STREAM(this->get_logger(), "scanning");

    neuSetLaserPower(m_param_laser_power);
    neuSetLaserInterval(m_param_laser_interval);
    neuSetFrameRate(m_param_frame_rate);
    neuSetColorMode(m_param_color_mode);
    neuVideoFusion(m_param_video_fusion);

    neuStartData();
    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(500)));

    if (neuvition::is_streaming()) {
      RCLCPP_INFO_STREAM(this->get_logger(), "streaming");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(1000)));
  } else {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Couldn't connect to lidar");
    return -1;
  }
  return 0;
}

void LidarStreamer::neuProcessPoint(PointCloudT &cloud) {
  // need convert to pcl::PointXYZI,
  // otherwise the ROS_subscriber will popup warning 'Failed to find match for
  // field intensity'
  sensor_msgs::msg::PointCloud2 lpc_msg;

  if (m_param_video_fusion) {
    pcl::PointCloud<PointT> pcl_points;
    pcl_points.resize(cloud.size());

    for (int i = 0; i < cloud.size(); i++) {
      pcl_points.at(i).x = cloud.at(i).z; // rotate the cordinate system to be
                                          // algined with ROS convention
      pcl_points.at(i).y = cloud.at(i).x;
      pcl_points.at(i).z = cloud.at(i).y;
      pcl_points.at(i).r = cloud.at(i).r;
      pcl_points.at(i).g = cloud.at(i).g;
      pcl_points.at(i).b = cloud.at(i).b;
      pcl_points.at(i).a = 255;
      pcl_points.at(i).time_sec = cloud.at(i).time_sec;
      pcl_points.at(i).time_usec = cloud.at(i).time_usec;
      pcl_points.at(i).intensity = cloud.at(i).intensity;
    }

    pcl::toROSMsg(pcl_points, lpc_msg);
  } else { // use pcl::PointXYZI to save message size
    pcl::PointCloud<pcl::PointXYZI> pcl_points;
    pcl_points.resize(cloud.size());

    for (int i = 0; i < cloud.size(); i++) {
      pcl_points.at(i).x = cloud.at(i).z; // rotate the cordinate system to be
                                          // algined with ROS convention
      pcl_points.at(i).y = cloud.at(i).x;
      pcl_points.at(i).z = cloud.at(i).y;
      pcl_points.at(i).intensity = cloud.at(i).intensity;
    }

    pcl::toROSMsg(pcl_points, lpc_msg);
  }

  lpc_msg.header.stamp = rclcpp::Node::now();
  lpc_msg.header.frame_id = m_param_frame_id_cloud;

  m_lpc_publisher->publish(std::move(lpc_msg));
}

void LidarStreamer::neuProcessIMU(sensor_msgs::msg::Imu &msg) {
  msg.header.stamp = rclcpp::Node::now();
  msg.header.frame_id = m_param_frame_id_imu;

  m_imu_publisher->publish(std::move(msg));
}

void LidarStreamer::neuProcessImage(sensor_msgs::msg::Image &msg) {
  msg.header.stamp = rclcpp::Node::now();
  msg.header.frame_id = m_param_frame_id_image;

  m_img_publisher->publish(std::move(msg));
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(LidarStreamer)
