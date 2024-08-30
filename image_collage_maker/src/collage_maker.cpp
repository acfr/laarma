#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <cstdio>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdexcept>

#define MAX_NUM_IMG_SUPPORTED 9

// compile time for loop, check
// https://www.reddit.com/r/cpp/comments/l37uui/hello_guys_i_made_compile_time_for_loop_library/
// requires c++17
template <typename T, T... S, typename F>
constexpr void for_sequence(std::integer_sequence<T, S...>, F f) {
  (static_cast<void>(f(std::integral_constant<T, S>{})), ...);
}

template <auto n, typename F> constexpr void for_sequence(F f) {
  for_sequence(std::make_integer_sequence<decltype(n), n>{}, f);
}

using std::placeholders::_1;

// main node
class CollageMaker : public ::rclcpp::Node {
public:
  explicit CollageMaker(rclcpp::NodeOptions node_options);
  ~CollageMaker(void);

private:
  template <int ImgN> void image_msg_callback(const sensor_msgs::msg::Image &);
  void merge_n_publish_image(rclcpp::Time stamp);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      m_collage_image_publisher{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      m_image_subscribers[MAX_NUM_IMG_SUPPORTED];
  rclcpp::Time m_image_last_recv_time[MAX_NUM_IMG_SUPPORTED];
  bool m_image_ever_recved[MAX_NUM_IMG_SUPPORTED];
  cv::Mat m_latest_image[MAX_NUM_IMG_SUPPORTED];
  std::chrono::nanoseconds m_throttle_period_ns;
  std::chrono::nanoseconds m_timeout_period_ns;
  rclcpp::Time m_collage_last_publish_time;
  rclcpp::QoS m_selected_qos;

  int m_param_collage_rows;
  int m_param_collage_cols;
  bool m_param_collage_show_stamp;
  int m_param_image_width;
  int m_param_image_height;
  double m_param_msgs_per_sec;
  double m_param_image_timeout_sec;
  bool m_param_use_wall_clock;
  std::string m_param_frame_id;
  bool m_param_use_sys_default_qos;
  bool m_param_viz; // if to show opencv image
};

template <int ImgN>
void CollageMaker::image_msg_callback(const sensor_msgs::msg::Image &msg) {
  const auto &now =
      m_param_use_wall_clock ? rclcpp::Clock{}.now() : this->now();

  cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(msg, msg.encoding);
  if (cv_image_ptr->image.cols != m_param_image_width or
      cv_image_ptr->image.rows != m_param_image_height) {
    cv::resize(cv_image_ptr->image, m_latest_image[ImgN],
               cv::Size(m_param_image_width, m_param_image_height));
  } else
    m_latest_image[ImgN] = cv_image_ptr->image.clone();
  m_image_last_recv_time[ImgN] = now;
  m_image_ever_recved[ImgN] = true;

  if (m_collage_last_publish_time > now) {
    RCLCPP_WARN(
        get_logger(),
        "Detected jump back in time, resetting throttle period to now for.");
    m_collage_last_publish_time = now;
  }
  if ((now - m_collage_last_publish_time).nanoseconds() >=
      m_throttle_period_ns.count()) {
    merge_n_publish_image(msg.header.stamp);
    m_collage_last_publish_time = now;
  }

  return;
}

void CollageMaker::merge_n_publish_image(rclcpp::Time stamp) {
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = m_param_frame_id;

  cv::Size collage_size = cv::Size(m_param_image_width * m_param_collage_cols,
                                   m_param_image_height * m_param_collage_rows);
  if (m_param_collage_show_stamp)
    collage_size.height += 24;
  cv::Mat collage = cv::Mat(collage_size, CV_8UC3);
  collage.setTo(cv::Scalar(255, 255, 255));
  const auto &now =
      m_param_use_wall_clock ? rclcpp::Clock{}.now() : this->now();
  for (int row = 0; row < m_param_collage_rows; row++) {
    for (int col = 0; col < m_param_collage_cols; col++) {
      int ImgN = row * m_param_collage_cols + col;
      if (ImgN > MAX_NUM_IMG_SUPPORTED)
        continue;

      if (m_image_ever_recved[ImgN]) {
        auto time_since_last_ns =
            (now - m_image_last_recv_time[ImgN]).nanoseconds();
        if (time_since_last_ns >= m_timeout_period_ns.count()) { // timeout
          m_latest_image[ImgN] = cv::Mat(
              cv::Size(m_param_image_width, m_param_image_height), CV_8UC3);
          m_latest_image[ImgN].setTo(cv::Scalar(255, 255, 255));
          std::stringstream centre_x_text;
          centre_x_text << "Loss " << static_cast<int>(time_since_last_ns / 1e9)
                        << "s";
          cv::Scalar text_color = CV_RGB(0, 0, 0);
          double scale = 1.0;
          int thickness = 1;
          cv::Size rect =
              cv::getTextSize(centre_x_text.str(), cv::FONT_HERSHEY_SIMPLEX,
                              scale, thickness, 0);
          cv::putText(m_latest_image[ImgN], centre_x_text.str(),
                      cv::Point((m_latest_image[ImgN].cols - rect.width) / 2,
                                (m_latest_image[ImgN].rows + rect.height) / 2),
                      cv::FONT_HERSHEY_SIMPLEX, scale, text_color, thickness);
        }
      }

      cv::Mat roi = collage(
          cv::Rect(m_param_image_width * col, m_param_image_height * row,
                   m_param_image_width, m_param_image_height));
      m_latest_image[ImgN].copyTo(roi);
    }
  }
  if (m_param_collage_show_stamp) {
    std::chrono::time_point<std::chrono::system_clock> now2(
        std::chrono::system_clock::duration(now.nanoseconds()));
    std::time_t t = std::chrono::system_clock::to_time_t(now2);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d %T");
    cv::putText(collage, oss.str(), cv::Point(2, collage.rows - 2),
                cv::FONT_HERSHEY_PLAIN, 2.0, CV_RGB(0, 0, 0), 2);
  }

  try {
    cv_bridge::CvImage img_bridge =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, collage);
    img_bridge.toImageMsg(img_msg);
  } catch (...) {
    throw std::runtime_error("Runtime error in publish_image()");
  }

  m_collage_image_publisher->publish(std::move(img_msg));

  if (m_param_viz) {
    cv::Size cv_windowSize = cv::Size(collage.cols, collage.rows);
    std::string window_title = "Image Collage";
    cv::namedWindow(window_title, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_title, cv_windowSize);
    cv::imshow(window_title, collage);
    cv::waitKey(1);
  }
}

CollageMaker::CollageMaker(rclcpp::NodeOptions node_options)
    : rclcpp::Node("collage_maker_node",
                   node_options.allow_undeclared_parameters(true)),
      m_selected_qos(rclcpp::SystemDefaultsQoS()) {
  declare_parameter("use_system_default_qos", true);
  declare_parameter("viz", false);
  m_param_use_sys_default_qos =
      get_parameter("use_system_default_qos").as_bool();
  m_param_viz = get_parameter("viz").as_bool();

  declare_parameter("collage_rows", 2);
  declare_parameter("collage_cols", 2);
  declare_parameter("collage_show_stamp", true);
  declare_parameter("image_width", 640);
  declare_parameter("image_height", 480);
  declare_parameter("msgs_per_sec", 1.0);
  declare_parameter("image_timeout_sec", 2.0);
  declare_parameter("use_wall_clock", true);
  declare_parameter("frame_id", "collage_image");
  m_param_collage_rows = get_parameter("collage_rows").as_int();
  m_param_collage_cols = get_parameter("collage_cols").as_int();
  m_param_collage_show_stamp = get_parameter("collage_show_stamp").as_bool();
  m_param_image_width = get_parameter("image_width").as_int();
  m_param_image_height = get_parameter("image_height").as_int();
  m_param_msgs_per_sec = get_parameter("msgs_per_sec").as_double();
  m_param_image_timeout_sec = get_parameter("image_timeout_sec").as_double();
  m_param_use_wall_clock = get_parameter("use_wall_clock").as_bool();
  m_param_frame_id = get_parameter("frame_id").as_string();

  if (m_param_collage_rows < 1)
    m_param_collage_rows = 1;
  if (m_param_collage_rows > 3)
    m_param_collage_rows = 3;
  if (m_param_collage_cols < 1)
    m_param_collage_cols = 1;
  if (m_param_collage_cols > 3)
    m_param_collage_cols = 3;

  m_throttle_period_ns = rclcpp::Rate(m_param_msgs_per_sec).period();
  m_timeout_period_ns = rclcpp::Rate(m_param_image_timeout_sec).period();
  m_collage_last_publish_time =
      m_param_use_wall_clock ? rclcpp::Clock{}.now() : this->now();

  for (int ImgN = 0; ImgN < MAX_NUM_IMG_SUPPORTED; ImgN++) {
    m_image_ever_recved[ImgN] = false;
    m_latest_image[ImgN] =
        cv::Mat(cv::Size(m_param_image_width, m_param_image_height), CV_8UC3);
    m_latest_image[ImgN].setTo(cv::Scalar(255, 255, 255));
    cv::Scalar text_color = CV_RGB(0, 0, 0);
    double scale = 1.0;
    int thickness = 1;
    std::string centre_x_text("No Signal");
    cv::Size rect = cv::getTextSize(centre_x_text, cv::FONT_HERSHEY_SIMPLEX,
                                    scale, thickness, 0);
    cv::putText(m_latest_image[ImgN], centre_x_text,
                cv::Point((m_latest_image[ImgN].cols - rect.width) / 2,
                          (m_latest_image[ImgN].rows + rect.height) / 2),
                cv::FONT_HERSHEY_SIMPLEX, scale, text_color, thickness);
  }

  rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
  system_default_qos.keep_last(
      10); // to allow intra-process comm in a composition container
  system_default_qos.durability_volatile(); // to allow intra-process comm in a
                                            // composition container
  rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
  m_selected_qos =
      m_param_use_sys_default_qos ? system_default_qos : sensor_data_qos;
  m_collage_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
      "out/image_collage", m_selected_qos);
  // compile time for loop, check
  // https://www.reddit.com/r/cpp/comments/l37uui/hello_guys_i_made_compile_time_for_loop_library/
  for_sequence<MAX_NUM_IMG_SUPPORTED>([this](auto ImgN) {
    // ImgN is an integral constant from 0 to MAX_NUM_IMG_SUPPORTED-1 inclusive,
    // can use it as template parameter
    std::stringstream topic_name;
    topic_name << "in/image" << ImgN;
    m_image_subscribers[ImgN] =
        this->create_subscription<sensor_msgs::msg::Image>(
            topic_name.str(), m_selected_qos,
            std::bind(&CollageMaker::image_msg_callback<ImgN>, this, _1));
  });
}

CollageMaker::~CollageMaker() {
  // clean up
  if (m_param_viz)
    cv::destroyAllWindows();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(CollageMaker)
