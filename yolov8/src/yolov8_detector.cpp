#define BOOST_BIND_NO_PLACEHOLDERS
#include "chrono"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "yolov8/yolov8.hpp"
#include <chrono>
#include <cstdio>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>

using std::placeholders::_1;

const std::vector<std::string> CLASS_NAMES = {
    "person",        "bicycle",      "car",
    "motorcycle",    "airplane",     "bus",
    "train",         "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench",        "bird",
    "cat",           "dog",          "horse",
    "sheep",         "cow",          "elephant",
    "bear",          "zebra",        "giraffe",
    "backpack",      "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",
    "skis",          "snowboard",    "sports ball",
    "kite",          "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket",
    "bottle",        "wine glass",   "cup",
    "fork",          "knife",        "spoon",
    "bowl",          "banana",       "apple",
    "sandwich",      "orange",       "broccoli",
    "carrot",        "hot dog",      "pizza",
    "donut",         "cake",         "chair",
    "couch",         "potted plant", "bed",
    "dining table",  "toilet",       "tv",
    "laptop",        "mouse",        "remote",
    "keyboard",      "cell phone",   "microwave",
    "oven",          "toaster",      "sink",
    "refrigerator",  "book",         "clock",
    "vase",          "scissors",     "teddy bear",
    "hair drier",    "toothbrush"};

const std::vector<std::vector<unsigned int>> COLORS = {
    {0, 114, 189},   {217, 83, 25},   {237, 177, 32},  {126, 47, 142},
    {119, 172, 48},  {77, 190, 238},  {162, 20, 47},   {76, 76, 76},
    {153, 153, 153}, {255, 0, 0},     {255, 128, 0},   {191, 191, 0},
    {0, 255, 0},     {0, 0, 255},     {170, 0, 255},   {85, 85, 0},
    {85, 170, 0},    {85, 255, 0},    {170, 85, 0},    {170, 170, 0},
    {170, 255, 0},   {255, 85, 0},    {255, 170, 0},   {255, 255, 0},
    {0, 85, 128},    {0, 170, 128},   {0, 255, 128},   {85, 0, 128},
    {85, 85, 128},   {85, 170, 128},  {85, 255, 128},  {170, 0, 128},
    {170, 85, 128},  {170, 170, 128}, {170, 255, 128}, {255, 0, 128},
    {255, 85, 128},  {255, 170, 128}, {255, 255, 128}, {0, 85, 255},
    {0, 170, 255},   {0, 255, 255},   {85, 0, 255},    {85, 85, 255},
    {85, 170, 255},  {85, 255, 255},  {170, 0, 255},   {170, 85, 255},
    {170, 170, 255}, {170, 255, 255}, {255, 0, 255},   {255, 85, 255},
    {255, 170, 255}, {85, 0, 0},      {128, 0, 0},     {170, 0, 0},
    {212, 0, 0},     {255, 0, 0},     {0, 43, 0},      {0, 85, 0},
    {0, 128, 0},     {0, 170, 0},     {0, 212, 0},     {0, 255, 0},
    {0, 0, 43},      {0, 0, 85},      {0, 0, 128},     {0, 0, 170},
    {0, 0, 212},     {0, 0, 255},     {0, 0, 0},       {36, 36, 36},
    {73, 73, 73},    {109, 109, 109}, {146, 146, 146}, {182, 182, 182},
    {219, 219, 219}, {0, 114, 189},   {80, 183, 189},  {128, 128, 0}};

// blocking queue
template <typename T> class BlockingQueue {
public:
  BlockingQueue(const BlockingQueue &) = delete; // make the class noncopyable
  BlockingQueue &
  operator=(const BlockingQueue &) = delete; // make the class noncopyable
  BlockingQueue(int queue_limit) { m_limit = queue_limit; }
  void push(const T &val) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_queue.push(val);
    while (m_queue.size() > m_limit)
      m_queue.pop();
    m_cond.notify_one();
  }
  void pop(T &val) {
    std::unique_lock<std::mutex> lock(m_mutex);
    while (m_queue.empty()) {
      m_cond.wait(lock);
    }
    val = m_queue.front();
    m_queue.pop();
  }
  bool try_pop(T &val) {
    std::unique_lock<std::mutex> lock(m_mutex);
    if (!m_queue.empty()) {
      val = m_queue.front();
      m_queue.pop();
      return true;
    }
    return false;
  }
  size_t size() const {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.size();
  }
  bool empty() const {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.empty();
  }
  bool full() const {
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_queue.size() >= m_limit;
  }

private:
  std::queue<T> m_queue;
  unsigned int m_limit;
  std::condition_variable m_cond;
  mutable std::mutex m_mutex;
};

struct CvImage {
  rclcpp::Time stamp;
  std::string frame_id;
  std::string encoding;
  cv::Mat img;
};

// main node

class Yolov8Detector : public ::rclcpp::Node {
public:
  explicit Yolov8Detector(rclcpp::NodeOptions node_options);
  ~Yolov8Detector(void);

private:
  void ProcessImages(void);
  void PublishImages(void);

  void image_topic_callback(const sensor_msgs::msg::Image &) const;
  void publish_image(const CvImage &);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr
      m_image_subscription{};
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_image_publisher{};
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr
      m_detection_publisher{};

  std::unique_ptr<std::thread> m_p_image_consumer;
  std::unique_ptr<std::thread> m_p_image_publisher;

  std::unique_ptr<YOLOv8> m_p_yolov8 = nullptr;

  std::unique_ptr<BlockingQueue<CvImage>> m_p_received_image_queue;
  std::unique_ptr<BlockingQueue<CvImage>> m_p_processed_image_queue;

  std::string
      m_param_engine_file_path; // file path to the yolov8 TensorRT engine
  std::vector<std::string>
      m_param_class_names; // a list of class names from yolov8 detector
  int m_param_num_labels;
  int m_param_topk;
  int m_param_image_height;
  double m_param_score_thres;
  double m_param_iou_thres;
  bool m_param_use_sys_default_qos;
  bool m_param_viz; // if to show opencv image

  std::string m_param_detector_ns; // camera namespace
};

void Yolov8Detector::image_topic_callback(
    const sensor_msgs::msg::Image &msg) const {
  rclcpp::Time stamp = msg.header.stamp;
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Callback: Received image msg: "
                                              << stamp.nanoseconds() / 1e6);

  cv_bridge::CvImagePtr cv_image_ptr =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "size " << cv_image_ptr->image.size[0] << ","
                              << cv_image_ptr->image.size[1] << ","
                              << cv_image_ptr->image.channels());
  CvImage received_image;
  received_image.stamp = msg.header.stamp;
  received_image.frame_id = msg.header.frame_id;
  received_image.encoding = sensor_msgs::image_encodings::BGR8;
  received_image.img = cv_image_ptr->image;
  m_p_received_image_queue->push(received_image);
  RCLCPP_DEBUG_STREAM(this->get_logger(), "Callback: Image pushed to queue");
}

void Yolov8Detector::ProcessImages(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Inference: Start thread");

  cv::Mat res, image;
  cv::Size size = cv::Size{640, 640};
  std::vector<Object> objs;

  for (int i = 0; rclcpp::ok(); i++) {
    if (m_p_received_image_queue->full()) {
      RCLCPP_WARN_STREAM_ONCE(
          this->get_logger(),
          "Inference: Image queue full, consider reducing FPS");
    }
    CvImage received_image;
    m_p_received_image_queue->pop(received_image);
    try {
      RCLCPP_DEBUG_STREAM(this->get_logger(),
                          "Start inference for received image ("
                              << received_image.img.cols << "x"
                              << received_image.img.rows << ")");
      objs.clear();
      m_p_yolov8->copy_from_Mat(received_image.img, size);
      auto start = std::chrono::system_clock::now();
      m_p_yolov8->infer();
      auto end = std::chrono::system_clock::now();
      m_p_yolov8->postprocess(objs, m_param_score_thres, m_param_iou_thres,
                              m_param_topk, m_param_num_labels);

      vision_msgs::msg::Detection2DArray detection_msg;
      detection_msg.header.stamp = received_image.stamp;
      detection_msg.header.frame_id = received_image.frame_id;
      for (auto &obj : objs) {
        vision_msgs::msg::Detection2D detection;
        vision_msgs::msg::ObjectHypothesisWithPose result;
        if (obj.label < m_param_class_names.size())
          result.hypothesis.class_id = m_param_class_names[obj.label];
        else
          result.hypothesis.class_id = "unknown";
        result.hypothesis.score = obj.prob;
        detection.results.push_back(result);
        detection.bbox.center.position.x = obj.rect.x + obj.rect.width / 2;
        detection.bbox.center.position.y = obj.rect.y + obj.rect.height / 2;
        detection.bbox.size_x = obj.rect.width;
        detection.bbox.size_y = obj.rect.height;
        detection_msg.detections.push_back(detection);
      }
      m_detection_publisher->publish(std::move(detection_msg));

      m_p_yolov8->draw_objects(received_image.img, res, objs,
                               m_param_class_names, COLORS);
      auto tc = (double)std::chrono::duration_cast<std::chrono::microseconds>(
                    end - start)
                    .count() /
                1000.;
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Object list:");
      for (auto &obj : objs) {
        std::string class_name;
        if (obj.label < m_param_class_names.size())
          class_name = m_param_class_names[obj.label];
        else
          class_name = "unknown";
        RCLCPP_DEBUG(this->get_logger(), " - %s %.1f%%", class_name.c_str(),
                     obj.prob * 100);
      }
      RCLCPP_DEBUG(this->get_logger(), "Time taken: %2.4lf ms", tc);
      RCLCPP_DEBUG_STREAM(this->get_logger(), "End inference");
      CvImage processed_image;
      processed_image.stamp = received_image.stamp;
      processed_image.frame_id = received_image.frame_id;
      processed_image.encoding = received_image.encoding;
      processed_image.img = res;
      m_p_processed_image_queue->push(processed_image);
    } catch (std::exception &ex) {
      throw std::runtime_error("Standard exception thrown: " +
                               std::string(ex.what()));
    } catch (...) {
      throw std::runtime_error("Unexpected exception thrown");
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Inference: Thread exits");
}

void Yolov8Detector::PublishImages(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Publish: Start thread");

  for (int i = 0; rclcpp::ok(); i++) {
    if (m_p_processed_image_queue->full()) {
      RCLCPP_WARN_STREAM_ONCE(
          this->get_logger(),
          "Publish: Image queue full, consider reducing FPS");
    }
    CvImage processed_image;
    m_p_processed_image_queue->pop(processed_image);
    publish_image(processed_image);

    if (m_param_viz) {
      cv::namedWindow(m_param_detector_ns, cv::WINDOW_AUTOSIZE);
      cv::imshow(m_param_detector_ns, processed_image.img);
      cv::waitKey(1);
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Publish: Thread exits");
}

void Yolov8Detector::publish_image(const CvImage &image) {
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = image.stamp;
  header.frame_id = image.frame_id;

  try {
    cv_bridge::CvImage img_bridge =
        cv_bridge::CvImage(header, image.encoding, image.img);
    img_bridge.toImageMsg(img_msg);

  } catch (...) {
    throw std::runtime_error("Runtime error in publish_image()");
  }

  m_image_publisher->publish(std::move(img_msg));
}

Yolov8Detector::Yolov8Detector(rclcpp::NodeOptions node_options)
    : rclcpp::Node("yolov8_detector_node",
                   node_options.allow_undeclared_parameters(true)) {
  declare_parameter("engine_file_path", "");
  declare_parameter("class_names", CLASS_NAMES);
  declare_parameter("num_labels", 80);
  declare_parameter("topk", 100);
  declare_parameter("score_thres", 0.25f);
  declare_parameter("iou_thres", 0.65f);
  declare_parameter("use_system_default_qos", true);
  declare_parameter("viz", false);
  m_param_engine_file_path = get_parameter("engine_file_path").as_string();
  m_param_class_names = get_parameter("class_names").as_string_array();
  m_param_num_labels = get_parameter("num_labels").as_int();
  m_param_topk = get_parameter("topk").as_int();
  m_param_score_thres = get_parameter("score_thres").as_double();
  m_param_iou_thres = get_parameter("iou_thres").as_double();
  m_param_use_sys_default_qos =
      get_parameter("use_system_default_qos").as_bool();
  m_param_viz = get_parameter("viz").as_bool();

  declare_parameter("detector_ns", "yolov8_detector");
  m_param_detector_ns = get_parameter("detector_ns").as_string();

  try {
    if (!m_param_engine_file_path.length()) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Empty engine file path");
      return;
    }

    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Loading engine file: " << m_param_engine_file_path);
    m_p_yolov8 = std::make_unique<YOLOv8>(m_param_engine_file_path);
    m_p_yolov8->make_pipe(true);
    RCLCPP_INFO_STREAM(this->get_logger(), "Success");

    m_p_received_image_queue = std::make_unique<BlockingQueue<CvImage>>(5);
    m_p_processed_image_queue = std::make_unique<BlockingQueue<CvImage>>(5);

    RCLCPP_INFO_STREAM(this->get_logger(), "Spinning up processing threads");
    m_p_image_consumer = std::unique_ptr<std::thread>(
        new std::thread(&Yolov8Detector::ProcessImages, this));
    m_p_image_publisher = std::unique_ptr<std::thread>(
        new std::thread(&Yolov8Detector::PublishImages, this));

    rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
    system_default_qos.keep_last(
        10); // to allow intra-process comm in a composition container
    system_default_qos.durability_volatile(); // to allow intra-process comm in
                                              // a composition container
    rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
    auto selected_qos =
        m_param_use_sys_default_qos ? system_default_qos : sensor_data_qos;
    m_image_subscription = this->create_subscription<sensor_msgs::msg::Image>(
        "/" + m_param_detector_ns + "/image", selected_qos,
        std::bind(&Yolov8Detector::image_topic_callback, this, _1));
    m_image_publisher = this->create_publisher<sensor_msgs::msg::Image>(
        "/" + m_param_detector_ns + "/image_with_detection", selected_qos);
    m_detection_publisher =
        this->create_publisher<vision_msgs::msg::Detection2DArray>(
            "/" + m_param_detector_ns + "/detection", selected_qos);

    RCLCPP_INFO_STREAM(this->get_logger(), "Ready to process images");
  } catch (std::exception &ex) {
    throw std::runtime_error("Standard exception thrown: " +
                             std::string(ex.what()));
  } catch (...) {
    throw std::runtime_error("Unexpected exception thrown");
  }
}

Yolov8Detector::~Yolov8Detector() {
  if (m_p_image_consumer != nullptr)
    m_p_image_consumer->join();
  if (m_p_image_publisher != nullptr)
    m_p_image_publisher->join();
  RCLCPP_INFO_STREAM(this->get_logger(), "All threads exit");

  // clean up
  if (m_p_yolov8 != nullptr)
    m_p_yolov8.release();
  if (m_param_viz)
    cv::destroyAllWindows();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(Yolov8Detector)
