#include "GenApi/GenApi.h"
#include "camera_info_manager/camera_info_manager.hpp"
#include "gevapi.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <cstdio>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <stdexcept>

#define MAX_NETIF 8
#define MAX_CAMERAS_PER_NETIF 32
#define MAX_CAMERAS (MAX_NETIF * MAX_CAMERAS_PER_NETIF)

// Enable/disable Bayer to RGB conversion
// (If disabled - Bayer format will be treated as Monochrome).
#define ENABLE_BAYER_CONVERSION 1

// Enable/disable buffer FULL/EMPTY handling (cycling)
#define USE_SYNCHRONOUS_BUFFER_CYCLING 0

// Enable/disable transfer tuning (buffering, timeouts, thread affinity).
#define TUNE_STREAMING_THREADS 0

#define NUM_BUF 8
void *m_latestBuffer = NULL;

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

struct ThermalImage {
  uint32_t seq;
  rclcpp::Time stamp;
  std::string encoding;
  cv::Mat img;
};

// main node

class CamStreamer : public ::rclcpp::Node {
public:
  explicit CamStreamer(rclcpp::NodeOptions node_options);
  ~CamStreamer(void);

private:
  int configure_camera(void);
  int connect_camera(void);

  void AcquireImages(void);
  void PublishImages(void);

  void publish_image(rclcpp::Time stamp, const cv::Mat &image,
                     std::string encoding = sensor_msgs::image_encodings::BGR8);
  void init_camera_info(std::string camera_name, std::string camera_info_url);

  std::shared_ptr<image_transport::CameraPublisher> m_p_camera_publisher{};
  std::shared_ptr<camera_info_manager::CameraInfoManager> m_p_camera_info{};

  std::unique_ptr<BlockingQueue<ThermalImage>> m_p_thermal_image_queue;

  std::unique_ptr<std::thread> m_p_image_producer;
  std::unique_ptr<std::thread> m_p_image_publisher;

  int m_param_timeout_ms;         // m_param_timeout_ms for grabbing images (in
                                  // milliseconds)
  int m_param_downsampling_ratio; // message downsampling ratio, default 1,
                                  // means no downsampling to the 30 FPS default
                                  // rate
  bool m_param_flip_horizontal;   // flip
  bool m_param_flip_vertical;     // flip
  bool m_param_use_sys_default_qos;
  bool m_param_viz; // if to show opencv image

  std::string m_param_camera_ns;       // camera namespace
  std::string m_param_camera_serial;   // camera serial number
  std::string m_param_camera_info_url; // camera info url
  std::string m_param_pixel_format;    // YUV422_8, Mono8
  std::string m_param_color_map;       // Fire, Grayscale, IronBlack
  std::string m_param_contrast_mode; // AdaptiveFixed, AdaptiveDynamic, Static,
                                     // StaticTemperature
  int m_param_contrast;              // for AdaptiveFixed, 0 to 255
  int m_param_brightness; // for AdaptiveFixed & AdaptiveDynamic, 0 to 255
  bool m_param_overlay;   // whether to turn on colorbar overlay on images
  std::string m_param_frame_id;

  GEV_CAMERA_HANDLE m_pDevice = NULL;
  bool m_isStreaming;
};

int CamStreamer::connect_camera(void) {
  GEV_DEVICE_INTERFACE pCamera[MAX_CAMERAS] = {0};
  GEV_STATUS status;
  int numCamera = 0;

  //===================================================================================
  // Set default options for the library.
  {
    GEVLIB_CONFIG_OPTIONS options = {0};

    GevGetLibraryConfigOptions(&options);
    options.logLevel = GEV_LOG_LEVEL_OFF;
    GevSetLibraryConfigOptions(&options);
  }

  if (GevGetCameraList(pCamera, MAX_CAMERAS, &numCamera)) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "Couldn't enumerate camera(s) on the network");
    return -1;
  }
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "There are " << numCamera
                                  << " camera(s) connected on the network");

  if (numCamera == 0) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "There is no connected cameras devices. Please connect "
                        "a device and try again");
    return -1;
  }

  int i;
  for (i = 0; i < numCamera; i++) {
    if (m_param_camera_serial == std::string(pCamera[i].serial))
      break;
  }
  if (i == numCamera) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
                        "There is no connected camera with serial no. "
                            << m_param_camera_serial);
    return -1;
  }

  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Trying to connect to camera with serial no. "
                         << m_param_camera_serial);
  if (GevOpenCamera(&pCamera[i], GevExclusiveMode, &m_pDevice)) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Couldn't open camera");
    return -1;
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "Success");

  return 0;
}

int CamStreamer::configure_camera(void) {
  if (!m_pDevice) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Camera not connected yet");
    return -1;
  }

  int i;
  int type;
  UINT32 height = 0;
  UINT32 width = 0;
  UINT32 format = 0;
  UINT32 maxHeight = 1600;
  UINT32 maxWidth = 2048;
  UINT32 maxDepth = 2;
  UINT64 size;
  UINT64 payload_size;
  int numBuffers = NUM_BUF;
  PUINT8 bufAddress[NUM_BUF];
  UINT32 pixFormat = 0;
  UINT32 pixDepth = 0;
  UINT32 convertedGevFormat = 0;

  GEV_CAMERA_OPTIONS camOptions = {0};

  // Adjust the camera interface options if desired (see the manual)
  GevGetCameraInterfaceOptions(m_pDevice, &camOptions);
  camOptions.heartbeat_timeout_ms =
      m_param_timeout_ms; // Disconnect detection (5 seconds)

#if TUNE_STREAMING_THREADS
  // Some tuning can be done here. (see the manual)
  camOptions.streamFrame_timeout_ms =
      1001; // Internal timeout for frame reception.
  camOptions.streamNumFramesBuffered = 4; // Buffer frames internally.
  camOptions.streamMemoryLimitMax =
      64 * 1024 * 1024;            // Adjust packet memory buffering limit.
  camOptions.streamPktSize = 9180; // Adjust the GVSP packet size.
  camOptions.streamPktDelay =
      10; // Add usecs between packets to pace arrival at NIC.

  // Assign specific CPUs to threads (affinity) - if required for better
  // performance.
  {
    int numCpus = _GetNumCpus();
    if (numCpus > 1) {
      camOptions.streamThreadAffinity = numCpus - 1;
      camOptions.serverThreadAffinity = numCpus - 2;
    }
  }
#endif
  // Write the adjusted interface options back.
  GevSetCameraInterfaceOptions(m_pDevice, &camOptions);

  if (m_param_pixel_format == "YUV422_8" or
      m_param_pixel_format == "Mono8") { // does not support Mono16 yet
    GevSetFeatureValueAsString(m_pDevice, "PixelFormat",
                               m_param_pixel_format.c_str());
    if (m_param_pixel_format == "YUV422_8") {
      if (m_param_color_map == "Fire" or m_param_color_map == "Grayscale" or
          m_param_color_map == "IronBlack")
        GevSetFeatureValueAsString(m_pDevice, "falseColorMap",
                                   m_param_color_map.c_str());
      else
        RCLCPP_WARN_STREAM(this->get_logger(),
                           "Color map not supported: " << m_param_color_map);
    }
  } else
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Pixel map not supported: " << m_param_pixel_format);

  if (m_param_contrast_mode == "AdaptiveFixed" or
      m_param_contrast_mode == "AdaptiveDynamic" or
      m_param_contrast_mode == "Static" or
      m_param_contrast_mode == "StaticTemperature") {
    GevSetFeatureValueAsString(m_pDevice, "contrastMode",
                               m_param_contrast_mode.c_str());
    if (m_param_contrast_mode == "AdaptiveFixed" or
        m_param_contrast_mode == "AdaptiveDynamic") {
      if (m_param_brightness < 0)
        m_param_brightness = 0;
      if (m_param_brightness > 255)
        m_param_brightness = 255;
      GevSetFeatureValue(m_pDevice, "brightness", sizeof(UINT32),
                         &m_param_brightness);
    }
    if (m_param_contrast_mode == "AdaptiveFixed") {
      if (m_param_contrast < 0)
        m_param_contrast = 0;
      if (m_param_contrast > 255)
        m_param_contrast = 255;
      GevSetFeatureValue(m_pDevice, "contrast", sizeof(UINT32),
                         &m_param_contrast);
    }
  } else
    RCLCPP_WARN_STREAM(this->get_logger(), "Contrast mode not supported: "
                                               << m_param_contrast_mode);

  if (m_param_overlay) {
    GevSetFeatureValueAsString(m_pDevice, "overlaySelector", "GraphColorMap");
    GevSetFeatureValueAsString(m_pDevice, "overlayMode", "Active");
    GevSetFeatureValueAsString(m_pDevice, "overlayGlobalMode", "Active");
  }

  GevGetFeatureValue(m_pDevice, "Width", &type, sizeof(UINT32), &width);
  GevGetFeatureValue(m_pDevice, "Height", &type, sizeof(UINT32), &height);
  GevGetFeatureValue(m_pDevice, "PayloadSize", &type, sizeof(UINT64),
                     &payload_size);
  GevGetFeatureValue(m_pDevice, "PixelFormat", &type, sizeof(UINT32), &format);
  char pixel_format[10] = {0};
  GevGetFeatureValueAsString(m_pDevice, "PixelFormat", &type,
                             sizeof(pixel_format), (char *)&pixel_format);

  //=================================================================
  // Set up a grab/transfer from this camera
  //
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Confirming camera ROI set for Height = "
                         << height << ", Width = " << width
                         << ", PixelFormat = " << pixel_format);
  if (std::string(pixel_format) != m_param_pixel_format) {
    RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Pixel format readout mismatch with the parameter, a strange issue");
    return -1;
  }

  maxHeight = height;
  maxWidth = width;
  maxDepth = GetPixelSizeInBytes(GevGetUnpackedPixelType(format));

  // Allocate image buffers (adjusting for any unpacking of packed pixels)
  // (Either the image size or the payload_size, whichever is larger).
  size = maxDepth * maxWidth * maxHeight;
  size = (payload_size > size) ? payload_size : size;
  for (i = 0; i < numBuffers; i++) {
    bufAddress[i] = (PUINT8)malloc(size);
    memset(bufAddress[i], 0, size);
  }

#if USE_SYNCHRONOUS_BUFFER_CYCLING
  // Initialize a transfer with synchronous buffer handling.
  GevInitializeTransfer(m_pDevice, SynchronousNextEmpty, size, numBuffers,
                        bufAddress);
#else
  // Initialize a transfer with asynchronous buffer handling.
  GevInitializeTransfer(m_pDevice, Asynchronous, size, numBuffers, bufAddress);
#endif

  for (i = 0; i < numBuffers; i++) {
    memset(bufAddress[i], 0, size);
  }

  return 0;
}

void CamStreamer::AcquireImages(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Start thread");

  while (rclcpp::ok()) {
    // handle camera connection and reconnection
    if (connect_camera()) {
      rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(1e9)));
      continue;
    }

    configure_camera();

    GevStartTransfer(m_pDevice, -1);
    RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Start stream");
    m_isStreaming = true;

    // receive images from the API and publish them.
    // While we are still running.
    for (uint32_t seq = 0; m_pDevice && rclcpp::ok(); seq++) {
      GEV_BUFFER_OBJECT *img = NULL;
      GEV_STATUS status = 0;

      // Wait for images to be received
      status = GevWaitForNextImage(m_pDevice, &img, 1000);

      if ((img == NULL) or (status != GEVLIB_OK)) {
        RCLCPP_DEBUG_STREAM(
            this->get_logger(),
            "Acquire: Error when receiving an image, gev status: " << status);
        if (status ==
            GEVLIB_ERROR_NO_CAMERA) { // connection breaks, need to reconnect
          RCLCPP_WARN_STREAM(
              this->get_logger(),
              "Acquire: Camera disconnected, try to reconnect...");
          rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(1e9)));
          // if(Gev_Reconnect(m_pDevice) == GEVLIB_OK) continue; // reconnect
          // after temp comm issue, but it is not working for some reason else
          break; // reconnect after camera power loss, tested to work fine
        }
        continue; // ignore other errors
      }
      if (img->status != 0) {
        // Image had an error (incomplete (timeout/overflow/lost)).
        // Do any handling of this condition necessary.
        RCLCPP_DEBUG_STREAM(
            this->get_logger(),
            "Acquire: Error when receiving an image, image status: "
                << img->status);
        continue; // ignore errors
      }

      auto stamp = rclcpp::Node::now();
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Acquire: Received a new image");

      m_latestBuffer = img->address;

      std::string encoding;
      cv::Mat image;
      if (m_param_pixel_format == "Mono8") {
        image = cv::Mat(static_cast<int>(img->h), static_cast<int>(img->w),
                        CV_8UC1, img->address);
        encoding = sensor_msgs::image_encodings::MONO8;
      } else if (m_param_pixel_format == "YUV422_8") {
        cv::Mat image_yuv =
            cv::Mat(static_cast<int>(img->h), static_cast<int>(img->w), CV_8UC2,
                    img->address);
        // Convert from yuv to rgb
        cv::cvtColor(image_yuv, image, cv::COLOR_YUV2BGR_YUYV);
        encoding = sensor_msgs::image_encodings::BGR8;
      }

      ThermalImage thermal_image;
      thermal_image.stamp = stamp;
      thermal_image.seq = seq;
      thermal_image.encoding = encoding;
      thermal_image.img = image;
      m_p_thermal_image_queue->push(thermal_image);

#if USE_SYNCHRONOUS_BUFFER_CYCLING
      if (img != NULL) {
        // Release the buffer back to the image transfer process.
        GevReleaseImage(displayContext->camHandle, img);
      }
#endif
      rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int>(1e6)));
    }

    m_isStreaming = false;
    // Stop
    RCLCPP_WARN_STREAM(this->get_logger(), "Acquire: Stopping transferring");
    GevStopTransfer(m_pDevice);
    GevAbortTransfer(m_pDevice);
    GevFreeTransfer(m_pDevice);
    GevCloseCamera(&m_pDevice);
  }

  // Close down the API.
  GevApiUninitialize();

  // Close socket API
  _CloseSocketAPI(); // must close API even on error

  RCLCPP_INFO_STREAM(this->get_logger(), "Acquire: Thread exits");
}

void CamStreamer::PublishImages(void) {
  RCLCPP_INFO_STREAM(this->get_logger(), "Publish: Start thread");

  cv::Size cv_windowSize = cv::Size(640, 480);
  std::string window_title = "Thermal Image";

  for (int i = 0; rclcpp::ok(); i++) {
    if (m_p_thermal_image_queue->full()) {
      RCLCPP_WARN_STREAM_ONCE(
          this->get_logger(),
          "Publish: Image queue full, consider reducing FPS");
    }
    ThermalImage processed_image;
    m_p_thermal_image_queue->pop(processed_image);
    if (!(processed_image.seq % m_param_downsampling_ratio)) {
      if (m_param_flip_vertical) {
        cv::flip(processed_image.img, processed_image.img, 0);
      }
      if (m_param_flip_horizontal) {
        cv::flip(processed_image.img, processed_image.img, 1);
      }
      publish_image(processed_image.stamp, processed_image.img,
                    processed_image.encoding);

      if (m_param_viz) {
        cv::namedWindow(window_title, cv::WINDOW_NORMAL);
        cv::resizeWindow(window_title, cv_windowSize);
        cv::imshow(window_title, processed_image.img);
        cv::waitKey(1);
      }
    }
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Publish: Thread exits");
}

void CamStreamer::publish_image(rclcpp::Time stamp, const cv::Mat &image,
                                std::string encoding) {
  sensor_msgs::msg::Image img_msg;
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = m_param_frame_id;

  try {
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, encoding, image);
    img_bridge.toImageMsg(img_msg);

  } catch (...) {
    throw std::runtime_error("Runtime error in publish_image()");
  }

  auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(
      m_p_camera_info->getCameraInfo());
  ci->header = img_msg.header;

  m_p_camera_publisher->publish(img_msg, *ci);
}

void CamStreamer::init_camera_info(std::string camera_name,
                                   std::string camera_info_url) {
  m_p_camera_info = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, camera_name);
  if (m_p_camera_info->validateURL(camera_info_url)) {
    m_p_camera_info->loadCameraInfo(camera_info_url);
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(),
                       "Invalid camera info URL: " << camera_info_url.c_str());
  }
}

CamStreamer::CamStreamer(rclcpp::NodeOptions node_options)
    : rclcpp::Node("thermal_camera_streamer_node",
                   node_options.allow_undeclared_parameters(true)),
      m_isStreaming(false) {
  declare_parameter("timeout_ms", 5000);
  declare_parameter("flip_horizontal", false);
  declare_parameter("flip_vertical", false);
  declare_parameter("downsampling_ratio", 1);
  declare_parameter("use_system_default_qos", true);
  declare_parameter("viz", false);
  m_param_timeout_ms = get_parameter("timeout_ms").as_int();
  m_param_flip_horizontal = get_parameter("flip_horizontal").as_bool();
  m_param_flip_vertical = get_parameter("flip_vertical").as_bool();
  m_param_downsampling_ratio = get_parameter("downsampling_ratio").as_int();
  m_param_use_sys_default_qos =
      get_parameter("use_system_default_qos").as_bool();
  m_param_viz = get_parameter("viz").as_bool();

  declare_parameter("camera_ns", "thermal");
  declare_parameter("camera_serial", "");
  declare_parameter("camera_info_url", "");
  declare_parameter("pixel_format", "YUV422_8");
  declare_parameter("color_map", "Fire");
  declare_parameter("contrast_mode", "AdaptiveFixed");
  declare_parameter("contrast", 150);
  declare_parameter("brightness", 100);
  declare_parameter("overlay", false);
  declare_parameter("frame_id", "");
  m_param_camera_ns = get_parameter("camera_ns").as_string();
  m_param_camera_serial = get_parameter("camera_serial").as_string();
  m_param_camera_info_url = get_parameter("camera_info_url").as_string();
  m_param_pixel_format = get_parameter("pixel_format").as_string();
  m_param_color_map = get_parameter("color_map").as_string();
  m_param_contrast_mode = get_parameter("contrast_mode").as_string();
  m_param_contrast = get_parameter("contrast").as_int();
  m_param_brightness = get_parameter("brightness").as_int();
  m_param_overlay = get_parameter("overlay").as_bool();
  m_param_frame_id = get_parameter("frame_id").as_string();

  if (!m_param_camera_serial.length()) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Camera serial not provided");
    throw std::runtime_error("Camera serial not provided");
  }

  if (m_param_downsampling_ratio < 1)
    m_param_downsampling_ratio = 1;
  if (m_param_downsampling_ratio > 100)
    m_param_downsampling_ratio = 100;

  m_p_thermal_image_queue = std::make_unique<BlockingQueue<ThermalImage>>(5);

  init_camera_info(m_param_camera_ns, m_param_camera_info_url);
  rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
  system_default_qos.keep_last(
      10); // to allow intra-process comm in a composition container
  system_default_qos.durability_volatile(); // to allow intra-process comm in a
                                            // composition container
  rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
  auto selected_qos =
      m_param_use_sys_default_qos ? system_default_qos : sensor_data_qos;
  m_p_camera_publisher = std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(
          this, "/" + m_param_camera_ns + "/image",
          selected_qos.get_rmw_qos_profile()));

  try {
    RCLCPP_INFO_STREAM(this->get_logger(), "Spinning up processing threads");
    m_p_image_producer = std::unique_ptr<std::thread>(
        new std::thread(&CamStreamer::AcquireImages, this));
    m_p_image_publisher = std::unique_ptr<std::thread>(
        new std::thread(&CamStreamer::PublishImages, this));
  } catch (GenICam::GenericException &ge) {
    throw std::runtime_error("GenICam exception thrown: " +
                             std::string(ge.what()));
  } catch (std::exception &ex) {
    throw std::runtime_error("Standard exception thrown: " +
                             std::string(ex.what()));
  } catch (...) {
    throw std::runtime_error("Unexpected exception thrown");
  }
}

CamStreamer::~CamStreamer() {
  if (m_p_image_producer != nullptr)
    m_p_image_producer->join();
  if (m_p_image_publisher != nullptr)
    m_p_image_publisher->join();
  RCLCPP_INFO_STREAM(this->get_logger(), "All threads exit");

  // clean up
  if (m_param_viz)
    cv::destroyAllWindows();
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(CamStreamer)
