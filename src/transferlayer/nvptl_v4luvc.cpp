#include <stdio.h>
#include <stdint.h>

// #include "backend.h"
// #include "types.h"

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <algorithm>
#include <array>
#include <functional>
#include <string>
#include <sstream>
#include <fstream>
#include <regex>
#include <thread>
#include <utility> // for pair
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <cmath>
#include <set>
#include <errno.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <regex>
#include <list>

#include "transferlayer/commondata.h"
#ifdef V4L2_META_FMT_UVC
constexpr bool metadata_node = true;
#else
#pragma message("\nLibrealsense notification: V4L2_META_FMT_UVC was not defined, adding metadata constructs")

constexpr bool metadata_node = false;

// Providing missing parts from videodev2.h
// V4L2_META_FMT_UVC >> V4L2_CAP_META_CAPTURE is also defined, but the opposite does not hold
#define V4L2_META_FMT_UVC v4l2_fourcc('U', 'V', 'C', 'H') /* UVC Payload Header */

#ifndef V4L2_CAP_META_CAPTURE
#define V4L2_CAP_META_CAPTURE 0x00800000 /* Specified in kernel header v4.16 */
#endif                                   // V4L2_CAP_META_CAPTURE

#endif // V4L2_META_FMT_UVC

#ifndef V4L2_META_FMT_D4XX
#define V4L2_META_FMT_D4XX v4l2_fourcc('D', '4', 'X', 'X') /* D400 Payload Header metadata */
#endif

// #define DEBUG_V4L
#ifdef DEBUG_V4L
#define LOG_DEBUG_V4L(...)                      \
  do {                                          \
    CLOG(DEBUG, "librealsense") << __VA_ARGS__; \
  } while (false)
#else
#define LOG_DEBUG_V4L(...)
#endif // DEBUG_V4L

// Use local definition of buf type to resolve for kernel versions
constexpr auto LOCAL_V4L2_BUF_TYPE_META_CAPTURE = (v4l2_buf_type)(13);

#pragma pack(push, 1)
// The struct definition is identical to uvc_meta_buf defined uvcvideo.h/ kernel 4.16 headers, and is provided to allow for cross-kernel compilation
struct uvc_meta_buffer {
  __u64 ns;    // system timestamp of the payload in nanoseconds
  __u16 sof;   // USB Frame Number
  __u8 length; // length of the payload metadata header
  __u8 flags;  // payload header flags
  __u8 *buf;   // device-specific metadata payload data
};
#pragma pack(pop)

const size_t MAX_DEV_PARENT_DIR = 10;
const double DEFAULT_KPI_FRAME_DROPS_PERCENTAGE = 0.05;
const uint16_t MAX_RETRIES = 100;
const uint8_t DEFAULT_V4L2_FRAME_BUFFERS = 4;
const uint16_t DELAY_FOR_RETRIES = 50;
const int DISCONNECT_PERIOD_MS = 6000;
const int POLLING_DEVICES_INTERVAL_MS = 2000;

const uint8_t MAX_META_DATA_SIZE = 0xff; // UVC Metadata total length

typedef double rs2_time_t; /**< Timestamp format. units are milliseconds */
struct frame_object {
  size_t frame_size;
  uint8_t metadata_size;
  const void *pixels;
  const void *metadata;
  rs2_time_t backend_time;
};
typedef std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> stream_profile_tuple;
// class wmf_backend;
struct stream_profile {
  uint32_t width;
  uint32_t height;
  uint32_t fps;
  uint32_t format;

  operator stream_profile_tuple() const {
    return std::make_tuple(width, height, fps, format);
  }
};
typedef std::function<void(stream_profile, frame_object, std::function<void()>)> frame_callback;

class named_mutex {
public:
  named_mutex(const std::string &device_path, unsigned timeout);

  named_mutex(const named_mutex &) = delete;

  ~named_mutex();

  void lock();

  void unlock();

  bool try_lock();

private:
  void acquire();
  void release();

  std::string _device_path;
  uint32_t _timeout;
  int _fildes;
  static std::recursive_mutex _init_mutex;
  static std::map<std::string, std::recursive_mutex> _dev_mutex;
  static std::map<std::string, int> _dev_mutex_cnt;
  int _object_lock_counter;
  std::mutex _mutex;
};
static int xioctl(int fh, unsigned long request, void *arg);

class buffer {
public:
  buffer(int fd, v4l2_buf_type type, bool use_memory_map, uint32_t index);

  void prepare_for_streaming(int fd);

  ~buffer();

  void attach_buffer(const v4l2_buffer &buf);

  void detach_buffer();

  void request_next_frame(int fd, bool force = false);

  uint32_t get_full_length() const { return _length; }
  uint32_t get_length_frame_only() const { return _original_length; }

  uint8_t *get_frame_start() const { return _start; }

  bool use_memory_map() const { return _use_memory_map; }

private:
  v4l2_buf_type _type;
  uint8_t *_start;
  uint32_t _length;
  uint32_t _original_length;
  bool _use_memory_map;
  uint32_t _index;
  v4l2_buffer _buf;
  std::mutex _mutex;
  bool _must_enqueue = false;
};

enum supported_kernel_buf_types : uint8_t {
  e_video_buf,
  e_metadata_buf,
  e_max_kernel_buf_type
};

// RAII handling of kernel buffers interchanges
/*class buffers_mgr
{
public:
    buffers_mgr(bool memory_mapped_buf) : _md_start(nullptr),
                                          _md_size(0),
                                          _mmap_bufs(memory_mapped_buf)
    {
    }

    ~buffers_mgr() {}

    void request_next_frame();
    void handle_buffer(supported_kernel_buf_types buf_type, int file_desc,
                       v4l2_buffer buf = v4l2_buffer(),
                       std::shared_ptr<buffer> data_buf = nullptr);

    uint8_t metadata_size() const { return _md_size; }
    void *metadata_start() const { return _md_start; }

    void set_md_attributes(uint8_t md_size, void *md_start)
    {
        _md_start = md_start;
        _md_size = md_size;
    }
    void set_md_from_video_node(bool compressed);
    bool verify_vd_md_sync() const;
    bool md_node_present() const;

    // Debug Evgeni
    //  RAII for buffer exchange with kernel
    struct kernel_buf_guard
    {
        ~kernel_buf_guard()
        {
            if (_data_buf && (!_managed))
            {
                if (_file_desc > 0)
                {
                    if (xioctl(_file_desc, (int)VIDIOC_QBUF, &_dq_buf) < 0)
                    {
                        printf("xioctl(VIDIOC_QBUF) guard failed for fd \n");//<< std::dec << _file_desc);
                        if (xioctl(_file_desc, (int)VIDIOC_DQBUF, &_dq_buf) >= 0)
                        {
                            printf("xioctl(VIDIOC_QBUF) Re-enqueue succeeded for fd \n");// << std::dec << _file_desc);
                            if (xioctl(_file_desc, (int)VIDIOC_QBUF, &_dq_buf) < 0)
                                printf("xioctl(VIDIOC_QBUF) re-deque  failed for fd \n");// << std::dec << _file_desc);
                            else
                                printf("xioctl(VIDIOC_QBUF) re-deque succeeded for fd \n");// << std::dec << _file_desc);
                        }
                        else
                            printf("xioctl(VIDIOC_QBUF) Re-enqueue failed for fd \n");// << std::dec << _file_desc);
                    }
                    else
                        printf("Enqueue (e) buf \n");// << std::dec << _dq_buf.index << " for fd " << _file_desc);
                }
            }
        }

        std::shared_ptr<buffer> _data_buf = nullptr;
        v4l2_buffer _dq_buf{};
        int _file_desc = -1;
        bool _managed = false;
    };

    std::array<kernel_buf_guard, e_max_kernel_buf_type> &get_buffers()
    {
        return buffers;
    }

private:
    void *_md_start;  // marks the address of metadata blob
    uint8_t _md_size; // metadata size is bounded by 255 bytes by design
    bool _mmap_bufs;

    std::array<kernel_buf_guard, e_max_kernel_buf_type> buffers;
};*/

class v4l_uvc_interface {
  virtual void capture_loop() = 0;

  virtual bool has_metadata() const = 0;

  virtual void streamon() const = 0;
  virtual void streamoff() const = 0;
  virtual void negotiate_kernel_buffers(size_t num) const = 0;

  virtual void allocate_io_buffers(size_t num) = 0;
  virtual void map_device_descriptor() = 0;
  virtual void unmap_device_descriptor() = 0;
  virtual void set_format(stream_profile profile) = 0;
  virtual void prepare_capture_buffers() = 0;
  virtual void stop_data_capture() = 0;
  // virtual void acquire_metadata(buffers_mgr &buf_mgr, fd_set &fds, bool compressed_format) = 0;
};

// The aim of the frame_drop_monitor is to check the frames drops kpi - which requires
// that no more than some percentage of the frames are dropped
// It is checked using the fps, and the previous corrupted frames, on the last 30 seconds
// for example, for frame rate of 30 fps, and kpi of 5%, the criteria will be:
// if at least 45 frames (= 30[fps] * 5%[kpi]* 30[sec]) drops have occured in the previous 30 seconds,
// then the kpi is violated
/*class frame_drop_monitor
{
public:
    frame_drop_monitor(double kpi_frames_drops_percentage) : _kpi_frames_drops_pct(kpi_frames_drops_percentage) {}
    // update_and_check_kpi method returns whether the kpi has been violated
    // it should be called each time a partial frame is caught
    bool update_and_check_kpi(const stream_profile &profile, const timeval &timestamp);

private:
    // container used to store the latest timestamps of the partial frames, per profile
    std::vector<std::pair<stream_profile, std::deque<long int>>> drops_per_stream;
    double _kpi_frames_drops_pct;
};*/
struct guid {
  uint32_t data1;
  uint16_t data2, data3;
  uint8_t data4[8];
};
// subdevice and node fields are assigned by Host driver; unit and GUID are hard-coded in camera firmware
struct extension_unit {
  int subdevice;
  uint8_t unit;
  int node;
  guid id;
};
enum power_state {
  D0,
  D3
};
struct control_range {
  control_range() {
  }

  control_range(int32_t in_min, int32_t in_max, int32_t in_step, int32_t in_def) {
    populate_raw_data(min, in_min);
    populate_raw_data(max, in_max);
    populate_raw_data(step, in_step);
    populate_raw_data(def, in_def);
  }
  control_range(std::vector<uint8_t> in_min, std::vector<uint8_t> in_max, std::vector<uint8_t> in_step, std::vector<uint8_t> in_def) {
    min = in_min;
    max = in_max;
    step = in_step;
    def = in_def;
  }
  std::vector<uint8_t> min;
  std::vector<uint8_t> max;
  std::vector<uint8_t> step;
  std::vector<uint8_t> def;

private:
  void populate_raw_data(std::vector<uint8_t> &vec, int32_t value);
};
typedef enum rs2_option {
  RS2_OPTION_BACKLIGHT_COMPENSATION,                  /**< Enable / disable color backlight compensation*/
  RS2_OPTION_BRIGHTNESS,                              /**< Color image brightness*/
  RS2_OPTION_CONTRAST,                                /**< Color image contrast*/
  RS2_OPTION_EXPOSURE,                                /**< Controls exposure time of color camera. Setting any value will disable auto exposure*/
  RS2_OPTION_GAIN,                                    /**< Color image gain*/
  RS2_OPTION_GAMMA,                                   /**< Color image gamma setting*/
  RS2_OPTION_HUE,                                     /**< Color image hue*/
  RS2_OPTION_SATURATION,                              /**< Color image saturation setting*/
  RS2_OPTION_SHARPNESS,                               /**< Color image sharpness setting*/
  RS2_OPTION_WHITE_BALANCE,                           /**< Controls white balance of color image. Setting any value will disable auto white balance*/
  RS2_OPTION_ENABLE_AUTO_EXPOSURE,                    /**< Enable / disable color image auto-exposure*/
  RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,               /**< Enable / disable color image auto-white-balance*/
  RS2_OPTION_VISUAL_PRESET,                           /**< Provide access to several recommend sets of option presets for the depth camera */
  RS2_OPTION_LASER_POWER,                             /**< Power of the laser emitter, with 0 meaning projector off*/
  RS2_OPTION_ACCURACY,                                /**< Set the number of patterns projected per frame. The higher the accuracy value the more patterns projected. Increasing the number of patterns help to achieve better accuracy. Note that this control is affecting the Depth FPS */
  RS2_OPTION_MOTION_RANGE,                            /**< Motion vs. Range trade-off, with lower values allowing for better motion sensitivity and higher values allowing for better depth range*/
  RS2_OPTION_FILTER_OPTION,                           /**< Set the filter to apply to each depth frame. Each one of the filter is optimized per the application requirements*/
  RS2_OPTION_CONFIDENCE_THRESHOLD,                    /**< The confidence level threshold used by the Depth algorithm pipe to set whether a pixel will get a valid range or will be marked with invalid range*/
  RS2_OPTION_EMITTER_ENABLED,                         /**< Emitter select: 0 – disable all emitters. 1 – enable laser. 2 – enable auto laser. 3 – enable LED.*/
  RS2_OPTION_FRAMES_QUEUE_SIZE,                       /**< Number of frames the user is allowed to keep per stream. Trying to hold-on to more frames will cause frame-drops.*/
  RS2_OPTION_TOTAL_FRAME_DROPS,                       /**< Total number of detected frame drops from all streams */
  RS2_OPTION_AUTO_EXPOSURE_MODE,                      /**< Auto-Exposure modes: Static, Anti-Flicker and Hybrid */
  RS2_OPTION_POWER_LINE_FREQUENCY,                    /**< Power Line Frequency control for anti-flickering Off/50Hz/60Hz/Auto */
  RS2_OPTION_ASIC_TEMPERATURE,                        /**< Current Asic Temperature */
  RS2_OPTION_ERROR_POLLING_ENABLED,                   /**< disable error handling */
  RS2_OPTION_PROJECTOR_TEMPERATURE,                   /**< Current Projector Temperature */
  RS2_OPTION_OUTPUT_TRIGGER_ENABLED,                  /**< Enable / disable trigger to be outputed from the camera to any external device on every depth frame */
  RS2_OPTION_MOTION_MODULE_TEMPERATURE,               /**< Current Motion-Module Temperature */
  RS2_OPTION_DEPTH_UNITS,                             /**< Number of meters represented by a single depth unit */
  RS2_OPTION_ENABLE_MOTION_CORRECTION,                /**< Enable/Disable automatic correction of the motion data */
  RS2_OPTION_AUTO_EXPOSURE_PRIORITY,                  /**< Allows sensor to dynamically ajust the frame rate depending on lighting conditions */
  RS2_OPTION_COLOR_SCHEME,                            /**< Color scheme for data visualization */
  RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED,          /**< Perform histogram equalization post-processing on the depth data */
  RS2_OPTION_MIN_DISTANCE,                            /**< Minimal distance to the target */
  RS2_OPTION_MAX_DISTANCE,                            /**< Maximum distance to the target */
  RS2_OPTION_TEXTURE_SOURCE,                          /**< Texture mapping stream unique ID */
  RS2_OPTION_FILTER_MAGNITUDE,                        /**< The 2D-filter effect. The specific interpretation is given within the context of the filter */
  RS2_OPTION_FILTER_SMOOTH_ALPHA,                     /**< 2D-filter parameter controls the weight/radius for smoothing.*/
  RS2_OPTION_FILTER_SMOOTH_DELTA,                     /**< 2D-filter range/validity threshold*/
  RS2_OPTION_HOLES_FILL,                              /**< Enhance depth data post-processing with holes filling where appropriate*/
  RS2_OPTION_STEREO_BASELINE,                         /**< The distance in mm between the first and the second imagers in stereo-based depth cameras*/
  RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP,             /**< Allows dynamically ajust the converge step value of the target exposure in Auto-Exposure algorithm*/
  RS2_OPTION_INTER_CAM_SYNC_MODE,                     /**< Impose Inter-camera HW synchronization mode. Applicable for D400/L500/Rolling Shutter SKUs */
  RS2_OPTION_STREAM_FILTER,                           /**< Select a stream to process */
  RS2_OPTION_STREAM_FORMAT_FILTER,                    /**< Select a stream format to process */
  RS2_OPTION_STREAM_INDEX_FILTER,                     /**< Select a stream index to process */
  RS2_OPTION_EMITTER_ON_OFF,                          /**< When supported, this option make the camera to switch the emitter state every frame. 0 for disabled, 1 for enabled */
  RS2_OPTION_ZERO_ORDER_POINT_X,                      /**< Deprecated!!! - Zero order point x*/
  RS2_OPTION_ZERO_ORDER_POINT_Y,                      /**< Deprecated!!! - Zero order point y*/
  RS2_OPTION_LLD_TEMPERATURE,                         /**< LDD temperature*/
  RS2_OPTION_MC_TEMPERATURE,                          /**< MC temperature*/
  RS2_OPTION_MA_TEMPERATURE,                          /**< MA temperature*/
  RS2_OPTION_HARDWARE_PRESET,                         /**< Hardware stream configuration */
  RS2_OPTION_GLOBAL_TIME_ENABLED,                     /**< disable global time  */
  RS2_OPTION_APD_TEMPERATURE,                         /**< APD temperature*/
  RS2_OPTION_ENABLE_MAPPING,                          /**< Enable an internal map */
  RS2_OPTION_ENABLE_RELOCALIZATION,                   /**< Enable appearance based relocalization */
  RS2_OPTION_ENABLE_POSE_JUMPING,                     /**< Enable position jumping */
  RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION,              /**< Enable dynamic calibration */
  RS2_OPTION_DEPTH_OFFSET,                            /**< Offset from sensor to depth origin in millimetrers*/
  RS2_OPTION_LED_POWER,                               /**< Power of the LED (light emitting diode), with 0 meaning LED off*/
  RS2_OPTION_ZERO_ORDER_ENABLED,                      /**< DEPRECATED! - Toggle Zero-Order mode */
  RS2_OPTION_ENABLE_MAP_PRESERVATION,                 /**< Preserve previous map when starting */
  RS2_OPTION_FREEFALL_DETECTION_ENABLED,              /**< Enable/disable sensor shutdown when a free-fall is detected (on by default) */
  RS2_OPTION_AVALANCHE_PHOTO_DIODE,                   /**< Changes the exposure time of Avalanche Photo Diode in the receiver */
  RS2_OPTION_POST_PROCESSING_SHARPENING,              /**< Changes the amount of sharpening in the post-processed image */
  RS2_OPTION_PRE_PROCESSING_SHARPENING,               /**< Changes the amount of sharpening in the pre-processed image */
  RS2_OPTION_NOISE_FILTERING,                         /**< Control edges and background noise */
  RS2_OPTION_INVALIDATION_BYPASS,                     /**< Enable\disable pixel invalidation */
  RS2_OPTION_AMBIENT_LIGHT,                           /**< DEPRECATED! - Use RS2_OPTION_DIGITAL_GAIN instead. */
  RS2_OPTION_DIGITAL_GAIN = RS2_OPTION_AMBIENT_LIGHT, /**< Change the depth digital gain see rs2_digital_gain for values */
  RS2_OPTION_SENSOR_MODE,                             /**< The resolution mode: see rs2_sensor_mode for values */
  RS2_OPTION_EMITTER_ALWAYS_ON,                       /**< Enable Laser On constantly (GS SKU Only) */
  RS2_OPTION_THERMAL_COMPENSATION,                    /**< Depth Thermal Compensation for selected D400 SKUs */
  RS2_OPTION_TRIGGER_CAMERA_ACCURACY_HEALTH,          /**< DEPRECATED as of 2.46! */
  RS2_OPTION_RESET_CAMERA_ACCURACY_HEALTH,            /**< DEPRECATED as of 2.46! */
  RS2_OPTION_HOST_PERFORMANCE,                        /**< Set host performance mode to optimize device settings so host can keep up with workload, for example, USB transaction granularity, setting option to low performance host leads to larger USB transaction size and reduced number of transactions which improves performance and stability if host is relatively weak as compared to workload */
  RS2_OPTION_HDR_ENABLED,                             /**< Enable / disable HDR */
  RS2_OPTION_SEQUENCE_NAME,                           /**< HDR Sequence name */
  RS2_OPTION_SEQUENCE_SIZE,                           /**< HDR Sequence size */
  RS2_OPTION_SEQUENCE_ID,                             /**< HDR Sequence ID - 0 is not HDR; sequence ID for HDR configuration starts from 1 */
  RS2_OPTION_HUMIDITY_TEMPERATURE,                    /**< Humidity temperature [Deg Celsius]*/
  RS2_OPTION_ENABLE_MAX_USABLE_RANGE,                 /**< Turn on/off the maximum usable depth sensor range given the amount of ambient light in the scene */
  RS2_OPTION_ALTERNATE_IR,                            /**< Turn on/off the alternate IR, When enabling alternate IR, the IR image is holding the amplitude of the depth correlation. */
  RS2_OPTION_NOISE_ESTIMATION,                        /**< Noise estimation - indicates the noise on the IR image */
  RS2_OPTION_ENABLE_IR_REFLECTIVITY,                  /**< Enables data collection for calculating IR pixel reflectivity  */
  RS2_OPTION_AUTO_EXPOSURE_LIMIT,                     /**< Set and get auto exposure limit in microseconds. If the requested exposure limit is greater than frame time, it will be set to frame time at runtime. Setting will not take effect until next streaming session. */
  RS2_OPTION_AUTO_GAIN_LIMIT,                         /**< Set and get auto gain limits ranging from 16 to 248. If the requested gain limit is less than 16, it will be set to 16. If the requested gain limit is greater than 248, it will be set to 248. Setting will not take effect until next streaming session. */
  RS2_OPTION_AUTO_RX_SENSITIVITY,                     /**< Enable receiver sensitivity according to ambient light, bounded by the Receiver Gain control. */
  RS2_OPTION_TRANSMITTER_FREQUENCY,                   /**<changes the transmitter frequencies increasing effective range over sharpness. */
  RS2_OPTION_VERTICAL_BINNING,                        /**< Enables vertical binning which increases the maximal sensed distance. */
  RS2_OPTION_RECEIVER_SENSITIVITY,                    /**< Control receiver sensitivity to incoming light, both projected and ambient (same as APD on L515). */
  RS2_OPTION_AUTO_EXPOSURE_LIMIT_TOGGLE,              /**< Enable / disable color image auto-exposure*/
  RS2_OPTION_AUTO_GAIN_LIMIT_TOGGLE,                  /**< Enable / disable color image auto-gain*/
  RS2_OPTION_COUNT                                    /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
} rs2_option;
enum usb_spec : uint16_t {
  usb_undefined = 0,
  usb1_type = 0x0100,
  usb1_1_type = 0x0110,
  usb2_type = 0x0200,
  usb2_01_type = 0x0201,
  usb2_1_type = 0x0210,
  usb3_type = 0x0300,
  usb3_1_type = 0x0310,
  usb3_2_type = 0x0320,
};
class uvc_device {
public:
  virtual void probe_and_commit(stream_profile profile, frame_callback callback, int buffers = DEFAULT_V4L2_FRAME_BUFFERS) = 0;
  virtual void stream_on() = 0;
  virtual void start_callbacks() = 0;
  virtual void stop_callbacks() = 0;
  virtual void close(stream_profile profile) = 0;

  virtual void set_power_state(power_state state) = 0;
  virtual power_state get_power_state() const = 0;

  virtual void init_xu(const extension_unit &xu) = 0;
  virtual bool set_xu(const extension_unit &xu, uint8_t ctrl, const uint8_t *data, int len) = 0;
  virtual bool get_xu(const extension_unit &xu, uint8_t ctrl, uint8_t *data, int len) const = 0;
  virtual control_range get_xu_range(const extension_unit &xu, uint8_t ctrl, int len) const = 0;

  virtual bool get_pu(rs2_option opt, int32_t &value) const = 0;
  virtual bool set_pu(rs2_option opt, int32_t value) = 0;
  virtual control_range get_pu_range(rs2_option opt) const = 0;

  virtual std::vector<stream_profile> get_profiles() const = 0;

  virtual void lock() const = 0;
  virtual void unlock() const = 0;

  virtual std::string get_device_location() const = 0;
  virtual usb_spec get_usb_specification() const = 0;

  virtual ~uvc_device() = default;

protected:
  std::function<void()> _error_handler;
};

struct uvc_device_info {
  std::string id = ""; // to distinguish between different pins of the same device
  uint16_t vid = 0;
  uint16_t pid = 0;
  uint16_t mi = 0;
  std::string unique_id = "";
  std::string device_path = "";
  std::string serial = "";
  usb_spec conn_spec = usb_undefined;
  uint32_t uvc_capabilities = 0;
  bool has_metadata_node = false;
  std::string metadata_node_id = "";

  operator std::string() {
    std::stringstream s;
    s << "id- " << id << "\nvid- " << std::hex << vid << "\npid- " << std::hex << pid << "\nmi- " << mi << "\nunique_id- " << unique_id << "\npath- " << device_path << "\nsusb specification- " << std::hex << (uint16_t)conn_spec << std::dec << (has_metadata_node ? ("\nmetadata node-" + metadata_node_id) : "");

    return s.str();
  }

  bool operator<(const uvc_device_info &obj) const {
    return (std::make_tuple(id, vid, pid, mi, unique_id, device_path) < std::make_tuple(obj.id, obj.vid, obj.pid, obj.mi, obj.unique_id, obj.device_path));
  }
};

typedef unsigned char byte;
void foreach_uvc_device(
    std::function<void(const uvc_device_info &,
                       const std::string &)>
        action);
class v4l_uvc_device : public uvc_device, public v4l_uvc_interface {
public:
  v4l_uvc_device(const uvc_device_info &info, bool use_memory_map = false);

  ~v4l_uvc_device() override;

  void probe_and_commit(stream_profile profile, frame_callback callback, int buffers) override;

  void stream_on() override;

  void start_callbacks() override;

  void stop_callbacks() override;

  void close(stream_profile) override;

  std::string fourcc_to_string(uint32_t id) const;

  void signal_stop();

  void poll();

  void set_power_state(power_state state) override;
  power_state get_power_state() const override { return _state; }

  void init_xu(const extension_unit &) override {}
  bool set_xu(const extension_unit &xu, uint8_t control, const uint8_t *data, int size) override;
  bool get_xu(const extension_unit &xu, uint8_t control, uint8_t *data, int size) const override;
  control_range get_xu_range(const extension_unit &xu, uint8_t control, int len) const override;

  bool get_pu(rs2_option opt, int32_t &value) const override;

  bool set_pu(rs2_option opt, int32_t value) override;

  control_range get_pu_range(rs2_option option) const override;

  std::vector<stream_profile> get_profiles() const override;

  void lock() const override;
  void unlock() const override;

  std::string get_device_location() const override { return _device_path; }
  usb_spec get_usb_specification() const override { return _device_usb_spec; }

protected:
  static uint32_t get_cid(rs2_option option);

  virtual void capture_loop() override;

  virtual bool has_metadata() const override;

  virtual void streamon() const override;
  virtual void streamoff() const override;
  virtual void negotiate_kernel_buffers(size_t num) const override;

  virtual void allocate_io_buffers(size_t num) override;
  virtual void map_device_descriptor() override;
  virtual void unmap_device_descriptor() override;
  virtual void set_format(stream_profile profile) override;
  virtual void prepare_capture_buffers() override;
  virtual void stop_data_capture() override;
  // virtual void acquire_metadata(buffers_mgr &buf_mgr, fd_set &fds, bool compressed_format = false) override;
  void subscribe_to_ctrl_event(uint32_t control_id);
  void unsubscribe_from_ctrl_event(uint32_t control_id);
  bool pend_for_ctrl_status_event();

  power_state _state = D3;
  std::string _name = "";
  std::string _device_path = "";
  usb_spec _device_usb_spec = usb_undefined;
  uvc_device_info _info;

  std::vector<std::shared_ptr<buffer>> _buffers;
  stream_profile _profile;
  frame_callback _callback;
  std::atomic<bool> _is_capturing;
  std::atomic<bool> _is_alive;
  std::atomic<bool> _is_started;
  std::unique_ptr<std::thread> _thread;
  // std::unique_ptr<named_mutex> _named_mtx;
  bool _use_memory_map;
  int _max_fd = 0;       // specifies the maximal pipe number the polling process will monitor
  std::vector<int> _fds; // list the file descriptors to be monitored during frames polling
                         // buffers_mgr _buf_dispatch; // Holder for partial (MD only) frames that shall be preserved between 'select' calls when polling v4l buffers
                         //  frame_drop_monitor _frame_drop_monitor; // used to check the frames drops kpi

private:
  int _fd = 0;          // prevent unintentional abuse in derived class
  int _stop_pipe_fd[2]; // write to _stop_pipe_fd[1] and read from _stop_pipe_fd[0]
};
/*
// Composition layer for uvc/metadata split nodes introduced with kernel 4.16
class v4l_uvc_meta_device : public v4l_uvc_device
{
public:
    v4l_uvc_meta_device(const uvc_device_info &info, bool use_memory_map = false);

    ~v4l_uvc_meta_device();

protected:
    void streamon() const;
    void streamoff() const;
    void negotiate_kernel_buffers(size_t num) const;
    void allocate_io_buffers(size_t num);
    void map_device_descriptor();
    void unmap_device_descriptor();
    void set_format(stream_profile profile);
    void prepare_capture_buffers();
    virtual void acquire_metadata(buffers_mgr &buf_mgr, fd_set &fds, bool compressed_format = false);

    int _md_fd = -1;
    std::string _md_name = "";

    std::vector<std::shared_ptr<buffer>> _md_buffers;
    stream_profile _md_profile;
};

class v4l_backend : public backend
{
public:
    std::shared_ptr<uvc_device> create_uvc_device(uvc_device_info info) const override;
    std::vector<uvc_device_info> query_uvc_devices() const override;

    std::shared_ptr<command_transfer> create_usb_device(usb_device_info info) const override;
    std::vector<usb_device_info> query_usb_devices() const override;

    std::shared_ptr<hid_device> create_hid_device(hid_device_info info) const override;
    std::vector<hid_device_info> query_hid_devices() const override;

    std::shared_ptr<time_service> create_time_service() const override;
    std::shared_ptr<device_watcher> create_device_watcher() const override;
};*/

void v4luvc_init();
typedef void (*UVCDEVICECALLBACK)(struct uvc_device_info *pinfo, void *userdata);
void query_uvc_devices(UVCDEVICECALLBACK callback, uint16_t vid, uint16_t pid, void *userdata);
int v4luvc_transfer(std::shared_ptr<v4l_uvc_device> dev, unsigned char *data, int len);

v4l_uvc_device *get_uvc_device_by_vidpid_path(uint16_t vid, uint16_t pid, const char *path);

typedef void (*STREAMCALLBACK)(const void *pixels, size_t framesize, void *userdata);
void v4luvc_start_stream(STREAMCALLBACK callback, std::shared_ptr<v4l_uvc_device> wmfdevice, void *userdata);
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

// #include "backend-hid.h"
// #include "backend.h"
// #include "types.h"
/*#if defined(USING_UDEV)
#include "udev-device-watcher.h"
#else
#include "../polling-device-watcher.h"
#endif
*/
// #include "usb/usb-enumerator.h"
// #include "usb/usb-device.h"

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <cstring>

#include <algorithm>
#include <functional>
#include <string>
#include <sstream>
#include <fstream>
#include <regex>
#include <thread>
#include <utility> // for pair
#include <chrono>
#include <thread>
#include <atomic>
#include <iomanip> // std::put_time

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>
#include <cmath>
#include <errno.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/sysmacros.h> // minor(...), major(...)
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>
#include <linux/videodev2.h>
#include <regex>
#include <list>

#include <sys/signalfd.h>
#include <signal.h>

#include <string>
#include <chrono>
#include <sstream>

struct to_string {
  std::ostringstream ss;
  template <class T>
  to_string &operator<<(const T &val) {
    ss << val;
    return *this;
  }
  operator std::string() const { return ss.str(); }
};

#pragma GCC diagnostic ignored "-Woverflow"

// #include "../tm2/tm-boot.h"

#ifdef ANDROID

// https://android.googlesource.com/platform/bionic/+/master/libc/include/bits/lockf.h
#define F_ULOCK 0
#define F_LOCK 1
#define F_TLOCK 2
#define F_TEST 3

// https://android.googlesource.com/platform/bionic/+/master/libc/bionic/lockf.cpp
int lockf64(int fd, int cmd, off64_t length) {
  // Translate POSIX lockf into fcntl.
  struct flock64 fl;
  memset(&fl, 0, sizeof(fl));
  fl.l_whence = SEEK_CUR;
  fl.l_start = 0;
  fl.l_len = length;

  if (cmd == F_ULOCK) {
    fl.l_type = F_UNLCK;
    cmd = F_SETLK64;
    return fcntl(fd, F_SETLK64, &fl);
  }

  if (cmd == F_LOCK) {
    fl.l_type = F_WRLCK;
    return fcntl(fd, F_SETLKW64, &fl);
  }

  if (cmd == F_TLOCK) {
    fl.l_type = F_WRLCK;
    return fcntl(fd, F_SETLK64, &fl);
  }
  if (cmd == F_TEST) {
    fl.l_type = F_RDLCK;
    if (fcntl(fd, F_GETLK64, &fl) == -1)
      return -1;
    if (fl.l_type == F_UNLCK || fl.l_pid == getpid())
      return 0;
    errno = EACCES;
    return -1;
  }

  errno = EINVAL;
  return -1;
}

int lockf(int fd, int cmd, off_t length) {
  return lockf64(fd, cmd, length);
}
#endif

std::recursive_mutex named_mutex::_init_mutex;
std::map<std::string, std::recursive_mutex> named_mutex::_dev_mutex;
std::map<std::string, int> named_mutex::_dev_mutex_cnt;

named_mutex::named_mutex(const std::string &device_path, unsigned timeout)
    : _device_path(device_path),
      _timeout(timeout), // TODO: try to lock with timeout
      _fildes(-1),
      _object_lock_counter(0) {
  _init_mutex.lock();
  _dev_mutex[_device_path]; // insert a mutex for _device_path
  if (_dev_mutex_cnt.find(_device_path) == _dev_mutex_cnt.end()) {
    _dev_mutex_cnt[_device_path] = 0;
  }
  _init_mutex.unlock();
}

named_mutex::~named_mutex() {
  unlock();
}

void named_mutex::lock() {
  std::lock_guard<std::mutex> lock(_mutex);
  acquire();
}

void named_mutex::unlock() {
  std::lock_guard<std::mutex> lock(_mutex);
  release();
}

bool named_mutex::try_lock() {
  std::lock_guard<std::mutex> lock(_mutex);
  if (-1 == _fildes) {
    _fildes = open(_device_path.c_str(), O_RDWR, 0); // TODO: check
    if (_fildes < 0)
      return false;
  }

  auto ret = lockf(_fildes, F_TLOCK, 0);
  if (ret != 0)
    return false;

  return true;
}

void named_mutex::acquire() {
  _dev_mutex[_device_path].lock();
  _dev_mutex_cnt[_device_path] += 1; // Advance counters even if throws because catch calls release()
  _object_lock_counter += 1;
  if (_dev_mutex_cnt[_device_path] == 1) {
    if (-1 == _fildes) {
      _fildes = open(_device_path.c_str(), O_RDWR, 0); // TODO: check
      if (0 > _fildes) {
        release();
        // throw linux_backend_exception(to_string() << __FUNCTION__ << ": Cannot open '" << _device_path);
      }
    }

    auto ret = lockf(_fildes, F_LOCK, 0);
    if (0 != ret) {
      release();
      // throw linux_backend_exception(to_string() << __FUNCTION__ << ": Acquire failed");
    }
  }
}

void named_mutex::release() {
  _object_lock_counter -= 1;
  if (_object_lock_counter < 0) {
    _object_lock_counter = 0;
    return;
  }
  _dev_mutex_cnt[_device_path] -= 1;
  std::string err_msg;
  if (_dev_mutex_cnt[_device_path] < 0) {
    _dev_mutex_cnt[_device_path] = 0;
    //  throw linux_backend_exception(to_string() << "Error: _dev_mutex_cnt[" << _device_path << "] < 0");
  }

  if ((_dev_mutex_cnt[_device_path] == 0) && (-1 != _fildes)) {
    auto ret = lockf(_fildes, F_ULOCK, 0);
    if (0 != ret)
      err_msg = to_string() << "lockf(...) failed";
    else {
      ret = close(_fildes);
      if (0 != ret)
        err_msg = to_string() << "close(...) failed";
      else
        _fildes = -1;
    }
  }
  _dev_mutex[_device_path].unlock();

  //   if (!err_msg.empty())
  //     throw linux_backend_exception(err_msg);
}

static int xioctl(int fh, unsigned long request, void *arg) {
  int r = 0;
  do {
    r = ioctl(fh, request, arg);
  } while (r < 0 && errno == EINTR);
  return r;
}

buffer::buffer(int fd, v4l2_buf_type type, bool use_memory_map, uint32_t index)
    : _type(type), _use_memory_map(use_memory_map), _index(index) {
  v4l2_buffer buf = {};
  buf.type = _type;
  buf.memory = use_memory_map ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;
  buf.index = index;
  printf("will query buffer!\n");
  if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
    // throw linux_backend_exception("xioctl(VIDIOC_QUERYBUF) failed");
  }

  // Prior to kernel 4.16 metadata payload was attached to the end of the video payload
  uint8_t md_extra = (V4L2_BUF_TYPE_VIDEO_CAPTURE == type) ? MAX_META_DATA_SIZE : 0;
  _original_length = buf.length;
  _length = _original_length + md_extra;

  if (use_memory_map) {
    _start = static_cast<uint8_t *>(mmap(nullptr, buf.length,
                                         PROT_READ | PROT_WRITE, MAP_SHARED,
                                         fd, buf.m.offset));
    if (_start == MAP_FAILED) {
      // throw linux_backend_exception("mmap failed");
    }
  } else {
    //_length += (V4L2_BUF_TYPE_VIDEO_CAPTURE==type) ? MAX_META_DATA_SIZE : 0;
    _start = static_cast<uint8_t *>(malloc(_length));
    if (!_start) {
      //   throw linux_backend_exception("User_p allocation failed!");
    }
    memset(_start, 0, _length);
  }
}

void buffer::prepare_for_streaming(int fd) {
  v4l2_buffer buf = {};
  buf.type = _type;
  buf.memory = _use_memory_map ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;
  buf.index = _index;
  buf.length = _length;

  if (!_use_memory_map) {
    buf.m.userptr = reinterpret_cast<unsigned long>(_start);
  }
  printf("qauery buffer in prepare_for_streaming!\n");
  if (xioctl(fd, VIDIOC_QBUF, &buf) < 0) {
    // throw linux_backend_exception("xioctl(VIDIOC_QBUF) failed");
  } else {
    //     //LOG_DEBUG("prepare_for_streaming fd " << std::dec << fd);
  }
}

buffer::~buffer() {
  if (_use_memory_map) {
    if (munmap(_start, _length) < 0) {
      //   linux_backend_exception("munmap");
    }
  } else {
    free(_start);
  }
}

void buffer::attach_buffer(const v4l2_buffer &buf) {
  std::lock_guard<std::mutex> lock(_mutex);
  _buf = buf;
  _must_enqueue = true;
}

void buffer::detach_buffer() {
  std::lock_guard<std::mutex> lock(_mutex);
  _must_enqueue = false;
}

void buffer::request_next_frame(int fd, bool force) {
  std::lock_guard<std::mutex> lock(_mutex);

  if (_must_enqueue || force) {
    if (!_use_memory_map) {
      auto metadata_offset = get_full_length() - MAX_META_DATA_SIZE;
      memset((byte *)(get_frame_start()) + metadata_offset, 0, MAX_META_DATA_SIZE);
    }

    printf("enqueue buf!!!\n");
    ////LOG_DEBUG("Enqueue buf " << std::dec << _buf.index << " for fd " << fd);
    if (xioctl(fd, VIDIOC_QBUF, &_buf) < 0) {
      // LOG_ERROR(("xioctl(VIDIOC_QBUF) failed when requesting new frame! fd: " << fd << " error: " << strerror(errno));
    }

    _must_enqueue = false;
  }
}
/*
void buffers_mgr::handle_buffer(supported_kernel_buf_types buf_type,
                                int file_desc,
                                v4l2_buffer v4l_buf,
                                std::shared_ptr<buffer> data_buf)
{
    if (e_max_kernel_buf_type <= buf_type)
    {
        // throw linux_backend_exception("invalid kernel buffer type request");
    }

    if (file_desc < 1)
    {
        // QBUF to be performed by a 3rd party
        this->buffers.at(buf_type)._managed = true;
    }
    else
    {
        buffers.at(buf_type)._file_desc = file_desc;
        buffers.at(buf_type)._managed = false;
        buffers.at(buf_type)._data_buf = data_buf;
        buffers.at(buf_type)._dq_buf = v4l_buf;
    }
}

void buffers_mgr::request_next_frame()
{
    for (auto &buf : buffers)
    {
        if (buf._data_buf && (buf._file_desc >= 0))
            buf._data_buf->request_next_frame(buf._file_desc);
    };
    _md_start = nullptr;
    _md_size = 0;
}*/
template <typename T>
bool val_in_range(const T &val, const std::initializer_list<T> &list) {
  for (const auto &i : list) {
    if (val == i) {
      return true;
    }
  }
  return false;
}
/*
void buffers_mgr::set_md_from_video_node(bool compressed)
{
    void *md_start = nullptr;
    auto md_size = 0;

    if (buffers.at(e_video_buf)._file_desc >= 0)
    {
        static const int d4xx_md_size = 248;
        auto buffer = buffers.at(e_video_buf)._data_buf;
        auto dq = buffers.at(e_video_buf)._dq_buf;
        auto fr_payload_size = buffer->get_length_frame_only();

        // For compressed data assume D4XX metadata struct
        // TODO - devise SKU-agnostic heuristics
        auto md_appendix_sz = 0L;
        if (compressed && (dq.bytesused < fr_payload_size))
            md_appendix_sz = d4xx_md_size;
        else
            md_appendix_sz = long(dq.bytesused) - fr_payload_size;

        if (md_appendix_sz > 0)
        {
            md_start = buffer->get_frame_start() + dq.bytesused - md_appendix_sz;
            md_size = (*(static_cast<uint8_t *>(md_start)));
            int md_flags = (*(static_cast<uint8_t *>(md_start) + 1));
            // Use heuristics for metadata validation
            if ((md_appendix_sz != md_size) || (!val_in_range(md_flags, {0x8e, 0x8f})))
            {
                md_size = 0;
                md_start = nullptr;
            }
        }
    }

    if (nullptr == md_start)
    {
        // LOG_DEBUG("Could not parse metadata");
    }
    set_md_attributes(static_cast<uint8_t>(md_size), md_start);
}

bool buffers_mgr::verify_vd_md_sync() const
{
    if ((buffers[e_video_buf]._file_desc > 0) && (buffers[e_metadata_buf]._file_desc > 0))
        if (buffers[e_video_buf]._dq_buf.sequence != buffers[e_metadata_buf]._dq_buf.sequence)
        {
            // LOG_ERROR(("Non-sequential Video and Metadata v4l buffers");
            return false;
        }
    return true;
}

bool buffers_mgr::md_node_present() const
{
    return (buffers[e_metadata_buf]._file_desc > 0);
}*/
static const std::map<usb_spec, std::string> usb_spec_names = {
    {usb_undefined, "Undefined"},
    {usb1_type, "1.0"},
    {usb1_1_type, "1.1"},
    {usb2_type, "2.0"},
    {usb2_01_type, "2.01"},
    {usb2_1_type, "2.1"},
    {usb3_type, "3.0"},
    {usb3_1_type, "3.1"},
    {usb3_2_type, "3.2"}};
// retrieve the USB specification attributed to a specific USB device.
// This functionality is required to find the USB connection type for UVC device
// Note that the input parameter is passed by value
static usb_spec get_usb_connection_type(std::string path) {
  usb_spec res{usb_undefined};

  char usb_actual_path[PATH_MAX] = {0};
  if (realpath(path.c_str(), usb_actual_path) != nullptr) {
    path = std::string(usb_actual_path);
    std::string val;
    if (!(std::ifstream(path + "/version") >> val)) {
      //   throw linux_backend_exception("Failed to read usb version specification");
    }

    auto kvp = std::find_if(usb_spec_names.begin(), usb_spec_names.end(),
                            [&val](const std::pair<usb_spec, std::string> &kpv) { return (std::string::npos != val.find(kpv.second)); });
    if (kvp != std::end(usb_spec_names))
      res = kvp->first;
  }
  return res;
}

// Retrieve device video capabilities to discriminate video capturing and metadata nodes
static uint32_t get_dev_capabilities(const std::string dev_name) {
  // RAII to handle exceptions
  std::unique_ptr<int, std::function<void(int *)>> fd(
      new int(open(dev_name.c_str(), O_RDWR | O_NONBLOCK, 0)),
      [](int *d) { if (d && (*d)) {::close(*d); } delete d; });

  if (*fd < 0) {
    //   throw linux_backend_exception(to_string() << __FUNCTION__ << ": Cannot open '" << dev_name);
  }

  v4l2_capability cap = {};
  if (xioctl(*fd, VIDIOC_QUERYCAP, &cap) < 0) {
    if (errno == EINVAL) {
      //  throw linux_backend_exception(to_string() << __FUNCTION__ << " " << dev_name << " is no V4L2 device");
    } else {
      //   throw linux_backend_exception(to_string() << __FUNCTION__ << " xioctl(VIDIOC_QUERYCAP) failed");
    }
  }

  return cap.device_caps;
}

void stream_ctl_on(int fd, v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE) {
  if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    //  throw linux_backend_exception(to_string() << "xioctl(VIDIOC_STREAMON) failed for buf_type=" << type);
  }
}

void stream_off(int fd, v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE) {
  if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
    //   throw linux_backend_exception(to_string() << "xioctl(VIDIOC_STREAMOFF) failed for buf_type=" << type);
  }
}

void req_io_buff(int fd, uint32_t count, std::string dev_name,
                 v4l2_memory mem_type, v4l2_buf_type type) {
  struct v4l2_requestbuffers req = {count, type, mem_type, {}};

  printf("req io buff!\n");
  if (xioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    if (errno == EINVAL) {
      // LOG_ERROR((dev_name + " does not support memory mapping");
    } else {
      //     throw linux_backend_exception("xioctl(VIDIOC_REQBUFS) failed");
    }
  }
}
typedef enum _usb_class {
  RS2_USB_CLASS_UNSPECIFIED = 0x00,
  RS2_USB_CLASS_AUDIO = 0x01,
  RS2_USB_CLASS_COM = 0x02,
  RS2_USB_CLASS_HID = 0x03,
  RS2_USB_CLASS_PID = 0x05,
  RS2_USB_CLASS_IMAGE = 0x06,
  RS2_USB_CLASS_PRINTER = 0x07,
  RS2_USB_CLASS_MASS_STORAGE = 0x08,
  RS2_USB_CLASS_HUB = 0x09,
  RS2_USB_CLASS_CDC_DATA = 0x0A,
  RS2_USB_CLASS_SMART_CARD = 0x0B,
  RS2_USB_CLASS_CONTENT_SECURITY = 0x0D,
  RS2_USB_CLASS_VIDEO = 0x0E,
  RS2_USB_CLASS_PHDC = 0x0F,
  RS2_USB_CLASS_AV = 0x10,
  RS2_USB_CLASS_BILLBOARD = 0x11,
  RS2_USB_CLASS_DIAGNOSTIC_DEVIE = 0xDC,
  RS2_USB_CLASS_WIRELESS_CONTROLLER = 0xE0,
  RS2_USB_CLASS_MISCELLANEOUS = 0xEF,
  RS2_USB_CLASS_APPLICATION_SPECIFIC = 0xFE,
  RS2_USB_CLASS_VENDOR_SPECIFIC = 0xFF
} usb_class;
struct usb_device_info {
  std::string id;

  uint16_t vid;
  uint16_t pid;
  uint16_t mi;
  std::string unique_id;
  std::string serial;
  usb_spec conn_spec;
  usb_class cls;

  operator std::string() {
    std::stringstream s;

    s << "vid- " << std::hex << vid << "\npid- " << std::hex << pid << "\nmi- " << mi << "\nsusb specification- " << std::hex << (uint16_t)conn_spec << std::dec << "\nunique_id- " << unique_id;

    return s.str();
  }
};
inline bool operator==(const uvc_device_info &a,
                       const uvc_device_info &b) {
  return (a.vid == b.vid) &&
         (a.pid == b.pid) &&
         (a.mi == b.mi) &&
         (a.unique_id == b.unique_id) &&
         (a.id == b.id) &&
         (a.device_path == b.device_path) &&
         (a.conn_spec == b.conn_spec);
}

inline bool operator==(const usb_device_info &a,
                       const usb_device_info &b) {
  return (a.id == b.id) &&
         (a.vid == b.vid) &&
         (a.pid == b.pid) &&
         (a.mi == b.mi) &&
         (a.unique_id == b.unique_id) &&
         (a.conn_spec == b.conn_spec);
}

void foreach_uvc_device(
    std::function<void(const uvc_device_info &,
                       const std::string &)>
        action) {
  // Enumerate all subdevices present on the system
  DIR *dir = opendir("/sys/class/video4linux");
  if (!dir) {
    // LOG_INFO(("Cannot access /sys/class/video4linux");
    return;
  }

  // Collect UVC nodes info to bundle metadata and video
  typedef std::pair<uvc_device_info, std::string> node_info;
  std::vector<node_info> uvc_nodes, uvc_devices;

  while (dirent *entry = readdir(dir)) {
    std::string name = entry->d_name;
    if (name == "." || name == "..")
      continue;

    // Resolve a pathname to ignore virtual video devices
    std::string path = "/sys/class/video4linux/" + name;
    std::string real_path{};
    char buff[PATH_MAX] = {0};
    if (realpath(path.c_str(), buff) != nullptr) {
      real_path = std::string(buff);
      if (real_path.find("virtual") != std::string::npos)
        continue;
    }

    try {
      uint16_t vid, pid, mi;
      std::string busnum, devnum, devpath;

      auto dev_name = "/dev/" + name;

      struct stat st = {};
      if (stat(dev_name.c_str(), &st) < 0) {
        //  throw linux_backend_exception(to_string() << "Cannot identify '" << dev_name);
      }
      if (!S_ISCHR(st.st_mode)) {
        //   throw linux_backend_exception(dev_name + " is no device");
      }

      // Search directory and up to three parent directories to find busnum/devnum
      std::ostringstream ss;
      ss << "/sys/dev/char/" << major(st.st_rdev) << ":" << minor(st.st_rdev) << "/device/";
      auto path = ss.str();
      auto valid_path = false;
      for (auto i = 0U; i < MAX_DEV_PARENT_DIR; ++i) {
        if (std::ifstream(path + "busnum") >> busnum) {
          if (std::ifstream(path + "devnum") >> devnum) {
            if (std::ifstream(path + "devpath") >> devpath) {
              valid_path = true;
              break;
            }
          }
        }
        path += "../";
      }
      if (!valid_path) {
#ifndef RS2_USE_CUDA
        /* On the Jetson TX, the camera module is CSI & I2C and does not report as this code expects
        Patch suggested by JetsonHacks: https://github.com/jetsonhacks/buildLibrealsense2TX */
        // LOG_INFO(("Failed to read busnum/devnum. Device Path: " << path);
#endif
        continue;
      }

      std::string modalias;
      if (!(std::ifstream("/sys/class/video4linux/" + name + "/device/modalias") >> modalias)) {
        //    throw linux_backend_exception("Failed to read modalias");
      }
      if (modalias.size() < 14 || modalias.substr(0, 5) != "usb:v" || modalias[9] != 'p') {
        //    throw linux_backend_exception("Not a usb format modalias");
      }
      if (!(std::istringstream(modalias.substr(5, 4)) >> std::hex >> vid)) {
        //     throw linux_backend_exception("Failed to read vendor ID");
      }
      if (!(std::istringstream(modalias.substr(10, 4)) >> std::hex >> pid)) {

        //    throw linux_backend_exception("Failed to read product ID");
      }
      if (!(std::ifstream("/sys/class/video4linux/" + name + "/device/bInterfaceNumber") >> std::hex >> mi)) {
        //   throw linux_backend_exception("Failed to read interface number");
      }

      // Find the USB specification (USB2/3) type from the underlying device
      // Use device mapping obtained in previous step to traverse node tree
      // and extract the required descriptors
      // Traverse from
      // /sys/devices/pci0000:00/0000:00:xx.0/ABC/M-N/3-6:1.0/video4linux/video0
      // to
      // /sys/devices/pci0000:00/0000:00:xx.0/ABC/M-N/version
      usb_spec usb_specification = get_usb_connection_type(real_path + "/../../../");

      uvc_device_info info{};
      info.pid = pid;
      info.vid = vid;
      info.mi = mi;
      info.id = dev_name;
      info.device_path = std::string(buff);
      info.unique_id = busnum + "-" + devpath + "-" + devnum;
      info.conn_spec = usb_specification;
      info.uvc_capabilities = get_dev_capabilities(dev_name);

      uvc_nodes.emplace_back(info, dev_name);
    } catch (const std::exception &e) {
      // LOG_INFO(("Not a USB video device: " << e.what());
    }
  }
  closedir(dir);

  // Matching video and metadata nodes
  // UVC nodes shall be traversed in ascending order for metadata nodes assignment ("dev/video1, Video2..
  // Replace lexicographic with numeric sort to ensure "video2" is listed before "video11"
  std::sort(begin(uvc_nodes), end(uvc_nodes), [](const node_info &lhs, const node_info &rhs) {
                            std::stringstream index_l(lhs.first.id.substr(lhs.first.id.find_first_of("0123456789")));
                            std::stringstream index_r(rhs.first.id.substr(rhs.first.id.find_first_of("0123456789")));
                            int left_id = 0;  index_l >> left_id;
                            int right_id = 0;  index_r >> right_id;
                            return left_id < right_id; });

  // Assume for each metadata node with index N there is a origin streaming node with index (N-1)
  for (auto &&cur_node : uvc_nodes) {
    try {
      if (!(cur_node.first.uvc_capabilities & V4L2_CAP_META_CAPTURE))
        uvc_devices.emplace_back(cur_node);
      else {
        if (uvc_devices.empty()) {
          // LOG_ERROR(("uvc meta-node with no video streaming node encountered: " << std::string(cur_node.first));
          continue;
        }

        // Update the preceding uvc item with metadata node info
        auto uvc_node = uvc_devices.back();

        if (uvc_node.first.uvc_capabilities & V4L2_CAP_META_CAPTURE) {
          // LOG_ERROR(("Consequtive uvc meta-nodes encountered: " << std::string(uvc_node.first) << " and " << std::string(cur_node.first));
          continue;
        }

        if (uvc_node.first.has_metadata_node) {
          // LOG_ERROR(("Metadata node for uvc device: " << std::string(uvc_node.first) << " was previously assigned ");
          continue;
        }

        // modify the last element with metadata node info
        uvc_node.first.has_metadata_node = true;
        uvc_node.first.metadata_node_id = cur_node.first.id;
        uvc_devices.back() = uvc_node;
      }
    } catch (const std::exception &e) {
      // LOG_ERROR(("Failed to map meta-node " << std::string(cur_node.first) << ", error encountered: " << e.what());
    }
  }

  try {
    // Dispatch registration for enumerated uvc devices
    for (auto &&dev : uvc_devices)
      action(dev.first, dev.second);
  } catch (const std::exception &e) {
    // LOG_ERROR(("Registration of UVC device failed: " << e.what());
  }
}
/*
bool frame_drop_monitor::update_and_check_kpi(const stream_profile &profile, const timeval &timestamp)
{
    bool is_kpi_violated = false;
    long int timestamp_usec = static_cast<long int>(timestamp.tv_sec * 1000000 + timestamp.tv_usec);

    // checking if the current profile is already in the drops_per_stream container
    auto it = std::find_if(drops_per_stream.begin(), drops_per_stream.end(),
                           [profile](std::pair<stream_profile, std::deque<long int>> &sp_deq)
                           { return profile == sp_deq.first; });

    // if the profile is already in the drops_per_stream container,
    // checking kpi with the new partial frame caught
    if (it != drops_per_stream.end())
    {
        // setting the kpi checking to be done on the last 30 seconds
        int time_limit = 30;

        // max number of drops that can be received in the time_limit, without violation of the kpi
        int max_num_of_drops = profile.fps * _kpi_frames_drops_pct * time_limit;

        auto &queue_for_profile = it->second;
        // removing too old timestamps of partial frames
        while (queue_for_profile.size() > 0)
        {
            auto delta_ts_usec = timestamp_usec - queue_for_profile.front();
            if (delta_ts_usec > (time_limit * 1000000))
            {
                queue_for_profile.pop_front();
            }
            else
                break; // correct because the frames are added chronologically
        }
        // checking kpi violation
        if (queue_for_profile.size() >= max_num_of_drops)
        {
            is_kpi_violated = true;
            queue_for_profile.clear();
        }
        else
            queue_for_profile.push_back(timestamp_usec);
    }
    else
    {
        // adding the the current partial frame's profile and timestamp to the container
        std::deque<long int> deque_to_add;
        deque_to_add.push_back(timestamp_usec);
        drops_per_stream.push_back(std::make_pair(profile, deque_to_add));
    }
    return is_kpi_violated;
}*/

v4l_uvc_device::v4l_uvc_device(const uvc_device_info &info, bool use_memory_map)
    : _name(""), _info(),
      _is_capturing(false),
      _is_alive(true),
      _is_started(false),
      _thread(nullptr),
      //   _named_mtx(nullptr),
      _use_memory_map(use_memory_map),
      _fd(-1),
      _stop_pipe_fd{} //,
                      //_buf_dispatch(use_memory_map) ,
                      //_frame_drop_monitor(DEFAULT_KPI_FRAME_DROPS_PERCENTAGE)
{
  foreach_uvc_device([&info, this](const uvc_device_info &i, const std::string &name) {
                if (i == info)
                {
                    _name = name;
                    _info = i;
                    _device_path = i.device_path;
                    _device_usb_spec = i.conn_spec;
                } });
  if (_name == "") {
    //    throw linux_backend_exception("device is no longer connected!");
  }

  // = std::unique_ptr<named_mutex>(new named_mutex(_name, 5000));
}

v4l_uvc_device::~v4l_uvc_device() {
  _is_capturing = false;
  if (_thread && _thread->joinable())
    _thread->join();
  for (auto &&fd : _fds) {
    try {
      if (fd)
        ::close(fd);
    } catch (...) {
    }
  }
}
#pragma pack(push, 1)
template <class T>
class big_endian {
  T be_value;

public:
  operator T() const {
    T le_value = 0;
    for (unsigned int i = 0; i < sizeof(T); ++i)
      *(reinterpret_cast<char *>(&le_value) + i) = *(reinterpret_cast<const char *>(&be_value) + sizeof(T) - i - 1);
    return le_value;
  }
};
#pragma pack(pop)
void v4l_uvc_device::probe_and_commit(stream_profile profile, frame_callback callback, int buffers) {
  if (!_is_capturing && !_callback) {
    v4l2_fmtdesc pixel_format = {};
    pixel_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    while (ioctl(_fd, VIDIOC_ENUM_FMT, &pixel_format) == 0) {
      v4l2_frmsizeenum frame_size = {};
      frame_size.pixel_format = pixel_format.pixelformat;

      uint32_t fourcc = (const big_endian<int> &)pixel_format.pixelformat;

      if (pixel_format.pixelformat == 0) {
        // Microsoft Depth GUIDs for R400 series are not yet recognized
        // by the Linux kernel, but they do not require a patch, since there
        // are "backup" Z16 and Y8 formats in place
        static const std::set<std::string> pending_formats = {
            "00000050-0000-0010-8000-00aa003",
            "00000032-0000-0010-8000-00aa003",
        };

        if (std::find(pending_formats.begin(),
                      pending_formats.end(),
                      (const char *)pixel_format.description) ==
            pending_formats.end()) {
          const std::string s(to_string() << "!" << pixel_format.description);
          std::regex rgx("!([0-9a-f]+)-.*");
          std::smatch match;

          if (std::regex_search(s.begin(), s.end(), match, rgx)) {
            std::stringstream ss;
            ss << match[1];
            int id;
            ss >> std::hex >> id;
            fourcc = (const big_endian<int> &)id;

            if (fourcc == profile.format) {
              //    throw linux_backend_exception(to_string() << "The requested pixel format '" << fourcc_to_string(id)
              //                                            << "' is not natively supported by the running Linux kernel and likely requires a patch");
            }
          }
        }
      }
      ++pixel_format.index;
    }

    set_format(profile);

    v4l2_streamparm parm = {};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(_fd, VIDIOC_G_PARM, &parm) < 0) {
      //     throw linux_backend_exception("xioctl(VIDIOC_G_PARM) failed");
    }

    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = profile.fps;
    if (xioctl(_fd, VIDIOC_S_PARM, &parm) < 0) {
      //     throw linux_backend_exception("xioctl(VIDIOC_S_PARM) failed");
    }

    // Init memory mapped IO
    negotiate_kernel_buffers(static_cast<size_t>(buffers));
    allocate_io_buffers(static_cast<size_t>(buffers));

    _profile = profile;
    _callback = callback;
  } else {
    //   throw wrong_api_call_sequence_exception("Device already streaming!");
  }
}

void v4l_uvc_device::stream_on() {
  if (!_is_capturing) {
    // _error_handler = error_handler;

    // Start capturing
    prepare_capture_buffers();

    // Synchronise stream requests for meta and video data.
    streamon();

    _is_capturing = true;
    _thread = std::unique_ptr<std::thread>(new std::thread([this]() { capture_loop(); }));
  }
}

void v4l_uvc_device::prepare_capture_buffers() {
  for (auto &&buf : _buffers)
    buf->prepare_for_streaming(_fd);
}

void v4l_uvc_device::stop_data_capture() {
  _is_capturing = false;
  _is_started = false;

  // Stop nn-demand frames polling
  signal_stop();

  _thread->join();
  _thread.reset();

  // Notify kernel
  streamoff();
}

void v4l_uvc_device::start_callbacks() {
  _is_started = true;
}

void v4l_uvc_device::stop_callbacks() {
  _is_started = false;
}

void v4l_uvc_device::close(stream_profile) {
  if (_is_capturing) {
    stop_data_capture();
  }

  if (_callback) {
    // Release allocated buffers
    allocate_io_buffers(0);

    // Release IO
    negotiate_kernel_buffers(0);

    _callback = nullptr;
  }
}
void copy(void *dst, void const *src, size_t size) {
  auto from = reinterpret_cast<uint8_t const *>(src);
  std::copy(from, from + size, reinterpret_cast<uint8_t *>(dst));
}
std::string v4l_uvc_device::fourcc_to_string(uint32_t id) const {
  uint32_t device_fourcc = id;
  char fourcc_buff[sizeof(device_fourcc) + 1];
  copy(fourcc_buff, &device_fourcc, sizeof(device_fourcc));
  fourcc_buff[sizeof(device_fourcc)] = 0;
  return fourcc_buff;
}

void v4l_uvc_device::signal_stop() {
  char buff[1] = {};
  if (write(_stop_pipe_fd[1], buff, 1) < 0) {
    //    throw linux_backend_exception("Could not signal video capture thread to stop. Error write to pipe.");
  }
}

std::string time_in_HH_MM_SS_MMM() {
  using namespace std::chrono;

  // get current time
  auto now = system_clock::now();

  // get number of milliseconds for the current second
  // (remainder after division into seconds)
  auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

  // convert to std::time_t in order to convert to std::tm (broken time)
  auto timer = system_clock::to_time_t(now);

  // convert to broken time
  std::tm bt = *std::localtime(&timer);

  std::ostringstream oss;

  oss << std::put_time(&bt, "%H:%M:%S"); // HH:MM:SS
  oss << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}
double monotonic_to_realtime(double monotonic) {
  auto realtime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  auto time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  return monotonic + (realtime - time_since_epoch);
}
void v4l_uvc_device::poll() {
  fd_set fds{};
  FD_ZERO(&fds);
  //  printf("_fds size:%d\n",_fds.size());
  for (auto fd : _fds) {
    FD_SET(fd, &fds);
  }

  struct timespec mono_time;
  int ret = clock_gettime(CLOCK_MONOTONIC, &mono_time);
  if (ret) {
    printf("could not query time!\n");
  }

  struct timeval expiration_time = {mono_time.tv_sec + 5, mono_time.tv_nsec / 1000};
  int val = 0;

  auto realtime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  auto time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  // LOG_DEBUG("Select initiated at " << time_in_HH_MM_SS_MMM() << ", mono time " << time_since_epoch << ", host time " << realtime);
  do {
    struct timeval remaining;
    ret = clock_gettime(CLOCK_MONOTONIC, &mono_time);
    if (ret) {
      printf("could not query time!\n");
    }

    struct timeval current_time = {mono_time.tv_sec, mono_time.tv_nsec / 1000};
    timersub(&expiration_time, &current_time, &remaining);
    if (timercmp(&current_time, &expiration_time, <)) {
      time_t now = time(NULL);
      //  printf("now:%d,will select\n",now);
      val = select(_max_fd + 1, &fds, nullptr, nullptr, &remaining);

      //  printf("select last:%d seconds,return:%d!\n",time(NULL)-now,val);
    } else {
      val = 0;
      //   printf("Select timeouted\n");
    }

    if (val < 0) {
      printf("Select interrupted, val = %d ,error = %d", val, errno);
    }
  } while (val < 0 && errno == EINTR);

  // LOG_DEBUG("Select done, val = " << val << " at " << time_in_HH_MM_SS_MMM());
  if (val < 0) {
    _is_capturing = false;
    _is_started = false;

    printf("streamoff..\n");
    // Notify kernel
    streamoff();
  } else {
    if (val > 0) {
      // printf("val>0\n");
      if (FD_ISSET(_stop_pipe_fd[0], &fds) || FD_ISSET(_stop_pipe_fd[1], &fds)) {
        if (!_is_capturing) {
          printf("V4L stream is closed\n");
          return;
        } else {
          printf("Stop pipe was signalled during streaming\n");
          return;
        }
      } else // Check and acquire data buffers from kernel
      {
        // printf("Check and acquire data buffers from kernel\n");
        bool md_extracted = false;
        bool keep_md = false;
        bool wa_applied = false;
        /*  buffers_mgr buf_mgr(_use_memory_map);
          if (_buf_dispatch.metadata_size())
          {
              printf("metadata size not zero!\n");
              buf_mgr = _buf_dispatch; // Handle over MD buffer from the previous cycle
              md_extracted = true;
              wa_applied = true;
              _buf_dispatch.set_md_attributes(0, nullptr);
          }
          // RAII to handle exceptions
          std::unique_ptr<int, std::function<void(int *)>> md_poller(new int(0),
                                                                     [this, &buf_mgr, &md_extracted, &keep_md, &fds](int *d)
                                                                     {
                                                                         if (!md_extracted)
                                                                         {
                                                                              printf("MD Poller read md \n");
                                                                             acquire_metadata(buf_mgr, fds);
                                                                             if (buf_mgr.metadata_size())
                                                                             {
                                                                                 if (keep_md) // store internally for next poll cycle
                                                                                 {
                                                                                     auto fn = *(uint32_t *)((char *)(buf_mgr.metadata_start()) + 28);
                                                                                     auto mdb = buf_mgr.get_buffers().at(e_metadata_buf);

                                                                                     _buf_dispatch = buf_mgr;                   // TODO keep metadata only as dispatch may hold video buf from previous cycle
                                                                                     buf_mgr.handle_buffer(e_metadata_buf, -1); // transfer new buffer request to next cycle
                                                                                 }
                                                                                 else // Discard collected metadata buffer
                                                                                 {
                                                                                     printf("Discard md buffer\n");
                                                                                     auto md_buf = buf_mgr.get_buffers().at(e_metadata_buf);
                                                                                     if (md_buf._data_buf)
                                                                                         md_buf._data_buf->request_next_frame(md_buf._file_desc, true);
                                                                                 }
                                                                             }
                                                                         }
                                                                         delete d;
                                                                     });*/

        if (FD_ISSET(_fd, &fds)) {
          FD_CLR(_fd, &fds);
          v4l2_buffer buf = {};
          buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
          buf.memory = _use_memory_map ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;
          if (xioctl(_fd, VIDIOC_DQBUF, &buf) < 0) {

            //  printf("xioctl..\n");
            // printf("Dequeued empty buf for fd " << std::dec << _fd);
          }

          auto timestamp = (double)buf.timestamp.tv_sec * 1000.f + (double)buf.timestamp.tv_usec / 1000.f;
          timestamp = monotonic_to_realtime(timestamp);

          // Read metadata. Metadata node performs a blocking call to ensure video and metadata sync
          /*acquire_metadata(buf_mgr, fds, compressed_format);
          md_extracted = true;

          if (wa_applied)
          {
              auto fn = *(uint32_t *)((char *)(buf_mgr.metadata_start()) + 28);
              printf("Extracting md buff, fn = %u\n" , fn);
          }*/

          auto frame_sz = buf.bytesused; // buf_mgr.md_node_present() ? buf.bytesused : std::min(buf.bytesused - buf_mgr.metadata_size(), buffer->get_length_frame_only());
          frame_object fo{frame_sz, 0,   // buf_mgr.metadata_size(),
                          (const void *)buf.m.userptr, NULL, timestamp};
          _callback(_profile, fo, []() mutable { /*buf_mgr.request_next_frame();*/ });
          //  printf("Dequeued buf.index:%u seq:%u,byte used:%u\n",buf.index , buf.sequence,buf.bytesused);
          xioctl(_fd, VIDIOC_QBUF, &buf);
          /*     auto buffer = _buffers[buf.index];
               buf_mgr.handle_buffer(e_video_buf, _fd, buf, buffer);

               if (_is_started)
               {
                   if (buf.bytesused == 0)
                   {
                       printf("Empty video frame arrived, index:%u\n" , buf.index);
                       return;
                   }

                   // Relax the required frame size for compressed formats, i.e. MJPG, Z16H
                   // Drop partial and overflow frames (assumes D4XX metadata only)
                   bool compressed_format = val_in_range(_profile.format, {0x4d4a5047U, 0x5a313648U});
                   bool partial_frame = (!compressed_format && (buf.bytesused < buffer->get_full_length() - MAX_META_DATA_SIZE));
                   bool overflow_frame = (buf.bytesused == buffer->get_length_frame_only() + MAX_META_DATA_SIZE);
                   if (partial_frame || overflow_frame)
                   {
                       auto percentage = (100 * buf.bytesused) / buffer->get_full_length();
                       std::stringstream s;
                       if (partial_frame)
                       {
                           s << "Incomplete video frame detected!\nSize " << buf.bytesused
                             << " out of " << buffer->get_full_length() << " bytes (" << percentage << "%)";
                           if (overflow_frame)
                           {
                               s << ". Overflow detected: payload size " << buffer->get_length_frame_only();
                               printf("Corrupted UVC frame data, underflow and overflow reported:%s\n",s.str().c_str());
                           }
                       }
                       else
                       {
                           if (overflow_frame)
                               s << "overflow video frame detected!\nSize " << buf.bytesused
                                 << ", payload size " << buffer->get_length_frame_only();
                       }
                       printf("Incomplete frame received: %s\n" ,s.str().c_str()); // Ev -try1
                       //    bool kpi_violated = _frame_drop_monitor.update_and_check_kpi(_profile, buf.timestamp);


                       // Check if metadata was already allocated
                       if (buf_mgr.metadata_size())
                       {
                           printf("Metadata was present when partial frame arrived, mark md as extracted\n");
                           md_extracted = true;
                           printf("Discarding md due to invalid video payload\n");
                           auto md_buf = buf_mgr.get_buffers().at(e_metadata_buf);
                           md_buf._data_buf->request_next_frame(md_buf._file_desc, true);
                       }
                   }
                   else
                   {
                       auto timestamp = (double)buf.timestamp.tv_sec * 1000.f + (double)buf.timestamp.tv_usec / 1000.f;
                       timestamp = monotonic_to_realtime(timestamp);

                       // Read metadata. Metadata node performs a blocking call to ensure video and metadata sync
                       acquire_metadata(buf_mgr, fds, compressed_format);
                       md_extracted = true;

                       if (wa_applied)
                       {
                           auto fn = *(uint32_t *)((char *)(buf_mgr.metadata_start()) + 28);
                           printf("Extracting md buff, fn = %u\n" , fn);
                       }

                       auto frame_sz = buf_mgr.md_node_present() ? buf.bytesused : std::min(buf.bytesused - buf_mgr.metadata_size(), buffer->get_length_frame_only());
                       frame_object fo{frame_sz, buf_mgr.metadata_size(),
                                       buffer->get_frame_start(), buf_mgr.metadata_start(), timestamp};

                       buffer->attach_buffer(buf);
                       buf_mgr.handle_buffer(e_video_buf, -1); // transfer new buffer request to the frame callback

                       if (buf_mgr.verify_vd_md_sync())
                       {
                           time_t start=time(NULL);
                           // Invoke user callback and enqueue next frame
                           _callback(_profile, fo, [buf_mgr]() mutable
                                     { buf_mgr.request_next_frame(); });
                           printf("now:%d,_callback..last:%d\n",time(NULL),time(NULL)-start);
                       }
                       else
                       {
                           printf("Video frame dropped, video and metadata buffers inconsistency\n");
                       }
                   }
               }
               else
               {
                   printf("Video frame arrived in idle mode.\n"); // TODO - verification
               }
               */
        } else {
          //    if (_is_started)
          //      keep_md = true;
          printf("FD_ISSET: no data on video node sink\n");
        }
      }
    } else // (val==0)
    {
      printf("Frames didn't arrived within 5 seconds\n");
      /* librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_FRAMES_TIMEOUT, 0, RS2_LOG_SEVERITY_WARN, "Frames didn't arrived within 5 seconds"};

       _error_handler(n);*/
    }
  }
}
/*
void v4l_uvc_device::acquire_metadata(buffers_mgr &buf_mgr, fd_set &, bool compressed_format)
{
    if (has_metadata())
        buf_mgr.set_md_from_video_node(compressed_format);
    else
        buf_mgr.set_md_attributes(0, nullptr);
}*/

void v4l_uvc_device::set_power_state(power_state state) {
  if (state == D0 && _state == D3) {
    map_device_descriptor();
  }
  if (state == D3 && _state == D0) {
    close(_profile);
    unmap_device_descriptor();
  }
  _state = state;
}

bool v4l_uvc_device::set_xu(const extension_unit &xu, uint8_t control, const uint8_t *data, int size) {
  uvc_xu_control_query q = {static_cast<uint8_t>(xu.unit), control, UVC_SET_CUR,
                            static_cast<uint16_t>(size), const_cast<uint8_t *>(data)};
  if (xioctl(_fd, UVCIOC_CTRL_QUERY, &q) < 0) {
    if (errno == EIO || errno == EAGAIN) // TODO: Log?
      return false;

    // throw linux_backend_exception("set_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed");
  }

  return true;
}
bool v4l_uvc_device::get_xu(const extension_unit &xu, uint8_t control, uint8_t *data, int size) const {
  memset(data, 0, size);
  uvc_xu_control_query q = {static_cast<uint8_t>(xu.unit), control, UVC_GET_CUR,
                            static_cast<uint16_t>(size), const_cast<uint8_t *>(data)};
  if (xioctl(_fd, UVCIOC_CTRL_QUERY, &q) < 0) {
    if (errno == EIO || errno == EAGAIN || errno == EBUSY) // TODO: Log?
      return false;

    //      throw linux_backend_exception("get_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed");
  }

  return true;
}
void control_range::populate_raw_data(std::vector<uint8_t> &vec, int32_t value) {
  vec.resize(sizeof(value));
  auto data = reinterpret_cast<const uint8_t *>(&value);
  std::copy(data, data + sizeof(value), vec.data());
}
control_range v4l_uvc_device::get_xu_range(const extension_unit &xu, uint8_t control, int len) const {
  control_range result{};
  __u16 size = 0;
  //__u32 value = 0;      // all of the real sense extended controls are up to 4 bytes
  // checking return value for UVC_GET_LEN and allocating
  // appropriately might be better
  //__u8 * data = (__u8 *)&value;
  // MS XU controls are partially supported only
  struct uvc_xu_control_query xquery = {};
  memset(&xquery, 0, sizeof(xquery));
  xquery.query = UVC_GET_LEN;
  xquery.size = 2; // size seems to always be 2 for the LEN query, but
                   // doesn't seem to be documented. Use result for size
                   // in all future queries of the same control number
  xquery.selector = control;
  xquery.unit = xu.unit;
  xquery.data = (__u8 *)&size;

  if (-1 == ioctl(_fd, UVCIOC_CTRL_QUERY, &xquery)) {
    //  throw linux_backend_exception("xioctl(UVC_GET_LEN) failed");
  }

  assert(size <= len);

  std::vector<uint8_t> buf;
  auto buf_size = std::max((size_t)len, sizeof(__u32));
  buf.resize(buf_size);

  xquery.query = UVC_GET_MIN;
  xquery.size = size;
  xquery.selector = control;
  xquery.unit = xu.unit;
  xquery.data = buf.data();
  if (-1 == ioctl(_fd, UVCIOC_CTRL_QUERY, &xquery)) {
    //  throw linux_backend_exception("xioctl(UVC_GET_MIN) failed");
  }
  result.min.resize(buf_size);
  std::copy(buf.begin(), buf.end(), result.min.begin());

  xquery.query = UVC_GET_MAX;
  xquery.size = size;
  xquery.selector = control;
  xquery.unit = xu.unit;
  xquery.data = buf.data();
  if (-1 == ioctl(_fd, UVCIOC_CTRL_QUERY, &xquery)) {
    //  throw linux_backend_exception("xioctl(UVC_GET_MAX) failed");
  }
  result.max.resize(buf_size);
  std::copy(buf.begin(), buf.end(), result.max.begin());

  xquery.query = UVC_GET_DEF;
  xquery.size = size;
  xquery.selector = control;
  xquery.unit = xu.unit;
  xquery.data = buf.data();
  if (-1 == ioctl(_fd, UVCIOC_CTRL_QUERY, &xquery)) {
    //   throw linux_backend_exception("xioctl(UVC_GET_DEF) failed");
  }
  result.def.resize(buf_size);
  std::copy(buf.begin(), buf.end(), result.def.begin());

  xquery.query = UVC_GET_RES;
  xquery.size = size;
  xquery.selector = control;
  xquery.unit = xu.unit;
  xquery.data = buf.data();
  if (-1 == ioctl(_fd, UVCIOC_CTRL_QUERY, &xquery)) {
    //  throw linux_backend_exception("xioctl(UVC_GET_CUR) failed");
  }
  result.step.resize(buf_size);
  std::copy(buf.begin(), buf.end(), result.step.begin());

  return result;
}

bool v4l_uvc_device::get_pu(rs2_option opt, int32_t &value) const {
  struct v4l2_control control = {get_cid(opt), 0};
  if (xioctl(_fd, VIDIOC_G_CTRL, &control) < 0) {
    if (errno == EIO || errno == EAGAIN) // TODO: Log?
      return false;

    //   throw linux_backend_exception("xioctl(VIDIOC_G_CTRL) failed");
  }

  if (RS2_OPTION_ENABLE_AUTO_EXPOSURE == opt) {
    control.value = (V4L2_EXPOSURE_MANUAL == control.value) ? 0 : 1;
  }
  value = control.value;

  return true;
}

bool v4l_uvc_device::set_pu(rs2_option opt, int32_t value) {
  struct v4l2_control control = {get_cid(opt), value};
  if (RS2_OPTION_ENABLE_AUTO_EXPOSURE == opt) {
    control.value = value ? V4L2_EXPOSURE_APERTURE_PRIORITY : V4L2_EXPOSURE_MANUAL;
  }

  // We chose not to protect the subscribe / unsubscribe with mutex due to performance reasons,
  // we prefer returning on timeout (and let the retry mechanism try again if exist) than blocking the main thread on every set command

  // RAII to handle unsubscribe in case of exceptions
  std::unique_ptr<uint32_t, std::function<void(uint32_t *)>> unsubscriber(
      new uint32_t(control.id),
      [this](uint32_t *id) {
        if (id) {
          // `unsubscribe_from_ctrl_event()` may throw so we first release the memory allocated and than call it.
          auto local_id = *id;
          delete id;
          unsubscribe_from_ctrl_event(local_id);
        }
      });

  subscribe_to_ctrl_event(control.id);

  // Set value
  if (xioctl(_fd, VIDIOC_S_CTRL, &control) < 0) {
    if (errno == EIO || errno == EAGAIN) // TODO: Log?
      return false;

    //    throw linux_backend_exception("xioctl(VIDIOC_S_CTRL) failed");
  }

  if (!pend_for_ctrl_status_event())
    return false;

  return true;
}

control_range v4l_uvc_device::get_pu_range(rs2_option option) const {
  // Auto controls range is trimed to {0,1} range
  if (option >= RS2_OPTION_ENABLE_AUTO_EXPOSURE && option <= RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) {
    static const int32_t min = 0, max = 1, step = 1, def = 1;
    control_range range(min, max, step, def);

    return range;
  }

  struct v4l2_queryctrl query = {};
  query.id = get_cid(option);
  if (xioctl(_fd, VIDIOC_QUERYCTRL, &query) < 0) {
    // Some controls (exposure, auto exposure, auto hue) do not seem to work on V4L2
    // Instead of throwing an error, return an empty range. This will cause this control to be omitted on our UI sample.
    // TODO: Figure out what can be done about these options and make this work
    query.minimum = query.maximum = 0;
  }

  control_range range(query.minimum, query.maximum, query.step, query.default_value);

  return range;
}

std::vector<stream_profile> v4l_uvc_device::get_profiles() const {
  std::vector<stream_profile> results;

  // Retrieve the caps one by one, first get pixel format, then sizes, then
  // frame rates. See http://linuxtv.org/downloads/v4l-dvb-apis for reference.
  v4l2_fmtdesc pixel_format = {};
  pixel_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  while (ioctl(_fd, VIDIOC_ENUM_FMT, &pixel_format) == 0) {
    v4l2_frmsizeenum frame_size = {};
    frame_size.pixel_format = pixel_format.pixelformat;

    uint32_t fourcc = (const big_endian<int> &)pixel_format.pixelformat;

    if (pixel_format.pixelformat == 0) {
      // Microsoft Depth GUIDs for R400 series are not yet recognized
      // by the Linux kernel, but they do not require a patch, since there
      // are "backup" Z16 and Y8 formats in place
      std::set<std::string> known_problematic_formats = {
          "00000050-0000-0010-8000-00aa003",
          "00000032-0000-0010-8000-00aa003",
      };

      if (std::find(known_problematic_formats.begin(),
                    known_problematic_formats.end(),
                    (const char *)pixel_format.description) ==
          known_problematic_formats.end()) {
        const std::string s(to_string() << "!" << pixel_format.description);
        std::regex rgx("!([0-9a-f]+)-.*");
        std::smatch match;

        if (std::regex_search(s.begin(), s.end(), match, rgx)) {
          std::stringstream ss;
          ss << match[1];
          int id;
          ss >> std::hex >> id;
          fourcc = (const big_endian<int> &)id;

          auto format_str = fourcc_to_string(id);
          //  LOG_WARNING("Pixel format " << pixel_format.description << " likely requires patch for fourcc code " << format_str << "!");
        }
      }
    } else {
      // LOG_DEBUG("Recognized pixel-format " << pixel_format.description);
    }

    while (ioctl(_fd, VIDIOC_ENUM_FRAMESIZES, &frame_size) == 0) {
      v4l2_frmivalenum frame_interval = {};
      frame_interval.pixel_format = pixel_format.pixelformat;
      frame_interval.width = frame_size.discrete.width;
      frame_interval.height = frame_size.discrete.height;
      while (ioctl(_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frame_interval) == 0) {
        if (frame_interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
          if (frame_interval.discrete.numerator != 0) {
            auto fps =
                static_cast<float>(frame_interval.discrete.denominator) /
                static_cast<float>(frame_interval.discrete.numerator);

            stream_profile p{};
            p.format = fourcc;
            p.width = frame_size.discrete.width;
            p.height = frame_size.discrete.height;
            p.fps = fps;
            if (fourcc != 0)
              results.push_back(p);
          }
        }

        ++frame_interval.index;
      }

      ++frame_size.index;
    }

    ++pixel_format.index;
  }
  return results;
}

void v4l_uvc_device::lock() const {
  //   _named_mtx->lock();
}
void v4l_uvc_device::unlock() const {
  //   _named_mtx->unlock();
}

uint32_t v4l_uvc_device::get_cid(rs2_option option) {
  switch (option) {
  case RS2_OPTION_BACKLIGHT_COMPENSATION:
    return V4L2_CID_BACKLIGHT_COMPENSATION;
  case RS2_OPTION_BRIGHTNESS:
    return V4L2_CID_BRIGHTNESS;
  case RS2_OPTION_CONTRAST:
    return V4L2_CID_CONTRAST;
  case RS2_OPTION_EXPOSURE:
    return V4L2_CID_EXPOSURE_ABSOLUTE; // Is this actually valid? I'm getting a lot of VIDIOC error 22s...
  case RS2_OPTION_GAIN:
    return V4L2_CID_GAIN;
  case RS2_OPTION_GAMMA:
    return V4L2_CID_GAMMA;
  case RS2_OPTION_HUE:
    return V4L2_CID_HUE;
  case RS2_OPTION_SATURATION:
    return V4L2_CID_SATURATION;
  case RS2_OPTION_SHARPNESS:
    return V4L2_CID_SHARPNESS;
  case RS2_OPTION_WHITE_BALANCE:
    return V4L2_CID_WHITE_BALANCE_TEMPERATURE;
  case RS2_OPTION_ENABLE_AUTO_EXPOSURE:
    return V4L2_CID_EXPOSURE_AUTO; // Automatic gain/exposure control
  case RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE:
    return V4L2_CID_AUTO_WHITE_BALANCE;
  case RS2_OPTION_POWER_LINE_FREQUENCY:
    return V4L2_CID_POWER_LINE_FREQUENCY;
  case RS2_OPTION_AUTO_EXPOSURE_PRIORITY:
    return V4L2_CID_EXPOSURE_AUTO_PRIORITY;
  default:; //  throw linux_backend_exception(to_string() << "no v4l2 cid for option " << option);
  }
}

void v4l_uvc_device::capture_loop() {
  try {
    while (_is_capturing) {
      poll();
    }
  } catch (const std::exception &ex) {
    // LOG_ERROR((ex.what());

    /*    librealsense::notification n = {RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR, 0, RS2_LOG_SEVERITY_ERROR, ex.what()};

        _error_handler(n);*/
  }
  printf("capture_loop exit!\n");
}

bool v4l_uvc_device::has_metadata() const {
  return !_use_memory_map;
}

void v4l_uvc_device::streamon() const {
  stream_ctl_on(_fd);
}

void v4l_uvc_device::streamoff() const {
  stream_off(_fd);
}

void v4l_uvc_device::negotiate_kernel_buffers(size_t num) const {
  req_io_buff(_fd, num, _name,
              _use_memory_map ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR,
              V4L2_BUF_TYPE_VIDEO_CAPTURE);
}

void v4l_uvc_device::allocate_io_buffers(size_t buffers) {
  if (buffers) {
    for (size_t i = 0; i < buffers; ++i) {
      _buffers.push_back(std::make_shared<buffer>(_fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, _use_memory_map, i));
    }
  } else {
    for (size_t i = 0; i < _buffers.size(); i++) {
      _buffers[i]->detach_buffer();
    }
    _buffers.resize(0);
  }
}

void v4l_uvc_device::map_device_descriptor() {
  printf("will open:%s\n", _name.c_str());
  _fd = open(_name.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (_fd < 0) {
    printf("can not open!\n");
    //   throw linux_backend_exception(to_string() << __FUNCTION__ << " Cannot open '" << _name);
  }

  if (pipe(_stop_pipe_fd) < 0) {
    printf("can not create pipe!\n");
    //  throw linux_backend_exception(to_string() << __FUNCTION__ << " Cannot create pipe!");
  }

  if (_fds.size()) {
    printf("_fds is already allocated!\n");
    //    throw linux_backend_exception(to_string() << __FUNCTION__ << " Device descriptor is already allocated");
  }
  printf("will insert _fd _stop_pipd_fd[0] _stop_pipe_fd[1]\n");
  _fds.insert(_fds.end(), {_fd, _stop_pipe_fd[0], _stop_pipe_fd[1]});
  _max_fd = *std::max_element(_fds.begin(), _fds.end());

  v4l2_capability cap = {};
  if (xioctl(_fd, VIDIOC_QUERYCAP, &cap) < 0) {
    if (errno == EINVAL) {
      //    throw linux_backend_exception(_name + " is not V4L2 device");
    } else {
      //   throw linux_backend_exception("xioctl(VIDIOC_QUERYCAP) failed");
    }
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {

    //    throw linux_backend_exception(_name + " is no video capture device");
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    //    throw linux_backend_exception(_name + " does not support streaming I/O");
  }

  // Select video input, video standard and tune here.
  v4l2_cropcap cropcap = {};
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(_fd, VIDIOC_CROPCAP, &cropcap) == 0) {
    v4l2_crop crop = {};
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; // reset to default
    if (xioctl(_fd, VIDIOC_S_CROP, &crop) < 0) {
      switch (errno) {
      case EINVAL:
        break; // Cropping not supported
      default:
        break; // Errors ignored
      }
    }
  } else {
  } // Errors ignored
}

void v4l_uvc_device::unmap_device_descriptor() {
  if (::close(_fd) < 0) {
    //   throw linux_backend_exception("v4l_uvc_device: close(_fd) failed");
  }

  if (::close(_stop_pipe_fd[0]) < 0) {
    //  throw linux_backend_exception("v4l_uvc_device: close(_stop_pipe_fd[0]) failed");
  }
  if (::close(_stop_pipe_fd[1]) < 0) {
    //  throw linux_backend_exception("v4l_uvc_device: close(_stop_pipe_fd[1]) failed");
  }

  _fd = 0;
  _stop_pipe_fd[0] = _stop_pipe_fd[1] = 0;
  _fds.clear();
}

void v4l_uvc_device::set_format(stream_profile profile) {
  v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = profile.width;
  fmt.fmt.pix.height = profile.height;
  fmt.fmt.pix.pixelformat = (const big_endian<int> &)profile.format;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;
  if (xioctl(_fd, VIDIOC_S_FMT, &fmt) < 0) {
    //    throw linux_backend_exception("xioctl(VIDIOC_S_FMT) failed");
  } else {

    // LOG_INFO(("Video node was successfully configured to " << fourcc_to_string(fmt.fmt.pix.pixelformat) << " format"
    //                                            << ", fd " << std::dec << _fd);
  }

  // LOG_INFO(("Trying to configure fourcc " << fourcc_to_string(fmt.fmt.pix.pixelformat));
}

void v4l_uvc_device::subscribe_to_ctrl_event(uint32_t control_id) {
  struct v4l2_event_subscription event_subscription;
  event_subscription.flags = V4L2_EVENT_SUB_FL_ALLOW_FEEDBACK;
  event_subscription.type = V4L2_EVENT_CTRL;
  event_subscription.id = control_id;
  memset(event_subscription.reserved, 0, sizeof(event_subscription.reserved));
  if (xioctl(_fd, VIDIOC_SUBSCRIBE_EVENT, &event_subscription) < 0) {
    //   throw linux_backend_exception(to_string() << "xioctl(VIDIOC_SUBSCRIBE_EVENT) with control_id = " << control_id << " failed");
  }
}

void v4l_uvc_device::unsubscribe_from_ctrl_event(uint32_t control_id) {
  struct v4l2_event_subscription event_subscription;
  event_subscription.flags = V4L2_EVENT_SUB_FL_ALLOW_FEEDBACK;
  event_subscription.type = V4L2_EVENT_CTRL;
  event_subscription.id = control_id;
  memset(event_subscription.reserved, 0, sizeof(event_subscription.reserved));
  if (xioctl(_fd, VIDIOC_UNSUBSCRIBE_EVENT, &event_subscription) < 0) {
    //    throw linux_backend_exception(to_string() << "xioctl(VIDIOC_UNSUBSCRIBE_EVENT) with control_id = " << control_id << " failed");
  }
}

bool v4l_uvc_device::pend_for_ctrl_status_event() {
  struct v4l2_event event;
  memset(&event, 0, sizeof(event));

  // Poll registered events and verify that set control event raised (wait max of 10 * 2 = 20 [ms])
  static int MAX_POLL_RETRIES = 10;
  for (int i = 0; i < MAX_POLL_RETRIES && event.type != V4L2_EVENT_CTRL; i++) {
    if (xioctl(_fd, VIDIOC_DQEVENT, &event) < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }

  return event.type == V4L2_EVENT_CTRL;
}
/*
v4l_uvc_meta_device::v4l_uvc_meta_device(const uvc_device_info &info, bool use_memory_map) : v4l_uvc_device(info, use_memory_map),
                                                                                             _md_fd(0),
                                                                                             _md_name(info.metadata_node_id)
{
}

v4l_uvc_meta_device::~v4l_uvc_meta_device()
{
}

void v4l_uvc_meta_device::streamon() const
{
    // Metadata stream shall be configured first to allow sync with video node
    stream_ctl_on(_md_fd, LOCAL_V4L2_BUF_TYPE_META_CAPTURE);

    // Invoke UVC streaming request
    v4l_uvc_device::streamon();
}

void v4l_uvc_meta_device::streamoff() const
{
    v4l_uvc_device::streamoff();

    stream_off(_md_fd, LOCAL_V4L2_BUF_TYPE_META_CAPTURE);
}

void v4l_uvc_meta_device::negotiate_kernel_buffers(size_t num) const
{
    v4l_uvc_device::negotiate_kernel_buffers(num);

    req_io_buff(_md_fd, num, _name,
                _use_memory_map ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR,
                LOCAL_V4L2_BUF_TYPE_META_CAPTURE);
}

void v4l_uvc_meta_device::allocate_io_buffers(size_t buffers)
{
    v4l_uvc_device::allocate_io_buffers(buffers);

    if (buffers)
    {
        for (size_t i = 0; i < buffers; ++i)
        {
            _md_buffers.push_back(std::make_shared<buffer>(_md_fd, LOCAL_V4L2_BUF_TYPE_META_CAPTURE, _use_memory_map, i));
        }
    }
    else
    {
        for (size_t i = 0; i < _buffers.size(); i++)
        {
            _md_buffers[i]->detach_buffer();
        }
        _md_buffers.resize(0);
    }
}

void v4l_uvc_meta_device::map_device_descriptor()
{
    v4l_uvc_device::map_device_descriptor();

    if (_md_fd > 0)
        throw linux_backend_exception(to_string() << _md_name << " descriptor is already opened");

    _md_fd = open(_md_name.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (_md_fd < 0)
        throw linux_backend_exception(to_string() << "Cannot open '" << _md_name);

    // The minimal video/metadata nodes syncer will be implemented by using two blocking calls:
    //  1. Obtain video node data.
    //  2. Obtain metadata
    //      To revert to multiplexing mode uncomment the next line
    _fds.push_back(_md_fd);
    _max_fd = *std::max_element(_fds.begin(), _fds.end());

    v4l2_capability cap = {};
    if (xioctl(_md_fd, VIDIOC_QUERYCAP, &cap) < 0)
    {
        if (errno == EINVAL)
            throw linux_backend_exception(_md_name + " is no V4L2 device");
        else
            throw linux_backend_exception(_md_name + " xioctl(VIDIOC_QUERYCAP) for metadata failed");
    }

    if (!(cap.capabilities & V4L2_CAP_META_CAPTURE))
        throw linux_backend_exception(_md_name + " is not metadata capture device");

    if (!(cap.capabilities & V4L2_CAP_STREAMING))
        throw linux_backend_exception(_md_name + " does not support metadata streaming I/O");
}

void v4l_uvc_meta_device::unmap_device_descriptor()
{
    v4l_uvc_device::unmap_device_descriptor();

    if (::close(_md_fd) < 0)
        throw linux_backend_exception("v4l_uvc_meta_device: close(_md_fd) failed");

    _md_fd = 0;
}

void v4l_uvc_meta_device::set_format(stream_profile profile)
{
    // Select video node streaming format
    v4l_uvc_device::set_format(profile);

    // Configure metadata node stream format
    v4l2_format fmt{};
    fmt.type = LOCAL_V4L2_BUF_TYPE_META_CAPTURE;

    if (xioctl(_md_fd, VIDIOC_G_FMT, &fmt))
        throw linux_backend_exception(_md_name + " ioctl(VIDIOC_G_FMT) for metadata node failed");

    if (fmt.type != LOCAL_V4L2_BUF_TYPE_META_CAPTURE)
        throw linux_backend_exception("ioctl(VIDIOC_G_FMT): " + _md_name + " node is not metadata capture");

    bool success = false;
    for (const uint32_t &request : {V4L2_META_FMT_D4XX, V4L2_META_FMT_UVC})
    {
        // Configure metadata format - try d4xx, then fallback to currently retrieve UVC default header of 12 bytes
        memcpy(fmt.fmt.raw_data, &request, sizeof(request));

        if (xioctl(_md_fd, VIDIOC_S_FMT, &fmt) >= 0)
        {
            //LOG_INFO(("Metadata node was successfully configured to " << fourcc_to_string(request) << " format"
                                                                     << ", fd " << std::dec << _md_fd);
            success = true;
            break;
        }
        else
        {
            LOG_WARNING("Metadata node configuration failed for " << fourcc_to_string(request));
        }
    }

    if (!success)
        throw linux_backend_exception(_md_name + " ioctl(VIDIOC_S_FMT) for metadata node failed");
}

void v4l_uvc_meta_device::prepare_capture_buffers()
{
    // Meta node to be initialized first to enforce initial sync
    for (auto &&buf : _md_buffers)
        buf->prepare_for_streaming(_md_fd);

    // Request streaming for video node
    v4l_uvc_device::prepare_capture_buffers();
}

// Retrieve metadata from a dedicated UVC node. For kernels 4.16+
void v4l_uvc_meta_device::acquire_metadata(buffers_mgr &buf_mgr, fd_set &fds, bool)
{
    // Use non-blocking metadata node polling
    if (FD_ISSET(_md_fd, &fds))
    {
        // In scenario if [md+vid] ->[md] ->[md,vid] the third md should not be retrieved but wait for next select
        if (buf_mgr.metadata_size())
        {
            LOG_WARNING("Metadata override requested but avoided skipped");
            return;
        }
        FD_CLR(_md_fd, &fds);

        v4l2_buffer buf{};
        buf.type = LOCAL_V4L2_BUF_TYPE_META_CAPTURE;
        buf.memory = _use_memory_map ? V4L2_MEMORY_MMAP : V4L2_MEMORY_USERPTR;

        // W/O multiplexing this will create a blocking call for metadata node
        if (xioctl(_md_fd, VIDIOC_DQBUF, &buf) < 0)
        {
            //LOG_DEBUG("Dequeued empty buf for md fd " << std::dec << _md_fd);
        }

        // V4l debugging message
        auto mdbuf = _md_buffers[buf.index]->get_frame_start();
        auto hwts = *(uint32_t *)((mdbuf + 2));
        auto fn = *(uint32_t *)((mdbuf + 38));
        //LOG_DEBUG("Dequeued md buf " << std::dec << buf.index << " for fd " << _md_fd << " seq " << buf.sequence
                                         << " fn " << fn << " hw ts " << hwts
                                         << " v4lbuf ts usec " << buf.timestamp.tv_usec);

        auto buffer = _md_buffers[buf.index];
        buf_mgr.handle_buffer(e_metadata_buf, _md_fd, buf, buffer);

        if (!_is_started)
            //LOG_INFO(("Metadata frame arrived in idle mode.");

        static const size_t uvc_md_start_offset = sizeof(uvc_meta_buffer::ns) + sizeof(uvc_meta_buffer::sof);

        if (buf.bytesused > uvc_md_start_offset)
        {
            // The first uvc_md_start_offset bytes of metadata buffer are generated by host driver
            buf_mgr.set_md_attributes(buf.bytesused - uvc_md_start_offset,
                                      buffer->get_frame_start() + uvc_md_start_offset);

            buffer->attach_buffer(buf);
            buf_mgr.handle_buffer(e_metadata_buf, -1); // transfer new buffer request to the frame callback
        }
        else
        {
            //LOG_DEBUG("Invalid md size: bytes used =  " << buf.bytesused << " ,start offset=" << uvc_md_start_offset);
            // Zero-size buffers generate empty md. Non-zero partial bufs handled as errors
            if (buf.bytesused > 0)
            {
                std::stringstream s;
                s << "Invalid metadata payload, size " << buf.bytesused;
                LOG_WARNING(s.str());
                _error_handler({RS2_NOTIFICATION_CATEGORY_FRAME_CORRUPTED, 0, RS2_LOG_SEVERITY_WARN, s.str()});
            }
        }
    }
}

std::shared_ptr<uvc_device> v4l_backend::create_uvc_device(uvc_device_info info) const
{
    auto v4l_uvc_dev = (!info.has_metadata_node) ? std::make_shared<v4l_uvc_device>(info) : std::make_shared<v4l_uvc_meta_device>(info);

    return std::make_shared<retry_controls_work_around>(v4l_uvc_dev);
}
*/
void v4luvc_init() {
}

static void devicecallback(uvc_device_info *pinfo, void *userdata) {
  NVPTL_DEVICE_INFO **pptmpdevice = (NVPTL_DEVICE_INFO **)userdata;
  // printf("got falcon device!\n");
  NVPTL_DEVICE_INFO *thedevice = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
  thedevice->next = *pptmpdevice;
  sprintf((char *)thedevice->usb_camera_name, "%s", pinfo->device_path.c_str());
  thedevice->type = NVPTL_UVC_INTERFACE;
  *pptmpdevice = thedevice;
}
void query_uvc_devices(UVCDEVICECALLBACK callback, uint16_t vid, uint16_t pid, void *userdata)
// std::vector<uvc_device_info> query_uvc_devices()
{
  std::vector<uvc_device_info> devices;

  auto action = [&devices](const uvc_device_info &i, const std::string &) {
    devices.push_back(i);
  };

  foreach_uvc_device(action);

  for (int i = 0; i < devices.size(); i++) {
    if (devices[i].vid == vid && devices[i].pid == pid) {
      printf("got:\nvid:0x%04X\npid:0x%04X\n", devices[i].vid, devices[i].pid);
      //	auto dev = std::make_shared<wmf_uvc_device>(devices[i]);
      callback(&devices[i], userdata);
    }
  }
}
/*
std::vector<uvc_device_info> v4l_backend::query_uvc_devices() const
{
    std::vector<uvc_device_info> uvc_nodes;
    v4l_uvc_device::foreach_uvc_device(
        [&uvc_nodes](const uvc_device_info &i, const std::string &)
        {
            uvc_nodes.push_back(i);
        });

    return uvc_nodes;
}*/
const extension_unit my_xu = {0, 6, 3, {0x41769ea2, 0x04de, 0xe347, {0x8b, 0x2b, 0xf4, 0x34, 0x1a, 0xff, 0x00, 0x3b}}};
int v4luvc_transfer(v4l_uvc_device *dev, unsigned char *data, int len) {
  // 每次发送60字节
  v4l_uvc_device *tmpdev = dev;
  int offset = 0;
  int tmplen = 0;
  int index = 0;
  unsigned char *tmpbuf = (unsigned char *)calloc(1, 64);
  while (offset < len) {
    if ((len - offset) >= 58) {
      tmplen = 58;
    } else {
      tmplen = len - offset;
    }
    memset(tmpbuf, 0, 60);
    *tmpbuf = tmplen + 2;
    *tmpbuf |= ((((len - offset) <= 58) ? 1 : 0) << 6);
    *(tmpbuf + 1) = index;
    memcpy(tmpbuf + 2, data + offset, tmplen);
    /*
    printf("will send index:%d, pkg size:%d bytes",index, tmplen + 2);
    printf("\n--------------------\n");
    for(int i = 0; i < tmplen + 2; i++)
    {
        printf("%lu ", tmpbuf[i]);
    }
    printf("\n--------------------\n");
    */
    if (tmpdev->set_xu(my_xu, 1, tmpbuf, 60)) {
      offset += tmplen;
      index++;
      // printf("ok to send %d bytes\n",tmplen);
    } else {
      // printf("fail to send %d bytes\n", tmplen);
      break;
    }
  }
  if (offset < len) {
    return -1;
  } else
    return 0;
}

v4l_uvc_device *get_uvc_device_by_vidpid_path(uint16_t vid, uint16_t pid, const char *path)
// std::vector<uvc_device_info> query_uvc_devices()
{
  std::vector<uvc_device_info> devices;

  auto action = [&devices](const uvc_device_info &info, const std::string &) {
    uvc_device_info device_info = info;
    devices.push_back(device_info);
  };

  foreach_uvc_device(action);

  for (int i = 0; i < devices.size(); i++) {
    if (devices[i].vid == vid && devices[i].pid == pid && 0 == strcmp(path, devices[i].device_path.c_str())) {
      return new v4l_uvc_device(devices[i]);
    }
  }
  return NULL;
}
typedef enum rs2_format {
  RS2_FORMAT_ANY,           /**< When passed to enable stream, librealsense will try to provide best suited format */
  RS2_FORMAT_Z16,           /**< 16-bit linear depth values. The depth is meters is equal to depth scale * pixel value. */
  RS2_FORMAT_DISPARITY16,   /**< 16-bit float-point disparity values. Depth->Disparity conversion : Disparity = Baseline*FocalLength/Depth. */
  RS2_FORMAT_XYZ32F,        /**< 32-bit floating point 3D coordinates. */
  RS2_FORMAT_YUYV,          /**< 32-bit y0, u, y1, v data for every two pixels. Similar to YUV422 but packed in a different order - https://en.wikipedia.org/wiki/YUV */
  RS2_FORMAT_RGB8,          /**< 8-bit red, green and blue channels */
  RS2_FORMAT_BGR8,          /**< 8-bit blue, green, and red channels -- suitable for OpenCV */
  RS2_FORMAT_RGBA8,         /**< 8-bit red, green and blue channels + constant alpha channel equal to FF */
  RS2_FORMAT_BGRA8,         /**< 8-bit blue, green, and red channels + constant alpha channel equal to FF */
  RS2_FORMAT_Y8,            /**< 8-bit per-pixel grayscale image */
  RS2_FORMAT_Y16,           /**< 16-bit per-pixel grayscale image */
  RS2_FORMAT_RAW10,         /**< Four 10 bits per pixel luminance values packed into a 5-byte macropixel */
  RS2_FORMAT_RAW16,         /**< 16-bit raw image */
  RS2_FORMAT_RAW8,          /**< 8-bit raw image */
  RS2_FORMAT_UYVY,          /**< Similar to the standard YUYV pixel format, but packed in a different order */
  RS2_FORMAT_NV12,          /**< i add this */
  RS2_FORMAT_MOTION_RAW,    /**< Raw data from the motion sensor */
  RS2_FORMAT_MOTION_XYZ32F, /**< Motion data packed as 3 32-bit float values, for X, Y, and Z axis */
  RS2_FORMAT_GPIO_RAW,      /**< Raw data from the external sensors hooked to one of the GPIO's */
  RS2_FORMAT_6DOF,          /**< Pose data packed as floats array, containing translation vector, rotation quaternion and prediction velocities and accelerations vectors */
  RS2_FORMAT_DISPARITY32,   /**< 32-bit float-point disparity values. Depth->Disparity conversion : Disparity = Baseline*FocalLength/Depth */
  RS2_FORMAT_Y10BPACK,      /**< 16-bit per-pixel grayscale image unpacked from 10 bits per pixel packed ([8:8:8:8:2222]) grey-scale image. The data is unpacked to LSB and padded with 6 zero bits */
  RS2_FORMAT_DISTANCE,      /**< 32-bit float-point depth distance value.  */
  RS2_FORMAT_MJPEG,         /**< Bitstream encoding for video in which an image of each frame is encoded as JPEG-DIB   */
  RS2_FORMAT_Y8I,           /**< 8-bit per pixel interleaved. 8-bit left, 8-bit right.  */
  RS2_FORMAT_Y12I,          /**< 12-bit per pixel interleaved. 12-bit left, 12-bit right. Each pixel is stored in a 24-bit word in little-endian order. */
  RS2_FORMAT_INZI,          /**< multi-planar Depth 16bit + IR 10bit.  */
  RS2_FORMAT_INVI,          /**< 8-bit IR stream.  */
  RS2_FORMAT_W10,           /**< Grey-scale image as a bit-packed array. 4 pixel data stream taking 5 bytes */
  RS2_FORMAT_Z16H,          /**< Variable-length Huffman-compressed 16-bit depth values. */
  RS2_FORMAT_FG,            /**< 16-bit per-pixel frame grabber format. */
  RS2_FORMAT_Y411,          /**< 12-bit per-pixel. */
  RS2_FORMAT_COUNT          /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
} rs2_format;
template <typename T>
uint32_t rs_fourcc(const T a, const T b, const T c, const T d) {
  static_assert((std::is_integral<T>::value), "rs_fourcc supports integral built-in types only");
  return ((static_cast<uint32_t>(a) << 24) |
          (static_cast<uint32_t>(b) << 16) |
          (static_cast<uint32_t>(c) << 8) |
          (static_cast<uint32_t>(d) << 0));
}
const std::map<uint32_t, rs2_format> platform_color_fourcc_to_rs2_format = {
    {rs_fourcc('Y', 'U', 'Y', '2'), RS2_FORMAT_YUYV},
    {rs_fourcc('Y', 'U', 'Y', 'V'), RS2_FORMAT_YUYV},
    {rs_fourcc('N', 'V', '1', '2'), RS2_FORMAT_NV12},
    {rs_fourcc('M', 'J', 'P', 'G'), RS2_FORMAT_MJPEG},
};
#define UNKNOWN_VALUE "UNKNOWN"
const char *get_string(rs2_format value) {
#define CASE(X)        \
  case RS2_FORMAT_##X: \
    return #X;
  switch (value) {
    CASE(ANY)
    CASE(Z16)
    CASE(DISPARITY16)
    CASE(DISPARITY32)
    CASE(XYZ32F)
    CASE(YUYV)
    CASE(RGB8)
    CASE(BGR8)
    CASE(RGBA8)
    CASE(BGRA8)
    CASE(Y8)
    CASE(Y16)
    CASE(RAW10)
    CASE(RAW16)
    CASE(RAW8)
    CASE(UYVY)
    CASE(MOTION_RAW)
    CASE(MOTION_XYZ32F)
    CASE(GPIO_RAW)
    CASE(6DOF)
    CASE(Y10BPACK)
    CASE(DISTANCE)
    CASE(MJPEG)
    CASE(Y8I)
    CASE(Y12I)
    CASE(INZI)
    CASE(INVI)
    CASE(W10)
    CASE(Z16H)
    CASE(FG)
    CASE(Y411)
    CASE(NV12)
  default:
    // assert(!is_valid(value));
    return UNKNOWN_VALUE;
  }
#undef CASE
}

static void v4luvc_start_stream(STREAMCALLBACK myframecallback, v4l_uvc_device *v4ldevice, void *userdata) {
  v4l_uvc_device *dev = v4ldevice;
  dev->set_power_state(D0);
  /*  for (auto &&xu : _xus)
        dev->init_xu(xu);
*/
  // dev->set_power_state(D3);

  auto profiles = dev->get_profiles();
  for (int i = 0; i < profiles.size(); i++) {
    printf("support:%dx%d,%d,%s\n", profiles[i].width, profiles[i].height, profiles[i].fps, get_string(platform_color_fourcc_to_rs2_format.at(profiles[i].format)));
    if (profiles[i].width == 1920 &&
        profiles[i].height == 1080 &&
        profiles[i].fps == 60 && 0 == strcmp("MJPEG", get_string(platform_color_fourcc_to_rs2_format.at(profiles[i].format)))) {
      dev->probe_and_commit(
          profiles[i], [myframecallback, userdata](stream_profile p, frame_object f, std::function<void()> continuation) mutable {
                //	static unsigned int frameindex = 0;
               //     printf("recv raw data len:%u\n",f.frame_size);
                    myframecallback(f.pixels, f.frame_size, userdata); },
          DEFAULT_V4L2_FRAME_BUFFERS);
      dev->start_callbacks();
      dev->stream_on();
      break;
    }
  }
}
/*
std::shared_ptr<command_transfer> v4l_backend::create_usb_device(usb_device_info info) const
{
    auto dev = usb_enumerator::create_usb_device(info);
    if (dev)
        return std::make_shared<command_transfer_usb>(dev);
    return nullptr;
}

std::vector<usb_device_info> v4l_backend::query_usb_devices() const
{
    auto device_infos = usb_enumerator::query_devices_info();
    // Give the device a chance to restart, if we don't catch
    // it, the watcher will find it later.
    if (tm_boot(device_infos))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        device_infos = usb_enumerator::query_devices_info();
    }
    return device_infos;
}

std::shared_ptr<hid_device> v4l_backend::create_hid_device(hid_device_info info) const
{
    return std::make_shared<v4l_hid_device>(info);
}

std::vector<hid_device_info> v4l_backend::query_hid_devices() const
{
    std::vector<hid_device_info> results;
    v4l_hid_device::foreach_hid_device([&](const hid_device_info &hid_dev_info)
                                       { results.push_back(hid_dev_info); });

    return results;
}
std::shared_ptr<time_service> v4l_backend::create_time_service() const
{
    return std::make_shared<os_time_service>();
}

std::shared_ptr<device_watcher> v4l_backend::create_device_watcher() const
{
#if defined(USING_UDEV)
    return std::make_shared<udev_device_watcher>(this);
#else
    return std::make_shared<polling_device_watcher>(this);
#endif
}

std::shared_ptr<backend> create_backend()
{
    return std::make_shared<v4l_backend>();
}*/
void nvptl_v4luvc_init() {
}

void nvptl_v4luvc_deinit() {
}

NVPTL_RESULT nvptl_v4luvc_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices) {
  int countdevices = 0;
  /*static*/ NVPTL_DEVICE_INFO *tmpdevice = NULL;
  /*if (NULL != tmpdevice)
  {
    nvptl_freedevices(tmpdevice);
    tmpdevice = NULL;
  }*/

  query_uvc_devices(devicecallback, NVPTL_VID, NVPTL_PID, (void *)&tmpdevice);

  if (tmpdevice != NULL) {
    NVPTL_DEVICE_INFO *justdevice = tmpdevice;
    while (justdevice != NULL) {
      countdevices++;
      justdevice = justdevice->next;
    }
    *ptotal = countdevices;
    *ppdevices = tmpdevice;
    return NVPTL_OK;
  } else
    return NVPTL_FAILED;
}
static int usb_hal_open(NVPTL_INSTANCE *inst, int bus, char *path) {
  (void)bus;
  int countdevices = 0;
  (void)countdevices;
  static NVPTL_DEVICE_INFO *tmpdevice = NULL;
  if (NULL != tmpdevice) {
    nvptl_freedevices(tmpdevice);
    tmpdevice = NULL;
  }

  auto devinfo = get_uvc_device_by_vidpid_path(NVPTL_VID, NVPTL_PID, path);

  if (devinfo != NULL) {
    inst->device_usb_handle = (void *)devinfo;

    return NVPTL_OK;
  } else
    return NVPTL_FAILED;
}
static void nvptl_recv_frame_callback(NVPTL_DEVICE_HANDLE handle, uint8_t *data, size_t len, void *userdata) {
  //	printf("recv whole frame from v4luvc!\n");
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  if (inst->recvframecallback != NULL) {
    //   printf("33recv packet len:%u\n",len);
    inst->recvframecallback(handle, data, len, userdata);
  }
}
static void loop_recv_callback(const void *pixels, size_t framesize, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)userdata;
  // FRAMECALLBACK callback = (FRAMECALLBACK)inst->framecallback;
  // depthtool* toolobj = (depthtool*)pParam;
  if (inst->usb_buf == NULL)
    inst->usb_buf = (uint8_t *)malloc(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);
  // printf("after malloc usb_buf!\n");
  //	static unsigned char *tmpbuf = NULL;
  if (NULL == inst->tmpbuf)
    inst->tmpbuf = (unsigned char *)malloc(USB_PACKET_MAX_SIZE);
  // printf("after malloc tmpbuf!\n");

  if (inst->devinfo.type == NVPTL_UVC_INTERFACE) {
    //     printf("current data packet size:%u\n",framesize);
    // printf("type ok,will zubao!\n");
    // on_usb_data_receive(inst, (uint8_t*)pixels, framesize);
    size_t bytesTransffered = framesize;
    static size_t index = 0;
    static unsigned long currentpacketlen = 0;
    if (((long)bytesTransffered >= (long)sizeof(NVPTL_USBHeaderDataPacket)) && (index == 0) && (0 == strncmp("NEXT_VPU", (const char *)pixels, 8))) { //
      memcpy(inst->usb_buf, pixels, bytesTransffered);
      index += bytesTransffered;
      currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
      int type = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->type;
      int sub_type = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->sub_type;
      if (type == 11)
        printf("-----type:%d sub_type:%d-----\n", type, sub_type);
      if (index == (currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
        //     printf("11recv whole frame size:%u\n",index);
        nvptl_recv_frame_callback(inst, inst->usb_buf, index, inst->userdata);
        index = 0;
        currentpacketlen = 0;
      }
    } else if (index >= sizeof(NVPTL_USBHeaderDataPacket)) {
      if ((index + bytesTransffered) <= USB_PACKET_MAX_SIZE) {
        memcpy(inst->usb_buf + index, pixels, bytesTransffered);
        index += bytesTransffered;
        if (index == (currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
          //    printf("22recv whole frame size:%u\n",index);
          nvptl_recv_frame_callback(inst, inst->usb_buf, index, inst->userdata);
          index = 0;
          currentpacketlen = 0;
        } else if (index > (currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
          printf("fail data!!!\n");
          index = 0;
          currentpacketlen = 0;
        }
      } else {
        printf("false data!\n");
        index = 0;
        currentpacketlen = 0;
      }
    } else {
      printf("false data,give up....\n");
      index = 0;
      currentpacketlen = 0;
    }
    return;
  } else {
    printf("type fail!\n");
  }
}
NVPTL_DEVICE_HANDLE nvptl_v4luvc_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)calloc(1, sizeof(NVPTL_INSTANCE));
  inst->devinfo = *dev_info;
  if (usb_hal_open(inst, 0, dev_info->usb_camera_name) >= 0) {
    inst->eventcallback = NULL;
    inst->connectuserdata = NULL;

    inst->recvframecallback = callback;
    inst->status = STARTED;

    inst->userdata = userdata;
    v4luvc_start_stream(loop_recv_callback, (v4l_uvc_device *)inst->device_usb_handle, inst);
    nvptl_debug_printf("connect ok!!!!\n");
    return inst;
  }
  free(inst);
  return NULL;
}

void nvptl_v4luvc_close(NVPTL_DEVICE_HANDLE handle) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;

  inst->status = STOPPED;

  v4l_uvc_device *tmpdev = (v4l_uvc_device *)inst->device_usb_handle;
  delete tmpdev;

  if (inst->tmpbuf != NULL) {
    free(inst->tmpbuf);
    inst->tmpbuf = NULL;
  }
  if (inst->usb_buf != NULL) {
    free(inst->usb_buf);
    inst->usb_buf = NULL;
  }

  free(inst);
}

int nvptl_v4luvc_send(NVPTL_DEVICE_HANDLE handle, unsigned char *sendbuffer, size_t len) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  v4luvc_transfer((v4l_uvc_device *)inst->device_usb_handle, (unsigned char *)sendbuffer, len);
  return 0;
}