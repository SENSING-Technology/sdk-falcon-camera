#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
// #include <fmt/core.h>
#include <stdio.h>

#ifdef ENABLE_PERF
#include <gperftools/profiler.h>
#endif
namespace pyb = pybind11;

// sdk-related.
#include "nvpfm.h"
#include "nvpfm.hpp"

#include <memory>
#include <thread>
#include <string>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <bitset>
#include <fcntl.h>
#include <array>
#include <signal.h>

// #include "concurrentqueue.h"
//  #include "ringbuffer.hpp"
#include "ring_queue.h"

#define DEBUG

typedef struct
{
    uint32_t countfps;
    uint32_t lastfps;
    struct timeval last;
    float fps;
} COUNTFPSDATA;
#if 0
struct s_nvpfm_dev_info_trans {
    std::string sn;
    std::string product;
    std::string cpu_net_type;
    std::string comm_type;
    std::string projector_type;
    std::string imu_type0;
    std::string imu_type1;
    std::string software_version;
    std::string usb_speed;
    float cpu_temperature;
    float projector_temperature[2];
    bool sensor[MAX_SENSOR_NUMBER];//哪些sensor可用，TRUE表示可用,FALSE表示不可用
    bool depth[MAX_DEPTH_NUMBER];//总共最多2个深度,TRUE表示可用,FALSE表示不可用
    bool depth0[MAX_SENSOR_NUMBER];//第一路深度绑定的sensor序号
    bool depth1[MAX_SENSOR_NUMBER];//第二路深度绑定的sensor序号
    bool rgb[2];//总共最多2个rgb,TRUE表示可用,FALSE表示不可用
    bool rgb0[MAX_SENSOR_NUMBER];//第一路rgb绑定的sensor序号
    bool rgb1[MAX_SENSOR_NUMBER];//第二路rgb绑定的sensor序号
    RESCOMBO irsupported[MAXSUPPORTEDRES];
    RESCOMBO colorsupported[MAXSUPPORTEDRES];
};
#endif

static std::vector<bool> I2BDelegate(BOOL *in, std::size_t len)
{
    std::vector<bool> res(len, false);
    for (int i = 0; i < len; ++i)
    {
        res[i] = (bool)in[i];
    }
    return res;
}

static std::vector<float> F2VDelegate(float *in, std::size_t len)
{
    std::vector<float> res(len, 0.0);
    for (int i = 0; i < len; ++i)
    {
        res[i] = (float)in[i];
    }
    return res;
}

typedef struct
{
    std::thread dev_thread;
    std::thread align_thread;
    std::thread depth_thread;
    NVPTL_DEVICE_INFO rawdevinfo;
    std::unique_ptr<nvpfm> fm = nullptr;
    std::unique_ptr<depthtransformer> transformer = nullptr;
    std::vector<s_nvpfm_camera_param> camparam;
    // like volatile bool
    std::atomic<bool> willrun;
    // initialized.
    std::atomic<bool> initialized;
    s_nvpfm_dev_info devinfo;
    uint16_t *aligndata = NULL;
    int rgbwidth = 0;
    int rgbheight = 0;

    COUNTFPSDATA rgbcountfps;
    COUNTFPSDATA depthcountfps;
    COUNTFPSDATA leftircountfps;
    COUNTFPSDATA rightircountfps;
    void *container;

    Ring_Queue *framequeue;
    Ring_Queue *alignframequeue;
    Ring_Queue *depthqueue;
    // add 同步变量
    std::atomic<bool> in_callback;
    // 是否在回调中
    inline void in_a_cb()
    {
        in_callback.store(true, std::memory_order::memory_order_relaxed);
    }
    inline void out_a_cb()
    {
        in_callback.store(false, std::memory_order::memory_order_relaxed);
    }

} DEVICEINFO;

static uint64_t get_time_ms()
{
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    uint64_t res = tv.tv_sec * 1000;
    res += (tv.tv_usec / 1000);
    return res;
}

static void signal_cb(int signum)
{
#ifdef ENABLE_PERF
    if (signum == SIGUSR1)
    {
        nvpfm_info_printf("StartProfiler!\n");
        ProfilerStart("pybind.prof");
    }
    else if (signum == SIGUSR2)
    {
        nvpfm_info_printf("StopProfiler!\n");
        ProfilerStop();
    }
#endif
}

static void setup_signal()
{
    struct sigaction profstat;
    profstat.sa_handler = signal_cb;
    profstat.sa_flags = 0;
    sigemptyset(&profstat.sa_mask);
    sigaddset(&profstat.sa_mask, SIGUSR1);
    sigaddset(&profstat.sa_mask, SIGUSR2);
    int ret = sigaction(SIGUSR1, &profstat, NULL);
    if (ret < 0)
    {
        nvpfm_warn_printf("Failed setup signal!\n");
    }
    ret = sigaction(SIGUSR2, &profstat, NULL);
    if (ret < 0)
    {
        nvpfm_warn_printf("Failed setup signal!\n");
    }
}

//
#define TO_STRING(VAR) "\"" #VAR "\""

struct PaletteUnit
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

static constexpr double DEPTH_MIN = 100.0;
static constexpr double DEPTH_MAX = 5000.0;
static constexpr int PaletteSize = 65536;
static std::array<PaletteUnit, PaletteSize> g_pallete;

static void setup_palette()
{
    static constexpr double diff = DEPTH_MAX - DEPTH_MIN;
    PaletteUnit initial_v;
    initial_v.r = initial_v.g = initial_v.b = 0;
    std::fill(g_pallete.begin(), g_pallete.end(), initial_v);

    for (int i = DEPTH_MIN; i <= DEPTH_MAX; ++i)
    {
        double cur_diff = ((double)i - DEPTH_MIN) * 255.0 / diff;
        if (cur_diff >= 255)
        {
            cur_diff = 254;
        }
        uint8_t y, u, v;
        y = u = v = 0;
        if (nvpfm_getyuvfromindex(cur_diff, &y, &u, &v) < 0)
        {
            continue;
        }
        float fr = y + 1.4075 * (v - 128);

        float fg = y - 0.3455 * (u - 128) - 0.7169 * (v - 128);

        float fb = y + 1.779 * (u - 128);
        g_pallete[i].r = (uint8_t)fr;
        g_pallete[i].g = (uint8_t)fg;
        g_pallete[i].b = (uint8_t)fb;
    }
}

class py_nvpfm
{
public:
    struct cb_gurad
    {
        explicit cb_gurad(DEVICEINFO *info)
        {
            this->info = info;
            info->in_a_cb();
        }
        ~cb_gurad()
        {
            info->out_a_cb();
        }
        DEVICEINFO *info;
    };
    // note for construction
    static constexpr std::memory_order default_order = std::memory_order::memory_order_relaxed;
    // the events.
    // export to python
    // limited to 32
    enum nvpfm_events
    {
        // 深度数据
        NVP_EV_DEPTH = 0,
        // 深度伪彩
        NVP_EV_DEPTH_PSEUDO,
        // 深度alignrgb
        NVP_EV_DEPTH_ALIGN_RGB,
        // 深度alignrgb pseudo
        NVP_EV_DEPTH_ALIGN_RGB_PSEUDO,
        // RGB帧
        NVP_EV_RGB,
        // 左目红外
        NVP_EV_LEFT_IR,
        // 右目红外
        NVP_EV_RIGHT_IR,
        // ???
        NVP_EV_GROUP,
        // IMU数据
        NVP_EV_IMU,
        // device info 数据
        NVP_EV_DEV_INFO,

        NVP_EV_COUNT,
    };

    // the most import structure which will be exported to the python.
    // todo: modify to image-frame data later, use common and others
    struct event_data
    {
        event_data()
        {
            imu_factory_data = nullptr;
            imu_application_data = nullptr;
            frame_data = nullptr;
            dev_info = nullptr;
        }

        ~event_data()
        {
            // note: pybind 自动托管
#if 0
            if(imu_factory_data) {
                delete imu_factory_data;
            }
            if(imu_application_data) {
                delete imu_application_data;
            }
#endif
            if (frame_data)
            {
                delete frame_data;
            }
        }

        void reset()
        {
            imu_factory_data = nullptr;
            imu_application_data = nullptr;
            frame_data = nullptr;
            dev_info = nullptr;
            ev_type = NVP_EV_COUNT;
            valid = false;
        }

        void init(nvpfm_events ev, uint32_t frame_size, bool imu_is_factory = false)
        {
            reset();
            ev_type = ev;
            if (ev == NVP_EV_IMU)
            {
                this->imu_is_factory = imu_is_factory;
                if (imu_is_factory)
                {
                    imu_factory_data = new IMU_FACTORY_DATA_STRUC;
                }
                else
                {
                    imu_application_data = new IMU_APPLICATION_DATA_STRUC;
                }
            }
            else if (ev == NVP_EV_DEV_INFO)
            {
                dev_info = new s_nvpfm_dev_info;
            }
            else
            {
                frame_data = new std::vector<uint8_t>(frame_size); // new std::array<uint8_t, 1920 * 1080 * 2>();
                // removed by cfy
                // std::fill(frame_data->begin(), frame_data->end(), 0);
            }
            valid = true;
        }

        static void move_source(event_data &lhs, event_data &rhs)
        {
            if (rhs.imu_factory_data)
            {
                lhs.imu_factory_data = rhs.imu_factory_data;
                rhs.imu_factory_data = nullptr;
            }
            if (rhs.imu_application_data)
            {
                lhs.imu_application_data = rhs.imu_application_data;
                rhs.imu_application_data = nullptr;
            }
            if (rhs.frame_data)
            {
                lhs.frame_data = rhs.frame_data;
                rhs.frame_data = nullptr;
            }
            if (rhs.dev_info)
            {
                lhs.dev_info = rhs.dev_info;
                rhs.dev_info = nullptr;
            }
            rhs.valid = false;
        }

        void contrust_numpy_array()
        {
            if (!frame_data)
            {
                return;
            }
            frame_data_array = pyb::array(frame_data->size(), frame_data->data());
            delete frame_data;
            frame_data = nullptr;
        }

        inline bool is_imu()
        {
            return (ev_type == NVP_EV_IMU);
        }

        // 帧率
        float fps_data;
        // 事件类型
        nvpfm_events ev_type;
        //
        // 帧数据，仅数据类型为NVP_EV_RGB/NVP_EV_LEFT_IR/NVP_EV_RIGHT_IR 时有效
        //
        // 帧宽度
        uint32_t frame_width;
        // 帧高度
        uint32_t frame_height;
        // std::array<uint8_t, 1920 * 1080*2> *frame_data;
        std::vector<uint8_t> *frame_data;
        pyb::array frame_data_array;
        // 哪个设备
        std::string dev_name;
        //
        // imu 数据
        //
        uint8_t imu_channel;   // 通道号
        bool imu_is_factory;   // 是否为厂测IMU数据
        uint8_t imu_data_type; // bit0:gyro,bit1:accel,bit2:MANG ,bit3:temp.0-disable,1-enable
        uint8_t imu_data_number;
        // remember move.
        IMU_FACTORY_DATA_STRUC *imu_factory_data;
        IMU_APPLICATION_DATA_STRUC *imu_application_data;
        s_nvpfm_dev_info *dev_info;
        bool valid;
    };

    py_nvpfm()
    {
        m_valid = false;
        setup_palette();
        m_quit.store(false, default_order);
        m_modes.store(0, default_order);
        m_enum_quited.store(false, default_order);
    }
    // note for destruction
    ~py_nvpfm()
    {
        // close all the fds.
    }

    void release(void *data, bool blocking = true)
    {
        DEVICEINFO *info = (DEVICEINFO *)(data);
        // set the control flag to false, but it's not safe.
        info->willrun.store(false, default_order);
        // stop all the callbacks.
        // 确定是否停下来 ????
        // 当前无法生效
        info->fm->stop_depth();
        info->fm->stop_rgb();
        info->fm->stop_rightir();
        info->fm->stop_leftir();
        info->fm->stop_imu();
        while (info->in_callback.load(default_order))
        {
            std::this_thread::yield();
        }
        nvpfm_info_printf("All callback is finished!\n");
        nvpfm_info_printf("Wait for the threads exit.\n");
        if (info->dev_thread.joinable())
        {
            info->dev_thread.join();
        }
        if (info->align_thread.joinable())
        {
            info->align_thread.join();
        }
        if (info->depth_thread.joinable())
        {
            info->depth_thread.join();
        }
        nvpfm_info_printf("Device thread quit!\n");
        // destroy ring queue
        if (info->framequeue != NULL)
        {
            // after this, should not call read data any more.
            info->initialized.store(false, std::memory_order::memory_order_release);
            Destroy_Ring_Queue(info->framequeue);
            info->framequeue = NULL;
        }
        if (info->alignframequeue != NULL)
        {
            // after this, should not call read data any more.
            info->initialized.store(false, std::memory_order::memory_order_release);
            Destroy_Ring_Queue(info->alignframequeue);
            info->alignframequeue = NULL;
        }
        if (info->depthqueue != NULL)
        {
            // after this, should not call read data any more.
            info->initialized.store(false, std::memory_order::memory_order_release);
            Destroy_Ring_Queue(info->depthqueue);
            info->depthqueue = NULL;
        }
        if (blocking)
        {
            m_mutex.lock();
        }
        {
            //  std::unique_lock<std::mutex> lg(m_mutex);
            auto it = m_dev_infos.find(info->rawdevinfo.usb_camera_name);
            if (it != m_dev_infos.end())
            {
                m_dev_infos.erase(it);
            }
            if (m_dev_infos.empty())
            {
                // notify the main thread exit
                nvpfm_info_printf("All threads quit!\n");
                m_valid.store(false, std::memory_order::memory_order_relaxed);
            }
        }
        if (blocking)
        {
            m_mutex.unlock();
        }
        delete info;
        nvpfm_info_printf("Thread released!\n");
    }

    inline bool get_event_enabled(nvpfm_events evt)
    {
        uint32_t evb = 1 << evt;
        return m_modes.load(std::memory_order::memory_order_relaxed) & evb;
    }

    static bool count_fps(COUNTFPSDATA *pdata, const std::string &streamtype, float *pfps)
    {
        if (!pdata->last.tv_sec)
        {
            gettimeofday(&pdata->last, NULL);
        }
        pdata->countfps++;
        struct timeval tv;
        gettimeofday(&tv, NULL);
        double offset = (double)tv.tv_sec - (double)pdata->last.tv_sec + ((double)tv.tv_usec - (double)pdata->last.tv_usec) / 1000000.0;

        if (1.0 < offset)
        {
            float fps = (float)(pdata->countfps - pdata->lastfps) / offset;
            // replace with callback or sth.
            // 打出来是16-20fps
            // printf("%s fps:%f\n", streamtype.c_str(), fps);
            pdata->last = tv;
            pdata->lastfps = pdata->countfps;
            *pfps = fps;
            return true;
        }
        return false;
    }

    // DO NOT DELETE!
    // get data first, then add mode control.

    static void setup_ir_evd(DEVICEINFO *info, NVPFM_USB_IMAGE_HEADER *header, event_data *ev_data)
    {
        // ev_data->fps_data = info->leftircountfps.fps;
        ev_data->dev_name = info->rawdevinfo.usb_camera_name;
        ev_data->frame_height = header->height;
        ev_data->frame_width = header->width;
        uint8_t *image_data = (uint8_t *)header + sizeof(NVPFM_USB_IMAGE_HEADER);

        memcpy(ev_data->frame_data->data(), image_data, ev_data->frame_height * ev_data->frame_width);
    }

    static void left_ir_cb(void *data, void *userdata)
    {
        // info 可能会失效
        DEVICEINFO *info = (DEVICEINFO *)userdata;
        // not enabled.
        cb_gurad g(info);
        py_nvpfm *this_ = reinterpret_cast<py_nvpfm *>(info->container);
        if (!this_->get_event_enabled(nvpfm_events::NVP_EV_LEFT_IR))
        {
            return;
        }
        // filter.
#if 1
        NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
        if (tmppack->sub_type != IMAGE_CHANNEL0_CALIBRATED &&
            tmppack->sub_type != IMAGE_CHANNEL0_ORIGNAL)
        {
            return;
        }
#endif
        count_fps(&info->leftircountfps, "leftir", &info->leftircountfps.fps);
        NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));
        event_data *ev_data = (event_data *)SOLO_Write(info->framequeue);
        if (!ev_data)
        {
            return;
        }
        ev_data->init(NVP_EV_LEFT_IR, leftirheader->width * leftirheader->height);
        setup_ir_evd(info, leftirheader, ev_data);
        ev_data->fps_data = info->leftircountfps.fps;
        // mono8 单色 8bit图像 mono10 单色10bit图像 工业相机此外还有rgb标准等

        SOLO_Write_Over(info->framequeue);
    }

    static void right_ir_cb(void *data, void *userdata)
    {
        DEVICEINFO *info = (DEVICEINFO *)userdata;
        cb_gurad g(info);
        py_nvpfm *this_ = reinterpret_cast<py_nvpfm *>(info->container);
        if (!this_->get_event_enabled(nvpfm_events::NVP_EV_RIGHT_IR))
        {
            return;
        }
        count_fps(&info->rightircountfps, "rightir", &info->rightircountfps.fps);
        NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));
        event_data *ev_data = (event_data *)SOLO_Write(info->framequeue);
        if (!ev_data)
        {
            return;
        }
        ev_data->init(NVP_EV_RIGHT_IR, rightirheader->height * rightirheader->width);
        setup_ir_evd(info, rightirheader, ev_data);
        ev_data->fps_data = info->rightircountfps.fps;
        SOLO_Write_Over(info->framequeue);
    }

    static void rgb_cb(void *data, void *userdata)
    {
        DEVICEINFO *info = (DEVICEINFO *)(userdata);
        cb_gurad g(info);
        py_nvpfm *this_ = reinterpret_cast<py_nvpfm *>(info->container);
        if (!this_->get_event_enabled(nvpfm_events::NVP_EV_RGB))
        {
            return;
        }
        if (count_fps(&info->rgbcountfps, "rgb", &info->rgbcountfps.fps))
        {
            // printf("got new rgb fps!\n");
        }

        // return;
        NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
        NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));
        info->rgbwidth = rgbheader->width;
        info->rgbheight = rgbheader->height;
        uint8_t *rgbrawdata = (uint8_t *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket) + sizeof(NVPFM_USB_IMAGE_HEADER));
        // printf("got rgb frame!\n");
        event_data *ev_data = (event_data *)SOLO_Write(info->framequeue);
        if (!ev_data)
        {
            return;
        }
        // 分析卡顿原因是什么
        uint32_t dlen = tmppack->len - sizeof(NVPFM_USB_IMAGE_HEADER);
        ev_data->init(NVP_EV_RGB, dlen);
        ev_data->fps_data = info->rgbcountfps.fps;
        ev_data->ev_type = nvpfm_events::NVP_EV_RGB;
        // strcpy(ev_data->dev_name, info->rawdevinfo.usb_camera_name);
        ev_data->dev_name = info->rawdevinfo.usb_camera_name;
        ev_data->frame_width = rgbheader->width;
        ev_data->frame_height = rgbheader->height;
        memcpy(ev_data->frame_data->data(), rgbrawdata, dlen);
        SOLO_Write_Over(info->framequeue);
    }

    static void depth_cb(void *data, void *userdata)
    {
        DEVICEINFO *info = (DEVICEINFO *)userdata;
        cb_gurad g(info);
        NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data +
                                                                         sizeof(NVPTL_USBHeaderDataPacket));
        uint16_t *depthraw = (uint16_t *)((unsigned char *)depthheader +
                                          sizeof(NVPFM_USB_IMAGE_HEADER));
        py_nvpfm *this_ = reinterpret_cast<py_nvpfm *>(info->container);
        event_data *evd = nullptr;
        uint16_t *d = (uint16_t *)((uint8_t *)depthheader + sizeof(NVPFM_USB_IMAGE_HEADER));

        NVPFM_USB_IMAGE_HEADER *tmpdepth = NULL;
        if ((tmpdepth = (NVPFM_USB_IMAGE_HEADER *)SOLO_Write(info->depthqueue)) != NULL)
        {
            memcpy(tmpdepth, depthheader, depthheader->width * depthheader->height * sizeof(uint16_t) + sizeof(NVPFM_USB_IMAGE_HEADER));
            SOLO_Write_Over(info->depthqueue);
        }

        if (this_->get_event_enabled(nvpfm_events::NVP_EV_DEPTH_PSEUDO))
        {
            // pesudo
            evd = (event_data *)SOLO_Write(info->framequeue);
            if (!evd)
            {
                return;
            }

            uint32_t dlen = depthheader->height * depthheader->width;
            evd->init(NVP_EV_DEPTH_PSEUDO, dlen * 3);
            auto &frame_data = *evd->frame_data;
            evd->frame_height = depthheader->height;
            evd->frame_width = depthheader->width;
            evd->dev_name = info->rawdevinfo.usb_camera_name;
            for (int i = 0; i < dlen; ++i)
            {
                uint16_t depth_v = d[i];
                const PaletteUnit &pu = g_pallete[depth_v];
                frame_data[i * 3] = pu.r;
                frame_data[i * 3 + 1] = pu.g;
                frame_data[i * 3 + 2] = pu.b;
            }
            SOLO_Write_Over(info->framequeue);
        }

        if (this_->get_event_enabled(nvpfm_events::NVP_EV_DEPTH))
        {
            evd = (event_data *)SOLO_Write(info->framequeue);
            if (!evd)
            {
                // for debug
#ifdef DEBUG
                // printf("No evd avaliable!\n");
#endif
                return;
            }
            uint32_t dlen = depthheader->height * depthheader->width * sizeof(uint16_t);
            evd->init(NVP_EV_DEPTH, dlen);
            evd->frame_height = depthheader->height;
            evd->frame_width = depthheader->width;
            evd->dev_name = info->rawdevinfo.usb_camera_name;
            // d= (uint16_t *)((uint8_t *)depthheader + sizeof(NVPFM_USB_IMAGE_HEADER));
            memcpy(evd->frame_data->data(), d, evd->frame_width * evd->frame_height * sizeof(uint16_t));
            SOLO_Write_Over(info->framequeue);
            // printf("Depth data is written!\n");
        }
        else
        {
            // for debug
#ifdef DEBUG
            // printf("Depth data is not enabled!\n");
#endif
        }
    }

    /// @brief imu callback.
    /// @param data
    /// @param userdata
    static void imu_cb(void *data, void *userdata)
    {
        DEVICEINFO *info = (DEVICEINFO *)userdata;
        cb_gurad g(info);
        NVPTL_USBHeaderDataPacket *pack = (NVPTL_USBHeaderDataPacket *)data;
        s_nvpfm_imu_data *imu_data = (s_nvpfm_imu_data *)pack->data;
        py_nvpfm *this_ = reinterpret_cast<py_nvpfm *>(info->container);
        if (!this_->get_event_enabled(NVP_EV_IMU))
        {
#ifdef DEBUG
            // printf("!!!!imu is not enabled!\n");
#endif
            return;
        }
        if (pack->type != NVPFM_IMU_DATA)
        {
            // printf("!!!!imu data is not valid, type:%d\n", imu_data->data_type);
            return;
        }
        if (pack->len < sizeof(s_nvpfm_imu_data) || imu_data->data_number <= 0)
        {
            // printf("!!!!check failed!\n");
            return;
        }
        event_data *evd = (event_data *)SOLO_Write(info->framequeue);
        if (!evd)
        {
            return;
        }
        evd->init(NVP_EV_IMU, 0, imu_data->factory_data);
        // 其后数据怎么填?
        evd->imu_data_type = imu_data->data_type;
        evd->imu_data_number = imu_data->data_number;
        evd->imu_channel = imu_data->channel;
        // evd->imu_is_factory = (bool) imu_data->factory_data;

        if (evd->imu_is_factory)
        {
            memcpy(evd->imu_factory_data, &imu_data->data, sizeof(IMU_FACTORY_DATA_STRUC));
        }
        else
        {
            memcpy(evd->imu_application_data, &imu_data->data, sizeof(IMU_APPLICATION_DATA_STRUC));
        }
        SOLO_Write_Over(info->framequeue);
    }

    static void init(DEVICEINFO *info)
    {
        if (!info->fm->isplugged())
        {
            return;
        }
        nvpfm_info_printf("FC1 is plugged!\n");
        while (info->willrun)
        {
            if (NVPTL_OK == info->fm->get_devinfo(&info->devinfo))
            {
                // nvpfm_info_printf("device info got!\n");
                break;
            }
            else
            {
                sleep(1);
            }
        }
        if (!info->willrun)
        {
            return;
        }
        info->fm->start_rgb(&py_nvpfm::rgb_cb);
        info->fm->start_leftir(&py_nvpfm::left_ir_cb);
        info->fm->start_rightir(&py_nvpfm::right_ir_cb);
        info->fm->start_depth(&py_nvpfm::depth_cb);
        info->fm->start_imu(&py_nvpfm::imu_cb);
        nvpfm_info_printf("init finished!\n");
    }
    static void event_cb(EVENTREASON reason, void *data)
    {
        DEVICEINFO *info = (DEVICEINFO *)(data);
        info->in_a_cb();
        if (reason == INFORMREBOOT)
        {
            info->fm->send_reboot();
            info->out_a_cb();
            return;
        }
        else
        {
            nvpfm_info_printf("plugout!\n");
        }
        // 提前释放占用
        info->out_a_cb();
        py_nvpfm *this_ = reinterpret_cast<py_nvpfm *>(info->container);
        this_->release(data);
    }
    void align_thread_loop(void *para)
    {
        static constexpr int sleep_time = 33 * 1000;
        DEVICEINFO *info = static_cast<DEVICEINFO *>(para);
        py_nvpfm *this_ = reinterpret_cast<py_nvpfm *>(info->container);
        // when will run going to be a false?
        while (info->willrun)
        {
            NVPFM_USB_IMAGE_HEADER *depthheader = NULL;
            if ((depthheader = (NVPFM_USB_IMAGE_HEADER *)SOLO_Read(info->depthqueue)) != NULL)
            {
                if (this_->get_event_enabled(nvpfm_events::NVP_EV_DEPTH_ALIGN_RGB))
                {
                    uint16_t *depth_raw = (uint16_t *)((uint8_t *)depthheader + sizeof(NVPFM_USB_IMAGE_HEADER));
                    if (info->camparam.size() > 0 && info->rgbwidth > 0 && info->rgbheight > 0)
                    {
                        s_nvpfm_camera_param param = info->camparam.at(0);
                        if (info->aligndata == NULL)
                            info->aligndata = (uint16_t *)malloc(info->rgbwidth * info->rgbheight * sizeof(uint16_t));
                        info->transformer->compute_depth_rgb_align(depth_raw,
                                                                   depthheader->width,
                                                                   depthheader->height,
                                                                   info->aligndata,
                                                                   info->rgbwidth, info->rgbheight, NULL,
                                                                   param.left_ir_focus,
                                                                   param.left_ir_photocenter,
                                                                   param.color_focus,
                                                                   param.color_photocenter,
                                                                   param.left2color_matrix);

                        event_data *evd = (event_data *)SOLO_Write(info->alignframequeue);
                        if (!evd)
                        {
                            SOLO_Read_Over(info->depthqueue);
                            continue;
                        }
                        //
                        uint32_t dlen = info->rgbwidth * info->rgbheight * sizeof(uint16_t);
                        evd->init(NVP_EV_DEPTH_ALIGN_RGB, dlen);
                        evd->frame_height = info->rgbheight;
                        evd->frame_width = info->rgbwidth;
                        evd->dev_name = info->rawdevinfo.usb_camera_name;
                        // d= (uint16_t *)((uint8_t *)depthheader + sizeof(NVPFM_USB_IMAGE_HEADER));
                        memcpy(evd->frame_data->data(), info->aligndata, evd->frame_width * evd->frame_height * sizeof(uint16_t));
                        SOLO_Write_Over(info->alignframequeue);

                        if (this_->get_event_enabled(nvpfm_events::NVP_EV_DEPTH_ALIGN_RGB_PSEUDO))
                        {
                            // pesudo
                            evd = (event_data *)SOLO_Write(info->alignframequeue);
                            if (!evd)
                            {
                                SOLO_Read_Over(info->depthqueue);
                                continue;
                            }

                            uint32_t dlen = info->rgbheight * info->rgbwidth;
                            evd->init(NVP_EV_DEPTH_ALIGN_RGB_PSEUDO, dlen * 3);
                            auto &frame_data = *evd->frame_data;
                            evd->frame_height = info->rgbheight;
                            evd->frame_width = info->rgbwidth;
                            evd->dev_name = info->rawdevinfo.usb_camera_name;
                            for (int i = 0; i < dlen; ++i)
                            {
                                uint16_t depth_v = info->aligndata[i];
                                const PaletteUnit &pu = g_pallete[depth_v];
                                frame_data[i * 3] = pu.r;
                                frame_data[i * 3 + 1] = pu.g;
                                frame_data[i * 3 + 2] = pu.b;
                            }
                            SOLO_Write_Over(info->alignframequeue);
                        }
                        // printf("Depth data is written!\n");
                    }
                }

                SOLO_Read_Over(info->depthqueue);
            }
            COMMONUSLEEP(sleep_time);
        }
    }
    void dev_thread_loop(void *para)
    {
        static constexpr int sleep_time = 1000 * 1000;
        DEVICEINFO *info = static_cast<DEVICEINFO *>(para);
        info->container = this;
        if (!info->fm)
        {
            nvpfm_info_printf("construct a new nvpfm!\n");
            info->fm = std::make_unique<nvpfm>(&info->rawdevinfo, &py_nvpfm::event_cb, info);
            info->transformer = std::make_unique<depthtransformer>();
        }

        info->align_thread = std::thread(std::bind(&py_nvpfm::align_thread_loop, this, info));

        if (info->willrun)
        {
            nvpfm_info_printf("will init nvpfm for device %s \n", info->rawdevinfo.usb_camera_name);
            init(info);
        }
        info->initialized.store(true, std::memory_order::memory_order_release);
        // when will run going to be a false?
        while (info->willrun)
        {
            s_nvpfm_get_sensor_config_ret cfg;
            // this will crash
            if (NVPTL_OK == info->fm->get_sensorcfg(&cfg))
            {
                if (!cfg.ret)
                {
                }
            }
            // find a place to insert.
            while (info->willrun && info->camparam.size() == 0)
            {
                s_nvpfm_camera_param para;
                if (NVPTL_OK == info->fm->get_camera_param(&para))
                {
                    nvpfm_info_printf("got parameter!\n");
                    info->camparam.push_back(para);
                    break;
                }
                else
                {
                    nvpfm_warn_printf("fail to get camera parameter!\n");
                    COMMONUSLEEP(sleep_time);
                }
            }
            COMMONUSLEEP(sleep_time);
        }
    }
    // 逻辑有问题
    void enum_thread_loop()
    {
        NVPTL_DEVICE_INFO *device;
        while (!m_quit.load())
        {
            int total = 0;
            if (NVPTL_OK == nvptl_enum_sync(&total, &device))
            {
                if (total <= 0)
                {
                    continue;
                }
                NVPTL_DEVICE_INFO *tmp_dev = device;
                {
                    std::unique_lock<std::mutex> lg(m_mutex);
                    while (tmp_dev) //&&0==strcmp("falcon-1-3",tmp_dev->usb_camera_name))
                    {
                        auto it = m_dev_infos.find(tmp_dev->usb_camera_name);
                        if (it != m_dev_infos.end())
                        {
                            tmp_dev = tmp_dev->next;
                            continue;
                        }
                        nvpfm_info_printf("no FC1:%s in map,so create new camera instance\n", tmp_dev->usb_camera_name);
                        // note the difference between new and malloc.
                        DEVICEINFO *info = new DEVICEINFO;
                        info->rawdevinfo = *tmp_dev;

                        info->willrun.store(true, default_order);
                        // info->camparam = new std::vector<s_nvpfm_camera_param>();
                        auto tmp_vec = std::vector<s_nvpfm_camera_param>();
                        // the data in camparam will be released automatically.
                        info->camparam.swap(tmp_vec);
                        // modify to 10.
                        info->framequeue = Create_Ring_Queue(32, sizeof(event_data));
                        info->alignframequeue = Create_Ring_Queue(4, sizeof(event_data));
                        info->depthqueue = Create_Ring_Queue(4, 1280 * 800 * 2 + sizeof(NVPFM_USB_IMAGE_HEADER));
                        nvpfm_info_printf("Will setup device!\n");
                        info->dev_thread = std::thread(std::bind(&py_nvpfm::dev_thread_loop, this, info));
                        m_dev_infos.insert(std::make_pair(tmp_dev->usb_camera_name, info));
                        info->initialized.store(false, default_order);
                        tmp_dev = tmp_dev->next;
                    }
                }
                nvptl_freedevices(tmp_dev);
            }
            COMMONUSLEEP(500 * 1000);
        }
        m_enum_quited.store(true, default_order);
        nvpfm_info_printf("Enum thread quit!\n");
    }

    // enable/disable events
    // block on the select, if wake up, send data or sth.
    int init_and_start(const std::string &log_path, uint32_t log_size)
    {
        setup_signal();
        m_valid.store(true, std::memory_order::memory_order_relaxed);
        printf("will save log:%s\n", log_path.c_str());
        nvpfm_init(log_path.c_str(), log_size);
        // 创建枚举线程
        std::thread enum_thread(std::bind(&py_nvpfm::enum_thread_loop, this));
        enum_thread.detach();

        return 0;
    }
    // this function will call sleep internally
    // !!!! 实现存在问题：仅返回一个设备信息
    std::vector<s_nvpfm_dev_info> get_dev_info()
    {
        std::vector<s_nvpfm_dev_info> results;
        for (int i = 0; i < 5; i++)
        {
            {
                std::unique_lock<std::mutex> lg(m_mutex);
                results.reserve(m_dev_infos.size());
                for (std::unordered_map<std::string, DEVICEINFO *>::iterator it = m_dev_infos.begin(); it != m_dev_infos.end(); it++)
                {
                    DEVICEINFO *tmpinfo = it->second;
                    if (!tmpinfo->initialized.load(std::memory_order::memory_order_acquire))
                    {
                        nvpfm_error_printf("fail to load memory_order_acquire!\n");
                        continue;
                    }
                    // std::unique_ptr<s_nvpfm_dev_info> info(new s_nvpfm_dev_info);
                    s_nvpfm_dev_info info;
                    if (NVPTL_OK == tmpinfo->fm->get_devinfo(&info))
                    {
                        results.emplace_back(info);
                    }
                    else
                    {
                        nvpfm_error_printf("Fail to get camera info!!!\n");
                        continue;
                    }
                }
            }
            if (results.size() > 0)
            {
                break;
            }
            else
            {
                sleep(1);
            }
        }
        return results;
    }

    /// @brief set the certain type data enabled
    /// @param evt event data type
    /// @param en enable
    void set_event_enabled(nvpfm_events evt, bool en)
    {
        if (en)
        {
            m_modes.fetch_or((1 << evt), default_order);
        }
        else
        {
#ifdef DEBUG
            printf("Mode :%d is disabled!\n", evt);
#endif
            m_modes.fetch_and(~(1 << evt), default_order);
        }
    }

    /// @brief read cam param from DEVICEINFO.
    ///
    ///
    std::vector<s_nvpfm_camera_param> read_cam_param()
    {
        std::vector<s_nvpfm_camera_param> results;
        for (auto it = m_dev_infos.begin(); it != m_dev_infos.end(); it++)
        {
            DEVICEINFO *info = it->second;
            if (info->camparam.size() > 0)
            {
                results.emplace_back(info->camparam[0]);
                break;
            }
        }
        return results;
    }

    /// @brief using select and wakeup if it gets data.
    /// could add timeout parameter.
    /// return value优化 改移动语义....
    std::vector<event_data> read_data()
    {
        std::vector<event_data> results;
        {
            std::unique_lock<std::mutex> lg(m_mutex);
            for (auto it = m_dev_infos.begin(); it != m_dev_infos.end(); it++)
            {
                DEVICEINFO *info = it->second;
                // 未初始化完成, 或者设备因出错退出
                if (!info->initialized.load(std::memory_order::memory_order_acquire))
                {
                    continue;
                }
                event_data *ev_data = nullptr;
                while ((ev_data = (event_data *)SOLO_Read(info->framequeue)))
                {
                    if (!ev_data->valid)
                    {
                        nvpfm_warn_printf("Got an invalid data!\n");
                    }
                    results.push_back(*ev_data);
                    event_data::move_source(results.back(), *ev_data);
                    results.back().contrust_numpy_array();
                    SOLO_Read_Over(info->framequeue);
                }

                while ((ev_data = (event_data *)SOLO_Read(info->alignframequeue)))
                {
                    if (!ev_data->valid)
                    {
                        nvpfm_warn_printf("Got an invalid data!\n");
                    }
                    results.push_back(*ev_data);
                    event_data::move_source(results.back(), *ev_data);
                    results.back().contrust_numpy_array();
                    SOLO_Read_Over(info->alignframequeue);
                }
            }
        }
        return results;
    }

    void stop_all_dev_threads()
    {
        std::unique_lock<std::mutex> lg(m_mutex);
        std::vector<DEVICEINFO *> infos;
        for (auto it = m_dev_infos.begin(); it != m_dev_infos.end(); it++)
        {
            infos.push_back(it->second);
        }
        // 防止退不出
        if (infos.empty())
        {
            m_valid.store(false, default_order);
            return;
        }
        for (auto it = infos.begin(); it != infos.end(); ++it)
        {
            release(*it, false);
        }
    }

    /// @brief stop the sdk
    /// teminate all threads
    /// destroy all allocated resources.
    ///
    void stop()
    {
        // quit enum thread.
        m_quit.store(true, default_order);
        while (!m_enum_quited.load(default_order))
        {
            std::this_thread::yield();
        }
        // all device thread quit.
        stop_all_dev_threads();
        nvpfm_info_printf("waiting all thread exit!\n");
        // make sure sdk is stopped.
        while (m_valid.load(std::memory_order::memory_order_relaxed))
        {
            std::this_thread::yield();
        }
        // call deinit, when all deivces was removed,deinit is finished.
        nvpfm_deinit();
        nvpfm_info_printf("All threads is stopped!\n");
    }

private:
    std::atomic<bool> m_quit;
    std::mutex m_mutex;
    std::unordered_map<std::string, DEVICEINFO *> m_dev_infos;
    std::atomic<bool> m_valid;
    std::atomic<int> m_modes;
    std::atomic<int> m_enum_quited;
};

#define ARR_LEN(a) (sizeof(a) / sizeof(a[0]))

/// PythonSetup
PYBIND11_MODULE(falcon_sdk, m)
{
    // 就像lua bridge一样，自动转

    // data
    pyb::class_<py_nvpfm::event_data>(m, "event_data")
        .def(pyb::init())
        .def_readonly("dev_name", &py_nvpfm::event_data::dev_name)
        .def_readonly("event_type", &py_nvpfm::event_data::ev_type)
        .def_readonly("fps", &py_nvpfm::event_data::fps_data)
        .def_readonly("frame_width", &py_nvpfm::event_data::frame_width)
        .def_readonly("frame_height", &py_nvpfm::event_data::frame_height)
        .def_readonly("frame_data_array", &py_nvpfm::event_data::frame_data_array)
        .def_readonly("imu_channel", &py_nvpfm::event_data::imu_channel)
        .def_readonly("imu_data_number", &py_nvpfm::event_data::imu_data_number)
        .def_readonly("imu_data_type", &py_nvpfm::event_data::imu_data_type)
        .def_readonly("imu_is_factory", &py_nvpfm::event_data::imu_is_factory)
        .def_readonly("imu_factory_data", &py_nvpfm::event_data::imu_factory_data)
        .def_readonly("imu_application_data", &py_nvpfm::event_data::imu_application_data);
    /*
    int irwidth;
            int irheight;
            int rgbwidth;
            int rgbheight;
            float left_ir_focus[2];//0:fx,1:fy
            float left_ir_photocenter[2];//0:px,1:py
            float right_ir_focus[2];
            float right_ir_photocenter[2];
            float color_focus[2];
            float color_photocenter[2];
            float left2right_matrix[12];
            float left2color_matrix[12];
            int result;*/
    pyb::class_<s_nvpfm_camera_param>(m, "s_nvpfm_camera_param")
        .def(pyb::init())
        .def_readonly("irwidth", &s_nvpfm_camera_param::irwidth)
        .def_readonly("irheight", &s_nvpfm_camera_param::irheight)
        .def_readonly("rgbwidth", &s_nvpfm_camera_param::rgbwidth)
        .def_readonly("rgbheight", &s_nvpfm_camera_param::rgbheight)
        .def_property_readonly("left_ir_focus", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->left_ir_focus, ARR_LEN(this_->left_ir_focus)); })
        .def_property_readonly("left_ir_photocenter", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->left_ir_photocenter, ARR_LEN(this_->left_ir_photocenter)); })
        .def_property_readonly("right_ir_focus", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->right_ir_focus, ARR_LEN(this_->right_ir_focus)); })
        .def_property_readonly("right_ir_photocenter", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->right_ir_photocenter, ARR_LEN(this_->right_ir_photocenter)); })
        .def_property_readonly("color_focus", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->color_focus, ARR_LEN(this_->color_focus)); })
        .def_property_readonly("color_photocenter", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->color_photocenter, ARR_LEN(this_->color_photocenter)); })
        .def_property_readonly("left2right_matrix", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->left2right_matrix, ARR_LEN(this_->left2right_matrix)); })
        .def_property_readonly("left2color_matrix", [](s_nvpfm_camera_param *this_)
                               { return F2VDelegate(this_->left2color_matrix, ARR_LEN(this_->left2color_matrix)); })
        .def_readonly("result", &s_nvpfm_camera_param::result);

    pyb::class_<IMU_APPLICATION_DATA>(m, "IMU_APPLICATION_DATA")
        .def(pyb::init())
        .def_readonly("fx", &IMU_APPLICATION_DATA::fX)
        .def_readonly("fy", &IMU_APPLICATION_DATA::fY)
        .def_readonly("fz", &IMU_APPLICATION_DATA::fZ);

    pyb::class_<IMU_FACTORY_DATA_STRUC> g_factory_data(m, "IMU_FACTORY_DATA_STRUC");
    g_factory_data.def(pyb::init())
        .def_readonly("tem_cali_data", &IMU_FACTORY_DATA_STRUC::fTemCaliData)
        .def_readonly("accel_cali_data", &IMU_FACTORY_DATA_STRUC::stAccelCaliData)
        .def_readonly("gyro_cali_data", &IMU_FACTORY_DATA_STRUC::stGyroCaliData)
        .def_readonly("mag_cali_data", &IMU_FACTORY_DATA_STRUC::stMagnCaliData)
        .def_readonly("timestamp", &IMU_FACTORY_DATA_STRUC::timestamp);

    pyb::class_<IMU_FACTORY_DATA>(m, "IMU_FACTORY_DATA")
        .def(pyb::init())
        .def_readonly("fx", &IMU_FACTORY_DATA::fX)
        .def_readonly("fy", &IMU_FACTORY_DATA::fY)
        .def_readonly("fz", &IMU_FACTORY_DATA::fZ);

    pyb::class_<IMU_APPLICATION_DATA_STRUC> g_applicaiton_data(m, "IMU_APPLICATION_DATA_STRUC");
    g_applicaiton_data.def(pyb::init())
        .def_readonly("tem_cali_data", &IMU_APPLICATION_DATA_STRUC::fTemCaliData)
        .def_readonly("accel_cali_data", &IMU_APPLICATION_DATA_STRUC::stAccelCaliData)
        .def_readonly("gyro_cali_data", &IMU_APPLICATION_DATA_STRUC::stGyroCaliData)
        .def_readonly("mag_cali_data", &IMU_APPLICATION_DATA_STRUC::stMagnCaliData)
        .def_readonly("timestamp", &IMU_APPLICATION_DATA_STRUC::timestamp);

    pyb::class_<py_nvpfm> gnvpfm(m, "py_nvpfm");
    gnvpfm.def(pyb::init())
        .def("stop", &py_nvpfm::stop)
        .def("init_and_start", &py_nvpfm::init_and_start)
        .def("get_dev_info", &py_nvpfm::get_dev_info)                                               // cfy: changed.
        .def("read_data", &py_nvpfm::read_data, pyb::return_value_policy::take_ownership)           // cfy: optimize.
        .def("read_cam_param", &py_nvpfm::read_cam_param, pyb::return_value_policy::take_ownership) // cfy: optimize.
        .def("set_event_enabled", &py_nvpfm::set_event_enabled)
        .def("get_event_enabled", &py_nvpfm::get_event_enabled);

    // enum
    pyb::enum_<py_nvpfm::nvpfm_events>(gnvpfm, "ev_type")
        .value("Depth", py_nvpfm::nvpfm_events::NVP_EV_DEPTH)
        .value("RGB", py_nvpfm::nvpfm_events::NVP_EV_RGB)
        .value("Group", py_nvpfm::nvpfm_events::NVP_EV_GROUP)
        .value("IMU", py_nvpfm::nvpfm_events::NVP_EV_IMU)
        .value("LeftIR", py_nvpfm::nvpfm_events::NVP_EV_LEFT_IR)
        .value("RightIR", py_nvpfm::nvpfm_events::NVP_EV_RIGHT_IR)
        .value("DepthPseudo", py_nvpfm::nvpfm_events::NVP_EV_DEPTH_PSEUDO)
        .value("DepthAlignRGB", py_nvpfm::nvpfm_events::NVP_EV_DEPTH_ALIGN_RGB)
        .value("DepthAlignRGBPseudo", py_nvpfm::nvpfm_events::NVP_EV_DEPTH_ALIGN_RGB_PSEUDO)
        .export_values();

    pyb::enum_<NVPFM_IMAGE_SIZE>(gnvpfm, "NVPFM_IMAGE_SIZE")
        .value("IMAGE_1280_800", IMAGE_1280_800)
        .value("IMAGE_1280_720", IMAGE_1280_720)
        .value("IMAGE_640_480", IMAGE_640_480)
        .value("IMAGE_640_400", IMAGE_640_400)
        .value("IMAGE_320_200", IMAGE_320_200)
        .value("IMAGE_640_360", IMAGE_640_360)
        .value("IMAGE_320_240", IMAGE_320_240)
        .value("IMAGE_960_600", IMAGE_960_600)
        .value("IMAGE_480_300", IMAGE_480_300)
        .value("IMAGE_1600_1200", IMAGE_1600_1200)
        .value("IMAGE_1280_1080", IMAGE_1280_1080)
        .value("IMAGE_1280_960", IMAGE_1280_960)
        .value("IMAGE_800_600", IMAGE_800_600)
        .value("IMAGE_848_480", IMAGE_848_480)
        .value("IMAGE_768_480", IMAGE_768_480)
        .value("IMAGE_1280_480", IMAGE_1280_480)
        .value("IMAGE_1920_1080", IMAGE_1920_1080)
        .value("IMAGE_960_1280", IMAGE_960_1280)
        .value("IMAGE_480_640", IMAGE_480_640)
        .value("IMAGE_UNKNOWN", IMAGE_UNKNOWN);

    pyb::class_<RESCOMBO>(m, "RESCOMBO")
        .def(pyb::init())
        .def_readonly("valid", &RESCOMBO::valid)
        .def_readonly("fps", &RESCOMBO::fps)
        .def_readonly("ispres", &RESCOMBO::ispres)
        .def_readonly("sensorres", &RESCOMBO::sensorres);

    pyb::class_<s_nvpfm_dev_info>(m, "s_nvpfm_dev_info")
        .def(pyb::init())
        .def_readonly("sn", &s_nvpfm_dev_info::sn)
        .def_readonly("product", &s_nvpfm_dev_info::product)
        .def_readonly("cpu_net_type", &s_nvpfm_dev_info::cpu_net_type)
        .def_readonly("comm_type", &s_nvpfm_dev_info::comm_type)
        .def_readonly("projector_type", &s_nvpfm_dev_info::projector_type)
        .def_readonly("imu_type0", &s_nvpfm_dev_info::imu_type0)
        .def_readonly("imu_type1", &s_nvpfm_dev_info::imu_type1)
        .def_readonly("software_version", &s_nvpfm_dev_info::software_version)
        .def_readonly("cpu_temperature", &s_nvpfm_dev_info::cpu_temperature)
        .def_readonly("projector_temperature", &s_nvpfm_dev_info::projector_temperature)
        .def_property_readonly("sensor", [](s_nvpfm_dev_info *this_)
                               { return I2BDelegate(this_->sensor, ARR_LEN(this_->sensor)); })
        .def_property_readonly("depth", [](s_nvpfm_dev_info *this_)
                               { return I2BDelegate(this_->depth, ARR_LEN(this_->depth)); })
        .def_property_readonly("depth0", [](s_nvpfm_dev_info *this_)
                               { return I2BDelegate(this_->depth0, ARR_LEN(this_->depth0)); })
        .def_property_readonly("depth1", [](s_nvpfm_dev_info *this_)
                               { return I2BDelegate(this_->depth1, ARR_LEN(this_->depth1)); })
        .def_property_readonly("rgb", [](s_nvpfm_dev_info *this_)
                               { return I2BDelegate(this_->rgb, ARR_LEN(this_->rgb)); })
        .def_property_readonly("rgb0", [](s_nvpfm_dev_info *this_)
                               { return I2BDelegate(this_->rgb0, ARR_LEN(this_->rgb0)); })
        .def_property_readonly("rgb1", [](s_nvpfm_dev_info *this_)
                               { return I2BDelegate(this_->rgb1, ARR_LEN(this_->rgb1)); })
        .def_readonly("irsupported", &s_nvpfm_dev_info::irsupported)
        .def_readonly("colorsupported", &s_nvpfm_dev_info::colorsupported);
}
