#include <vector>
#include <memory>
#include <string>
#include <memory.h>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "commondef.h"
// #include "transferlayer/nvptl_serial.h"
#include "transferlayer/commondata.h"
#include "utils.h"
#ifdef _LINUX
#include <asm-generic/ioctl.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#endif
#define MAXBUFFERSIZE 65536
#define UPGRADE_MAX_BLK_SIZE (1024 * 46)
static const VIDPID VPS[] = {
    {0x4E56, 0x594E}};

#include "ring_queue.h"
typedef enum {
  SERIAL_READ,
  SERIAL_WRITE,
  SERIAL_CLOSE,
} SERIAL_RWTYPE;
typedef struct {
  NVPTL_INSTANCE *handle;
  HANDLE closeevent;
  SERIAL_RWTYPE type;
} SERIAL_RW;
static Ring_Queue *s_rwqueue = NULL;

#if defined(_WINDOWS)

#include "windows.h" // CreateFile GetTickCount64
#include "tchar.h"   // _sntprintf _T

#include <Setupapi.h> //SetupDiGetClassDevs Setup*
#include <initguid.h> //GUID

#pragma comment(lib, "setupapi.lib")

using namespace std;

#ifndef GUID_DEVINTERFACE_COMPORT
DEFINE_GUID(GUID_DEVINTERFACE_COMPORT, 0x86E0D1E0L, 0x8089, 0x11D0, 0x9C, 0xE4, 0x08, 0x00, 0x3E, 0x30, 0x1F, 0x73);
#endif

struct SerialPortInfo {
  std::string portName;
  std::string description;
};

std::string wstringToString(const std::wstring &wstr) {
  // https://stackoverflow.com/questions/4804298/how-to-convert-wstring-into-string
  if (wstr.empty()) {
    return std::string();
  }

  int size = WideCharToMultiByte(CP_ACP, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL);
  std::string ret = std::string(size, 0);
  WideCharToMultiByte(CP_ACP, 0, &wstr[0], (int)wstr.size(), &ret[0], size, NULL, NULL); // CP_UTF8

  return ret;
}

bool enumDetailsSerialPorts(vector<SerialPortInfo> &portInfoList) {
  // https://docs.microsoft.com/en-us/windows/win32/api/setupapi/nf-setupapi-setupdienumdeviceinfo

  bool bRet = false;
  SerialPortInfo m_serialPortInfo;

  std::string strFriendlyName;
  std::string strPortName;

  HDEVINFO hDevInfo = INVALID_HANDLE_VALUE;

  // Return only devices that are currently present in a system
  // The GUID_DEVINTERFACE_COMPORT device interface class is defined for COM ports. GUID
  // {86E0D1E0-8089-11D0-9CE4-08003E301F73}
  hDevInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_COMPORT, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);

  if (INVALID_HANDLE_VALUE != hDevInfo) {
    SP_DEVINFO_DATA devInfoData;
    // The caller must set DeviceInfoData.cbSize to sizeof(SP_DEVINFO_DATA)
    devInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    for (DWORD i = 0; SetupDiEnumDeviceInfo(hDevInfo, i, &devInfoData); i++) {
      // get pid/vid,USB\VID_4E56&PID_594E&REV_0409&MI_00
      std::string pidvid = "";
      TCHAR hardwareid[256];
      SetupDiGetDeviceRegistryProperty(hDevInfo, &devInfoData, SPDRP_HARDWAREID, NULL, (PBYTE)hardwareid,
                                       sizeof(hardwareid), NULL);
#ifdef UNICODE
      pidvid = wstringToString(hardwareid);
#else
      pidvid = std::string(portName);
#endif
      bool gotpidvid = false;
      for (int i = 0; i < sizeof(VPS) / sizeof(VIDPID); i++) {
        char tmpvid[64], tmppid[64];
        sprintf(tmpvid, "VID_%04X", VPS[i].VID);
        sprintf(tmppid, "PID_%04X", VPS[i].PID);
        if (pidvid.find(tmpvid) != string::npos && pidvid.find(tmppid) != string::npos) {
          gotpidvid = true;
          break;
        }
      }
      if (!gotpidvid)
        continue;

      // get port name
      TCHAR portName[256];
      HKEY hDevKey = SetupDiOpenDevRegKey(hDevInfo, &devInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
      if (INVALID_HANDLE_VALUE != hDevKey) {
        DWORD dwCount = 255; // DEV_NAME_MAX_LEN
        RegQueryValueEx(hDevKey, _T("PortName"), NULL, NULL, (BYTE *)portName, &dwCount);
        RegCloseKey(hDevKey);
      }

      // get friendly name
      TCHAR fname[256];
      SetupDiGetDeviceRegistryProperty(hDevInfo, &devInfoData, SPDRP_FRIENDLYNAME, NULL, (PBYTE)fname,
                                       sizeof(fname), NULL);

#ifdef UNICODE
      strPortName = wstringToString(portName);
      strFriendlyName = wstringToString(fname);
#else
      strPortName = std::string(portName);
      strFriendlyName = std::string(fname);
#endif
      // remove (COMxx)
      strFriendlyName = strFriendlyName.substr(0, strFriendlyName.find(("(COM")));

      m_serialPortInfo.portName = strPortName;
      m_serialPortInfo.description = strFriendlyName;
      portInfoList.push_back(m_serialPortInfo);
    }

    if (ERROR_NO_MORE_ITEMS == GetLastError()) {
      bRet = true; // no more item
    }
  }

  SetupDiDestroyDeviceInfoList(hDevInfo);

  return bRet;
}
#else

#include <linux/limits.h>
#include <sys/stat.h>
#include <dirent.h>
#include <assert.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <atomic>
#include <functional>
#include <sys/epoll.h>
// 可能会开多个 ....
#define EPOLL_NUM 8
static libusb_context *s_context = NULL;
/*
class nvp_serial
{
    int fd_ = -1;
    std::string port_;
    bool valid_ = false;
    std::thread ev_loop_thread_;
    std::atomic<bool> quit_;
    static constexpr std::memory_order default_order = std::memory_order::memory_order_relaxed;
    static constexpr int EPOLL_NUM = 8;
    int efd_;
    std::atomic<int> status_;
    // callback args
    NVPTL_RECV_FRAME_CALLBACK frame_cb_ = nullptr;
    NVPTL_DEVICE_HANDLE *inst_ = nullptr;
    void *ud_ = nullptr;

    std::vector<uint8_t> recv_buf_;
    uint8_t *buff_data_ = nullptr;
    int buf_offset_ = 0;
    // options is not avaliable now.
    // options area

public:
    explicit nvp_serial(const std::string &port) : port_(port)
    {
        quit_.store(false, default_order);
        recv_buf_.resize(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);
        buff_data_ = recv_buf_.data();
    }

    ~nvp_serial()
    {
#if 0
        ::close(fd_);
        if (ev_loop_thread_.joinable())
        {
            ev_loop_thread_.join();
        }
#endif
    }

#define printf(fmt, ...) printf(fmt, ##__VA_ARGS__)
    bool setup_serial(speed_t baudRate, int dataBits, int stopBits, int parity)
    {
        struct termios serialAttr;
        if (tcgetattr(fd_, &serialAttr) != 0)
        {
            printf("Failed to get serial port attributes\n");
            ::close(fd_);
            return false;
        }

        cfsetospeed(&serialAttr, baudRate);
        cfsetispeed(&serialAttr, baudRate);

        serialAttr.c_cflag &= ~CSIZE;
        serialAttr.c_cflag |= dataBits;

        if (stopBits == 1)
        {
            serialAttr.c_cflag &= ~CSTOPB;
        }
        else if (stopBits == 2)
        {
            serialAttr.c_cflag |= CSTOPB;
        }

        serialAttr.c_cflag &= ~(PARENB | PARODD);
        if (parity != 0)
        {
            serialAttr.c_cflag |= PARENB;
            if (parity == 2)
            {
                serialAttr.c_cflag |= PARODD;
            }
        }

        if (tcsetattr(fd_, TCSANOW, &serialAttr) != 0)
        {
            printf("Failed to set serial port attributes");
            ::close(fd_);
            return false;
        }
        // tcflush(serialFD, TCIOFLUSH);
        return true;
    }

    void do_data(int recvd)
    {

        static constexpr long header_padding = sizeof(NVPTL_USBHeaderDataPacket);
        buf_offset_ += recvd;
        // assert(buff_offset_ <= USB_PACKET_MAX_SIZE);
        // nothing to do
        if (buf_offset_ < header_padding)
        {
            // too less.
            return;
        }
        if (strncmp("NEXT_VPU", (const char *)buff_data_, 8) != 0)
        {
            int searched_idx = -1;
            for (int i = 0; i < buf_offset_ - 8; ++i)
            {
                if (memcmp(&buff_data_[i], "NEXT_VPU", 8) != 0)
                {
                    continue;
                }
                buf_offset_ -= i;
                memmove(buff_data_, buff_data_ + i, buf_offset_);
                searched_idx = i;
                break;
            }
            if (searched_idx < 0)
            {
                printf("Can't find header in this packet\n");
                buf_offset_ = 0;
                return;
            }
        }
        int cur_pack_len = ((NVPTL_USBHeaderDataPacket *)buff_data_)->len;
        int total_len = cur_pack_len + header_padding;
        if (buf_offset_ >= total_len)
        {
            // the callback.
            frame_cb_(inst_, buff_data_, total_len, ud_);
            buf_offset_ -= total_len;
            if (buf_offset_ > 0)
            {
                memmove(buff_data_, buff_data_ + total_len, buf_offset_);
            }
        }
    }

    inline void epoll_quit()
    {
        epoll_ctl(efd_, EPOLL_CTL_DEL, fd_, 0);
    }

    int do_read()
    {
        // do sth.
        int ret = 0;
        for (;;)
        {
            int recvd = read(fd_, buff_data_ + buf_offset_, USB_PACKET_MAX_SIZE - buf_offset_);
            if (recvd > 0)
            {
                do_data(recvd);
            }
            else if (recvd == -1)
            {
                // 读完了（无数据可读）
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    break;
                }
                else if (errno == EINTR) // 读的过于频繁
                {
                    continue;
                }
                else
                {
                    // 失败
                    epoll_quit();
                    ret = -1;
                    printf("Read from port failed!\n");
                    break;
                }
            }
            else if (recvd == 0)
            {
                epoll_quit();
                ret = -1;
                break;
            }
        }
        return ret;
    }

    int do_send(struct epoll_event *ev)
    {
        std::vector<uint8_t> *data = reinterpret_cast<std::vector<uint8_t> *>(ev->data.ptr);
        if (!data)
        {
            printf("Exit event is triggered!\n");
            epoll_quit();
            return -2;
        }
        uint8_t *d = data->data();
        uint32_t len = data->size();
        int send_offset = 0;
        int res = 0;
        for (;;)
        {
            int sent = write(fd_, d + send_offset, len - send_offset);
            if (sent > 0)
            {
                send_offset += sent;
                if (send_offset == len)
                {
                    break;
                }
            }
            else if (sent == -1)
            {
                // 缓冲区满，重试
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                {
                    continue;
                }
                else if (errno == EINTR)
                {
                    continue;
                }
                else
                {
                    epoll_quit();
                    res = -1;
                }
            }
            else if (sent == 0)
            {
                epoll_quit();
                res = -1;
            }
        }
    }

    void event_loop()
    {
        int n;
        struct epoll_event events[EPOLL_NUM];
        // using CONFIGURED as RUNING state.
        status_.store(CONFIGURED, default_order);
        while (!quit_.load(default_order))
        {
            n = epoll_wait(efd_, events, EPOLL_NUM, -1);
            if (-1 == n)
            {
                if (errno == EAGAIN)
                {
                    continue;
                }
                printf("EPOLL Failed!\n");
                break;
            }
            // event process.
            for (int i = 0; i < n; ++i)
            {
                if (events[i].events & EPOLLOUT)
                {
                    if (do_send(&events[i]) < 0)
                    {
                        printf("Send failed!\n");
                        break;
                    }
                }
                else if (events[i].events & EPOLLIN)
                {
                    int r = 0;
                    if ((r = do_read()) < 0)
                    {
                        if (r != -2)
                        {
                            printf("Read failed!\n");
                        }
                        break;
                    }
                }
                else if ((events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) || (events[i].events & EPOLLRDHUP))
                {
                    // error.
                    printf("Camera %s epoll stopped", port_.c_str());
                    break;
                }
                else
                {
                    printf("Unknown event %d polled.", events[i].events);
                }
            }
        }
        status_.store(STOPPED, std::memory_order_release);
    }

    void set_frame_cb(NVPTL_RECV_FRAME_CALLBACK cb, NVPTL_DEVICE_HANDLE *inst, void *ud)
    {
        frame_cb_ = cb;
        inst_ = inst;
        ud_ = ud;
    }

    bool open()
    {
        fd_ = ::open(port_.c_str(), O_RDWR);
        if (-1 == fd_)
        {
            printf("Failed to open port %s", port_.c_str());
            return false;
        }
        setup_serial(115200, 8, 1, 0);
        efd_ = epoll_create(EPOLL_NUM);
        struct epoll_event evt;
        evt.data.ptr = NULL;
        evt.events = EPOLLIN | EPOLLET;
        if ((epoll_ctl(efd_, EPOLL_CTL_ADD, fd_, &evt)) == 0)
        {
            // inst->recvframecallback = callback;
            status_.store(STARTED, default_order);
            // 启动ev_loop
            this->ev_loop_thread_ = std::thread(std::bind(&nvp_serial::ev_loop_thread_, this));
            printf("camera %s epoll started", this->port_.c_str());
        }
        valid_ = true;
        return valid_;
    }
    // the close operation.
    void close()
    {
        if (status_.load(default_order) == CONFIGURED)
        {
            struct epoll_event evt;
            evt.data.ptr = NULL;
            evt.events = EPOLLET | EPOLLOUT;
            epoll_ctl(efd_, EPOLL_CTL_MOD, fd_, &evt);
        }
        if (ev_loop_thread_.joinable())
        {
            ev_loop_thread_.join();
        }
        ::close(fd_);
        fd_ = -1;
        status_.store(INITIAL, default_order);
        valid_ = false;
    }

    int async_send_data(uint8_t *buffer, size_t len)
    {
        if (status_.load(default_order) != CONFIGURED)
        {
            printf("Serial is not ready to transfer!\n");
            return -1;
        }
        struct epoll_event evt;
        std::vector<uint8_t> *pt;
        evt.data.ptr = pt = new std::vector<uint8_t>(len);
        memcpy(pt->data(), buffer, len);
        evt.events = EPOLLOUT | EPOLLET;
        int ret = epoll_ctl(efd_, EPOLL_CTL_MOD, fd_, &evt);
        if (ret)
        {
            delete pt;
            // 此处不主动quit，等待evloop 触发
            printf("Failed to add epoll event.\n");
        }
        return ret;
    }
    static libusb_context *g_context;
};
*/
// 找到tty*，然后打开/dev/tty*
int is_slink(const char *file) {
  struct stat file_stat = {0};

  if (lstat(file, &file_stat) == -1)
    return 0;

  return S_ISLNK(file_stat.st_mode) ? 1 : 0;
}

static std::string dev_is_serial(libusb_device *dev) {
  struct libusb_device_descriptor desc;
  libusb_get_device_descriptor(dev, &desc);
  if (desc.bDeviceClass == LIBUSB_CLASS_HUB) {
    return "";
  }
  int gotit = 0;
  for (unsigned int i = 0; i < sizeof(VPS) / sizeof(VIDPID); i++) {
    if (desc.idVendor == VPS[i].VID && desc.idProduct == VPS[i].PID) {
      gotit = 1;
      break;
    }
  }
  if (!gotit) {
    return "";
  }
  // 总线编号
  int bus_num = libusb_get_bus_number(dev);
  std::string port_num_path_base = "/" + std::to_string(bus_num);
  std::string port_num_path = std::to_string(bus_num);
  std::array<uint8_t, 32> port_num;
  port_num.fill(0);
  // bus-port.port.port:config.interface
  // 1-1/1-1:1.0/ttyUSB0/tty/ttyUSB0
  // 返回经过的端口数量，对于挂载在3.0集线器上的设备，这个数字可能就不是1，所以存在多级拼接
  int port_num_len = libusb_get_port_numbers(dev, port_num.data(), 32);
  // assert(port_num_len == 1);
  // printf("port num length:%d\n", port_num_len);
  if (port_num_len) {
    port_num_path = port_num_path + "-";
    port_num_path_base = port_num_path_base + "-";
    for (int i = 0; i < port_num_len; ++i) {
      port_num_path_base = port_num_path_base + std::to_string(port_num[i]);
      port_num_path = port_num_path + std::to_string(port_num[i]);
      if (i + 1 < port_num_len) {
        port_num_path_base = port_num_path_base + ".";
        port_num_path = port_num_path + ".";
      }
    }
  }
  port_num_path_base = port_num_path_base + "/";
  DIR *dir;
  if ((dir = opendir("/sys/class/tty")) == NULL) {
    return "";
  }
  struct dirent *ptr;
  std::string file;
  std::array<char, PATH_MAX> link_path;

  std::string ttyname;
  std::string res;
  while ((ptr = readdir(dir)) != NULL) {
    std::string dname = ptr->d_name;
    if (dname == "." || dname == "..") /// current dir OR parrent dir
      continue;
    // 某些平台下面可能不叫tty
    file = "/sys/class/tty/" + dname;
    link_path.fill(0);
    // problem: 复合设备时，可能存在多个tty实例，可能出现多个匹配, 需要测试或者明确
    // 盲猜复合设备可能会产生多个匹配，此时就很难搞了.... 因为无法确认复合设备中的哪个端口是干啥用的（或者暂时不知道）
    if (is_slink(file.c_str()) && readlink(file.c_str(), link_path.data(), PATH_MAX)) {
      if (strstr(link_path.data(), port_num_path_base.c_str())) {
        // printf("Dev is tty! name:%s \n", dname.c_str());
        // printf("raw path:%s, link path:%s\n", file.c_str(), link_path.data());
        res = "/dev/" + dname;
      }
    }
  }

  // printf("port name path base:%s\n", port_num_path_base.c_str());
  // printf("port name path:%s\n", port_num_path.c_str());
  closedir(dir);
  return res;
}

#endif

static THREADHANDLE s_serial_thread;
static bool s_serial_running = true;

static void nvptl_recv_frame_callback(NVPTL_DEVICE_HANDLE handle, uint8_t *data, unsigned long len, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  if (inst->recvframecallback != NULL) {
    inst->recvframecallback(handle, data, len, userdata);
  }
}

#ifdef _WINDOWS
static void WINAPI transfercallback(DWORD dwErrorCode,
                                    DWORD dwNumberOfBytesTransfered,
                                    LPOVERLAPPED lpOverlapped);
static unsigned __stdcall serial_rw_thread(void *param) {
#else
static void *serial_rw_thread(void *param) {
#endif

#ifndef _WINDOWS
  struct epoll_event events[EPOLL_NUM];
  int efd_ = epoll_create(EPOLL_NUM);
#endif

  while (s_serial_running) {
    SERIAL_RW *p = (SERIAL_RW *)SOLO_Read(s_rwqueue);
    if (p) {
      NVPTL_INSTANCE *nvptlhandle = (NVPTL_INSTANCE *)p->handle;
      if (p->type == SERIAL_READ) {
#ifdef _WINDOWS
        // 建立一个重叠结构
        ZeroMemory(&nvptlhandle->wrOverlapped, sizeof(nvptlhandle->wrOverlapped));
        nvptlhandle->wrOverlapped.Pointer = nvptlhandle;

        if (ReadFileEx(nvptlhandle->device_usb_handle, nvptlhandle->usb_buf, MAXBUFFERSIZE, &nvptlhandle->wrOverlapped, transfercallback)) {
        } else {
          printf("fail to read file:%d\n", GetLastError());
        }
#else
        struct epoll_event evt;
        evt.data.ptr = nvptlhandle;
        evt.events = EPOLLIN | EPOLLET;
        if ((epoll_ctl(efd_, EPOLL_CTL_ADD, nvptlhandle->device_serial_handle, &evt)) == 0) {
          printf("camera %s epoll started\n", nvptlhandle->devinfo.usb_camera_name);
        }
#endif
      } else if (p->type == SERIAL_CLOSE) {
      }
      SOLO_Read_Over(s_rwqueue);
    }
#ifdef _WINDOWS
    SleepEx(30, TRUE);
#else
    int n = epoll_wait(efd_, events, EPOLL_NUM, 15);
    if (-1 == n) {
      if (errno == EAGAIN) {
        printf("EAGAIN!\n");
        continue;
      }
      printf("EPOLL Failed!\n");
      break;
    }
    // printf("%lld,n:%d\n",time(NULL), n);
    //  event process.
    for (int i = 0; i < n; ++i) {
      NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)events[i].data.ptr;
      if (events[i].events & EPOLLOUT) {
        /*  if (do_send(&events[i]) < 0)
          {
              printf("Send failed!\n");
              break;
          }*/
      } else if (events[i].events & EPOLLIN) {
        while (true) {
          int recvd = read(inst->device_serial_handle, inst->usb_buf + inst->index, 64 * 1024);
          if (recvd > 0) {
            // printf("got:%d\n", recvd);

            ////////////////////here,we compose packets to ok frame
            inst->index += recvd;
            // printf("now buffer data length:%d\n", inst->index);
            if ((long)inst->index >= (long)sizeof(NVPTL_USBHeaderDataPacket)) {
              // printf("data len is more than head len:%d\n", sizeof(NVPTL_USBHeaderDataPacket));
              if (0 == strncmp("NEXT_VPU", (const char *)inst->usb_buf, 8)) {
                // printf("got frame start!\n");
                inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
                if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
                  // printf("1got frame!\n");
                  nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);

                  inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
                  // printf("buffer data left:%d\n", inst->index);
                  if (inst->index > 0) {
                    printf("after frame recved,still has %d bytes\n", inst->index);
                    //   printf("move data to front\n");
                    memmove(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
                  } else {
                    //     printf("no data left!\n");
                    //	printf("no more data left:index=%d!\n", inst->index);
                  }
                } else {
                  //  printf("1,buffer data len smaller than frame size:%d\n", (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)));
                  //    printf("1,need more data!!!%d\n",(inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))-inst->index);
                  //	printf("index too small tobe a complete frame:want:%d,has:%d\n", (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
                }
              } else {
                printf("first is not NEXT_VPU!will search\n");
              searchnextvpu:
                if ((long)inst->index >= (long)sizeof(NVPTL_USBHeaderDataPacket)) {
                  int searchednextvpu = 0;
                  for (unsigned long i = 0; i < (inst->index - 8); i++) {
                    if (0 == memcmp(&inst->usb_buf[i], "NEXT_VPU", 8)) {
                      searchednextvpu = 1;
                      memmove(inst->usb_buf, &inst->usb_buf[i], inst->index - i);
                      inst->index = inst->index - i;
                      printf("i:%d,searched NEXT_VPU!buffer data len=%d\n", i, inst->index);

                      if ((long)inst->index >= (long)sizeof(NVPTL_USBHeaderDataPacket)) {
                        //   printf("buffer data len is more than head!\n");
                        inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
                        if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
                          // printf("got frame!\n");
                          nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);
                          inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
                          if (inst->index > 0) {
                            memmove(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
                          }
                          goto searchnextvpu;
                        } else { // 虽然有头，但是长度不够，需要更多数据
                          //       printf("has head,but data not enough,need more data!%d\n",(inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))-inst->index);
                          break;
                        }
                        // printf("continue search!\n");
                      } else { // 虽然有头，但是长度不够，需要更多数据
                        //      printf("has start,but data not enough,need more data!\n");
                        break;
                      }
                    }
                  }

                  // printf("end search!\n");
                  if (!searchednextvpu) {
                    //   printf("has no nextvpu,so be it!!!\n");
                    inst->index = 0;
                  }
                }
              }
            } else {
              //  printf("index too small tobe header:%d\n",inst->index);
            }
          } else {
            // printf("read return 0 or <0:%d,errno:%d\n", recvd, errno);
            break;
          }
        }
        // printf("read end!\n");
        //  tcdrain(inst->device_serial_handle);
        tcflush(inst->device_serial_handle, TCIOFLUSH);
      } else if ((events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) || (events[i].events & EPOLLRDHUP)) {
        // error.
        printf("Camera %s epoll stopped", inst->devinfo.usb_camera_name);
        break;
      } else {
        printf("Camera %s Unknown event %d polled.\n", inst->devinfo.usb_camera_name, events[i].events);
      }
    }
#endif
  }
  return 0;
}

void nvptl_serial_init() {
  s_serial_running = true;
  if (s_rwqueue == NULL)
    s_rwqueue = Create_Ring_Queue(12, sizeof(SERIAL_RW));
#ifndef _WINDOWS
  libusb_init(&s_context);
  pthread_create(&s_serial_thread, 0, serial_rw_thread, NULL);
#else
  unsigned int threadID;
  s_serial_thread = (HANDLE)_beginthreadex(NULL, 0, serial_rw_thread, (LPVOID)NULL, 0, &threadID);
#endif
}

void nvptl_serial_deinit() {
  s_serial_running = false;
#ifndef _WINDOWS
  pthread_join(s_serial_thread, NULL);
  libusb_exit(s_context);
#else
  WaitForSingleObject(s_serial_thread, INFINITE);
#endif
  Destroy_Ring_Queue(s_rwqueue);
  s_rwqueue = NULL;
}

NVPTL_RESULT nvptl_serial_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices) {
#ifdef _WINDOWS
  int countdevices = 0;
  NVPTL_DEVICE_INFO *tmpdevice = NULL;
  vector<SerialPortInfo> spInfo;
  enumDetailsSerialPorts(spInfo);
  for (int i = 0; i < spInfo.size(); i++) {
    NVPTL_DEVICE_INFO *thedevice = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
    thedevice->next = tmpdevice;
    sprintf((char *)thedevice->usb_camera_name, "%s", spInfo[i].portName.c_str());
    thedevice->type = NVPTL_SERIAL_INTERFACE;
    tmpdevice = thedevice;
    countdevices++;

    // std::cout << i << " - " << spInfo[i].portName << " " << spInfo[i].description << std::endl;
  }
  *ptotal = countdevices;
  *ppdevices = tmpdevice;
  return NVPTL_OK;
#else
  libusb_device **devs;
  ssize_t cnt;
  NVPTL_DEVICE_INFO *head = NULL;
  cnt = libusb_get_device_list(s_context, &devs);
  // printf("Found %d devices", static_cast<int>(cnt));
  uint32_t valid_dev_cnt = 0;
  NVPTL_RESULT res = NVPTL_FAILED;
  for (int i = 0; i < cnt; ++i) {
    std::string name = dev_is_serial(devs[i]);
    if (name != "") {
      // 添加到设备列表中
      NVPTL_DEVICE_INFO *new_dev_node = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
      new_dev_node->next = head;
      strcpy(new_dev_node->usb_camera_name, name.c_str());
      new_dev_node->type = NVPTL_SERIAL_INTERFACE;
      head = new_dev_node;

      ++valid_dev_cnt;
    }
  }
  libusb_free_device_list(devs, 1);
  if (head) {
    *ppdevices = head;
    *ptotal = valid_dev_cnt;
    // printf("Enumeration %d valid serial devices totally!", valid_dev_cnt);
    res = NVPTL_OK;
  }
  return res;
#endif
}

static int usb_serial_open(NVPTL_INSTANCE *inst) {
#ifdef _WINDOWS
  char tmpserialname[64];
  sprintf(tmpserialname, "\\\\.\\%s", inst->devinfo.usb_camera_name);
  inst->device_usb_handle = CreateFileA(tmpserialname,                // 串口名，COM10及以上的串口名格式应为："\\\\.\\COM10"
                                        GENERIC_READ | GENERIC_WRITE, // 允许读或写
                                        0,                            // 独占方式
                                        NULL,
                                        OPEN_EXISTING,        // 打开而不是创建
                                        FILE_FLAG_OVERLAPPED, // 异步方式
                                        NULL);
  if (inst->device_usb_handle == INVALID_HANDLE_VALUE) {
    return -1;
  }

  inst->closeevent = CreateEvent(NULL, TRUE, FALSE, NULL);
  // SetupComm(inst->device_usb_handle, 0, 0);// MAXBUFFERSIZE, UPGRADE_MAX_BLK_SIZE); // 设置输入输出缓冲
  /*
        COMMTIMEOUTS TimeOuts;										// 超时设置
        TimeOuts.ReadIntervalTimeout = 100;
        TimeOuts.ReadTotalTimeoutMultiplier = 500;
        TimeOuts.ReadTotalTimeoutConstant = 0;

        TimeOuts.WriteTotalTimeoutMultiplier = 0;
        TimeOuts.WriteTotalTimeoutConstant = 0;

        SetCommTimeouts(inst->device_usb_handle, &TimeOuts);
        DCB dcb;
      // 设置串口属性
      GetCommState(inst->device_usb_handle, &dcb);	//串口属性配置

    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;


      if (!SetCommState(inst->device_usb_handle, &dcb))
      {
        CloseHandle(inst->device_usb_handle);
        std::cout << "串口配置失败" << std::endl;
        return -1;
      }*/
  PurgeComm(inst->device_usb_handle, PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_RXABORT);
#else
  inst->device_serial_handle = open(inst->devinfo.usb_camera_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (-1 == inst->device_serial_handle) {
    printf("Failed to open port %s\n", inst->devinfo.usb_camera_name);
    return -1;
  }
#endif
  return 0;
}

#ifdef _WINDOWS
static void WINAPI transfercallback(DWORD dwErrorCode,
                                    DWORD dwNumberOfBytesTransfered,
                                    LPOVERLAPPED lpOverlapped) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)lpOverlapped->Pointer;
  if (dwErrorCode != 0) {
    printf("transfer error code:%d\n", dwErrorCode);
  }
  if ((dwErrorCode == 0) && (dwNumberOfBytesTransfered != 0)) {
    // printf("got %d bytes from serial\n", dwNumberOfBytesTransfered);
    ////////////////////here,we compose packets to ok frame
    long bytesTransffered = dwNumberOfBytesTransfered;
    inst->index += bytesTransffered;
    if ((long)inst->index >= (long)sizeof(NVPTL_USBHeaderDataPacket)) {
      if (0 == strncmp("NEXT_VPU", (const char *)inst->usb_buf, 8)) {
        inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
        if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
          //	printf("1userdata:0x%X\n", inst);
          nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);
          inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
          if (inst->index > 0) {
            memmove(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
          } else {
            //	printf("no more data left:index=%d!\n", inst->index);
          }
        } else {
          //	printf("index too small tobe a complete frame:want:%d,has:%d\n", (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
        }
      } else {
        // printf("first is not NEXT_VPU!will search\n");
      searchnextvpu:
        int searchednextvpu = 0;
        for (unsigned long i = 0; i < (inst->index - 8); i++) {
          if (0 == memcmp(&inst->usb_buf[i], "NEXT_VPU", 8)) {
            // printf("searched NEXT_VPU!\n");
            searchednextvpu = 1;
            memmove(inst->usb_buf, &inst->usb_buf[i], inst->index - i);
            inst->index = inst->index - i;

            inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
            if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
              //	printf("2userdata:0x%X\n", inst);
              nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);
              inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
              if (inst->index > 0) {
                memmove(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
              }
            } else { // 虽然有头，但是长度不够，需要更多数据
              break;
            }
            goto searchnextvpu;
          }
        }
        if (!searchednextvpu) {
          printf("has no nextvpu,so be it!!!\n");
          inst->index = 0;
        }
      }
    } else {
      //	printf("index too small tobe header:%d\n",inst->index);
    }
  }

  memset(&inst->wrOverlapped, 0, sizeof(inst->wrOverlapped));
  inst->wrOverlapped.Pointer = inst;
  /*if (nvptlhandle->wrOverlapped.hEvent != NULL)
  {
    ResetEvent(nvptlhandle->wrOverlapped.hEvent);
    nvptlhandle->wrOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    s_handlevector.push_back(nvptlhandle->wrOverlapped.hEvent);
  }
  else {
    nvptlhandle->wrOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    s_handlevector.push_back(nvptlhandle->wrOverlapped.hEvent);
  }*/
  int len = USB_PACKET_MAX_SIZE - inst->index - MAXBUFFERSIZE;
  if (len < 0) {
    printf("buffer full!!!\n");
    /*	SetEvent(inst->closeevent);
      if (inst->eventcallback != NULL) {
        inst->eventcallback(PLUGOUT, inst->connectuserdata);
      }
      return;*/
    inst->index = 0;
  }
  if (inst->willclose) {
    printf("will close in transfercallback!\n");
    if (inst->eventcallback != NULL) {
      inst->eventcallback(PLUGOUT, inst->connectuserdata);
    }
    SetEvent(inst->closeevent);
    return;
  }
  if (ReadFileEx(inst->device_usb_handle, inst->usb_buf + inst->index, MAXBUFFERSIZE, &inst->wrOverlapped, transfercallback)) {
    // printf("transfercallback:ok to read file!\n");

  } else {
    SetEvent(inst->closeevent);
    DWORD err = GetLastError();
    printf("index:%d,error:%d\n", inst->index, err);
    if (inst->eventcallback != NULL) {
      inst->eventcallback(PLUGOUT, inst->connectuserdata);
    }
  }
}
#endif

static void async_read(NVPTL_INSTANCE *nvptlhandle) {
  if (nvptlhandle->usb_buf == NULL)
    nvptlhandle->usb_buf = (uint8_t *)malloc(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);

  SERIAL_RW *p = (SERIAL_RW *)SOLO_Write(s_rwqueue);
  if (p) {
    p->handle = nvptlhandle;
    p->type = SERIAL_READ;
    SOLO_Write_Over(s_rwqueue);
  }
}
NVPTL_DEVICE_HANDLE nvptl_serial_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)calloc(1, sizeof(NVPTL_INSTANCE));
  inst->devinfo = *dev_info;
  if (usb_serial_open(inst) >= 0) {
    inst->eventcallback = NULL;
    inst->connectuserdata = NULL;

    inst->recvframecallback = callback;
    inst->status = STARTED;
    inst->userdata = userdata;

    async_read(inst);
    // nvptl_debug_printf("connect ok!!!!\n");
    return inst;
  }
  free(inst);
  return NULL;
  /*
      int bus;
      char path[256];
      sscanf(dev_info->usb_camera_name, "falcon-%d-%[^-]", &bus, path);
      NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)calloc(1, sizeof(NVPTL_INSTANCE));

      nvp_serial *serial = new nvp_serial(dev_info->usb_camera_name);
      inst->devinfo = *dev_info;
      serial->set_frame_cb(callback, (NVPTL_DEVICE_HANDLE *)inst, userdata);
      if (!serial->open())
      {
          printf("Failed to open port %s!\n", dev_info->usb_camera_name);
          free(inst);
          delete serial;
          return NULL;
      }
      inst->nvptl_serail_port_inst = serial;
      inst->eventcallback = NULL;
      inst->connectuserdata = NULL;

      inst->recvframecallback = callback;
      // 此状态是否需要动态调整?
      inst->status = STARTED;
      inst->userdata = userdata;
      return inst;
  #endif*/
}
void nvptl_serial_close(NVPTL_DEVICE_HANDLE handle) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
#ifdef _WINDOWS
  inst->willclose = 1;
  /*SERIAL_RW* p = (SERIAL_RW*)SOLO_Write(s_rwqueue);
  if (p) {
    p->handle = inst;
    p->closeevent = closeevent;
    p->type = SERIAL_CLOSE;
    SOLO_Write_Over(s_rwqueue);
  }*/
  WaitForSingleObject(inst->closeevent, INFINITE);
  CloseHandle(inst->closeevent);
  CloseHandle(inst->device_usb_handle);
  free(inst);
#else
  /* nvp_serial *serial = reinterpret_cast<nvp_serial *>(inst->nvptl_serail_port_inst);
   if (!serial)
   {
       return;
   }
   serial->close();
   delete serial;
   inst->nvptl_serail_port_inst = NULL;
   inst->status = STOPPED;*/
#endif
}
int nvptl_serial_send(NVPTL_DEVICE_HANDLE handle, unsigned char *sendBuffer, size_t len) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
#ifdef _WINDOWS
  // 建立一个重叠结构
  // OVERLAPPED wrOverlapped;
  memset(&inst->wOverlapped, 0, sizeof(inst->wOverlapped));
  /*if (inst->wOverlapped.hEvent != NULL)
  {
    ResetEvent(inst->wOverlapped.hEvent);
    inst->wOverlapped.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
  }*/

  // 发送
  // DWORD dwError;
  DWORD dwSend = 0;

  /*if (ClearCommError(inst->device_usb_handle, &dwError, NULL))
  {
    PurgeComm(inst->device_usb_handle, PURGE_TXABORT | PURGE_TXCLEAR);
  }*/
  if (!WriteFile(inst->device_usb_handle, sendBuffer, len, &dwSend, &inst->wOverlapped)) {
    /*if (GetLastError() == ERROR_IO_PENDING)
    {
      while (!GetOverlappedResult(inst->device_usb_handle, &wrOverlapped, &dwSend, FALSE))
      {
        if (GetLastError() == ERROR_IO_INCOMPLETE)
        {
          continue;
        }
        else
        {
          std::cout << "发送失败" << std::endl;
          ClearCommError(inst->device_usb_handle, &dwError, NULL);
          break;
        }
      }
    }*/
  }
#else
  /*  nvp_serial *serial = reinterpret_cast<nvp_serial *>(inst->nvptl_serail_port_inst);
    if (!serial)
    {
        return -1;
    }
    return serial->async_send_data(sendBuffer, len);*/
#endif
  free(sendBuffer);
  return 0;
}