#include <stdio.h>

#ifdef _WINDOWS
#include <Winsock2.h>
#include <Windows.h>
#include <time.h>
#pragma comment(lib, "ws2_32.lib")
#include <process.h>

#else
#include <sys/stat.h>

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h> /* See NOTES */
#include <sys/socket.h>
#include <semaphore.h>
#include <pthread.h>
#include <unistd.h>
#ifdef USEDLOPENLIBUSB
#include <dlfcn.h>
#endif
#include <sys/time.h>
#include <jpeglib.h>
#endif
#include <stdint.h>
#include "transferlayer/commondata.h"
#include "nvpfm.h"

#define JPEG_QUALITY 75
#include <libusb.h>
#include "utils.h"
#include "timestamp.h"
#include "ring_queue.h"
// #include <vector>
#include <stdlib.h>

#ifndef USEYAMLCPP
#include <yaml.h>
#endif
#include "timestamp.h"

#include "nvpfm_internal.h"

#define MAXINST 256
#pragma pack(8)

#define USB_START_REQUEST 0x01
#define USB_START_RESPONSE 0x02

#define USB_STOP_REQUEST 0x03
#define USB_STOP_RESPONSE 0x04

#define USB_RESET_REQUEST 0x05
#define USB_RESET_RESPONSE 0x06

typedef struct
{
  int isopen;
  NVPFM_INSTANCE *inst;
  int bus;
  char path[64];
  libusb_device *device;
} OPENUSBMSG;
#ifdef _WINDOWS

#include <time.h>
const __int64 DELTA_EPOCH_IN_MICROSECS = 11644473600000000;

int gettimeofday(struct timeval2 *tv /*in*/, struct timezone2 *tz /*in*/)
{
  FILETIME ft;
  __int64 tmpres = 0;
  TIME_ZONE_INFORMATION tz_winapi;
  int rez = 0;

  ZeroMemory(&ft, sizeof(ft));
  ZeroMemory(&tz_winapi, sizeof(tz_winapi));

  GetSystemTimeAsFileTime(&ft);

  tmpres = ft.dwHighDateTime;
  tmpres <<= 32;
  tmpres |= ft.dwLowDateTime;

  /*converting file time to unix epoch*/
  tmpres /= 10; /*convert into microseconds*/
  tmpres -= DELTA_EPOCH_IN_MICROSECS;
  tv->tv_sec = (__int32)(tmpres * 0.000001);
  tv->tv_usec = (tmpres % 1000000);

  if (tz != NULL)
  {
    //_tzset(),don't work properly, so we use GetTimeZoneInformation
    rez = GetTimeZoneInformation(&tz_winapi);
    tz->tz_dsttime = (rez == 2) ? TRUE : FALSE;
    tz->tz_minuteswest = tz_winapi.Bias + ((rez == 2) ? tz_winapi.DaylightBias : 0);
  }
  return 0;
}
#endif

typedef struct _pkt_info_custom_t
{
  unsigned char buffer[USB_PACKET_MAX_SIZE];
  int len;
} pkt_info_custom_t;

#include <math.h>
#ifndef _WINDOWS
#include <libgen.h>
#endif
const unsigned char colortable[256][3] = {{16, 128, 128},
                                          {26, 170, 122},
                                          {26, 173, 121},
                                          {27, 176, 121},
                                          {28, 178, 120},
                                          {28, 181, 120},
                                          {29, 184, 119},
                                          {29, 186, 119},
                                          {30, 189, 119},
                                          {30, 192, 118},
                                          {31, 194, 118},
                                          {32, 196, 118},
                                          {32, 197, 117},
                                          {32, 199, 117},
                                          {33, 201, 117},
                                          {33, 203, 117},
                                          {33, 204, 116},
                                          {34, 206, 116},
                                          {34, 208, 116},
                                          {35, 209, 116},
                                          {35, 211, 115},
                                          {35, 213, 115},
                                          {36, 215, 114},
                                          {37, 217, 115},
                                          {36, 218, 114},
                                          {37, 220, 114},
                                          {37, 222, 114},
                                          {38, 223, 113},
                                          {38, 225, 113},
                                          {39, 226, 113},
                                          {39, 229, 113},
                                          {39, 231, 112},
                                          {40, 233, 112},
                                          {40, 235, 111},
                                          {40, 236, 111},
                                          {41, 238, 111},
                                          {41, 239, 110},
                                          {41, 240, 110},
                                          {43, 239, 109},
                                          {45, 238, 107},
                                          {47, 237, 106},
                                          {49, 235, 104},
                                          {51, 234, 103},
                                          {53, 233, 101},
                                          {55, 232, 100},
                                          {57, 231, 98},
                                          {59, 230, 97},
                                          {61, 228, 96},
                                          {63, 227, 94},
                                          {65, 226, 93},
                                          {67, 225, 92},
                                          {69, 224, 90},
                                          {71, 222, 89},
                                          {73, 221, 87},
                                          {75, 220, 86},
                                          {77, 219, 84},
                                          {79, 218, 84},
                                          {81, 217, 81},
                                          {83, 216, 80},
                                          {86, 214, 78},
                                          {88, 213, 77},
                                          {90, 212, 75},
                                          {92, 211, 74},
                                          {94, 210, 72},
                                          {96, 208, 71},
                                          {98, 207, 69},
                                          {100, 206, 68},
                                          {102, 205, 66},
                                          {104, 204, 65},
                                          {106, 203, 63},
                                          {108, 202, 63},
                                          {110, 200, 60},
                                          {112, 199, 59},
                                          {114, 198, 57},
                                          {116, 197, 56},
                                          {118, 196, 54},
                                          {120, 194, 53},
                                          {122, 193, 51},
                                          {124, 192, 50},
                                          {126, 191, 48},
                                          {128, 190, 47},
                                          {130, 189, 45},
                                          {131, 188, 44},
                                          {134, 186, 43},
                                          {136, 185, 41},
                                          {138, 184, 40},
                                          {140, 183, 39},
                                          {142, 182, 37},
                                          {144, 181, 36},
                                          {146, 180, 34},
                                          {148, 178, 33},
                                          {150, 177, 31},
                                          {152, 176, 30},
                                          {154, 175, 28},
                                          {156, 174, 27},
                                          {158, 173, 25},
                                          {160, 171, 24},
                                          {162, 170, 22},
                                          {164, 169, 21},
                                          {166, 168, 19},
                                          {168, 167, 18},
                                          {170, 166, 16},
                                          {171, 164, 18},
                                          {171, 162, 20},
                                          {172, 159, 22},
                                          {173, 157, 25},
                                          {173, 155, 26},
                                          {174, 153, 28},
                                          {174, 150, 31},
                                          {175, 148, 32},
                                          {176, 146, 34},
                                          {177, 143, 37},
                                          {177, 141, 39},
                                          {178, 138, 41},
                                          {179, 136, 43},
                                          {179, 134, 45},
                                          {180, 131, 47},
                                          {180, 129, 49},
                                          {181, 127, 51},
                                          {182, 124, 54},
                                          {182, 122, 55},
                                          {183, 119, 57},
                                          {183, 117, 59},
                                          {184, 115, 61},
                                          {184, 112, 63},
                                          {185, 110, 65},
                                          {186, 108, 67},
                                          {187, 106, 69},
                                          {187, 103, 71},
                                          {188, 101, 73},
                                          {189, 99, 75},
                                          {189, 96, 77},
                                          {190, 94, 79},
                                          {190, 91, 82},
                                          {191, 89, 84},
                                          {191, 87, 85},
                                          {191, 85, 86},
                                          {192, 83, 88},
                                          {192, 81, 90},
                                          {193, 78, 91},
                                          {193, 76, 93},
                                          {193, 74, 94},
                                          {194, 72, 96},
                                          {194, 70, 98},
                                          {194, 67, 99},
                                          {195, 65, 101},
                                          {196, 66, 102},
                                          {197, 68, 103},
                                          {199, 72, 103},
                                          {201, 76, 104},
                                          {203, 80, 105},
                                          {204, 84, 105},
                                          {207, 92, 105},
                                          {209, 96, 106},
                                          {211, 100, 106},
                                          {213, 107, 106},
                                          {214, 107, 108},
                                          {215, 106, 109},
                                          {215, 106, 110},
                                          {216, 105, 112},
                                          {217, 105, 113},
                                          {218, 104, 114},
                                          {219, 104, 116},
                                          {220, 103, 118},
                                          {221, 103, 120},
                                          {222, 102, 121},
                                          {223, 101, 123},
                                          {224, 101, 125},
                                          {225, 100, 127},
                                          {226, 100, 128},
                                          {227, 99, 130},
                                          {228, 98, 132},
                                          {228, 98, 133},
                                          {227, 93, 134},
                                          {226, 88, 134},
                                          {225, 84, 135},
                                          {224, 79, 136},
                                          {223, 74, 137},
                                          {222, 69, 138},
                                          {221, 64, 138},
                                          {220, 60, 139},
                                          {219, 55, 140},
                                          {218, 50, 141},
                                          {210, 17, 146},
                                          {208, 18, 148},
                                          {207, 19, 149},
                                          {205, 19, 150},
                                          {204, 21, 151},
                                          {202, 21, 152},
                                          {200, 22, 153},
                                          {199, 22, 154},
                                          {198, 24, 155},
                                          {196, 25, 156},
                                          {195, 26, 157},
                                          {193, 26, 159},
                                          {192, 27, 160},
                                          {190, 28, 161},
                                          {188, 29, 162},
                                          {187, 29, 163},
                                          {185, 31, 164},
                                          {184, 32, 165},
                                          {182, 32, 166},
                                          {181, 33, 167},
                                          {179, 34, 168},
                                          {178, 35, 170},
                                          {176, 36, 171},
                                          {175, 37, 172},
                                          {173, 38, 173},
                                          {172, 39, 174},
                                          {170, 39, 175},
                                          {169, 40, 176},
                                          {167, 41, 177},
                                          {166, 42, 178},
                                          {164, 43, 180},
                                          {163, 44, 181},
                                          {161, 45, 182},
                                          {160, 46, 183},
                                          {158, 46, 184},
                                          {157, 47, 185},
                                          {155, 48, 186},
                                          {154, 49, 187},
                                          {152, 50, 188},
                                          {151, 51, 189},
                                          {149, 52, 191},
                                          {148, 53, 192},
                                          {146, 53, 193},
                                          {145, 54, 194},
                                          {143, 55, 195},
                                          {142, 56, 196},
                                          {140, 57, 197},
                                          {139, 58, 198},
                                          {137, 59, 199},
                                          {136, 60, 200},
                                          {134, 60, 202},
                                          {133, 61, 203},
                                          {131, 62, 204},
                                          {130, 63, 205},
                                          {128, 64, 206},
                                          {126, 65, 207},
                                          {125, 66, 208},
                                          {123, 67, 209},
                                          {122, 67, 210},
                                          {120, 68, 212},
                                          {115, 71, 216},
                                          {109, 75, 220},
                                          {104, 78, 224},
                                          {98, 81, 228},
                                          {93, 84, 232},
                                          {92, 85, 233},
                                          {91, 85, 233},
                                          {90, 86, 234},
                                          {89, 87, 235},
                                          {88, 87, 235},
                                          {87, 88, 236},
                                          {86, 88, 237},
                                          {16, 128, 128}};

static libusb_context *s_context = NULL;
extern int update_result;
extern uint8_t upgrade_update_flag[256];

// #ifndef _WINDOWS
typedef ssize_t (*LIBUSB_GET_DEVICE_LIST)(libusb_context *ctx,
                                          libusb_device ***list);

static LIBUSB_GET_DEVICE_LIST _LIBUSB_GET_DEVICE_LIST;

typedef int (*LIBUSB_GET_DEVICE_DESCRIPTOR)(libusb_device *dev,
                                            struct libusb_device_descriptor *desc);
static LIBUSB_GET_DEVICE_DESCRIPTOR _LIBUSB_GET_DEVICE_DESCRIPTOR;

typedef void (*LIBUSB_CLOSE)(libusb_device_handle *dev_handle);
static LIBUSB_CLOSE _LIBUSB_CLOSE;

typedef void (*LIBUSB_EXIT)(libusb_context *ctx);
static LIBUSB_EXIT _LIBUSB_EXIT;

typedef int (*LIBUSB_BULK_TRANSFER)(libusb_device_handle *dev_handle,
                                    unsigned char endpoint, unsigned char *data, int length,
                                    int *actual_length, unsigned int timeout);
static LIBUSB_BULK_TRANSFER _LIBUSB_BULK_TRANSFER;

typedef int (*LIBUSB_INIT)(libusb_context **ctx);
static LIBUSB_INIT _LIBUSB_INIT;

typedef int (*LIBUSB_CLAIM_INTERFACE)(libusb_device_handle *dev_handle,
                                      int interface_number);
static LIBUSB_CLAIM_INTERFACE _LIBUSB_CLAIM_INTERFACE;

typedef uint8_t (*LIBUSB_GET_BUS_NUMBER)(libusb_device *dev);
static LIBUSB_GET_BUS_NUMBER _LIBUSB_GET_BUS_NUMBER;
typedef uint8_t (*LIBUSB_GET_PORT_NUMBER)(libusb_device *dev);
static LIBUSB_GET_PORT_NUMBER _LIBUSB_GET_PORT_NUMBER;

typedef int (*LIBUSB_GET_PORT_NUMBERS)(libusb_device *dev,
                                       uint8_t *port_numbers, int port_numbers_len);
static LIBUSB_GET_PORT_NUMBERS _LIBUSB_GET_PORT_NUMBERS;

typedef uint8_t (*LIBUSB_GET_DEVICE_ADDRESS)(libusb_device *dev);
static LIBUSB_GET_DEVICE_ADDRESS _LIBUSB_GET_DEVICE_ADDRESS;

typedef void (*LIBUSB_FREE_DEVICE_LIST)(libusb_device **list,
                                        int unref_devices);
static LIBUSB_FREE_DEVICE_LIST _LIBUSB_FREE_DEVICE_LIST;

typedef int (*LIBUSB_OPEN)(libusb_device *dev,
                           libusb_device_handle **dev_handle);
static LIBUSB_OPEN _LIBUSB_OPEN;

typedef libusb_device_handle *(*LIBUSB_OPEN_DEVICE_WITH_VID_PID)(
    libusb_context *ctx, uint16_t vendor_id, uint16_t product_id);
static LIBUSB_OPEN_DEVICE_WITH_VID_PID _LIBUSB_OPEN_DEVICE_WITH_VID_PID;
#ifdef USEDLOPENLIBUSB
static void *libusbhandle = NULL;
#endif
// #endif

#ifndef _WINDOWS
#include "log.h"
static log_t *g_loghandle = NULL;
#endif
#include <stdarg.h>

void nvpfm_debug_printf(const char *fmt, ...)
{
#ifdef RAWPRINTF
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  return;
#endif
  {
#ifndef _WINDOWS
    if (NULL == g_loghandle)
      return;
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 511, fmt, args);
    _log_debug(g_loghandle, buffer); // fmt,##__VA_ARGS__)
    va_end(args);
#else
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#endif
  }
}
void nvpfm_info_printf(const char *fmt, ...)
{
#ifdef RAWPRINTF
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  return;
#endif
  {
#ifndef _WINDOWS
    if (NULL == g_loghandle)
      return;
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 511, fmt, args);
    _log_info(g_loghandle, buffer); // fmt,##__VA_ARGS__)
    va_end(args);

#else
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#endif
  }
}
void nvpfm_warn_printf(const char *fmt, ...)
{
#ifdef RAWPRINTF
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  return;
#endif
  {
#ifndef _WINDOWS
    if (NULL == g_loghandle)
      return;
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 511, fmt, args);
    _log_warn(g_loghandle, buffer); // fmt,##__VA_ARGS__)
    va_end(args);

#else
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#endif
  }
}
void nvpfm_error_printf(const char *fmt, ...)
{
#ifdef RAWPRINTF
  char buffer[512];
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  return;
#endif
  {
#ifndef _WINDOWS
    if (NULL == g_loghandle)
      return;
    char buffer[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, 511, fmt, args);
    _log_error(g_loghandle, buffer); // fmt,##__VA_ARGS__)
    va_end(args);
#else
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#endif
  }
}

static int real_usb_hal_open(NVPFM_INSTANCE *inst, int bus, char *path);
static int real_usb_hal_close(NVPFM_INSTANCE *inst);

int nvpfm_init(const char *logfile, int maxlogfilesize)
{
#ifndef _WINDOWS
  /////////////////initial log module
  if (g_loghandle == NULL && logfile != NULL)
  {
    void *sh1 = stream_handle_create(ERROR_STDERR);
    sh1 = set_stream_param(sh1, LOG_ERROR, FRED, NULL, NULL);
    sh1 = set_stream_param(sh1, LOG_DEBUG, FGREEN, NULL, NULL);
    sh1 = set_stream_param(sh1, LOG_WARN, FCYAN, NULL, NULL);
    g_loghandle = add_to_handle_list(NULL, sh1); // fh1);
    if (logfile != NULL)
    {
      void *fh1 = create_log_file(logfile, maxlogfilesize, 1, 256);

      g_loghandle = add_to_handle_list(g_loghandle, fh1); // sh1);
    }
    _log_start(g_loghandle);
  }
#endif
  nvptl_init();
  return 0;
}

void nvpfm_deinit()
{
  nvpfm_debug_printf("deinit in falcon!\n");
#ifdef USEDLOPENLIBUSB
  /*3. �رտ��ļ�*/
  dlclose(libusbhandle);
#endif
#ifndef _WINDOWS
  if (g_loghandle != NULL)
  {
    _log_close();
    log_destory(g_loghandle);
    g_loghandle = NULL;
  }
#endif
  nvptl_deinit();
}

/*void usb_upgrade_callback(NVPFM_UPGRADE_SUB_TYPE type, void *data, uint32_t size)
{
  extern int32_t get_upgrade_update_flag_index(NVPFM_UPGRADE_SUB_TYPE type);
  int32_t index = get_upgrade_update_flag_index(type);
  if (index >= 0)
  {
    switch (type)
    {
    case NVPFM_COMMAND_USB_UPGRADE_NVPFM_RETURN:
    {
      update_result = ((s_nvpfm_upgrade_result *)data)->result;

      nvpfm_debug_printf("recv upgrade falcon ack, %s\r\n",
             (update_result == 1) ? "success" : ((update_result == -1) ? "err,retry" : ((update_result == -2) ? "can't write, stop" : "can't operate")));

      upgrade_update_flag[index] = 0;

      break;
    }
    case NVPFM_COMMAND_USB_UPGRADE_LIB_RETURN:
    {
      update_result = ((s_nvpfm_upgrade_result *)data)->result;

      nvpfm_debug_printf("recv upgrade lib ack, %s\r\n",
             (update_result == 1) ? "success" : ((update_result == -1) ? "err,retry" : ((update_result == -2) ? "can't write, stop" : "can't operate")));

      upgrade_update_flag[index] = 0;

      break;
    }
    case NVPFM_COMMAND_USB_UPGRADE_FILE_RETURN:
    {
      update_result = ((s_nvpfm_upgrade_result *)data)->result;

      nvpfm_debug_printf("recv upgrade file ack, %s\r\n",
             (update_result == 1) ? "success" : ((update_result == -1) ? "err,retry" : ((update_result == -2) ? "can't write, stop" : "can't operate")));

      upgrade_update_flag[index] = 0;

      break;
    }
    }
  }
}*/

int nvpfm_getyuvfromindex(int index, unsigned char *py, unsigned char *pu, unsigned char *pv)
{
  if (index < 255 && index >= 0)
  {
    *py = colortable[index][0];
    *pu = colortable[index][1];
    *pv = colortable[index][2];
    return 0;
  }
  else
  {
    return -1;
  }
}
//////////////////////////////////////
#pragma pack()
static void hal_close(NVPFM_DEVICE_HANDLE handle)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  nvptl_close((NVPTL_DEVICE_HANDLE)inst->transferlayerhandle);
}
BOOL nvpfm_hasconnect(NVPFM_DEVICE_HANDLE handle)
{
  //	nvpfm_debug_printf("in nvpfm_hasconnect\n");
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  return inst->transferlayerhandle->status >= CONNECTED && inst->transferlayerhandle->status != STOPPED;
}

static uint8_t *compose_packet(NVPFM_DEVICE_HANDLE handle, uint16_t type, uint16_t sub_type, uint32_t len, const uint8_t *pData, int *plen)
{
  (void)handle;
  // NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  // pthread_mutex_lock(inst->setgetmutex);
  // int bytesTransffered;
  NVPTL_USBHeaderDataPacket *usbPacket = (NVPTL_USBHeaderDataPacket *)calloc(1, sizeof(NVPTL_USBHeaderDataPacket) + len);
  uint32_t checksum = 0;

  if (len > ONE_PACKET_DATA_MAX_SIZE)
  {
    free(usbPacket);
    nvpfm_warn_printf("packet data too long\r\n");
    //	MUTEXUNLOCK(inst->setgetmutex);
    return NULL;
  }
  //	memset(&usbPacket, 0, sizeof(usbPacket));

  usbPacket->magic[0] = 'N';
  usbPacket->magic[1] = 'E';
  usbPacket->magic[2] = 'X';
  usbPacket->magic[3] = 'T';
  usbPacket->magic[4] = '_';
  usbPacket->magic[5] = 'V';
  usbPacket->magic[6] = 'P';
  usbPacket->magic[7] = 'U';
  usbPacket->type = type;
  usbPacket->sub_type = sub_type;
  usbPacket->len = len;

  if (len > 0)
  {
    for (uint32_t i = 0; i < len; i++)
    {
      checksum += pData[i];
    }
    memcpy((void *)usbPacket->data, pData, len);
  }
  usbPacket->checksum = checksum;
  if (type == NVPFM_USER_DATA)
  {
    nvptl_debug_printf("type:%d, subtype:%d, len:%d, checksum:%x, data:%s\n",
                       usbPacket->type, usbPacket->sub_type, usbPacket->len, usbPacket->checksum, (char *)usbPacket->data);
  }

  /*ret = hal_write(handle, (uint8_t *)usbPacket, sizeof(NVPTL_USBHeaderDataPacket) + len, &bytesTransffered, timeout);
  free(usbPacket);
  if (ret < 0 || ((uint32_t)bytesTransffered) != (sizeof(NVPTL_USBHeaderDataPacket) + len))
  {
    nvpfm_debug_printf("[%s]: ret = %d\n", __FUNCTION__, ret);
//		MUTEXUNLOCK(inst->setgetmutex);
    return -1;
  }*/
  //	MUTEXUNLOCK(inst->setgetmutex);
  *plen = sizeof(NVPTL_USBHeaderDataPacket) + len;
  return (uint8_t *)usbPacket;
}
static void asyncwrite(NVPFM_DEVICE_HANDLE handle, unsigned char *sendBuffer, size_t len);
static int send_one_packet(NVPFM_DEVICE_HANDLE handle, uint16_t type, uint16_t sub_type, uint32_t len, const uint8_t *pData, int timeout)
{
  (void)timeout;
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;

  int tmplen = 0;
  uint8_t *data = compose_packet(handle, type, sub_type, len, (const uint8_t *)pData, &tmplen);
  if (data != NULL)
  {
    nvptl_send(inst->transferlayerhandle, data, tmplen);
  }
  else
  {
    return -1;
  }

  return 0;
}

NVPTL_RESULT nvpfm_sendecho(NVPFM_DEVICE_HANDLE handle)
{
  if (nvpfm_hasconnect(handle))
  {
    int ret = send_one_packet(handle, NVPFM_COMMAND_DATA, NVPFM_COMMAND_ECHO_TEST_COMMAND_RETURN, 0, (const uint8_t *)NULL, 2000);
    if (ret >= 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm_sendreboot(NVPFM_DEVICE_HANDLE handle)
{
  if (nvpfm_hasconnect(handle))
  {
    int ret = send_one_packet(handle, NVPFM_COMMAND_DATA, NVPFM_COMMAND_REBOOT_COMMAND, 0, (const uint8_t *)NULL, 2000);
    if (ret >= 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm_senduserdata(NVPFM_DEVICE_HANDLE handle, void *data, int size)
{
  if (nvpfm_hasconnect(handle))
  {
    int ret = send_one_packet(handle, NVPFM_USER_DATA, 0, size, (const uint8_t *)data, 2000);
    if (ret >= 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}

void nvpfm_freedevices(NVPTL_DEVICE_INFO *thedevice)
{
  if (thedevice == NULL)
    return;

  NVPTL_DEVICE_INFO *tmpdevice = thedevice;
  NVPTL_DEVICE_INFO *nextdevice = NULL;
  do
  {
    NVPTL_DEVICE_INFO *nextdevice = tmpdevice->next;
    free(tmpdevice);
    tmpdevice = nextdevice;
  } while (nextdevice != NULL);
}

static void asyncwrite(NVPFM_DEVICE_HANDLE handle, unsigned char *sendBuffer, size_t len)

{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  int rc = 0;

  nvptl_send(inst->transferlayerhandle, sendBuffer, len);
}
uint64_t myabs(NVPFM_USB_IMAGE_HEADER *first, NVPFM_USB_IMAGE_HEADER *second)
{
  if (first->timestamp > second->timestamp)
  {
    return first->timestamp - second->timestamp;
  }
  else
  {
    return second->timestamp - first->timestamp;
  }
}
static int issametime(NVPFM_INSTANCE *inst, int *prgbindex, int *pleftirindex, int *prightirindex)
{
  NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      for (int k = 0; k < 4; k++)
      {
        NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + i * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
        NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + j * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
        NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + k * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
        if (myabs(depthheader, rgbheader) < 20000 &&
            myabs(depthheader, leftirheader) < 20000 &&
            myabs(depthheader, rightirheader) < 20000)
        {
          //	printf("offset:%llu,%llu,%llu\n", myabs(depthheader, rgbheader), myabs(depthheader, leftirheader), myabs(depthheader, rightirheader));
          *prgbindex = i;
          *pleftirindex = j;
          *prightirindex = k;
          return 1;
        }
        /*else
        {
          printf("current:%llu,%llu,%llu,%llu\n", depthheader->timestamp, rgbheader->timestamp, leftirheader->timestamp, rightirheader->timestamp);
          printf("fail offset:%llu,%llu,%llu\n", myabs(depthheader, rgbheader), myabs(depthheader, leftirheader), myabs(depthheader, rightirheader));
        }*/
      }
    }
  }
  return 0;
}
static uint64_t getcurrentus()
{
#ifdef _WINDOWS
  struct timeval2 tv;
  gettimeofday(&tv, NULL);
#else
  struct timeval tv;
  gettimeofday(&tv, NULL);
#endif
  uint64_t current = (uint64_t)tv.tv_sec * 1000000;
  current += tv.tv_usec;
  return current;
}
#ifdef _WINDOWS
// #pragma comment(lib,"liblz4_static.lib")
#pragma comment(lib, "turbojpeg-static.lib")
#endif
#include "turbojpeg.h"

void YUV420PtoNV12(unsigned char *Src, unsigned char *Dst, int Width, int Height)
{

  unsigned char *SrcU = Src + Width * Height;

  unsigned char *SrcV = SrcU + Width * Height / 4;

  memcpy(Dst, Src, Width * Height);

  unsigned char *DstU = Dst + Width * Height;

  for (int i = 0; i < Width * Height / 4; i++)
  {

    (*DstU++) = (*SrcU++);

    (*DstU++) = (*SrcV++);
  }
}
static int mjpeg_decompress_frame(uint8_t *jpeg_buffer, int jpeg_size, uint8_t *yuv_buffer, int *yuv_size, int *yuv_type)
// int tjpeg2yuv(unsigned char* jpeg_buffer, int jpeg_size, unsigned char** yuv_buffer, int* yuv_size, int* yuv_type)
{
  tjhandle handle = NULL;
  int width, height, subsample, colorspace;
  int flags = 0;
  int padding = 1; // 1或4均可，但不能是0
  int ret = 0;

  handle = tjInitDecompress();
  tjDecompressHeader3(handle, jpeg_buffer, jpeg_size, &width, &height, &subsample, &colorspace);

  nvpfm_debug_printf("w: %d h: %d subsample: %d color: %d\n", width, height, subsample, colorspace);

  flags |= 0;

  *yuv_type = subsample;
  // 注：经测试，指定的yuv采样格式只对YUV缓冲区大小有影响，实际上还是按JPEG本身的YUV格式来转换的
  *yuv_size = tjBufSizeYUV2(width, padding, height, subsample);
  /* *yuv_buffer = (unsigned char *)malloc(*yuv_size);
   if (*yuv_buffer == NULL)
   {
     printf("malloc buffer for rgb failed.\n");
     return -1;
   }*/

  ret = tjDecompressToYUV2(handle, jpeg_buffer, jpeg_size, yuv_buffer, width,
                           padding, height, flags);
  if (ret < 0)
  {
    nvpfm_debug_printf("decompress to jpeg failed: %s\n", tjGetErrorStr());
  }
  tjDestroy(handle);

  return 0; // ret;
}

static void on_data_receive(NVPTL_DEVICE_HANDLE tlhandle, unsigned char *data, unsigned long len, void *userdata)
{
  // printf("enter on_data_receive!\n");
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)userdata;
  NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;

  if (tmppack->type == NVPFM_IMAGE_DATA)
  {
    NVPFM_USB_IMAGE_HEADER *tmpimgheader = (NVPFM_USB_IMAGE_HEADER *)(tmppack->data);
    tmpimgheader->arrivaltimestamp = getcurrentus();
    // printf("receive image frame:%d!\n", tmppack->sub_type);
  }
  if (tmppack->type == NVPFM_COMMAND_DATA && tmppack->sub_type == NVPFM_COMMAND_REPORT_REBOOT_COMMAND)
  {
    if (inst->transferlayerhandle->eventcallback != NULL)
    {
      inst->transferlayerhandle->eventcallback(INFORMREBOOT, inst->transferlayerhandle->connectuserdata);
    }
    nvpfm_debug_printf("got echo msg!\n");
    return;
  }
  if (tmppack->type == NVPFM_COMMAND_DATA && tmppack->sub_type == NVPFM_COMMAND_ECHO_TEST_COMMAND)
  {
    nvpfm_debug_printf("got echo msg,then send echo return!\n");
    nvpfm_sendecho(inst);
    return;
  }
  if (tmppack->type == NVPFM_IMAGE_DATA)
  {
  }
  if (inst->transferlayerhandle == NULL)
    return;

  if (NULL == inst->responsequeue)
  {
    inst->responsequeue = Create_Ring_Queue(64, sizeof(sendcmd_t));
  }
  if (len < sizeof(NVPTL_USBHeaderDataPacket))
  {
    nvpfm_error_printf("data too short:%lu<=%lu\n", len, sizeof(NVPTL_USBHeaderDataPacket));
    return;
  }

  if (0 != strncmp("NEXT_VPU", (const char *)data, strlen("NEXT_VPU")))
  {
    nvpfm_error_printf("first 8 character is not NEXT_VPU!\n");
    return;
  }

  if (tmppack->len != (len - sizeof(NVPTL_USBHeaderDataPacket)))
  {
    nvpfm_error_printf("len not equal:%lu,%d\n", len - sizeof(NVPTL_USBHeaderDataPacket), tmppack->len);
    return;
  }

  // nvpfm_debug_printf("valid data from camera!\n");
  if (tmppack->type == NVPFM_UPDATE_DATA)
  {
    extern void usb_upgrade_callback(NVPFM_DEVICE_HANDLE handle, NVPFM_UPGRADE_SUB_TYPE type, void *data, uint32_t size);
    usb_upgrade_callback(inst, (NVPFM_UPGRADE_SUB_TYPE)tmppack->sub_type, tmppack->data, tmppack->len);
    return;
  }
  if (tmppack->type == NVPFM_IMAGE_DATA)
  {
    NVPFM_USB_IMAGE_HEADER *tmpimgheader = (NVPFM_USB_IMAGE_HEADER *)(tmppack->data);
    tmpimgheader->arrivaltimestamp = getcurrentus();
    if (tmpimgheader->compress_type == 1)
    {
      nvpfm_info_printf("lz4 support under construction!\n");
      return;
    }
    else if (tmpimgheader->compress_type == 2)
    {
      static uint8_t *buffer = NULL;
      if (buffer == NULL)
        buffer = (uint8_t *)malloc(USB_PACKET_MAX_SIZE);

      uint8_t *imgdata = (uint8_t *)tmppack->data + sizeof(NVPFM_USB_IMAGE_HEADER);
      int compressed_size = tmppack->len - sizeof(NVPFM_USB_IMAGE_HEADER);
      struct timeval before;
      gettimeofday(&before, NULL);
      int dst1_decompress_size, yuv_type;
      /*FILE* fp = fopen("c:/workspace/ok.jpg","wb");
      fwrite(imgdata, 1, compressed_size, fp);
      fclose(fp);
      exit(0);*/
      int ret = mjpeg_decompress_frame((const char *)imgdata, compressed_size, (char *)buffer, &dst1_decompress_size, &yuv_type);
      if (dst1_decompress_size > 0)
      {
        YUV420PtoNV12(buffer, imgdata, tmpimgheader->width, tmpimgheader->height);
        tmppack->len = dst1_decompress_size + sizeof(NVPFM_USB_IMAGE_HEADER);
        //    tmpimgheader->format = IMAGE_U8;
        // printf("ok to decode jpeg image frame!%dx%d\n", tmpimgheader->width,tmpimgheader->height);
        struct timeval now;
        gettimeofday(&now, NULL);
        NVP_U64 beforedecompress = (NVP_U64)before.tv_sec * 1000000 + (NVP_U64)before.tv_usec;
        NVP_U64 current = (NVP_U64)now.tv_sec * 1000000 + (NVP_U64)now.tv_usec;
        // if(((current - tmpimgheader->timestamp) / 1000)<400)
        //  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN);
        //  printf("compress ratio:%f,depth frame delay:%lu ms,decompress last:%lu us\n", (double)compressed_size / (double)dst1_decompress_size, (current - tmpimgheader->timestamp) / 1000, (current - beforedecompress));
        //  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
      }
      else
      {
        //  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED);
        printf("fail to decode image frame of jpeg!%d\n", dst1_decompress_size);
        // SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 7);
        return;
      }
    }
    // printf("receive image frame:%d!\n", tmppack->sub_type);
  }

  // nvpfm_info_printf("current type:%d,sub_type:%d\n", tmppack->type, tmppack->sub_type);

  if (inst->transferlayerhandle->status == CONNECTED) //&& tmppack->type == NVPFM_LOG_DATA)
  {
    //	if (nvpfm_transferdepth(inst, 0) >= 0 && nvpfm_transferir(inst, 0) >= 0 && nvpfm_transferrgb(inst, 0) >= 0 && nvpfm_imuenable(inst, 0) >= 0)
    {
      nvpfm_debug_printf("enter post connected!\n");
      inst->transferlayerhandle->status = POSTCONNECTED;
      inst->transferlayerhandle->status = CONFIGURED;
      SEM_POST(&inst->postconnectsem);

      //	return;
    }
  }

  // if(inst->transferlayerhandle->status == POSTCONNECTED && tmppack->type==NVPFM_IMAGE_DATA){
  //	return;
  //	}

  if (inst->transferlayerhandle->status == CONFIGURED && tmppack->type == NVPFM_IMAGE_DATA)
  {
    // test if width and height equal to inst->imgheight....,or reset pipeline
    if (tmppack->type == NVPFM_IMAGE_DATA)
    { // here,we will save data to rgb/depth/ir/save queue
      NVPFM_USB_IMAGE_HEADER *tmpimgheader = (NVPFM_USB_IMAGE_HEADER *)(tmppack->data);
      if (tmpimgheader->width != (uint32_t)inst->irwidth || tmpimgheader->height != (uint32_t)inst->irheight)
      {
        /*s_nvpfm_pipeline_cmd value;
        value.status = 3;
        int ret4 = send_one_packet(handle, NVPFM_COMMAND_DATA, NVPFM_COMMAND_USB_PIPELINE_COMMAND, sizeof(s_nvpfm_pipeline_cmd), (const uint8_t *)&value, 2000);
        if(ret4>=0)*/
        {
          inst->transferlayerhandle->status = STARTED;
          //	SEM_POST(&inst->postconnectsem);
          //	return;
        }
      }
      else
      {
        inst->transferlayerhandle->status = STARTED;
        //	SEM_POST(&inst->postconnectsem);
      }
    }
    //	return;
  }

  if (inst->b_timesync)
  {
    struct timeval now;
    gettimeofday(&now, NULL);
    time_t current = now.tv_sec;
    if ((current - inst->lasttimesync) > inst->timesynccycle)
    {
      s_nvpfm_time tmpvalue;
      tmpvalue.local_time_alignto_pc_time = (NVP_U64)now.tv_sec * 1000000 + (NVP_U64)now.tv_usec;
      tmpvalue.pc_time = (NVP_U64)now.tv_sec * 1000000 + (NVP_U64)now.tv_usec;

      if (inst->transferlayerhandle->devinfo.type == NVPTL_USB_INTERFACE)
      {
        int len = 0;
        uint8_t *data = compose_packet(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_SYS_TIME_COMMAND, sizeof(s_nvpfm_time) /*s_nvpfm_time_ret)*/, (const uint8_t *)&tmpvalue, &len);
        if (data != NULL)
        {
          asyncwrite(inst, data, len);
          nvpfm_debug_printf("send time sync ok!!!\n");
          inst->lasttimesync = current;
        }
      }
      else
      {
        if (0 <= send_one_packet(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_SYS_TIME_COMMAND, sizeof(s_nvpfm_time) /*s_nvpfm_time_ret)*/, (const uint8_t *)&tmpvalue, 33))
        {
          nvpfm_debug_printf("send time sync ok!!!\n");
          inst->lasttimesync = current;
        }
        else
        {
          nvpfm_warn_printf("send time sync failed!!!\n");
        }
      }
    }
  }
  /////////////////////////get all from responsequeue to responselisthead
  //	if(inst->sendthread!=NULL){
  {
    //	sendcmd_t tmpcmd;
    sendcmd_t *responsenode = 0;
    while (1)
    {
      responsenode = (sendcmd_t *)SOLO_Read(inst->responsequeue);
      if (responsenode)
      {
        NVPFM_LIST_NODE *tmpnode = (NVPFM_LIST_NODE *)calloc(1, sizeof(NVPFM_LIST_NODE));
        tmpnode->data = *responsenode;
        tmpnode->next = inst->responselisthead;
        inst->responselisthead = tmpnode;
        SOLO_Read_Over(inst->responsequeue);
      }
      else
      {
        break;
      }
    }
  }

  {
    //   if (tmppack->type == 4)
    //    printf("current sub_type:%d,sub_type:%d\n", tmppack->type, tmppack->sub_type);
    // nvpfm_info_printf("process respond sub_type:%d\n", tmppack->sub_type);
    //	nvpfm_debug_printf("vector size:%d\n",inst->responselisthead->size());
    //  std::vector<sendcmd_t>::iterator iter;
    NVPFM_LIST_NODE *tmpnode = inst->responselisthead;
    NVPFM_LIST_NODE *prevnode = NULL;
    while (tmpnode != NULL)
    // for (iter = inst->responselisthead->begin(); iter != inst->responselisthead->end();)
    {
      sendcmd_t tmpcmd = tmpnode->data;

      MYTIMEVAL tmpval;
      gettimeofday(&tmpval, NULL);

      ///////////////1.û���ڣ�����һ��
      if (((tmpval.tv_sec - tmpcmd.starttm.tv_sec) * 1000 + (tmpval.tv_usec - tmpcmd.starttm.tv_usec) / 1000) < tmpcmd.timeout &&
          tmppack->type == tmpcmd.responsetype &&
          tmppack->sub_type == tmpcmd.responsesubtype)
      {
        if (tmpcmd.callback == NULL)
        { // sync interface
          /* printf("sync interface,found reponse,copy data....,\n"
                  "ptype:%d,stype:%d,receive size:%d\n",
                  tmppack->type, tmppack->sub_type, tmppack->len);
 */
          if (tmppack->len > 0 && tmpcmd.responsedata != NULL)
          {
            if (tmppack->len <= *tmpcmd.responselen)
              memcpy(tmpcmd.responsedata, tmppack->data, tmppack->len);
            else
              memcpy(tmpcmd.responsedata, tmppack->data, *tmpcmd.responselen);
          }
          if (tmpcmd.responselen != NULL)
            *tmpcmd.responselen = tmppack->len;

          if (tmpcmd.waitsem != NULL)
            SEM_POST(tmpcmd.waitsem);
        }
        else
        { // async interface
          //	printf("call async interface:type:%d,sub_type:%d\n", tmpcmd.responsetype, tmpcmd.responsesubtype);
          tmpcmd.callback(tmppack->data, tmppack->len, tmpcmd.userdata);
        }
        // iter = inst->responselisthead->erase(iter);
        NVPFM_LIST_NODE *tobedelete = tmpnode;
        tmpnode = tmpnode->next;
        if (prevnode != NULL)
          prevnode->next = tmpnode;
        else
          inst->responselisthead = tmpnode;
        free(tobedelete);
      }
      else if (((tmpval.tv_sec - tmpcmd.starttm.tv_sec) * 1000 + (tmpval.tv_usec - tmpcmd.starttm.tv_usec) / 1000) >= tmpcmd.timeout)
      {
        nvpfm_warn_printf("timeout,just abanden!!!,response sub_type:%d\n", tmpnode->data.responsesubtype);
        
        // iter = inst->responselisthead->erase(iter);
        NVPFM_LIST_NODE *tobedelete = tmpnode;
        tmpnode = tmpnode->next;
        if (prevnode != NULL)
          prevnode->next = tmpnode;
        else
          inst->responselisthead = tmpnode;
        free(tobedelete);
      }
      else
      {
        prevnode = tmpnode;
        tmpnode = tmpnode->next;
      }
    }
  }

  if (tmppack->type == NVPFM_IMAGE_DATA && inst->transferlayerhandle->status == STARTED)
  { // here,we will save data to rgb/depth/ir/save queue
    if (inst->savecallback != NULL)
    {
      inst->savecallback(tmppack, inst->transferlayerhandle->connectuserdata);
    }
    NVPFM_USB_IMAGE_HEADER *tmpimgheader = (NVPFM_USB_IMAGE_HEADER *)(tmppack->data);

    if (tmppack->sub_type == IMAGE_DEPTH0 ||
        tmppack->sub_type == IMAGE_CHANNEL0_CALIBRATED ||
        tmppack->sub_type == IMAGE_CHANNEL1_CALIBRATED ||
        tmppack->sub_type == IMAGE_CHANNEL0_ORIGNAL ||
        tmppack->sub_type == IMAGE_CHANNEL1_ORIGNAL)
    {
      if (tmpimgheader->width != (uint32_t)inst->irwidth)
        inst->irwidth = tmpimgheader->width;
      if (tmpimgheader->height != (uint32_t)inst->irheight)
        inst->irheight = tmpimgheader->height;
    }
    else if (tmppack->sub_type == IMAGE_CHANNEL2_CALIBRATED || tmppack->sub_type == IMAGE_CHANNEL3_CALIBRATED)
    {
      if (tmpimgheader->width != (uint32_t)inst->rgbwidth)
        inst->rgbwidth = tmpimgheader->width;
      if (tmpimgheader->height != (uint32_t)inst->rgbheight)
        inst->rgbheight = tmpimgheader->height;
    }

    if (tmppack->sub_type == IMAGE_DEPTH0 || tmppack->sub_type == IMAGE_DEPTH1)
    {
      //	nvpfm_debug_printf("write depth frame to depthpkgqueue!\n");
#ifdef USEEXTRATHREAD
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Write(inst->depthpktqueue);
      if (p)
      {
        p->len = len;
        memcpy((unsigned char *)p->buffer, data, len);
        SOLO_Write_Over(inst->depthpktqueue);
        SEM_POST(&inst->depthsem);
      }
      else
      {
        // nvpfm_debug_printf("fail image one!\n");
      }
#else
      int width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket)))->width;
      int height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket)))->height;
      //	printf("inst:0x%X,depthcallback:0x%X,has depth frame!!!%dx%d:%dx%d\n", inst,inst->depthcallback,width,height,inst->irwidth,inst->irheight);
      if (inst->depthcallback != NULL && width == inst->irwidth && height == inst->irheight)
      {
        //		printf("will call depthcallback##################################\n");
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        depthheader->leavetimestamp = currentus;
        inst->depthcallback(tmppack, inst->transferlayerhandle->connectuserdata);
      }
#endif
      if (inst->groupcallback != NULL)
      {
        //  if groupbuffer has all four pkt same group,just send it
        NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
        NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + sizeof(NVPTL_USBHeaderDataPacket));
        NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
        NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
        memcpy(inst->groupbuffer.depthbuffer, data, len);
        int rgbindex, leftirindex, rightirindex;
        if (depthheader->width != 0 && issametime(inst, &rgbindex, &leftirindex, &rightirindex)) //&& depthheader->group_id == leftirheader->group_id && leftirheader->group_id == rightirheader->group_id && rightirheader->group_id == rgbheader->group_id)
        {
          if (inst->b_save && inst->savequeue != NULL)
          {
            grouppkt_info_custom_t *p = 0;
            p = (grouppkt_info_custom_t *)SOLO_Write(inst->savequeue);
            if (p)
            {
              p->len = USB_PACKET_MAX_SIZE * 13;
              memcpy((unsigned char *)p->buffer, inst->groupbuffer.buffer,
                     USB_PACKET_MAX_SIZE * 13);
              SOLO_Write_Over(inst->savequeue);
            }
            else
            {
              // nvpfm_debug_printf("fail image one!\n");
            }
          }

#ifdef USEEXTRATHREAD
          grouppkt_info_custom_t *p = 0;
          p = (grouppkt_info_custom_t *)SOLO_Write(inst->grouppktqueue);
          if (p)
          {
            p->len = USB_PACKET_MAX_SIZE * 4;
            memcpy((unsigned char *)p->buffer, inst->groupbuffer.buffer, USB_PACKET_MAX_SIZE * 4);
            SOLO_Write_Over(inst->grouppktqueue);
            SEM_POST(&inst->groupsem);
          }
          else
          {
            // nvpfm_debug_printf("fail image one!\n");
          }
#else
          if (inst->groupcallback != NULL)
          {
            uint64_t currentus = getcurrentus();
            depthheader->leavetimestamp = currentus;
            rgbheader->leavetimestamp = currentus;
            leftirheader->leavetimestamp = currentus;
            rightirheader->leavetimestamp = currentus;

            inst->groupcallback(inst->groupbuffer.depthbuffer,
                                (unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE,
                                (unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE,
                                (unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE,
                                inst->transferlayerhandle->connectuserdata);
            NVPFM_USB_IMAGE_HEADER *tmpdepthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
            NVPFM_USB_IMAGE_HEADER *tmprgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
            NVPFM_USB_IMAGE_HEADER *tmpleftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
            NVPFM_USB_IMAGE_HEADER *tmprightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
            tmpdepthheader->timestamp = 0;
            tmprgbheader->timestamp = 0;
            tmpleftirheader->timestamp = 0;
            tmprightirheader->timestamp = 0;
            tmpdepthheader->width = 0;
          }
#endif
        }
      }
    }
    if (tmppack->sub_type == IMAGE_CHANNEL0_CALIBRATED ||
        tmppack->sub_type == IMAGE_CHANNEL0_ORIGNAL)
    {
#ifdef USEEXTRATHREAD
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Write(inst->leftirpktqueue);
      if (p)
      {
        p->len = len;
        memcpy((unsigned char *)p->buffer, data, len);
        SOLO_Write_Over(inst->leftirpktqueue);
        SEM_POST(&inst->leftirsem);
      }
      else
      {
        // nvpfm_debug_printf("fail image one!\n");
      }
#else
      if (inst->leftircallback != NULL)
      {
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        leftirheader->leavetimestamp = currentus;
        inst->leftircallback(tmppack, inst->transferlayerhandle->connectuserdata);
      }
#endif
      if (inst->groupcallback != NULL)
      {
        if (tmppack->sub_type == IMAGE_CHANNEL0_CALIBRATED ||
            tmppack->sub_type == IMAGE_CHANNEL0_ORIGNAL)
        {
          // if groupbuffer has all four pkt same group,just send it
          NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          // NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *leftirheader = NULL;
          for (int i = 0; i < 4; i++)
          {
            NVPFM_USB_IMAGE_HEADER *tmpleftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + i * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
            if (leftirheader == NULL)
              leftirheader = tmpleftirheader;
            else if (leftirheader->timestamp > tmpleftirheader->timestamp)
            {
              leftirheader = tmpleftirheader;
            }
          }
          memcpy((unsigned char *)leftirheader - sizeof(NVPTL_USBHeaderDataPacket), data, len);
          int rgbindex, leftirindex, rightirindex;
          if (depthheader->width != 0 && issametime(inst, &rgbindex, &leftirindex, &rightirindex)) // depthheader->group_id == leftirheader->group_id && leftirheader->group_id == rightirheader->group_id && rightirheader->group_id == rgbheader->group_id)
          {
            if (inst->b_save && inst->savequeue != NULL)
            {
              grouppkt_info_custom_t *p = 0;
              p = (grouppkt_info_custom_t *)SOLO_Write(inst->savequeue);
              if (p)
              {
                p->len = USB_PACKET_MAX_SIZE * 4;
                memcpy((unsigned char *)p->buffer, inst->groupbuffer.buffer,
                       USB_PACKET_MAX_SIZE * 4);
                SOLO_Write_Over(inst->savequeue);
              }
              else
              {
                // nvpfm_debug_printf("fail image one!\n");
              }
            }

#ifdef USEEXTRATHREAD
            grouppkt_info_custom_t *p = 0;
            p = (grouppkt_info_custom_t *)SOLO_Write(inst->grouppktqueue);
            if (p)
            {
              p->len = USB_PACKET_MAX_SIZE * 4;
              memcpy((unsigned char *)p->buffer, inst->groupbuffer.buffer, USB_PACKET_MAX_SIZE * 4);
              SOLO_Write_Over(inst->grouppktqueue);
              SEM_POST(&inst->groupsem);
            }
            else
            {
              // nvpfm_debug_printf("fail image one!\n");
            }
#else
            if (inst->groupcallback != NULL)
            {
              uint64_t currentus = getcurrentus();
              depthheader->leavetimestamp = currentus;
              rgbheader->leavetimestamp = currentus;
              leftirheader->leavetimestamp = currentus;
              rightirheader->leavetimestamp = currentus;
              inst->groupcallback(inst->groupbuffer.depthbuffer,
                                  (unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE,
                                  (unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE,
                                  (unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE,
                                  inst->transferlayerhandle->connectuserdata);
              NVPFM_USB_IMAGE_HEADER *tmpdepthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmprgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmpleftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmprightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              tmpdepthheader->timestamp = 0;
              tmprgbheader->timestamp = 0;
              tmpleftirheader->timestamp = 0;
              tmprightirheader->timestamp = 0;
              tmpdepthheader->width = 0;
            }
#endif
          }
        }
      }
    }
    if (tmppack->sub_type == IMAGE_CHANNEL1_CALIBRATED ||
        tmppack->sub_type == IMAGE_CHANNEL1_ORIGNAL)
    {
#ifdef USEEXTRATHREAD
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Write(inst->rightirpktqueue);
      if (p)
      {
        p->len = len;
        memcpy((unsigned char *)p->buffer, data, len);
        SOLO_Write_Over(inst->rightirpktqueue);
        SEM_POST(&inst->rightirsem);
      }
      else
      {
        // nvpfm_debug_printf("fail image one!\n");
      }
#else
      if (inst->rightircallback != NULL)
      {
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        rightirheader->leavetimestamp = currentus;
        inst->rightircallback(tmppack, inst->transferlayerhandle->connectuserdata);
      }
#endif
      if (inst->groupcallback != NULL)
      {
        if (tmppack->sub_type == IMAGE_CHANNEL1_CALIBRATED ||
            tmppack->sub_type == IMAGE_CHANNEL1_ORIGNAL)
        {
          // if groupbuffer has all four pkt same group,just send it
          NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          //	NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *rightirheader = NULL;
          for (int i = 0; i < 4; i++)
          {
            NVPFM_USB_IMAGE_HEADER *tmprightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + i * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
            if (rightirheader == NULL)
              rightirheader = tmprightirheader;
            else if (rightirheader->timestamp > tmprightirheader->timestamp)
            {
              rightirheader = tmprightirheader;
            }
          }
          memcpy((unsigned char *)rightirheader - sizeof(NVPTL_USBHeaderDataPacket), data, len);
          int rgbindex, leftirindex, rightirindex;
          if (depthheader->width != 0 && issametime(inst, &rgbindex, &leftirindex, &rightirindex)) // depthheader->group_id == leftirheader->group_id && leftirheader->group_id == rightirheader->group_id && rightirheader->group_id == rgbheader->group_id)
          {
            if (inst->b_save && inst->savequeue != NULL)
            {
              grouppkt_info_custom_t *p = 0;
              p = (grouppkt_info_custom_t *)SOLO_Write(inst->savequeue);
              if (p)
              {
                p->len = USB_PACKET_MAX_SIZE * 4;
                memcpy((unsigned char *)p->buffer, inst->groupbuffer.buffer,
                       USB_PACKET_MAX_SIZE * 4);
                SOLO_Write_Over(inst->savequeue);
              }
              else
              {
                // nvpfm_debug_printf("fail image one!\n");
              }
            }

#ifdef USEEXTRATHREAD
            grouppkt_info_custom_t *p = 0;
            p = (grouppkt_info_custom_t *)SOLO_Write(inst->grouppktqueue);
            if (p)
            {
              p->len = USB_PACKET_MAX_SIZE * 4;
              memcpy((unsigned char *)p->buffer, inst->groupbuffer.buffer, USB_PACKET_MAX_SIZE * 4);
              SOLO_Write_Over(inst->grouppktqueue);
              SEM_POST(&inst->groupsem);
            }
            else
            {
              // nvpfm_debug_printf("fail image one!\n");
            }
#else
            if (inst->groupcallback != NULL)
            {
              uint64_t currentus = getcurrentus();
              depthheader->leavetimestamp = currentus;
              rgbheader->leavetimestamp = currentus;
              leftirheader->leavetimestamp = currentus;
              rightirheader->leavetimestamp = currentus;
              inst->groupcallback(inst->groupbuffer.depthbuffer,
                                  (unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE,
                                  (unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE,
                                  (unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE,
                                  inst->transferlayerhandle->connectuserdata);
              NVPFM_USB_IMAGE_HEADER *tmpdepthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmprgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmpleftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmprightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              tmpdepthheader->timestamp = 0;
              tmprgbheader->timestamp = 0;
              tmpleftirheader->timestamp = 0;
              tmprightirheader->timestamp = 0;
              tmpdepthheader->width = 0;
            }
#endif
          }
        }
      }
    }
    if (tmppack->sub_type == IMAGE_CHANNEL2_CALIBRATED ||
        tmppack->sub_type == IMAGE_CHANNEL3_CALIBRATED ||
        tmppack->sub_type == IMAGE_CHANNEL2_ORIGNAL ||
        tmppack->sub_type == IMAGE_CHANNEL3_ORIGNAL)
    {
      if (inst->groupcallback != NULL)
      {
        if (tmppack->sub_type == IMAGE_CHANNEL2_CALIBRATED ||
            tmppack->sub_type == IMAGE_CHANNEL2_ORIGNAL)
        {
          // if groupbuffer has all four pkt same group,just send it
          NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          // NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
          NVPFM_USB_IMAGE_HEADER *rgbheader = NULL;
          for (int i = 0; i < 4; i++)
          {
            NVPFM_USB_IMAGE_HEADER *tmprgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + i * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
            if (rgbheader == NULL)
              rgbheader = tmprgbheader;
            else if (rgbheader->timestamp > tmprgbheader->timestamp)
            {
              rgbheader = tmprgbheader;
            }
          }
          memcpy((unsigned char *)rgbheader - sizeof(NVPTL_USBHeaderDataPacket), data, len);
          int rgbindex, leftirindex, rightirindex;
          if (depthheader->width != 0 && issametime(inst, &rgbindex, &leftirindex, &rightirindex)) // depthheader->group_id == leftirheader->group_id && leftirheader->group_id == rightirheader->group_id && rightirheader->group_id == rgbheader->group_id)
          {
#ifdef USEEXTRATHREAD
            grouppkt_info_custom_t *p = 0;
            p = (grouppkt_info_custom_t *)SOLO_Write(inst->grouppktqueue);
            if (p)
            {
              p->len = USB_PACKET_MAX_SIZE * 4;
              memcpy((unsigned char *)p->buffer, inst->groupbuffer.buffer, USB_PACKET_MAX_SIZE * 4);
              SOLO_Write_Over(inst->grouppktqueue);
              SEM_POST(&inst->groupsem);
            }
            else
            {
              // nvpfm_debug_printf("fail image one!\n");
            }
#else
            if (inst->groupcallback != NULL)
            {
              uint64_t currentus = getcurrentus();
              depthheader->leavetimestamp = currentus;
              rgbheader->leavetimestamp = currentus;
              leftirheader->leavetimestamp = currentus;
              rightirheader->leavetimestamp = currentus;
              inst->groupcallback(inst->groupbuffer.depthbuffer,
                                  (unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE,
                                  (unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE,
                                  (unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE,
                                  inst->transferlayerhandle->connectuserdata);
              NVPFM_USB_IMAGE_HEADER *tmpdepthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmprgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rgbbuffer + rgbindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmpleftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.leftirbuffer + leftirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              NVPFM_USB_IMAGE_HEADER *tmprightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)inst->groupbuffer.rightirbuffer + rightirindex * USB_PACKET_MAX_SIZE + sizeof(NVPTL_USBHeaderDataPacket));
              tmpdepthheader->timestamp = 0;
              tmprgbheader->timestamp = 0;
              tmpleftirheader->timestamp = 0;
              tmprightirheader->timestamp = 0;
              tmpdepthheader->width = 0;
            }
#endif
          }
        }
      }
#ifdef USEEXTRATHREAD
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Write(inst->rgbpktqueue);
      if (p)
      {
        p->len = len;
        memcpy((unsigned char *)p->buffer, data, len);
        SOLO_Write_Over(inst->rgbpktqueue);
        SEM_POST(&inst->rgbsem);
      }
      else
      {
        // nvpfm_debug_printf("fail image one!\n");
      }
#else
      if (inst->rgbcallback != NULL)
      {
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        rgbheader->leavetimestamp = currentus;
        inst->rgbcallback(tmppack, inst->transferlayerhandle->connectuserdata);
      }

      if (tmppack->sub_type == IMAGE_CHANNEL2_CALIBRATED ||
          tmppack->sub_type == IMAGE_CHANNEL2_ORIGNAL)
      {
        if (inst->leftircallback != NULL)
        {
          uint64_t currentus = getcurrentus();
          NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
          leftirheader->leavetimestamp = currentus;
          inst->leftircallback(tmppack, inst->transferlayerhandle->connectuserdata);
        }
      }
      if (tmppack->sub_type == IMAGE_CHANNEL3_CALIBRATED ||
          tmppack->sub_type == IMAGE_CHANNEL3_ORIGNAL)
      {
        if (inst->rightircallback != NULL)
        {
          uint64_t currentus = getcurrentus();
          NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
          rightirheader->leavetimestamp = currentus;
          inst->rightircallback(tmppack, inst->transferlayerhandle->connectuserdata);
        }
      }
#endif
    }
  }
  else if (tmppack->type == NVPFM_IMU_DATA && inst->transferlayerhandle->status == STARTED)
  {
    // s_nvpfm_imu_data* pdata= (s_nvpfm_imu_data*)(tmppack->data);
/*		for(uint32_t i=0;i<pdata->data_number;i++){
      pdata->imu_data[i].timestamp = (NVP_U64)(get_frame_timestamp(handle, pdata->imu_data[i].timestamp) * 1000);
    }
*/
#ifdef USEEXTRATHREAD
    if (NULL == inst->imupktqueue)
    {
      inst->imupktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Write(inst->imupktqueue);
    if (p)
    {
      p->len = len;
      memcpy((unsigned char *)p->buffer, data, len);
      SOLO_Write_Over(inst->imupktqueue);
      SEM_POST(&inst->imusem);
    }
#else
    if (inst->imucallback != NULL)
      inst->imucallback(tmppack, inst->transferlayerhandle->connectuserdata);

#endif
  } /*else if (tmppack->type == NVPFM_CV_DATA && inst->transferlayerhandle->status == STARTED) {
    if (NULL == inst->goodfeaturepktqueue) {
      inst->goodfeaturepktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Write(inst->goodfeaturepktqueue);
    if (p) {
      p->len = len;
      memcpy((unsigned char *)p->buffer, data, len);
      SOLO_Write_Over(inst->goodfeaturepktqueue);
      SEM_POST(&inst->goodfeaturesem);
    }
  }*/
  else
  {

    if (tmppack->type == NVPFM_DEVICE_DATA)
    {
    }

#ifdef USEEXTRATHREAD
    if (NULL == inst->otherpktqueue)
    {
      inst->otherpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Write(inst->otherpktqueue);
    if (p)
    {
      p->len = len;
      memcpy((unsigned char *)p->buffer, data, len);
      SOLO_Write_Over(inst->otherpktqueue);
      SEM_POST(&inst->othersem);
    }
#else
    if (inst->othercallback != NULL)
    {
      inst->othercallback(tmppack, inst->transferlayerhandle->connectuserdata);
    }
#endif
  }
}

#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
static unsigned __stdcall imuprocessthread(void *param)
{
#else
static void *imuprocessthread(void *param)
{
#endif
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;
  while (inst->thread_running_flag && NULL == inst->imupktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }

  while (inst->thread_running_flag)
  {
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Read(inst->imupktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800);
      memcpy(tmpbuf, (unsigned char *)p->buffer, len);
      SOLO_Read_Over(inst->imupktqueue);

      inst->imucallback(tmpbuf, inst->connectuserdata);
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }
  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall goodfeatureprocessthread(void *param)
{
#else
static void *goodfeatureprocessthread(void *param)
{
#endif
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;
  while (inst->thread_running_flag && NULL == inst->goodfeaturepktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }

  while (inst->thread_running_flag)
  {
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Read(inst->goodfeaturepktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800);
      memcpy(tmpbuf, (unsigned char *)p->buffer, len);
      SOLO_Read_Over(inst->goodfeaturepktqueue);

      inst->goodfeaturecallback(tmpbuf, inst->connectuserdata);
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }
  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}
#ifdef _WINDOWS
static unsigned __stdcall groupprocessthread(void *param)
{
#else
static void *
groupprocessthread(void *param)
{
#endif
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;
  while (inst->thread_running_flag && NULL == inst->grouppktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }
  while (inst->thread_running_flag)
  {
    grouppkt_info_custom_t *p = 0;
    p = (grouppkt_info_custom_t *)SOLO_Read(inst->grouppktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(USB_PACKET_MAX_SIZE * 4);
      memcpy((void *)tmpbuf, (const void *)p->buffer, (size_t)len);
      SOLO_Read_Over(inst->grouppktqueue);

      if (inst->groupcallback != NULL)
        inst->groupcallback(tmpbuf, inst->connectuserdata);
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }
  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall rgbprocessthread(void *param)
{
#else
static void *
rgbprocessthread(void *param)
{
#endif
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;
  while (inst->thread_running_flag && NULL == inst->rgbpktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }
  while (inst->thread_running_flag)
  {
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Read(inst->rgbpktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 3);
      memcpy((void *)tmpbuf, (const void *)p->buffer, (size_t)len);
      SOLO_Read_Over(inst->rgbpktqueue);

      if (inst->rgbcallback != NULL)
      {
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        rgbheader->leavetimestamp = currentus;
        inst->rgbcallback(tmpbuf, inst->connectuserdata);
      }
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }
  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall leftirprocessthread(void *param)
{
#else
static void *
leftirprocessthread(void *param)
{
#endif
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;

  while (inst->thread_running_flag && NULL == inst->leftirpktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }
  while (inst->thread_running_flag)
  {
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Read(inst->leftirpktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 2);
      memcpy(tmpbuf, (unsigned char *)p->buffer, len);
      SOLO_Read_Over(inst->leftirpktqueue);

      if (inst->leftircallback != NULL)
      {
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        leftirheader->leavetimestamp = currentus;
        inst->leftircallback(tmpbuf, inst->connectuserdata);
      }
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }
  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall rightirprocessthread(void *param)
{
#else
static void *
rightirprocessthread(void *param)
{
#endif
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;

  while (inst->thread_running_flag && NULL == inst->rightirpktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }

  while (inst->thread_running_flag)
  {
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Read(inst->rightirpktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 2);
      memcpy(tmpbuf, (unsigned char *)p->buffer, len);
      SOLO_Read_Over(inst->rightirpktqueue);

      if (inst->rightircallback != NULL)
      {
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        rightirheader->leavetimestamp = currentus;
        inst->rightircallback(tmpbuf, inst->connectuserdata);
      }
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }
  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}
#ifdef _WINDOWS
static unsigned __stdcall depthprocessthread(void *param)
{
#else
static void *depthprocessthread(void *param)
{
#endif
  nvpfm_debug_printf("in depthprocessthread,wait for depthpktqueue\n");
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;
  while (inst->thread_running_flag && NULL == inst->depthpktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }

  while (inst->thread_running_flag)
  {
    //	nvpfm_debug_printf("wait for depthpktqueue to be ready!\n");
    int timeout = 1000;
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->depthsem, timeout); // &ts);
    if (ret == -1)
    {
      //	nvpfm_debug_printf("read depth frame timeout.....\n");
      continue;
    }

    //	nvpfm_debug_printf("in depthprocessthread,read depthpktqueue\n");
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Read(inst->depthpktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(1280 * 800 * 3);
      memcpy(tmpbuf, (unsigned char *)p->buffer, len);
      SOLO_Read_Over(inst->depthpktqueue);
      int width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmpbuf + sizeof(NVPTL_USBHeaderDataPacket)))->width;
      int height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmpbuf + sizeof(NVPTL_USBHeaderDataPacket)))->height;
      if (inst->depthcallback != NULL && width == inst->irwidth && height == inst->irheight)
      {
        uint64_t currentus = getcurrentus();
        NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
        depthheader->leavetimestamp = currentus;
        inst->depthcallback(tmpbuf, inst->connectuserdata);
      }
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }

  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}

#ifdef _WINDOWS
static unsigned __stdcall otherprocessthread(void *param)
{
#else
static void *otherprocessthread(void *param)
{
#endif
  nvpfm_debug_printf("in otherprocessthread,wait for otherpktqueue\n");
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)param;
  while (inst->thread_running_flag && NULL == inst->otherpktqueue)
  {
    COMMONUSLEEP(100 * 1000);
  }

  while (inst->thread_running_flag)
  {
    //	nvpfm_debug_printf("wait for otherpktqueue to be ready!\n");
    int timeout = 1000;
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->othersem, timeout); // &ts);
    if (ret == -1)
    {
      //	nvpfm_debug_printf("read depth frame timeout.....\n");
      continue;
    }

    //	nvpfm_debug_printf("in depthprocessthread,read depthpktqueue\n");
    pkt_info_custom_t *p = 0;
    p = (pkt_info_custom_t *)SOLO_Read(inst->otherpktqueue);
    if (p)
    {
      int len = p->len;
      uint8_t *tmpbuf = (uint8_t *)malloc(1024 * 1024 * 5);
      memcpy(tmpbuf, (unsigned char *)p->buffer, len);
      SOLO_Read_Over(inst->otherpktqueue);
      if (inst->othercallback != NULL)
        inst->othercallback(tmpbuf, inst->connectuserdata);
      free(tmpbuf);
    }
    else
    {
      COMMONUSLEEP(10 * 1000);
    }
  }
  if (inst->transferlayerhandle->status == STOPPED)
  {
    pthread_detach(pthread_self());
  }
  return 0;
}
#endif
static void transfercallback(struct libusb_transfer *transfer);

static void nvpfm_releasehandle(NVPFM_DEVICE_HANDLE handle)
{
  nvpfm_debug_printf("will release handle!\n");
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (inst->tminfo == NULL)
    return;

  while (inst->transfer != NULL)
  {
    printf("wait 500ms!\n");
    COMMONUSLEEP(500 * 1000);
  }
  while (inst->sendtransfer != NULL)
  {
    printf("wait 500ms!\n");
    COMMONUSLEEP(500 * 1000);
  }
  hal_close(inst);

  CLOSEMUTEX(inst->timestampmutex);
  CLOSEMUTEX(inst->setgetmutex);

  if (inst->tmpbuf != NULL)
  {
    free(inst->tmpbuf);
    inst->tmpbuf = NULL;
  }
  if (inst->usb_buf != NULL)
  {
    free(inst->usb_buf);
    inst->usb_buf = NULL;
  }
  if (NULL != inst->otherpktqueue)
  {
    Destroy_Ring_Queue(inst->otherpktqueue);
  }
  if (NULL != inst->depthpktqueue)
  {
    Destroy_Ring_Queue(inst->depthpktqueue);
  }
  if (inst->tminfo != NULL)
  {
    free(inst->tminfo);
    inst->tminfo = NULL;
  }
  if (NULL != inst->leftirpktqueue)
  {
    Destroy_Ring_Queue(inst->leftirpktqueue);
  }
  if (NULL != inst->rightirpktqueue)
  {
    Destroy_Ring_Queue(inst->rightirpktqueue);
  }
  if (NULL != inst->grouppktqueue)
  {
    Destroy_Ring_Queue(inst->grouppktqueue);
  }
  if (NULL != inst->rgbpktqueue)
  {
    Destroy_Ring_Queue(inst->rgbpktqueue);
  }
  if (NULL != inst->imupktqueue)
  {
    Destroy_Ring_Queue(inst->imupktqueue);
  }
  if (NULL != inst->goodfeaturepktqueue)
  {
    Destroy_Ring_Queue(inst->goodfeaturepktqueue);
  }
  if (NULL != inst->savepktqueue)
  {
    Destroy_Ring_Queue(inst->savepktqueue);
  }
  if (NULL != inst->responsequeue)
  {
    Destroy_Ring_Queue(inst->responsequeue);
  }

  SEM_DESTROY(&inst->postconnectsem);
  SEM_DESTROY(&inst->depthsem);
  SEM_DESTROY(&inst->upgradesem);
  SEM_DESTROY(&inst->othersem);
  SEM_DESTROY(&inst->rgbsem);
  SEM_DESTROY(&inst->groupsem);
  SEM_DESTROY(&inst->leftirsem);
  SEM_DESTROY(&inst->rightirsem);
  SEM_DESTROY(&inst->imusem);
  SEM_DESTROY(&inst->goodfeaturesem);

  // if (inst->camera_info != NULL)free(inst->camera_info);
  /*	if(inst->transfer!=NULL){
      nvpfm_debug_printf("will free transfer!\n");
      //libusb_cancel_transfer (inst->transfer);
        libusb_free_transfer (inst->transfer);
        inst->transfer = NULL;
    }*/
  if (inst->sendtransfer != NULL)
  {
    // libusb_cancel_transfer (inst->sendtransfer);
    libusb_free_transfer(inst->sendtransfer);
    inst->sendtransfer = NULL;
  }
#ifndef _WINDOWS
  libusb_free_transfer(inst->transfer);
#endif
  free(inst);
}
static void nvpfm_waitfordisconnect(NVPFM_DEVICE_HANDLE handle)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (nvpfm_hasconnect(handle))
  {
    nvpfm_debug_printf("still connected,will release handle in waitfordisconnect!\n");
#ifdef _WINDOWS
    nvpfm_debug_printf("in nvpfm_waitfordisconnect...\n");
    DWORD retfirst = WaitForSingleObject(inst->recvthread, INFINITE);
    // DWORD retfirst = WaitForSingleObject(inst->timestampthread, INFINITE);

#ifdef USEEXTRATHREAD
    nvpfm_debug_printf("will join leftirthread...\n");
    DWORD retsecond = WaitForSingleObject(inst->leftirthread, INFINITE);

    nvpfm_debug_printf("will join rightirthread...\n");
    retsecond = WaitForSingleObject(inst->rightirthread, INFINITE);

    nvpfm_debug_printf("will join rgbthread...\n");
    DWORD retthird = WaitForSingleObject(inst->rgbthread, INFINITE);
    nvpfm_debug_printf("will join groupthread...\n");
    retthird = WaitForSingleObject(inst->groupthread, INFINITE);

    nvpfm_debug_printf("will join imuthread...\n");
    DWORD retfourth = WaitForSingleObject(inst->imuthread, INFINITE);

    nvpfm_debug_printf("will join depththread...\n");
    DWORD retfifth = WaitForSingleObject(inst->depththread, INFINITE);
    nvpfm_debug_printf("will join otherthread...\n");
    DWORD retfifthdotfive = WaitForSingleObject(inst->otherthread, INFINITE);

    nvpfm_debug_printf("will join savethread...\n");
    DWORD retsixth = WaitForSingleObject(inst->savethread, INFINITE);
#endif
#else
    nvpfm_debug_printf("in nvpfm_waitfordisconnect...\n");
    pthread_join(inst->recvthread, NULL);
    // pthread_join(inst->timestampthread, NULL);
    // pthread_mutex_destroy(&inst->timestampmutex);
    // pthread_mutex_destroy(&inst->setgetmutex);

#ifdef USEEXTRATHREAD
    if (inst->leftirthread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join leftirthread...\n");
      pthread_join(inst->leftirthread, NULL);
    }
    if (inst->rightirthread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join rightirthread...\n");
      pthread_join(inst->rightirthread, NULL);
    }
    if (inst->rgbthread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join rgbthread...\n");
      pthread_join(inst->rgbthread, NULL);
    }
    if (inst->imuthread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join imuthread...\n");
      pthread_join(inst->imuthread, NULL);
    }
    if (inst->groupthread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join groupthread...\n");
      pthread_join(inst->groupthread, NULL);
    }
    if (inst->depththread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join depththread...\n");
      pthread_join(inst->depththread, NULL);
    }
    if (inst->otherthread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join otherthread...\n");
      pthread_join(inst->otherthread, NULL);
    }
    if (inst->savethread != (THREADHANDLE)NULL)
    {
      nvpfm_debug_printf("will join savethread...\n");
      pthread_join(inst->savethread, NULL);
    }
#endif
#endif
  }
  nvpfm_releasehandle(inst);
}

void nvpfm_close(NVPFM_DEVICE_HANDLE handle)
{
  if (nvpfm_hasconnect(handle))
  {
    nvpfm_debug_printf("still connected,will call waitfordisconnecT!\n");
    NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
    inst->transferlayerhandle->status = STOPPED;
    inst->thread_running_flag = 0;

    nvpfm_waitfordisconnect(handle);
  }
  else
  {
    nvpfm_debug_printf("will just release handle!\n");
    nvpfm_releasehandle(handle);
    nvpfm_debug_printf("falcon not connected,so disconnectcamera has no effect!\n");
  }
}

static int net_hal_open(const char *deviceip)
{
#ifdef _WINDOWS
  SOCKET tmpsock = socket(AF_INET, SOCK_STREAM, 0);
  if (tmpsock == INVALID_SOCKET)
  {
    nvpfm_error_printf("socket() called failed!, error code is %d\n", WSAGetLastError());
    return -1;
  }
  else
  {
    nvpfm_debug_printf("socket() called successful!\n");
  }
  // ��Ҫ���ӵķ�����׽��ֽṹ��Ϣ
  SOCKADDR_IN addrServer;
  // �趨������IP
  addrServer.sin_addr.S_un.S_addr = inet_addr(deviceip);
  addrServer.sin_family = AF_INET;
  // �趨�������Ķ˿ں�(ʹ�������ֽ���)
  addrServer.sin_port = htons((unsigned short)8000);
  // �������������������
  nvpfm_debug_printf("will connect to:%s:%d\n", deviceip, 8000);

  // ���÷�������ʽ����
  unsigned long ul = 1;
  int ret = ioctlsocket(tmpsock, FIONBIO, (unsigned long *)&ul);
  if (ret == SOCKET_ERROR)
    return -1;

  connect(tmpsock, (SOCKADDR *)&addrServer, sizeof(SOCKADDR));

  // select ģ�ͣ������ó�ʱ
  fd_set r;
  FD_ZERO(&r);
  FD_SET(tmpsock, &r);

  struct timeval timeout;
  timeout.tv_sec = 3; // ���ӳ�ʱ3��
  timeout.tv_usec = 0;
  ret = select(0, 0, &r, 0, &timeout);
  if (ret <= 0)
  {
    closesocket(tmpsock);
    return -1;
  }

  ul = 0;
  // �������ģʽ
  ret = ioctlsocket(tmpsock, FIONBIO, (unsigned long *)&ul);
  if (ret == SOCKET_ERROR)
  {
    closesocket(tmpsock);
    return -1;
  }

#else
  int re = -1;
  unsigned long ul;
  nvpfm_debug_printf("device ip:%s\n", deviceip);

  // ���� socket
  int tmpsock;
  //	nvpfm_debug_printf("create socket...\n");
  tmpsock = socket(AF_INET, SOCK_STREAM, 0);
  if (tmpsock < 0)
  {
    perror("create socket error");
    return -1;
  }

  //	nvpfm_debug_printf("set nonblock socket...\n");
  // ���÷�����
  ul = 1;
  ioctl(tmpsock, FIONBIO, &ul);

  // ���� socket
  struct sockaddr_in cliaddr;

  // ���socket��IP��˿�
  memset(&cliaddr, 0, sizeof(struct sockaddr));
  cliaddr.sin_family = AF_INET;
  cliaddr.sin_port = htons(8000);
  cliaddr.sin_addr.s_addr = inet_addr(deviceip);

  //	nvpfm_debug_printf("will connect to...\n");
  // �ͻ�������
  re = connect(tmpsock, (struct sockaddr *)&cliaddr, sizeof(struct sockaddr));
  if (re >= 0)
  {
    nvpfm_debug_printf("just connected!\n");
    return 1;
  }

  //	nvpfm_debug_printf("set sock to block...\n");
  if (re == 1)
  {
    // ����Ϊ����
    ul = 0;
    ioctl(tmpsock, FIONBIO, &ul);
    return tmpsock;
  }

  // ���ó�ʱʱ��
  re = 0;
  fd_set set;
  MYTIMEVAL tm;

  int len;
  int error = -1;

  tm.tv_sec = 3;
  tm.tv_usec = 0;
  FD_ZERO(&set);
  FD_SET(tmpsock, &set);
  // nvpfm_debug_printf("will select!!!\n");

  re = select(tmpsock + 1, NULL, &set, NULL, &tm);
  len = sizeof(int);
  if (re > 0)
  {

    //	nvpfm_debug_printf("after select!!!\n");

    // ��ȡsocket״̬
    getsockopt(tmpsock, SOL_SOCKET, SO_ERROR, &error, (socklen_t *)&len);
    if (error != 0)
    {
      nvpfm_error_printf("socket status not valid!!!\n");
      close(tmpsock);
      return -1;
    }
  }

  //	nvpfm_debug_printf("set to block!!!\n");
  // ����Ϊ����
  ul = 0;
  ioctl(tmpsock, FIONBIO, &ul);

#endif

#ifdef _WINDOWS
  int recvTimeout = 4 * 1000; // 30s
  int sendTimeout = 4 * 1000; // 30s
  setsockopt(tmpsock, SOL_SOCKET, SO_RCVTIMEO, (char *)&recvTimeout, sizeof(int));
  setsockopt(tmpsock, SOL_SOCKET, SO_SNDTIMEO, (char *)&sendTimeout, sizeof(int));
#else

  //	nvpfm_debug_printf("set send and recv timeout!!!\n");
  MYTIMEVAL recvTimeout = {4, 0};
  MYTIMEVAL sendTimeout = {4, 0};
  setsockopt(tmpsock, SOL_SOCKET, SO_RCVTIMEO, (char *)&recvTimeout, sizeof(struct timeval));
  setsockopt(tmpsock, SOL_SOCKET, SO_SNDTIMEO, (char *)&sendTimeout, sizeof(struct timeval));

#endif
  return tmpsock;
}

// 1.connect camera
// 2.shutdown streams including depth/ir/rgb/imu
// 3.return handle
NVPFM_DEVICE_HANDLE nvpfm_open(NVPTL_DEVICE_INFO *dev_info, EVENTCALLBACK eventcallback, void *userdata)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)calloc(1, sizeof(NVPFM_INSTANCE));

  inst->setgetmutex = CREATEMUTEX();
  NVPTL_DEVICE_HANDLE tlhandle = nvptl_open(dev_info, on_data_receive, inst);

  if (tlhandle != NULL)
  {
    inst->transferlayerhandle = tlhandle;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    // printf("after open:%ld\n",tv.tv_sec*1000000+tv.tv_usec);
    // free(path);
    inst->timestampmutex = CREATEMUTEX();
    inst->timestamp_is_ready = 0;

    inst->transferlayerhandle->eventcallback = eventcallback;
    inst->transferlayerhandle->connectuserdata = userdata;
    if (inst->transferlayerhandle->devinfo.type == NVPTL_UVC_INTERFACE)
    {
#ifdef _WINDOWS
      nvptl_mfuvc_set_eventcallback(inst->transferlayerhandle, eventcallback, userdata);
#endif
    }

    /*
    inst->depthcallback = NULL;
    inst->imucallback = NULL;
    inst->leftircallback = NULL;
    inst->rightircallback = NULL;
    inst->othercallback = NULL;
    inst->rgbcallback = NULL;
    inst->savecallback = NULL;
    inst->connectuserdata = NULL;*/

    inst->tminfo = (TIMESTAMPINFO *)calloc(1, sizeof(TIMESTAMPINFO));

    SEM_INIT(&inst->depthsem, 0, 0);
    SEM_INIT(&inst->upgradesem, 0, 0);
    SEM_INIT(&inst->othersem, 0, 0);
    SEM_INIT(&inst->rgbsem, 0, 0);
    SEM_INIT(&inst->groupsem, 0, 0);
    SEM_INIT(&inst->leftirsem, 0, 0);
    SEM_INIT(&inst->rightirsem, 0, 0);
    SEM_INIT(&inst->imusem, 0, 0);
    SEM_INIT(&inst->goodfeaturesem, 0, 0);
    SEM_INIT(&inst->postconnectsem, 0, 0);
    // inst->transferlayerhandle->status = CONNECTED;

    // ret = 1;

    inst->thread_running_flag = 1;

#ifdef _WINDOWS
    unsigned int threadID;

    //	inst->recvthread = (HANDLE)_beginthreadex(NULL, 0, loop_recv, (LPVOID)inst, 0, &threadID);
    // inst->timestampthread = (HANDLE)_beginthreadex(NULL, 0, timestamp_thread, (LPVOID)inst, 0, &threadID);
    if (NULL == inst->responsequeue)
    {
      inst->responsequeue = Create_Ring_Queue(64, sizeof(sendcmd_t));
    }
#ifdef USEEXTRATHREAD
    if (NULL == inst->rgbpktqueue)
    {
      inst->rgbpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
    if (NULL == inst->grouppktqueue)
    {
      inst->grouppktqueue = Create_Ring_Queue(4, sizeof(grouppkt_info_custom_t));
    }
    if (NULL == inst->depthpktqueue)
    {
      inst->depthpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }

    if (NULL == inst->leftirpktqueue)
    {
      inst->leftirpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
    if (NULL == inst->rightirpktqueue)
    {
      inst->rightirpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
#endif

#else
    // pthread_create(&inst->recvthread, 0, loop_recv, (void *)inst);
    if (NULL == inst->responsequeue)
    {
      inst->responsequeue = Create_Ring_Queue(9, sizeof(sendcmd_t));
    }
#ifdef USEEXTRATHREAD
    if (NULL == inst->rgbpktqueue)
    {
      inst->rgbpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
    if (NULL == inst->grouppktqueue)
    {
      inst->grouppktqueue = Create_Ring_Queue(4, sizeof(grouppkt_info_custom_t));
    }
    if (NULL == inst->depthpktqueue)
    {
      inst->depthpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }

    if (NULL == inst->leftirpktqueue)
    {
      inst->leftirpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
    if (NULL == inst->rightirpktqueue)
    {
      inst->rightirpktqueue = Create_Ring_Queue(4, sizeof(pkt_info_custom_t));
    }
#endif

#endif

    nvpfm_debug_printf("connect ok!!!!\n");
    return inst;
  }
  else
  {
    eventcallback(HASSTOP, userdata);
  }
  free(inst);
  return NULL;
}

NVPTL_RESULT nvpfm_start(NVPFM_DEVICE_HANDLE handle, NVPFM_STREAM_TYPE_E stream_type, void *callback)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (inst == NULL)
    return NVPTL_FAILED;

  if (nvpfm_hasconnect(handle) && inst->transferlayerhandle->status >= CONFIGURED)
  {
    if (stream_type == NVPFM_STREAM_DEPTH)
    {
      nvpfm_debug_printf("will start depth stream transfer!!\n");
      inst->depthcallback = callback;
      // if (0 != nvpfm_transferdepth(handle, 1))return NVPTL_FAILED;
      if (callback != NULL)
      {
        if (inst->depththread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->depththread = (THREADHANDLE)_beginthreadex(NULL, 0, depthprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->depththread, 0, depthprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_DEPTH_LEFTIR)
    {
      nvpfm_debug_printf("will leftir stream transfer!!\n");
      inst->leftircallback = callback;
      // if (0 != nvpfm_transferir(handle, 1))return NVPTL_FAILED;
      if (callback != NULL)
      {
        if (inst->leftirthread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->leftirthread = (THREADHANDLE)_beginthreadex(NULL, 0, leftirprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->leftirthread, 0, leftirprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_DEPTH_RIGHTIR)
    {
      nvpfm_debug_printf("will start rightir stream transfer!!\n");
      inst->rightircallback = callback;
      // if (0 != nvpfm_transferir(handle, 1))return NVPTL_FAILED;
      if (callback != NULL)
      {
        if (inst->rightirthread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->rightirthread = (THREADHANDLE)_beginthreadex(NULL, 0, rightirprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->rightirthread, 0, rightirprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_RGB)
    {
      nvpfm_debug_printf("will start rgb stream transfer!!\n");
      inst->rgbcallback = callback;
      // if (0 != nvpfm_transferrgb(handle, 1))return NVPTL_FAILED;
      if (callback != NULL)
      {
        if (inst->rgbthread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->rgbthread = (THREADHANDLE)_beginthreadex(NULL, 0, rgbprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->rgbthread, 0, rgbprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_GROUPIMAGES)
    {
      nvpfm_debug_printf("will start group images stream transfer!!\n");
      inst->groupcallback = callback;
      /*if (0 != nvpfm_transferir(handle, 1))return NVPTL_FAILED;
      if (0 != nvpfm_transferrgb(handle, 1))return NVPTL_FAILED;
      if (0 != nvpfm_transferdepth(handle, 1))return NVPTL_FAILED;
      */
      if (callback != NULL)
      {
        if (inst->groupthread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->groupthread = (THREADHANDLE)_beginthreadex(NULL, 0, groupprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->groupthread, 0, groupprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_IMU)
    {
      nvpfm_debug_printf("will start imu stream transfer!!\n");
      inst->imucallback = callback;
      //	if (0 != nvpfm_imuenable(handle, 1))return NVPTL_FAILED;
      if (callback != NULL)
      {
        if (inst->imuthread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->imuthread = (THREADHANDLE)_beginthreadex(NULL, 0, imuprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->imuthread, 0, imuprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_OTHER)
    {
      nvpfm_debug_printf("will start other stream transfer!!\n");
      inst->othercallback = callback;
      //	nvpfm_imuenable(handle, 1);
      if (callback != NULL)
      {
        if (inst->otherthread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->otherthread = (THREADHANDLE)_beginthreadex(NULL, 0, otherprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->otherthread, 0, otherprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_GOODFEATURE)
    {
      nvpfm_debug_printf("will start good feature stream transfer!!\n");
      inst->goodfeaturecallback = callback;
      //	nvpfm_imuenable(handle, 1);
      if (callback != NULL)
      {
        if (inst->goodfeaturethread == (THREADHANDLE)NULL)
        {
#ifdef USEEXTRATHREAD
#ifdef _WINDOWS
          DWORD threadID;
          inst->goodfeaturethread = (THREADHANDLE)_beginthreadex(NULL, 0, goodfeatureprocessthread, (LPVOID)inst, 0, &threadID);
#else
          pthread_create(&inst->goodfeaturethread, 0, goodfeatureprocessthread, (void *)inst);
#endif
#endif
        }
      }
    }
    else if (stream_type == NVPFM_STREAM_SAVE)
    {
      inst->savecallback = callback;
    }
    else
    {
      nvpfm_error_printf("unknown stream type!\n");
      return NVPTL_FAILED;
    }

    inst->transferlayerhandle->status = STARTED;

    return NVPTL_OK;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm_stop(NVPFM_DEVICE_HANDLE handle, NVPFM_STREAM_TYPE_E stream_type)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (nvpfm_hasconnect(handle) && inst->transferlayerhandle->status >= CONFIGURED)
  {
    int ret = -1;
    if (stream_type == NVPFM_STREAM_DEPTH)
    {
      // ret = nvpfm_transferdepth(handle, 0);
      //  inst->depthcallback = NULL;
    }
    else if (stream_type == NVPFM_STREAM_DEPTH_LEFTIR)
    {
      // ret = nvpfm_transferir(handle, 0);
      //  inst->leftircallback = NULL;
    }
    else if (stream_type == NVPFM_STREAM_DEPTH_RIGHTIR)
    {
      // ret = nvpfm_transferir(handle, 0);
      //  inst->rightircallback = NULL;
    }
    else if (stream_type == NVPFM_STREAM_RGB)
    {
      // ret = nvpfm_transferrgb(handle, 0);
      //  inst->rgbcallback = NULL;
    }
    else if (stream_type == NVPFM_STREAM_IMU)
    {
      // ret = nvpfm_imuenable(handle, 0);
      //  inst->imucallback = NULL;
    }
    else
    {
      return NVPTL_FAILED;
    }

    return (ret == 0 ? NVPTL_OK : NVPTL_FAILED);
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm_get_frame(NVPFM_DEVICE_HANDLE handle, NVPFM_STREAM_TYPE_E steamtype, NVPTL_USBHeaderDataPacket **ppframe, int timeout)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (steamtype == NVPFM_STREAM_DEPTH)
  {
    if (NULL == inst->depthpktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->depthsem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....,for depth\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->depthpktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 1280 * 800 * 2 + 1024);
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->depthpktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }
  else if (steamtype == NVPFM_STREAM_GROUPIMAGES)
  {
    if (NULL == inst->grouppktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->groupsem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....for groupimages\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->grouppktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 4 * (1280 * 800 * 2 + 1024));
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->grouppktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }
  else if (steamtype == NVPFM_STREAM_RGB)
  {
    if (NULL == inst->rgbpktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->rgbsem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....for rgb\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->rgbpktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 1280 * 800 * 2 + 1024);
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->rgbpktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }
  else if (steamtype == NVPFM_STREAM_DEPTH_LEFTIR)
  {
    if (NULL == inst->leftirpktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->leftirsem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....for leftir\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->leftirpktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 1280 * 800 * 2 + 1024);
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->leftirpktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }
  else if (steamtype == NVPFM_STREAM_DEPTH_RIGHTIR)
  {
    if (NULL == inst->rightirpktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*	struct timespec ts;
      clock_gettime(CLOCK_REALTIME, &ts);
      ts.tv_sec += timeout / 1000;
      ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->rightirsem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....for rightir\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->rightirpktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 1280 * 800 * 2 + 1024);
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->rightirpktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }
  else if (steamtype == NVPFM_STREAM_IMU)
  {
    if (NULL == inst->imupktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->imusem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....for imu\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->imupktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 1280 * 800 * 2 + 1024);
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->imupktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }
  else if (steamtype == NVPFM_STREAM_GOODFEATURE)
  {
    if (NULL == inst->goodfeaturepktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->goodfeaturesem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....for imu\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->goodfeaturepktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 1280 * 800 * 2 + 1024);
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->goodfeaturepktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }
  else if (steamtype == NVPFM_STREAM_OTHER)
  {
    if (NULL == inst->otherpktqueue)
    {
      COMMONUSLEEP(500 * 1000);
      return NVPTL_NOTREADY;
    }
    /*struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;*/
    int ret = SEM_TIMEDWAIT(&inst->othersem, timeout); // &ts);
    if (ret == -1 && errno == ETIMEDOUT)
    {
      nvpfm_error_printf("nvpfm_get_frame_sync timeout.....for other\n");
      if (inst->thread_running_flag)
        return NVPTL_TIMEOUT;
      else
        return NVPTL_NETFAILED;
    }
    else if (ret == -1 && errno != ETIMEDOUT)
    {
      nvpfm_error_printf("some else err happened:%d\n", errno);
    }

    if (inst->thread_running_flag)
    {
      pkt_info_custom_t *p = 0;
      p = (pkt_info_custom_t *)SOLO_Read(inst->otherpktqueue);
      if (p)
      {
        static NVPTL_USBHeaderDataPacket *tmpframe = NULL;
        if (NULL == tmpframe)
          tmpframe = (NVPTL_USBHeaderDataPacket *)calloc(1, 1280 * 800 * 2 + 1024);
        memcpy(tmpframe, &p->buffer[0], p->len);
        SOLO_Read_Over(inst->otherpktqueue);
        *ppframe = tmpframe;
        return NVPTL_OK;
      }
    }
    else
    {
      return NVPTL_NOTCONNECT;
    }
    return NVPTL_UNKNOWN;
  }

  else
  {
    return NVPTL_NOTSUPPORTED;
  }
}

NVPTL_RESULT NVPFM_GET_VALUE(NVPFM_INSTANCE *inst,
                             NVPFM_PRIMARY_TYPE CMD, NVPFM_COMMAND_SUB_TYPE SUBTYPE, uint8_t *senddata, int PARAMSIZE,
                             NVPFM_COMMAND_SUB_TYPE RSPTYPE, uint8_t *value, int *prespsize,
                             const char *TOMSG, int timeout)
{

  MUTEXLOCK(inst->setgetmutex);
  sendcmd_t *p = 0;
  p = (sendcmd_t *)SOLO_Write(inst->responsequeue);
  if (p)
  {
    SEM_T *tmpsem = (SEM_T *)calloc(1, sizeof(SEM_T));
    SEM_INIT(tmpsem, 0, 0);
    memset(p, 0, sizeof(sendcmd_t));
    void *tmpparam = calloc(1, *prespsize);
    int *responselen = (int *)calloc(1, sizeof(int));
    *responselen = *prespsize;
    p->responselen = responselen;
    p->responsetype = CMD;
    p->responsesubtype = RSPTYPE;
    p->responsedata = tmpparam;
    p->waitsem = tmpsem;
    MYTIMEVAL tmpstarttm;
    gettimeofday(&tmpstarttm, NULL);
    p->starttm = tmpstarttm;
    p->timeout = timeout;
    SOLO_Write_Over(inst->responsequeue);
    MUTEXUNLOCK(inst->setgetmutex);
    if (0 <= send_one_packet(inst, CMD, SUBTYPE, PARAMSIZE, senddata, timeout))
    {
      // printf("get type:%d,sub type:%d,send ok!!!\n", CMD, SUBTYPE);
    }
    while(1)
    {
      MYTIMEVAL endtm;
      gettimeofday(&endtm,NULL);
      int offset=(endtm.tv_sec-tmpstarttm.tv_sec)*1000+(endtm.tv_usec-tmpstarttm.tv_usec)/1000;
   
      if(offset<timeout){
        int retvalue = SEM_TRYWAIT(tmpsem);
        if (retvalue == -1)
        {
          nvpfm_warn_printf("still no answer!%d,%d\n", CMD, SUBTYPE);
          COMMONUSLEEP(500 * 1000);
        }
        else
        {
          MUTEXLOCK(inst->setgetmutex);
          
          //	printf("want 4125,sub_type:%d,want size:%d,response len:%d\n", RSPTYPE,*prespsize,*responselen);
          memcpy(value, tmpparam, (*prespsize <= *responselen ? *prespsize : *responselen));
          *prespsize = *responselen;

          SEM_DESTROY(tmpsem);
          free(tmpsem);
          free(tmpparam);
          free(responselen);
          MUTEXUNLOCK(inst->setgetmutex);
          return NVPTL_OK;
        }
      }else{
        break;
      }
    }

    while(1){
      MYTIMEVAL endtm;
      gettimeofday(&endtm,NULL);
      int offset=(endtm.tv_sec-tmpstarttm.tv_sec)*1000+(endtm.tv_usec-tmpstarttm.tv_usec)/1000;
      if(offset<(timeout+500)){
        COMMONUSLEEP(50*1000);
      }else{
        break;
      }
    }

    SEM_DESTROY(tmpsem);
    free(tmpsem);
    free(tmpparam);
    free(responselen);
    nvpfm_error_printf(TOMSG);
    // printf("get value type:%d,return timeout:sub_type:%d\n", CMD, SUBTYPE);
    return NVPTL_TIMEOUT;
  }
  else
  {
    MUTEXUNLOCK(inst->setgetmutex);
    return NVPTL_RESPONSEQUEUEFULL;
  }
  return NVPTL_FAILED;
}

NVPTL_RESULT NVPFM_GET_VALUE_ASYNC(NVPFM_INSTANCE *inst,
                                   NVPFM_PRIMARY_TYPE CMD, NVPFM_COMMAND_SUB_TYPE SUBTYPE, uint8_t *senddata, int PARAMSIZE,
                                   NVPFM_COMMAND_SUB_TYPE RSPTYPE, ASYNCCALLBACK callback, void *userdata, int timeout)
{
  if (0 <= send_one_packet(inst, CMD, SUBTYPE, PARAMSIZE, senddata, timeout))
  {
    // nvpfm_debug_printf("get sub type:%d,send ok!!!\n", SUBTYPE);
  }
  else
  {
    return NVPTL_NETFAILED;
  }
  MUTEXLOCK(inst->setgetmutex);
  sendcmd_t *p = 0;
  p = (sendcmd_t *)SOLO_Write(inst->responsequeue);
  if (p)
  {
    memset(p, 0, sizeof(sendcmd_t));
    p->responsetype = CMD;
    p->responsesubtype = RSPTYPE;
    MYTIMEVAL tmpstarttm;
    gettimeofday(&tmpstarttm, NULL);
    p->starttm = tmpstarttm;
    p->timeout = timeout;
    p->callback = callback;
    p->userdata = userdata;
    SOLO_Write_Over(inst->responsequeue);
    MUTEXUNLOCK(inst->setgetmutex);

    return NVPTL_OK;
  }
  else
  {
    MUTEXUNLOCK(inst->setgetmutex);
    return NVPTL_RESPONSEQUEUEFULL;
  }
  return NVPTL_FAILED;
}

NVPTL_RESULT nvpfm_set(NVPFM_DEVICE_HANDLE handle, NVPFM_VALUE_TYPE valuetype, void *sendvalue, int sendlen, void *value, int *presponselen, int timeout)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (NULL == inst)
  {
    return NVPTL_FAILED;
  }

  MUTEXLOCK(inst->setgetmutex);

  if (NULL == inst->responsequeue)
  {
    MUTEXUNLOCK(inst->setgetmutex);
    return NVPTL_NOTREADY;
  }

// cfy add
// cfy note: msvc 可能不支持语句块的返回结构
#define GET_COMMAND_VALUE(CMD, ERROR_STR) (                                   \
    printf(">>>>>>>>>>>>>>>>>>>>Send command %s with value %d\n", #CMD, CMD), \
    NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, CMD, sendvalue, sendlen, CMD##_RETURN, value, presponselen, ERROR_STR, timeout))

  MUTEXUNLOCK(inst->setgetmutex);
  if (nvpfm_hasconnect(handle))
  {
    switch (valuetype)
    {
    case NVPFM_TRIGGER_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_TRIGGER_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_TRIGGER_CONFIG_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set triggerconfig timeout", timeout);
      break;
    }

    case NVPFM_DEPTH_CALCULATE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_DEPTH_CALCULATE, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_DEPTH_CALCULATE_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set depth calculate timeout", timeout);
      break;
    }
    case NVPFM_EEPROM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_FACTORY_COMMAND, NVPFM_FACTORY_SET_EEPROM_SN, sendvalue, sendlen,
                             NVPFM_FACTORY_SET_EEPROM_SN_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set factory eeprom timeout", timeout);
      break;
    }
    case NVPFM_CLEAR_CALIB:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_FACTORY_COMMAND, NVPFM_FACTORY_CLEAR_CALIB, sendvalue, sendlen,
                             NVPFM_FACTORY_CLEAR_CALIB_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set factory clear calib timeout", timeout);
      break;
    }
    case NVPFM_ENTER_READY:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_ENTER_READY_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_ENTER_READY_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set ready mode timeout", timeout);
      break;
    }
    case NVPFM_LOWTEXTURE_REMOVAL:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_LOWTEXTURE_REMOVAL, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_LOWTEXTURE_REMOVAL_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set lowtexture removal timeout", timeout);
      break;
    }
    case NVPFM_EXIT_READY:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_EXIT_READY_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_EXIT_READY_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set ready mode timeout", timeout);
      break;
    }
    case NVPFM_UPLOAD_CALIB_START:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_CALIBRATION_DATA, NVPFM_COMMAND_USB_CALIBRATION_START, sendvalue, sendlen,
                             NVPFM_COMMAND_USB_CALIBRATION_START_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "start calibration timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_FACTORYSTARTIMAGE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_FACTORY_COMMAND, NVPFM_FACTORY_START_IMAGE, sendvalue, sendlen,
                             NVPFM_FACTORY_START_IMAGE_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "factory start image timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_FACTORYSTOPIMAGE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_FACTORY_COMMAND, NVPFM_FACTORY_STOP_IMAGE, sendvalue, sendlen,
                             NVPFM_FACTORY_STOP_IMAGE_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "factory stop image timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_FACTORYSTARTIMU:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_FACTORY_COMMAND, NVPFM_FACTORY_START_IMU, sendvalue, sendlen,
                             NVPFM_FACTORY_START_IMU_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "factory start imu timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_FACTORYSTOPIMU:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_FACTORY_COMMAND, NVPFM_FACTORY_STOP_IMU, sendvalue, sendlen,
                             NVPFM_FACTORY_STOP_IMU_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "factory stop imu timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_UPLOAD_CALIB_DATA:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_CALIBRATION_DATA, NVPFM_COMMAND_USB_CALIBRATION_DATA, sendvalue, sendlen,
                             NVPFM_COMMAND_USB_CALIBRATION_DATA_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "transfer calibration data timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_UPLOAD_CALIB_END:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_CALIBRATION_DATA, NVPFM_COMMAND_USB_CALIBRATION_END, sendvalue, sendlen,
                             NVPFM_COMMAND_USB_CALIBRATION_END_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "end calibration timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_PROJECTOR_STATUS:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set projector status timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND, sizeof(s_nvpfm_app_projector_config), NVPFM_COMMAND_SET_PROJECTOR_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_SMEAR_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SMEAR_FILTER_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_SMEAR_FILTER_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set smear timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SMEAR_FILTER_PARAM, sizeof(s_nvpfm_smear_filter), NVPFM_COMMAND_SET_SMEAR_FILTER_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_SENSOR_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_IMAGE_SET_SENSOR_CONFIG_COMMAND_RETURN, value, presponselen,
                             "set sensor cfg timeout", timeout);
      // send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_GET_SENSOR_CONFIG_COMMAND, 0, NULL, 2000);
    }
    break;
    case NVPFM_EFUSE_USER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_WRITE_EFUSE_USER, sendvalue, sendlen,
                             NVPFM_COMMAND_WRITE_EFUSE_USER_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set efuse user timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_WRITE_EFUSE_USER, sizeof(s_nvpfm_efuse_user_ret), NVPFM_COMMAND_WRITE_EFUSE_USER_RETURN, "", ((s_nvpfm_efuse_user_union*)value)->ret.status = *(int*)tmpparam; , 1)
    }
    case NVPFM_WRITE_KEY:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_WRITE_EFUSE_KEY, sendvalue, sendlen,
                             NVPFM_COMMAND_WRITE_EFUSE_KEY_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set write key timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_WRITE_EFUSE_KEY, sizeof(s_nvpfm_efuse_key), NVPFM_COMMAND_WRITE_EFUSE_KEY_RETURN, "", ; , 1)
    }
    case NVPFM_HIGH_PRECISION:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_HIGH_PRECISION_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_HIGH_PRECISION_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set high precision timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_HIGH_PRECISION_PARAM, sizeof(s_nvpfm_high_precision), NVPFM_COMMAND_SET_HIGH_PRECISION_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_GOODFEATURE_PARAM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_GOOD_FEATURE_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_GOOD_FEATURE_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set good feature timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_GOOD_FEATURE_PARAM, sizeof(s_nvpfm_good_feature), NVPFM_COMMAND_SET_GOOD_FEATURE_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_LK_PARAM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_LK_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_LK_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set lk timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_LK_PARAM, sizeof(s_nvpfm_lk), NVPFM_COMMAND_SET_LK_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_SAVE_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SAVE_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_SAVE_CONFIG_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set save config timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SAVE_CONFIG_COMMAND, sizeof(int), NVPFM_COMMAND_SAVE_CONFIG_COMMAND_RETURN, "", ; , 1)
    }

    case NVPFM_SET_CAMERA_IP:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_IP_ADDR, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_IP_ADDR_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set ip timeout", timeout);
    }
    case NVPFM_REPEAT_TEXTURE_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_REPEATED_TEXTURE_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_REPEATED_TEXTURE_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set repeat filter timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_REPEATED_TEXTURE_PARAM, sizeof(s_nvpfm_repeated_texture_filter), NVPFM_COMMAND_SET_REPEATED_TEXTURE_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_BAD_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_BAD_FILTER_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_BAD_FILTER_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set bad filter timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_BAD_FILTER_PARAM, sizeof(s_nvpfm_bad_filter), NVPFM_COMMAND_SET_BAD_FILTER_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_LIGHT_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_LIGHT_FILTER_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_LIGHT_FILTER_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set light filter timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_LIGHT_FILTER_PARAM, sizeof(s_nvpfm_light_filter), NVPFM_COMMAND_SET_LIGHT_FILTER_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_SENSOR_STATUS:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_SENSOR_STATUS, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_SENSOR_STATUS_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set sensor status timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_CONFIDENCE_PARAM, sizeof(s_nvpfm_confidence), NVPFM_COMMAND_SET_CONFIDENCE_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_IMU_STATUS:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_IMU_STATUS, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_IMU_STATUS_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set imu status timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_CONFIDENCE_PARAM, sizeof(s_nvpfm_confidence), NVPFM_COMMAND_SET_CONFIDENCE_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_IMU_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_IMU_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_IMU_CONFIG_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set imu config timeout", timeout);
      break;
    }
    case NVPFM_CAN_STATUS:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_CAN_STATUS, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_CAN_STATUS_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set can status timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_CONFIDENCE_PARAM, sizeof(s_nvpfm_confidence), NVPFM_COMMAND_SET_CONFIDENCE_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_CONFIDENCE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_CONFIDENCE_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_CONFIDENCE_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set confidence timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_CONFIDENCE_PARAM, sizeof(s_nvpfm_confidence), NVPFM_COMMAND_SET_CONFIDENCE_PARAM_RETURN, "", ; , 1)
    }
    case NVPFM_SPATIAL_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SPATIAL_FILTER_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_SPATIAL_FILTER_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set spatial fitler timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_SPATIAL_FILTER_PARAM, sizeof(s_nvpfm_spatial_filter), NVPFM_COMMAND_SET_SPATIAL_FILTER_PARAM_RETURN, "", ;, 1)
    }
    case NVPFM_TEMPORAL_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_TEMPORAL_FILTER_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_TEMPORAL_FILTER_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set temporal filter timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_TEMPORAL_FILTER_PARAM, sizeof(s_nvpfm_temporal_filter), NVPFM_COMMAND_SET_TEMPORAL_FILTER_PARAM_RETURN, "", ; , 1)
    }

    case NVPFM_SPECKLE_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SPECKLE_FILTER_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_SPECKLE_FILTER_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set speckle filter timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SPECKLE_FILTER_PARAM, sizeof(s_nvpfm_speckle_filter), NVPFM_COMMAND_SET_SPECKLE_FILTER_PARAM_RETURN, "", ;, 1)
    }
    case NVPFM_HIGHLIGHT_FILTER:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_DEPTH_HIGH_LIGHT_PROCESS_PARAM, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_DEPTH_HIGH_LIGHT_PROCESS_PARAM_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set highlight timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_DEPTH_HIGH_LIGHT_PROCESS_PARAM, sizeof(s_nvpfm_depth_high_light), NVPFM_COMMAND_SET_DEPTH_HIGH_LIGHT_PROCESS_PARAM_RETURN, "", ;, 1)
    }
    case NVPFM_DEPTH_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_DSP_SET_STATIC_CONFIG_COMMAND_RETURN, sendvalue, sendlen,
                             NVPFM_DSP_SET_STATIC_CONFIG_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set depth config timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_DSP_PROCESS, NVPFM_DSP_SET_STATIC_CONFIG_COMMAND_RETURN, sizeof(s_nvpfm_app_dsp_process_static_config), NVPFM_DSP_SET_STATIC_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_RUN_MODE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_APP_RUN_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_APP_RUN_CONFIG_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set run config timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_APP_RUN_CONFIG_COMMAND, sizeof(s_nvpfm_app_run_config), NVPFM_COMMAND_SET_APP_RUN_CONFIG_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_IR_EXPOSURE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND, sendvalue, sendlen,
                             NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set ir exposure timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND, sizeof(s_nvpfm_sensor_exposure), NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND_RETURN, "", ; , 1)
    }
    case NVPFM_RGB_EXPOSURE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND, sendvalue, sendlen,
                             NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set rgb exposure timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND, sizeof(s_nvpfm_sensor_exposure), NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND_RETURN, "", ;, 1)
    }
    case NVPFM_TIME_SYNC:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_SYS_TIME_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_SYS_TIME_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set set sys time timeout", timeout);
      break;
      // NVPFM_SET_VALUE(NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_SYS_TIME_COMMAND, sizeof(s_nvpfm_time_ret), NVPFM_COMMAND_SET_SYS_TIME_COMMAND_RETURN, "", (*(s_nvpfm_time_ret*)value = *(s_nvpfm_time_ret*)tmpparam); , 1)
    }
    case NVPFM_TRANSFER_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_TRANSFER_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_SET_TRANSFER_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "set transfer config timeout", timeout);
      break;
    }
    case NVPFM_LED_CONFIG:
    {
      return NVPFM_GET_VALUE(
          inst,
          NVPFM_COMMAND_DATA,
          NVPFM_COMMAND_SET_LED_CONFIG_COMMAND, sendvalue, sendlen,
          NVPFM_COMMAND_SET_LED_CONFIG_COMMAND_RETURN, value, presponselen,
          "set led config timeout", timeout);
      break;
    }
    case NVPFM_INFER_DEPTH_PROCESS:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS,
                             NVPFM_COMMAND_SET_INFER_DEPTH_PROCESS_PARAM,
                             sendvalue,
                             sendlen,
                             NVPFM_COMMAND_SET_INFER_DEPTH_PROCESS_PARAM_RETURN,
                             value,
                             presponselen,
                             "set infer depth process timeout!",
                             timeout);
    }
    // 应当通过脚本生成
    case NVPFM_REQ_IFRAME:
    {
      // GET_COMMAND_VALUE(NVPFM_COMMAND_REQUEST_H265_I_FRAME, "request 265 iframe timeout");
      // printf("Requst data: cmd:%d, st:%d .\n", NVPFM_COMMAND_DATA, NVPFM_COMMAND_REQUEST_H265_I_FRAME);
      return GET_COMMAND_VALUE(NVPFM_COMMAND_REQUEST_H265_I_FRAME, "request 265 iframe timeout");
    }
    case NVPFM_GET_USER_CFG:
    {
      return GET_COMMAND_VALUE(NVPFM_COMMAND_GET_USER_CUSTOM_CONFIG, "Get user custom config timeout!");
    }
    case NVPFM_GET_ALL_USER_CFG:
    {
      return GET_COMMAND_VALUE(NVPFM_COMMAND_GET_USER_ALL_CUSTOM_CONFIG_NAME, "Get user custom config timeout!");
    }
    case NVPFM_LOAD_USER_CFG:
    {
      return GET_COMMAND_VALUE(NVPFM_COMMAND_LOAD_USER_CUSTOM_CONFIG, "Get user all customized config names timeout!");
    }
    case NVPFM_DELETE_USER_CFG:
    {
      return GET_COMMAND_VALUE(NVPFM_COMMAND_DELETE_USER_CUSTOM_CONFIG, "Delete user customized config timeout!");
    }
    case NVPFM_SAVE_USER_CFG:
    {

      return GET_COMMAND_VALUE(NVPFM_COMMAND_SAVE_USER_CUSTOM_CONFIG, "Save user customized config timeout!");
    }
    case NVPFM_GET_PWR_MODE_DESC:
    {
      return GET_COMMAND_VALUE(NVPFM_COMMAND_GET_CLK_CONFIG_BY_POWER_MODE, "Get pwr mode config info timeout!");
    }
    default:
    {
      return NVPTL_NOTSUPPORTED;
    }
    }
  }
  else
  {
    nvpfm_error_printf("not connected,just return NVPTL_NOTCONNECT\n");
    return NVPTL_NOTCONNECT;
  }
  return NVPTL_FAILED;
#undef GET_COMMAND_VALUE
}

NVPTL_RESULT nvpfm_settimesynccycle(NVPFM_DEVICE_HANDLE handle, int enable, int seconds)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  inst->b_timesync = enable;
  inst->timesynccycle = seconds;
  return NVPTL_OK;
}

NVPTL_RESULT nvpfm_send_update_tftp(NVPFM_DEVICE_HANDLE handle)
{
  if (!nvpfm_hasconnect(handle))
  {
    return NVPTL_NOTCONNECT;
  }

  /*	if (0 <= send_one_packet(handle, NVPFM_UPDATE_TFTP, 0, 0, (const uint8_t *)NULL, 2000))
    {
      return NVPTL_OK;
    }*/
  return NVPTL_FAILED;
}

void nvpfm_set_async(NVPFM_DEVICE_HANDLE handle, NVPFM_VALUE_TYPE valuetype, uint8_t *data, int len, ASYNCCALLBACK callback, void *userdata)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  switch (valuetype)
  {
  case NVPFM_TRIGGER_CONFIG:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_COMMAND_DATA,
                          NVPFM_COMMAND_SET_TRIGGER_CONFIG_COMMAND,
                          data, len,
                          NVPFM_COMMAND_SET_TRIGGER_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    //	send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_CONFIG_COMMAND, len, data, 2000);
  }
  break;
  case NVPFM_CONFIDENCE:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_CONFIDENCE_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_CONFIDENCE_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // 这里要发送获取confidence的包
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_CONFIDENCE_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_SENSOR_CONFIG:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_IMAGE_PROCESS,
                          NVPFM_IMAGE_SET_SENSOR_CONFIG_COMMAND,
                          data, len,
                          NVPFM_IMAGE_SET_SENSOR_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    //	send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_CONFIG_COMMAND, len, data, 2000);
  }
  break;
  case NVPFM_HIGH_PRECISION:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_HIGH_PRECISION_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_HIGH_PRECISION_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_HIGH_PRECISION_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_REPEAT_TEXTURE_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_REPEATED_TEXTURE_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_REPEATED_TEXTURE_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_REPEATED_TEXTURE_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_DEPTH_CONFIG:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_DSP_SET_STATIC_CONFIG_COMMAND,
                          data, len,
                          NVPFM_DSP_SET_STATIC_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_DSP_SET_STATIC_CONFIG_COMMAND, len, data, 2000);
  }
  break;
  case NVPFM_IR_EXPOSURE:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_IMAGE_PROCESS,
                          NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND,
                          data, len,
                          NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND, len, data, 2000);
  }
  break;
  case NVPFM_RGB_EXPOSURE:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_IMAGE_PROCESS,
                          NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND,
                          data, len,
                          NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_SET_SENSOR_EXPOSURE_COMMAND, len, data, 2000);
  }
  break;
  case NVPFM_RUN_MODE:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_COMMAND_DATA,
                          NVPFM_COMMAND_SET_APP_RUN_CONFIG_COMMAND,
                          data, len,
                          NVPFM_COMMAND_SET_APP_RUN_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_SET_APP_RUN_CONFIG_COMMAND, len, data, 2000);
  }
  break;
  case NVPFM_SMEAR_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_SMEAR_FILTER_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_SMEAR_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SMEAR_FILTER_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_SPATIAL_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_SPATIAL_FILTER_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_SPATIAL_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SPATIAL_FILTER_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_TEMPORAL_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_TEMPORAL_FILTER_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_TEMPORAL_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_TEMPORAL_FILTER_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_SPECKLE_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_SPECKLE_FILTER_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_SPECKLE_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_SPECKLE_FILTER_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_HIGHLIGHT_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_DEPTH_HIGH_LIGHT_PROCESS_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_DEPTH_HIGH_LIGHT_PROCESS_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_DEPTH_HIGH_LIGHT_PROCESS_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_LIGHT_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_LIGHT_FILTER_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_LIGHT_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_LIGHT_FILTER_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_BAD_FILTER:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_BAD_FILTER_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_BAD_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_SET_BAD_FILTER_PARAM, len, data, 2000);
  }
  break;
  case NVPFM_IMU_CONFIG:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_COMMAND_DATA,
                          NVPFM_COMMAND_SET_IMU_CONFIG_COMMAND,
                          data, len,
                          NVPFM_COMMAND_SET_IMU_CONFIG_COMMAND_RETURN,
                          callback, userdata, 2000);
    break;
  }
  case NVPFM_INFER_DEPTH_PROCESS:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_SET_INFER_DEPTH_PROCESS_PARAM,
                          data, len,
                          NVPFM_COMMAND_SET_INFER_DEPTH_PROCESS_PARAM_RETURN,
                          callback, userdata, 2000);
    break;
  }
  default:
    break;
    ;
  }
}

void nvpfm_get_async(NVPFM_DEVICE_HANDLE handle, NVPFM_VALUE_TYPE valuetype, ASYNCCALLBACK callback, void *userdata)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (inst == NULL)
    return;
  switch (valuetype)
  {
  case NVPFM_DEVICE_INFO:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_COMMAND_DATA,
                          NVPFM_COMMAND_GET_DEVICE_INFO_COMMAND,
                          NULL, 0,
                          NVPFM_COMMAND_GET_DEVICE_INFO_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_DEVICE_INFO_COMMAND, senddata, NULL, 2000);
  }
  break;
  case NVPFM_DEPTH_PARAM:
  {

    s_get_depth_param senddata;
    senddata.channel = NVPFM_DEPTH_CHANNEL0;
    senddata.frame_size = IMAGE_480_640;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_COMMAND_DATA,
                          NVPFM_COMMAND_GET_DEPTH_PARAM_COMMAND,
                          (uint8_t *)&senddata, sizeof(s_get_depth_param),
                          NVPFM_COMMAND_GET_DEPTH_PARAM_COMMAND_RETURN,
                          callback, userdata,
                          2000);

    // send_one_packet(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_DEPTH_PARAM_COMMAND, sizeof(s_get_depth_param), &senddata, 2000);
  }
  break;
  case NVPFM_CONFIDENCE:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_CONFIDENCE_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_CONFIDENCE_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_CONFIDENCE_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_SENSOR_CONFIG:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_IMAGE_PROCESS,
                          NVPFM_IMAGE_GET_SENSOR_CONFIG_COMMAND,
                          NULL, 0,
                          NVPFM_IMAGE_GET_SENSOR_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_GET_SENSOR_CONFIG_COMMAND, 0, NULL, 2000);
  }
  break;
  case NVPFM_SPATIAL_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_SPATIAL_FILTER_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_SPATIAL_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);

    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_SPATIAL_FILTER_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_TEMPORAL_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_TEMPORAL_FILTER_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_TEMPORAL_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);

    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_TEMPORAL_FILTER_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_SPECKLE_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_SPECKLE_FILTER_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_SPECKLE_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_SPECKLE_FILTER_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_HIGHLIGHT_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_DEPTH_HIGH_LIGHT_PROCESS_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_DEPTH_HIGH_LIGHT_PROCESS_PARAM_RETURN,
                          callback, userdata,
                          2000);

    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_DEPTH_HIGH_LIGHT_PROCESS_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_REPEAT_TEXTURE_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_REPEATED_TEXTURE_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_REPEATED_TEXTURE_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_REPEATED_TEXTURE_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_DEPTH_CONFIG:
  {
    s_nvpfm_get_dsp_static_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_DSP_GET_STATIC_CONFIG_COMMAND,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dsp_static_config),
                          NVPFM_DSP_GET_STATIC_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    //	send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_DSP_GET_STATIC_CONFIG_COMMAND, sizeof(s_nvpfm_get_dsp_static_config), &cfg, 2000);
  }
  break;
  case NVPFM_IR_EXPOSURE:
  {
    s_nvpfm_get_sensor_exposure cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_IMAGE_PROCESS,
                          NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_sensor_exposure),
                          NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND, sizeof(s_nvpfm_get_sensor_exposure), &cfg, 2000);
  }
  break;
  case NVPFM_RGB_EXPOSURE:
  {
    s_nvpfm_get_sensor_exposure cfg;
    cfg.channel = CHANNEL2;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_IMAGE_PROCESS,
                          NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_sensor_exposure),
                          NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND, sizeof(s_nvpfm_get_sensor_exposure), &cfg, 2000);
  }
  break;
  case NVPFM_RUN_MODE:
  {
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_COMMAND_DATA,
                          NVPFM_COMMAND_GET_APP_RUN_CONFIG_COMMAND,
                          NULL, 0,
                          NVPFM_COMMAND_GET_APP_RUN_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_APP_RUN_CONFIG_COMMAND, 0, NULL, 2000);
  }
  break;
  case NVPFM_SMEAR_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_SMEAR_FILTER_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_SMEAR_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_SMEAR_FILTER_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_HIGH_PRECISION:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_HIGH_PRECISION_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_HIGH_PRECISION_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_HIGH_PRECISION_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_LIGHT_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_LIGHT_FILTER_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_LIGHT_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_LIGHT_FILTER_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_BAD_FILTER:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;

    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_BAD_FILTER_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_BAD_FILTER_PARAM_RETURN,
                          callback, userdata,
                          2000);
    // send_one_packet(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_BAD_FILTER_PARAM, sizeof(s_nvpfm_get_dynamic_config), &cfg, 2000);
  }
  break;
  case NVPFM_IMU_CONFIG:
  {
    s_get_imu_config cfg;
    cfg.channel = NVPFM_IMU_CHANNEL0;
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_COMMAND_DATA,
                          NVPFM_COMMAND_GET_IMU_CONFIG_COMMAND,
                          (uint8_t *)&cfg, sizeof(s_get_imu_config),
                          NVPFM_COMMAND_GET_IMU_CONFIG_COMMAND_RETURN,
                          callback, userdata,
                          2000);
    break;
  }
  case NVPFM_INFER_DEPTH_PROCESS:
  {
    s_nvpfm_get_dynamic_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;
    NVPFM_GET_VALUE_ASYNC(inst,
                          NVPFM_DSP_PROCESS,
                          NVPFM_COMMAND_GET_INFER_DEPTH_PROCESS_PARAM,
                          (uint8_t *)&cfg, sizeof(s_nvpfm_get_dynamic_config),
                          NVPFM_COMMAND_GET_INFER_DEPTH_PROCESS_PARAM_RETURN,
                          callback, userdata, 2000);
    break;
  }
  default:
    break;
  }
}

NVPTL_RESULT nvpfm_get(NVPFM_DEVICE_HANDLE handle, NVPFM_VALUE_TYPE valuetype, void *sendvalue, int sendlen, void *value, int *presponselen, int timeout)
{ // 这是一个c层的通用同步获取接口，发送一个，接收一个
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (NULL == inst)
  {
    return NVPTL_FAILED;
  }
  MUTEXLOCK(inst->setgetmutex);
  if (NULL == inst->responsequeue)
  {
    MUTEXUNLOCK(inst->setgetmutex);
    return NVPTL_NOTREADY;
  }
  MUTEXUNLOCK(inst->setgetmutex);
  if (nvpfm_hasconnect(handle))
  {
    switch (valuetype)
    {
    case NVPFM_TRIGGER_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_TRIGGER_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_TRIGGER_CONFIG_COMMAND_RETURN, value, presponselen,
                             "get trigger config timeout...", timeout);
    }
    case NVPFM_EEPROM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_FACTORY_COMMAND, NVPFM_FACTORY_GET_EEPROM_SN, sendvalue, sendlen,
                             NVPFM_FACTORY_GET_EEPROM_SN_RETURN, value, presponselen,
                             "get factory eeprom timeout...", timeout);
    }
    case NVPFM_DEPTH_CALCULATE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_DEPTH_CALCULATE, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_DEPTH_CALCULATE_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "get depth calculate timeout", timeout);
      break;
    }
    case NVPFM_LOWTEXTURE_REMOVAL:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_LOWTEXTURE_REMOVAL, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_LOWTEXTURE_REMOVAL_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "get lowtexture removal timeout", timeout);
      break;
    }
    case NVPFM_INNER_PARAM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_SENSOR_CHANNEL_INNER_PARAM_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_SENSOR_CHANNEL_INNER_PARAM_COMMAND_RETURN, value, presponselen,
                             "get cam inner timeout...", timeout);
    }
    case NVPFM_DOWNLOAD_CALIB:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_CALIBRATION_DATA, NVPFM_COMMAND_USB_CALIBRATION_DOWNLOAD, sendvalue, sendlen,
                             NVPFM_COMMAND_USB_CALIBRATION_DOWNLOAD_RETURN, value, presponselen,
                             "download calib timeout...", timeout);
    }
    case NVPFM_IMU_PARAM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_IMU_PARAM_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_IMU_PARAM_COMMAND_RETURN, value, presponselen,
                             "get imu param timeout...", timeout);
    }
    case NVPFM_IMU_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_IMU_CONFIG_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_IMU_CONFIG_COMMAND_RETURN, value, presponselen, // sizeof(s_nvpfm_app_projector_config),
                             "get imu config timeout", timeout);
      break;
    }
    case NVPFM_OUTER_PARAM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_CHANNELS_POS_PARAM_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_CHANNELS_POS_PARAM_COMMAND_RETURN, value, presponselen,
                             "get cam outer timeout...", timeout);
    }
    case NVPFM_DEPTH_PARAM:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_DEPTH_PARAM_COMMAND, sendvalue, sendlen,
                             NVPFM_COMMAND_GET_DEPTH_PARAM_COMMAND_RETURN, value, presponselen,
                             "get depth param timeout...", timeout);
    }
    case NVPFM_SMEAR_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_SMEAR_FILTER_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_SMEAR_FILTER_PARAM_RETURN, value, presponselen, "get smear_filter timeout...", timeout);
    }
    case NVPFM_SENSOR_CONFIG:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_GET_SENSOR_CONFIG_COMMAND, sendvalue, sendlen, NVPFM_IMAGE_GET_SENSOR_CONFIG_COMMAND_RETURN, value, presponselen, "get sensor cfg timeout...", timeout);
    }
    case NVPFM_EFUSE_USER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_READ_EFUSE_USER, sendvalue, sendlen, NVPFM_COMMAND_READ_EFUSE_USER_RETURN, value, presponselen, "get efuse_user timeout...", timeout);
    }
    case NVPFM_CYPHER_STATUS:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_READ_EFUSE_STATUS, sendvalue, sendlen, NVPFM_COMMAND_READ_EFUSE_STATUS_RETURN, value, presponselen, "get cypher_status timeout...", timeout);
    }
    case NVPFM_CAMERA_TIME:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_SYS_TIME_COMMAND, sendvalue, sendlen, NVPFM_COMMAND_GET_SYS_TIME_COMMAND_RETURN, value, presponselen, "get camera_time timeout...", timeout);
    }
    /*	case NVPFM_RGB_ELSE_PARAM:
      {
        NVPFM_GET_VALUE(inst,NVPFM_COMMAND_DATA,NVPFM_COMMAND_GET_RGB_ELSE_PARAM,sendvalue, sendlen, NVPFM_COMMAND_GET_RGB_ELSE_PARAM_RETURN, value, presponselen,  "get rgb_else_param timeout...")
      }*/
    case NVPFM_HIGH_PRECISION:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_HIGH_PRECISION_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_HIGH_PRECISION_PARAM_RETURN, value, presponselen, "get high_precision timeout...", timeout);
    }
    case NVPFM_GOODFEATURE_PARAM:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_GOOD_FEATURE_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_GOOD_FEATURE_PARAM_RETURN, value, presponselen, "get high_precision timeout...", timeout);
    }
    case NVPFM_LK_PARAM:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_LK_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_LK_PARAM_RETURN, value, presponselen, "get lk_param timeout...", timeout);
    }
    case NVPFM_DOWNLOAD_FILE:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DOWNLOAD_DATA, NVPFM_COMMAND_USB_DOWNLOAD_USER_FILE, sendvalue, sendlen,
                             NVPFM_COMMAND_USB_DOWNLOAD_USER_FILE_RETURN, (uint8_t *)value, presponselen,
                             "download timeout...", timeout);
    }
      /*	case NVPFM_IMU_INTERNALREF:
        {
          return NVPFM_GET_VALUE(inst,NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_IMU_INTERNAL_REFRRENCE, sendvalue, sendlen, NVPFM_COMMAND_GET_IMU_INTERNAL_REFRRENCE_RETURN, value, presponselen, "get imu_internalref timeout...",timeout);

        }*/
    case NVPFM_REPEAT_TEXTURE_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_REPEATED_TEXTURE_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_REPEATED_TEXTURE_PARAM_RETURN, value, presponselen, "get repeat_texture_filter timeout...", timeout);
    }
    /*		case NVPFM_EDGE_FILTER:
        {
          NVPFM_GET_VALUE(inst,NVPFM_COMMAND_DATA,NVPFM_COMMAND_GET_EDGE_FILTER_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_EDGE_FILTER_PARAM_RETURN,  value, presponselen, "get edge_filter timeout...")
        }*/
    case NVPFM_BAD_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_BAD_FILTER_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_BAD_FILTER_PARAM_RETURN, value, presponselen, "get bad_filter timeout...", timeout);
    }
    case NVPFM_LIGHT_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_LIGHT_FILTER_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_LIGHT_FILTER_PARAM_RETURN, value, presponselen, "get light_filter timeout...", timeout);
    }
    case NVPFM_CONFIDENCE:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_CONFIDENCE_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_CONFIDENCE_PARAM_RETURN, value, presponselen, "get confidence timeout...", timeout);
    }
    case NVPFM_SPATIAL_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_SPATIAL_FILTER_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_SPATIAL_FILTER_PARAM_RETURN, value, presponselen, "get spatial filter timeout...", timeout);
    }
    case NVPFM_TEMPORAL_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_TEMPORAL_FILTER_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_TEMPORAL_FILTER_PARAM_RETURN, value, presponselen, "get temporal filter timeout...", timeout);
    }
    case NVPFM_SPECKLE_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_SPECKLE_FILTER_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_SPECKLE_FILTER_PARAM_RETURN, value, presponselen, "get speckle filter timeout...", timeout);
    }
    case NVPFM_HIGHLIGHT_FILTER:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_COMMAND_GET_DEPTH_HIGH_LIGHT_PROCESS_PARAM, sendvalue, sendlen, NVPFM_COMMAND_GET_DEPTH_HIGH_LIGHT_PROCESS_PARAM_RETURN, value, presponselen, "get highlight filter timeout...", timeout);
    }
    case NVPFM_DEPTH_CONFIG:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_DSP_PROCESS, NVPFM_DSP_GET_STATIC_CONFIG_COMMAND, sendvalue, sendlen, NVPFM_DSP_GET_STATIC_CONFIG_COMMAND_RETURN, value, presponselen, "get depth_config timeout...", timeout);
    }
    case NVPFM_RUN_MODE:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_APP_RUN_CONFIG_COMMAND, sendvalue, sendlen, NVPFM_COMMAND_GET_APP_RUN_CONFIG_COMMAND_RETURN, value, presponselen, "get run_mode timeout...", timeout);
    }
    case NVPFM_IR_EXPOSURE:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND, sendvalue, sendlen, NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND_RETURN, value, presponselen, "get ir_exposure timeout...", timeout);
    }
    case NVPFM_RGB_EXPOSURE:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_IMAGE_PROCESS, NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND, sendvalue, sendlen, NVPFM_IMAGE_GET_SENSOR_EXPOSURE_COMMAND_RETURN, value, presponselen, "get rgb_exposure timeout...", timeout);
    }
    case NVPFM_PROJECTOR_STATUS:
    {
      return NVPFM_GET_VALUE(inst, NVPFM_COMMAND_DATA, NVPFM_COMMAND_GET_PROJECTOR_CONFIG_COMMAND, sendvalue, sendlen, NVPFM_COMMAND_GET_PROJECTOR_CONFIG_COMMAND_RETURN, value, presponselen, "get projector_status timeout...", timeout);
    }
    case NVPFM_DEVICE_INFO:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA,
                             NVPFM_COMMAND_GET_DEVICE_INFO_COMMAND,
                             sendvalue, sendlen,
                             NVPFM_COMMAND_GET_DEVICE_INFO_COMMAND_RETURN,
                             value, presponselen, "get camera info timeout",
                             2000);
    }
    case NVPFM_TRANSFER_CONFIG:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_COMMAND_DATA,
                             NVPFM_COMMAND_GET_TRANSFER_COMMAND,
                             sendvalue, sendlen,
                             NVPFM_COMMAND_GET_TRANSFER_COMMAND_RETURN,
                             value, presponselen, "get transfer config timeout",
                             2000);
    }
    case NVPFM_LED_CONFIG:
    {
      return NVPFM_GET_VALUE(
          inst,
          NVPFM_COMMAND_DATA,
          NVPFM_COMMAND_GET_LED_CONFIG_COMMAND,
          sendvalue, sendlen,
          NVPFM_COMMAND_GET_LED_CONFIG_COMMAND_RETURN,
          value, presponselen, "get led config timeout",
          2000);
    }
    case NVPFM_RUN_READY_MODE:
    {
      return NVPFM_GET_VALUE(
          inst,
          NVPFM_COMMAND_DATA,
          NVPFM_COMMAND_GET_RUN_AT_READY_MODE,
          sendvalue, sendlen,
          NVPFM_COMMAND_GET_RUN_AT_READY_MODE_RETURN,
          value, presponselen, "get run at ready mode timeout",
          2000);
    }
    case NVPFM_INFER_DEPTH_PROCESS:
    {
      return NVPFM_GET_VALUE(inst,
                             NVPFM_DSP_PROCESS,
                             NVPFM_COMMAND_GET_INFER_DEPTH_PROCESS_PARAM,
                             sendvalue,
                             sendlen,
                             NVPFM_COMMAND_GET_INFER_DEPTH_PROCESS_PARAM_RETURN,
                             value,
                             presponselen,
                             "get infer depth process timeout!",
                             timeout);
    }
    default:
    {
      return NVPTL_NOTSUPPORTED;
    }
    }
  }
  else
  {
    nvpfm_error_printf("not connected,just return NVPTL_NOTCONNECT\n");
    return NVPTL_NOTCONNECT;
  }
  return NVPTL_FAILED;
}

s_nvpfm_imu_internal_reference *nvpfm_getimuinternalref(NVPFM_DEVICE_HANDLE handle, s_nvpfm_imu_internal_reference *imuinternalref)
{
  if (!nvpfm_hasconnect(handle))
  {
    return NULL;
  }
  s_get_imu_param senddata;
  senddata.channel = NVPFM_IMU_CHANNEL0;
  senddata.type = 3;
  s_get_imu_param_result *recvdata = (s_get_imu_param_result *)calloc(1, sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE);
  int recvlen = sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE;
  if (NVPTL_OK == nvpfm_get(handle, NVPFM_IMU_PARAM, &senddata, sizeof(s_get_imu_param), recvdata, &recvlen, 2000))
  {
    // parse imu_internal_param.txt to get s_nvpfm_imu_internal_reference
    char *buffer = (char *)recvdata->data;
    // printf("recv imu internal yaml:len:%d,\n%s\n", recvlen, buffer);
    char *p = NULL;
    p = strtok(buffer, "\r\n: ");
    int count = 0;
    while (p != NULL)
    {
      char *tmpp = strdup(p);
      p = strtok(NULL, "\r\n: ");
      if (0 == strcmp(tmpp, "Accel_m[0][0]"))
      {
        //  printf("%s:%s\n", tmpp, p);
        imuinternalref->accel_m[0] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[0][1]"))
      {
        //   printf("%s:%s\n", tmpp, p);
        imuinternalref->accel_m[1] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[0][2]"))
      {
        //    printf("%s:%s\n", tmpp, p);
        imuinternalref->accel_m[2] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[1][0]"))
      {
        //    printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_m[3] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[1][1]"))
      {
        //   printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_m[4] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[1][2]"))
      {
        //    printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_m[5] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[2][0]"))
      {
        //   printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_m[6] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[2][1]"))
      {
        //    printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_m[7] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_m[2][2]"))
      {
        //   printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_m[8] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_b[0]"))
      {
        //   printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_b[0] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_b[1]"))
      {
        //   printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_b[1] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Accel_b[2]"))
      {
        //    printf("%s:%s\n", tmpp, p);

        imuinternalref->accel_b[2] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Gyro_b[0]"))
      {
        //     printf("%s:%s\n", tmpp, p);

        imuinternalref->gyro_b[0] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Gyro_b[1]"))
      {
        //    printf("%s:%s\n", tmpp, p);

        imuinternalref->gyro_b[1] = (float)atof(p);
        count++;
      }
      else if (0 == strcmp(tmpp, "Gyro_b[2]"))
      {
        //     printf("%s:%s\n", tmpp, p);

        imuinternalref->gyro_b[2] = (float)atof(p);
        count++;
      }

      free(tmpp);
    }

    free(recvdata);
    if (count == 15)
    {
      return imuinternalref;
    }
    else
    {
      printf("get imuinternalref failed!!!%d\n", count);
      return NULL;
    }
  }

  free(recvdata);
  return NULL;
}
#ifndef USEYAMLCPP
static void parseimuconfig(const char *buffer, int bufferlen, float *values)
{
  yaml_parser_t parser;
  yaml_token_t token;

  if (!yaml_parser_initialize(&parser))
    fputs("Failed to initialize parser!\n", stderr);
  yaml_parser_set_input_string(&parser, (const unsigned char *)buffer, bufferlen);

  int startindex = -1;
  int state = 0;
  do
  {
    char *tk;

    yaml_parser_scan(&parser, &token);
    switch (token.type)
    {
    case YAML_KEY_TOKEN:
      state = 0;
      break;
    case YAML_VALUE_TOKEN:
      state = 1;
      break;
    case YAML_SCALAR_TOKEN:
      tk = (char *)token.data.scalar.value;
      if (state == 0)
      {
        //	printf("got key token:%s\n", tk);

        if (!strcmp(tk, "accelerometer_noise_density"))
        {
          startindex = 0;
        }
        else if (!strcmp(tk, "accelerometer_random_walk"))
        {

          startindex = 1;
        }
        else if (!strcmp(tk, "gyroscope_noise_density"))
        {
          startindex = 2;
        }
        else if (!strcmp(tk, "gyroscope_random_walk"))
        {
          startindex = 3;
        }
        else
        {
          startindex = -1;
        }
      }
      else
      {
        //	printf("got value token:%s\n", tk);
        if (startindex >= 0 && startindex <= 3)
        {
          float floatvalue = atof(tk);
          //		printf("got float:%f\n", floatvalue);
          values[startindex] = floatvalue;
          startindex++;
        }
      }
      break;
    default:
      break;
    }
    if (token.type != YAML_STREAM_END_TOKEN)
      yaml_token_delete(&token);
  } while (token.type != YAML_STREAM_END_TOKEN);
  yaml_token_delete(&token);

  yaml_parser_delete(&parser);
}
static float parsecamimushift(const char *buffer, int bufflen)
{
  float ret = 0;
  yaml_parser_t parser;
  yaml_token_t token;

  if (!yaml_parser_initialize(&parser))
    fputs("Failed to initialize parser!\n", stderr);
  yaml_parser_set_input_string(&parser, (const unsigned char *)buffer, bufflen);

  int startindex = -1;
  int state = 0;
  do
  {
    char *tk;

    yaml_parser_scan(&parser, &token);
    switch (token.type)
    {
    case YAML_KEY_TOKEN:
      state = 0;
      break;
    case YAML_VALUE_TOKEN:
      state = 1;
      break;
    case YAML_SCALAR_TOKEN:
      tk = (char *)token.data.scalar.value;
      if (state == 0)
      {
        //	printf("got key token:%s\n", tk);
        if (!strcmp(tk, "timeshift_cam_imu"))
        {
          startindex = 0;
        }
      }
      else
      {
        //	printf("got value token:%s\n", tk);
        if (startindex >= 0 && startindex <= 15)
        {
          float floatvalue = atof(tk);
          //	printf("got float:%f\n", floatvalue);
          ret = floatvalue;
          break;
        }
      }
      break;
    default:
      break;
    }
    if (token.type != YAML_STREAM_END_TOKEN)
      yaml_token_delete(&token);
  } while (token.type != YAML_STREAM_END_TOKEN);
  yaml_token_delete(&token);

  yaml_parser_delete(&parser);

  return ret;
}
static void parsecamimuconfig(const char *buffer, int bufflen, float *values)
{
  yaml_parser_t parser;
  yaml_token_t token;

  if (!yaml_parser_initialize(&parser))
    fputs("Failed to initialize parser!\n", stderr);
  yaml_parser_set_input_string(&parser, (const unsigned char *)buffer, bufflen);

  int startindex = -1;
  int state = 0;
  do
  {
    char *tk;

    yaml_parser_scan(&parser, &token);
    switch (token.type)
    {
    case YAML_KEY_TOKEN:
      state = 0;
      break;
    case YAML_VALUE_TOKEN:
      state = 1;
      break;
    case YAML_SCALAR_TOKEN:
      tk = (char *)token.data.scalar.value;
      if (state == 0)
      {
        //	printf("got key token:%s\n", tk);
        if (!strcmp(tk, "T_cam_imu"))
        {
          startindex = 0;
        }
      }
      else
      {
        //	printf("got value token:%s\n", tk);
        if (startindex >= 0 && startindex <= 15)
        {
          float floatvalue = atof(tk);
          //	printf("got float:%f\n", floatvalue);
          values[startindex] = floatvalue;
          startindex++;
        }
      }
      break;
    default:
      break;
    }
    if (token.type != YAML_STREAM_END_TOKEN)
      yaml_token_delete(&token);
  } while (token.type != YAML_STREAM_END_TOKEN);
  yaml_token_delete(&token);

  yaml_parser_delete(&parser);
}
s_nvpfm_imu_external_reference *nvpfm_getimuexternalref(NVPFM_DEVICE_HANDLE handle, s_nvpfm_imu_external_reference *imuexternalref)
{
  if (!nvpfm_hasconnect(handle))
  {
    return NULL;
  }

  BOOL gotimuyaml = FALSE;
  BOOL gotcamchain = FALSE;
  // 1.get imu.yaml
  s_get_imu_param senddata1;
  senddata1.channel = NVPFM_IMU_CHANNEL0;
  senddata1.type = 1;
  s_get_imu_param_result *recvdata1 = (s_get_imu_param_result *)calloc(1, sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE);
  int recvlen1 = sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE;
  if (NVPTL_OK == nvpfm_get(handle, NVPFM_IMU_PARAM, &senddata1, sizeof(s_get_imu_param), recvdata1, &recvlen1, 2000))
  {
    if (recvdata1->result == 0)
      gotimuyaml = TRUE;
    else
      printf("got response of imu param,but resuilt is not 0!\n");
  }
  else
  {
    printf("fail to get imu param of type 1!\n");
  }

  s_get_imu_param senddata2;
  senddata2.channel = NVPFM_IMU_CHANNEL0;
  senddata2.type = 2;
  s_get_imu_param_result *recvdata2 = (s_get_imu_param_result *)calloc(1, sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE);
  int recvlen2 = sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE;
  if (NVPTL_OK == nvpfm_get(handle, NVPFM_IMU_PARAM, &senddata2, sizeof(s_get_imu_param), recvdata2, &recvlen2, 2000))
  {
    // 2.get camchain-imucam.yaml
    if (recvdata2->result == 0)
      gotcamchain = TRUE;
    else
      printf("got response of type 2 of imu param,but resuilt is not 0!\n");
  }
  else
  {
    printf("fail to get imu param of type 2!\n");
  }

  if (gotimuyaml && gotcamchain)
  {
    float camimus[16];
    parsecamimuconfig(recvdata2->data, strlen(recvdata2->data), &camimus[0]);

    float imus[4];
    parseimuconfig(recvdata1->data, strlen(recvdata1->data), &imus[0]);

    for (int row = 0; row < 4; row++)
      for (int col = 0; col < 4; col++)
      {
        imuexternalref->t_cam_imu[row * 4 + col] = camimus[row * 4 + col];
      }
    // printf("will parse acc and gyro!\n");
    imuexternalref->acc_noise_density = imus[0];  // imuconfig["imu0"]["accelerometer_noise_density"].as<float>();
    imuexternalref->acc_random_walk = imus[1];    // imuconfig["imu0"]["accelerometer_random_walk"].as<float>();
    imuexternalref->gyro_noise_density = imus[2]; // imuconfig["imu0"]["gyroscope_noise_density"].as<float>();
    imuexternalref->gyro_random_walk = imus[3];   // imuconfig["imu0"]["gyroscope_random_walk"].as<float>();
    imuexternalref->timeshift_cam_imu = parsecamimushift(recvdata2->data, strlen(recvdata2->data));
    ; // imuconfig["imu0"]["gyroscope_random_walk"].as<float>();

    // printf("got imuexternalref:\n");
    free(recvdata1);
    free(recvdata2);
    return imuexternalref;
  }
  free(recvdata1);
  free(recvdata2);
  // 1.download two yaml file
  return NULL;
}
#endif

const char *nvpfm_getsdkversion()
{
  return SDKVERSION(MAINVER, SUBVER1, SUBVER2);
}

void nvpfm_zerorect(uint16_t *depthdata, int width, int height, int fromcolindex, int tocolindex, int fromrowindex, int torowindex)
{
  if (fromcolindex < 0 || fromcolindex >= width ||
      tocolindex < 0 || tocolindex >= width ||
      fromrowindex < 0 || fromrowindex >= height ||
      torowindex < 0 || torowindex >= height ||
      fromcolindex > tocolindex || fromrowindex > torowindex)
  {
    return;
  }
  for (int i = fromrowindex; i <= torowindex; i++)
  {
    uint16_t *linestart = depthdata + i * width + fromcolindex;
    memset((uint8_t *)linestart, 0, (tocolindex - fromcolindex + 1) * 2);
  }
}
void nvpfm_globaltimeenable(NVPFM_DEVICE_HANDLE handle, int enable)
{
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  if (inst != NULL)
  {
    inst->global_time_enabled = enable;
  }
}
NVPTL_RESULT nvpfm_savergb2jpg(NVPTL_USBHeaderDataPacket *packet, const char *jpgfile)
{
  NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)packet;
  const char *pYUVBuffer = (const char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket) + sizeof(NVPFM_USB_IMAGE_HEADER);
  int nWidth = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  int nHeight = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket)))->height;
#ifndef _WINDOWS
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  JSAMPROW row_pointer[1];
  FILE *pJpegFile = NULL;
  unsigned char *yuvbuf = NULL;
  unsigned char *ybase = NULL, *ubase = NULL;
  int i = 0, j = 0;
  int idx = 0;

  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_compress(&cinfo);

  if ((pJpegFile = fopen(jpgfile, "wb")) == NULL)
  {
    return NVPTL_FAILED;
  }
  jpeg_stdio_dest(&cinfo, pJpegFile);

  // image width and height, in pixels

  cinfo.image_width = nWidth;
  cinfo.image_height = nHeight;
  cinfo.input_components = 3;       // # of color components per pixel
  cinfo.in_color_space = JCS_YCbCr; // colorspace of input image
  jpeg_set_defaults(&cinfo);
  jpeg_set_quality(&cinfo, JPEG_QUALITY, TRUE);

  cinfo.jpeg_color_space = JCS_YCbCr;
  cinfo.comp_info[0].h_samp_factor = 2;
  cinfo.comp_info[0].v_samp_factor = 2;
  jpeg_start_compress(&cinfo, TRUE);

  yuvbuf = (unsigned char *)malloc(nWidth * 3);
  if (yuvbuf == NULL)
  {
    return NVPTL_FAILED;
  }
  memset(yuvbuf, 0, nWidth * 3);

  ybase = (unsigned char *)pYUVBuffer;
  ubase = (unsigned char *)(pYUVBuffer + nWidth * nHeight);
  while (cinfo.next_scanline < cinfo.image_height)
  {
    idx = 0;
    for (i = 0; i < nWidth; i++)
    {
      yuvbuf[idx++] = ybase[i + j * nWidth];
      yuvbuf[idx++] = ubase[j / 2 * nWidth + (i / 2) * 2];
      yuvbuf[idx++] = ubase[j / 2 * nWidth + (i / 2) * 2 + 1];
    }
    row_pointer[0] = yuvbuf;
    jpeg_write_scanlines(&cinfo, row_pointer, 1);
    j++;
  }

  jpeg_finish_compress(&cinfo);
  jpeg_destroy_compress(&cinfo);
  fclose(pJpegFile);
  free(yuvbuf);
#endif
  return NVPTL_OK;
}
