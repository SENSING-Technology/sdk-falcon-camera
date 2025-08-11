#ifndef __NVPTL_COMMONDEF__
#define __NVPTL_COMMONDEF__
#ifdef _WINDOWS
#include <Windows.h>
#include <process.h>
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
typedef int int32_t;
typedef short int16_t;
typedef unsigned long long uint64_t;
#else
#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#endif
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  uint16_t VID;
  uint16_t PID;
} VIDPID;

typedef enum {
  PLUGOUT,
  CANCELLED,
  HASSTOP,
  INFORMREBOOT,
} EVENTREASON;
typedef void (*FRAMECALLBACK)(void *data, void *userdata);
typedef void (*GROUPFRAMECALLBACK)(void *depthframe, void *rgbframe, void *leftirframe, void *rightirframe, void *userdata);
typedef void (*EVENTCALLBACK)(EVENTREASON reason, void *userdata);
typedef void (*DEVICECALLBACK)(const char *devicename, void *userdata);

#ifdef _WINDOWS
typedef void *FD_HANDLE;
// typedef usb_dev_handle *USBHANDLE;
typedef HANDLE THREADHANDLE;
typedef HANDLE SEM_T;
typedef struct timeval2 MYTIMEVAL;
#else
#include <unistd.h>
typedef int FD_HANDLE;
typedef void *HANDLE;
#define INVALID_SOCKET (-1)
typedef int SOCKET;
typedef pthread_t THREADHANDLE;
typedef sem_t SEM_T;
typedef struct timeval MYTIMEVAL;
#endif
#define NVPTL_VID 0x1d6b
#define NVPTL_PID 0x0102

#ifdef __cplusplus
}
#endif
#endif