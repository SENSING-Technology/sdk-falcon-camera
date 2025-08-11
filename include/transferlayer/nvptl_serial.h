#ifndef NVPTL_SERIAL_H
#define NVPTL_SERIAL_H 1
#ifndef _WINDOWS
#include "libusb-1.0/libusb.h"
#include "nvptl.h"
#endif

//#include "libusb-1.0/libusb.h"

#ifdef __cplusplus
extern "C"
{
#endif
/*
  extern void nvptl_serial_init();
  extern void nvptl_serial_deinit();
  extern NVPTL_RESULT nvptl_serial_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices);
  extern NVPTL_DEVICE_HANDLE nvptl_serial_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata);
  extern void nvptl_serial_close(NVPTL_DEVICE_HANDLE handle);
  extern int nvptl_serial_send(NVPTL_DEVICE_HANDLE handle, unsigned char *sendBuffer, size_t len);
  */
    void nvptl_serial_init();
    void nvptl_serial_deinit();
    NVPTL_RESULT nvptl_serial_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices);
	NVPTL_DEVICE_HANDLE nvptl_serial_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void* userdata);
	void nvptl_serial_close(NVPTL_DEVICE_HANDLE handle);
	int nvptl_serial_send(NVPTL_DEVICE_HANDLE handle, unsigned char* sendBuffer, size_t len);

#ifdef __cplusplus
}
#endif

#endif