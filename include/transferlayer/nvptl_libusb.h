#ifndef __NVPTL_LIBUSB__
#define __NVPTL_LIBUSB__
#include <libusb.h>
#ifdef __cplusplus
extern "C" {
#endif
	
#define NVPTL_ENDPOINT_IN (LIBUSB_ENDPOINT_IN + 1)
#define NVPTL_ENDPOINT_OUT (LIBUSB_ENDPOINT_OUT + 1)
#define USB_PACKET_MAX_SIZE (4 * 1024 * 1024 + 1024)

	typedef void *USBHANDLE;

	void nvptl_libusb_init();

	void nvptl_libusb_deinit();
	NVPTL_RESULT nvptl_libusb_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices);
	NVPTL_DEVICE_HANDLE nvptl_libusb_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback,void* userdata);
	void nvptl_libusb_close(NVPTL_DEVICE_HANDLE handle);
	int nvptl_libusb_send(NVPTL_DEVICE_HANDLE handle, unsigned char* sendBuffer, size_t len);
#ifdef __cplusplus
}
#endif
#endif