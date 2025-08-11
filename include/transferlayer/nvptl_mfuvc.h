#ifndef __NVPTL_MFUVC__
#define __NVPTL_MFUVC__
#include <libusb.h>
#ifdef __cplusplus
extern "C" {
#endif
	//class wmf_uvc_device;
	//typedef std::shared_ptr<wmf_uvc_device> UVCHANDLE;

	void nvptl_mfuvc_init();
	void nvptl_mfuvc_set_eventcallback(NVPTL_DEVICE_HANDLE handle,EVENTCALLBACK eventcallback, void* userdata);
	void nvptl_mfuvc_deinit();
	NVPTL_RESULT nvptl_mfuvc_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices);
	NVPTL_DEVICE_HANDLE nvptl_mfuvc_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback,void* userdata);
	void nvptl_mfuvc_close(NVPTL_DEVICE_HANDLE handle);
	int nvptl_mfuvc_send(NVPTL_DEVICE_HANDLE handle, unsigned char* sendBuffer, size_t len);
#ifdef __cplusplus
}
#endif
#endif