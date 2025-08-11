#ifndef __NVPTL_NET__
#define __NVPTL_NET__


#ifdef __cplusplus
extern "C" {
#endif

	void nvptl_net_init();

	void nvptl_net_deinit();

	NVPTL_RESULT nvptl_net_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices);

	NVPTL_DEVICE_HANDLE nvptl_net_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void* userdata);

	void nvptl_net_close(NVPTL_DEVICE_HANDLE handle);

	int nvptl_net_send(NVPTL_DEVICE_HANDLE handle, unsigned char* sendbuffer, size_t len);

#ifdef __cplusplus
}
#endif
#endif