#ifndef __NVPTL_V4LUVC__
#define __NVPTL_V4LUVC__

#ifdef __cplusplus
extern "C"
{
#endif

    // Metadata streaming nodes are available with kernels 4.16+

    void nvptl_v4luvc_init();

    void nvptl_v4luvc_deinit();

    NVPTL_RESULT nvptl_v4luvc_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices);

    NVPTL_DEVICE_HANDLE nvptl_v4luvc_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata);

    void nvptl_v4luvc_close(NVPTL_DEVICE_HANDLE handle);

    int nvptl_v4luvc_send(NVPTL_DEVICE_HANDLE handle, unsigned char *sendbuffer, size_t len);

#ifdef __cplusplus
}
#endif
#endif