#ifndef __NVPTL_COMMONDATA__
#define __NVPTL_COMMONDATA__
#include "commondef.h"
#include "nvptl.h"
#include "nvptl_libusb.h"
#include "nvptl_net.h"
#ifdef _WINDOWS
#include "nvptl_serial.h"
#include "nvptl_mfuvc.h"
#else
#include "nvptl_v4luvc.h"
#endif
#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
    //	struct timeval lastsubmittv;
    void *userdata;
    NVPTL_STATUS status;
    NVPTL_DEVICE_INFO devinfo;
    USBHANDLE device_usb_handle;
#ifdef _WINDOWS
    int device_serial_handle;
    OVERLAPPED wrOverlapped;
    OVERLAPPED wOverlapped;
#endif
    // UVCHANDLE device_uvc_handle;
    unsigned long index;
    int currentpacketlen;

		SOCKET device_net_socket;
		FD_HANDLE device_net_handle;
		HANDLE device_net_thread;
		volatile int device_net_connect;
		void* device_net_sendlist;
		void* device_net_sendlast;
		volatile long device_net_sendcnt;
		void* device_net_recvlist;
		void* device_net_recvlast;
		volatile long device_net_recvcnt;

		int willclose;
		HANDLE closeevent;
		int timestamp_is_ready;

		//int thread_running_flag;
		int global_time_enabled;
		int irwidth;
		int irheight;
		int rgbwidth;
		int rgbheight;
		int rgb_rectify_mode;
		EVENTCALLBACK eventcallback;
		void *connectuserdata;
		//	Ring_Queue *sendqueue;

		int b_save;

		//	THREADHANDLE sendthread;
		uint8_t *usb_buf;
		uint8_t *tmpbuf;

		struct libusb_transfer *transfer;
		struct libusb_transfer *sendtransfer;

		int timesynccycle;
		int b_timesync;
		time_t lasttimesync;
		NVPTL_RECV_FRAME_CALLBACK recvframecallback;
	} NVPTL_INSTANCE;

	typedef struct
	{
		uint8_t magic[8];  /**< "NEXT_VPU" magic word */
		uint16_t type;	   /**< primary type */
		uint16_t sub_type; /**< sub type */
		uint32_t checksum; /**< checksum */
		uint32_t len;	   /**< length of data member in bytes*/
		uint8_t data[0];   /**< pure data,mean depends type and sub_type,length depends len */
	} NVPTL_HeaderDataPacket;


#ifdef __cplusplus
}
#endif
#endif