#ifndef __NVPTL__
#define __NVPTL__

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "libusb.h"

#ifdef __cplusplus
extern "C" {
#endif
#define FEYNMAN_ONE_PACKET_DATA_MAX_SIZE (0x400000 - 0x80)
	typedef struct
	{
		uint8_t magic[8];  /**< "NEXT_VPU" magic word */
		uint16_t type;	   /**< primary type */
		uint16_t sub_type; /**< sub type */
		uint32_t checksum; /**< checksum */
		uint32_t len;	   /**< length of data member in bytes*/
		uint8_t data[0];   /**< pure data,mean depends type and sub_type,length depends len */
	} NVPTL_USBHeaderDataPacket;


#define nvptl_debug_printf(fmt,...) printf(fmt,##__VA_ARGS__)
#define nvptl_warn_printf(fmt,...) printf(fmt,##__VA_ARGS__)
#define nvptl_error_printf(fmt,...) printf(fmt,##__VA_ARGS__)

	typedef void *NVPTL_DEVICE_HANDLE;
	typedef enum
	{
		NVPTL_OK,					 /**< result ok:0 */
		NVPTL_NOTCONNECT,			 /**< result not connected:1 */
		NVPTL_RESPONSEQUEUEFULL,	 /**< result response queue full:2 */
		NVPTL_TIMEOUT,				 /**< result timeout:3 */
		NVPTL_NETFAILED,			 /**< result net failed:4 */
		NVPTL_UNKNOWN,				 /**< result unknown:5 */
		NVPTL_NOTSUPPORTED,			 /**< result not supported:6 */
		NVPTL_FAILED,				 /**< result failed:7 */
		NVPTL_NOTREADY,				 /**< result not ready:8 */
		NVPTL_NOTREACHPOSTCONNECTED, /**< result not reach post connected:9 */
	} NVPTL_RESULT;
	typedef enum
	{
		INITIAL = 0,   /**< open procedure status initial:0 */
		CONNECTED,	   /**< open procedure status connected:1 */
		POSTCONNECTED, /**< open procedure status post connected:2 */
		CONFIGURED,	   /**< open procedure status configured:3 */
		STARTED,	   /**< open procedure status started:4 */
		STOPPED,	   /**< open procedure status stopped:5 */
	} NVPTL_STATUS;
	typedef enum
	{
		NVPTL_UNKNOWN_INTERFACE, /**< network interface type of camera:0 */
		NVPTL_NETWORK_INTERFACE, /**< network interface type of camera:1 */
		NVPTL_USB_INTERFACE,	 /**< usb interface type of camera:2 */
		NVPTL_UVC_INTERFACE,	 /**< uvc interface type of camera:3 */
		NVPTL_SERIAL_INTERFACE,
	} NVPTL_INTERFACE_TYPE;

	typedef void(*NVPTL_RECV_FRAME_CALLBACK)(NVPTL_DEVICE_HANDLE handle, unsigned char* framedata, unsigned long len, void* userdata);
	

	// Binary-coded decimal represent the USB specification to which the UVC device complies
	typedef enum  {
		USB_UNDEFINED = 0,
		USB1_TYPE = 0x0100,
		USB1_1_TYPE = 0x0110,
		USB2_TYPE= 0x0200,
		USB2_01_TYPE= 0x0201,
		USB2_1_TYPE = 0x0210,
		USB3_TYPE = 0x0300,
		USB3_1_TYPE = 0x0310,
		USB3_2_TYPE = 0x0320,
	}EM_NVPTL_USB_SPEC;

	typedef struct _nvptl_device_info
	{
		NVPTL_INTERFACE_TYPE type; /**< camera interface type,network or usb */
		union
		{
			char usb_camera_name[256]; /**< usb camera name,only for identify cameras plugged to current computer */
			char net_camera_ip[256];	  /**< network camera name,this is ip of camera */
		};
		struct _nvptl_device_info *next; /**< pointer to next camera device */
		int notexists;//cout for not enumerates
		struct libusb_device_descriptor usb_desc;
	} NVPTL_DEVICE_INFO;

	typedef enum {
		ENUMUSB=1,
		ENUMNET=2,
		ENUMSERIAL=4,
		ENUMUVC=8,
	}ENUMCAMERA;
	void nvptl_init();
	void nvptl_deinit();
	int nvptl_send(NVPTL_DEVICE_HANDLE handle, unsigned char* sendBuffer, size_t len);
	void nvptl_close(NVPTL_DEVICE_HANDLE handle);
	NVPTL_DEVICE_HANDLE nvptl_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback,void* userdata);
	typedef void (*NVPTL_ENUM_CALLBACK)(int total, NVPTL_DEVICE_INFO *pdevices,void* userdata);
	NVPTL_RESULT nvptl_enum(NVPTL_ENUM_CALLBACK callback,void* userdata);// int *ptotal, NVPTL_DEVICE_INFO **ppdevices);
	NVPTL_RESULT nvptl_enum_sync(int *ptotal, NVPTL_DEVICE_INFO **ppdevices);
	NVPTL_RESULT nvptl_enum_sync_mask(int *ptotal, NVPTL_DEVICE_INFO **ppdevices,uint8_t mask);
	void nvptl_freedevices(NVPTL_DEVICE_INFO *thedevice);
	bool nvptl_hasconnect(NVPTL_DEVICE_HANDLE handle);
#ifdef __cplusplus
}
#endif
#endif