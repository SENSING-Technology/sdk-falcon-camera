#ifdef _WINDOWS
#include <Windows.h>
#include <process.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "commondef.h"
#include "transferlayer/commondata.h"
#include "utils.h"

void nvptl_init()
{
	nvptl_libusb_init();
#ifdef _WINDOWS
	nvptl_serial_init();
	nvptl_mfuvc_init();
#else
	nvptl_v4luvc_init();
#endif
	nvptl_net_init();
}

void nvptl_deinit()
{
	nvptl_libusb_deinit();
#ifdef _WINDOWS
	nvptl_serial_deinit();
	nvptl_mfuvc_deinit();
#else
	nvptl_v4luvc_deinit();
#endif
	nvptl_net_deinit();
}

void nvptl_freedevices(NVPTL_DEVICE_INFO *thedevice)
{
	if (thedevice == NULL)
		return;
	free(thedevice);

	/*	NVPTL_DEVICE_INFO *tmpdevice = thedevice;
		NVPTL_DEVICE_INFO *nextdevice = NULL;
		do
		{
			NVPTL_DEVICE_INFO *nextdevice = tmpdevice->next;
			free(tmpdevice);
			tmpdevice = nextdevice;
		} while (nextdevice != NULL);*/
}
typedef struct
{
	NVPTL_ENUM_CALLBACK callback;
	void *userdata;
} ENUMPARAM;
#ifdef _WINDOWS
static unsigned __stdcall enum_falcon_thread(void *param)
#else
static void *enum_falcon_thread(void *param)
#endif
{
	ENUMPARAM *tmpparam = (ENUMPARAM *)param;
	ENUMPARAM theparam = *tmpparam;
	free(tmpparam);
	int total = 0;
	NVPTL_DEVICE_INFO *resultdevices = NULL;
	{
		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_libusb_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices = ptmpdevices;
				total += tmptotal;
			}
		}
	}
	{
		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
#ifdef _WINDOWS
		if (NVPTL_OK == nvptl_mfuvc_enum(&tmptotal, &ptmpdevices))
#else
		if (NVPTL_OK == nvptl_v4luvc_enum(&tmptotal, &ptmpdevices))
#endif
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}
	}
	{
		// LARGE_INTEGER nFreq;
		// LARGE_INTEGER t1;
		// LARGE_INTEGER t2;
		// double dt;
		// QueryPerformanceFrequency(&nFreq);
		// QueryPerformanceCounter(&t1);

		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_net_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}

		// QueryPerformanceCounter(&t2);
		// dt = (t2.QuadPart - t1.QuadPart) / (double)nFreq.QuadPart;
		// printf("nvptl_net_enum timeout %.02f \n", dt);
	}
	// if (total > 0) {
	theparam.callback(total, resultdevices, theparam.userdata);
	//}
	return 0;
}

NVPTL_RESULT nvptl_enum_sync_mask(int *ptotal, NVPTL_DEVICE_INFO **ppdevices, uint8_t mask)
{
	int total = 0;
	NVPTL_DEVICE_INFO *resultdevices = NULL;
	if (mask & ENUMUSB)
	{
		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_libusb_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}
	}
	if (mask & ENUMUVC)
	{
		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
#ifdef _WINDOWS
		if (NVPTL_OK == nvptl_mfuvc_enum(&tmptotal, &ptmpdevices))
#else
		if (NVPTL_OK == nvptl_v4luvc_enum(&tmptotal, &ptmpdevices))
#endif
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}
	}

	if (mask & ENUMNET)
	{
		/*	struct timeval2 starttm,endtm;
		gettimeofday(&starttm, NULL);*/

		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_net_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}

		/*	gettimeofday(&endtm, NULL);
		double netenumcost = (double)endtm.tv_sec - (double)starttm.tv_sec + ((double)endtm.tv_usec - (double)starttm.tv_usec)/1000000.0;
		printf("nvptl_net_enum timeout %.02f \n", netenumcost);*/
	}
	if (mask & ENUMSERIAL)
	{
#ifdef _WINDOWS
		/*	struct timeval2 starttm,endtm;
		gettimeofday(&starttm, NULL);*/

		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_serial_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}

		/*	gettimeofday(&endtm, NULL);
		double netenumcost = (double)endtm.tv_sec - (double)starttm.tv_sec + ((double)endtm.tv_usec - (double)starttm.tv_usec)/1000000.0;
		printf("nvptl_net_enum timeout %.02f \n", netenumcost);*/
#endif
	}

	*ptotal = total;
	if (total == 0)
	{
		*ppdevices = NULL;
	}
	else
	{
		*ppdevices = (NVPTL_DEVICE_INFO *)calloc(total, sizeof(NVPTL_DEVICE_INFO));
		NVPTL_DEVICE_INFO *tmpinfo = resultdevices;
		int i = 0;
		while (tmpinfo != NULL)
		{
			(*ppdevices)[i] = *tmpinfo;
			if ((*ppdevices)[i].next != NULL)
				(*ppdevices)[i].next = &(*ppdevices)[i + 1];
			NVPTL_DEVICE_INFO *tobedelete = tmpinfo;
			tmpinfo = tmpinfo->next;
			free(tobedelete);
			i++;	
		}
	}

	return NVPTL_OK;
}
NVPTL_RESULT nvptl_enum_sync(int *ptotal, NVPTL_DEVICE_INFO **ppdevices)
{
	int total = 0;
	NVPTL_DEVICE_INFO *resultdevices = NULL;
	{
		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_libusb_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices = ptmpdevices;
				total += tmptotal;
			}
		}
	}
	{
		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
#ifdef _WINDOWS
		if (NVPTL_OK == nvptl_mfuvc_enum(&tmptotal, &ptmpdevices))
#else
		if (NVPTL_OK == nvptl_v4luvc_enum(&tmptotal, &ptmpdevices))
#endif
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}
	}
	{
		/*	struct timeval2 starttm,endtm;
			gettimeofday(&starttm, NULL);*/

		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_net_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}

		/*	gettimeofday(&endtm, NULL);
			double netenumcost = (double)endtm.tv_sec - (double)starttm.tv_sec + ((double)endtm.tv_usec - (double)starttm.tv_usec)/1000000.0;
			printf("nvptl_net_enum timeout %.02f \n", netenumcost);*/
	}
#ifdef _WINDOWS
	{
		/*	struct timeval2 starttm,endtm;
		gettimeofday(&starttm, NULL);*/

		int tmptotal = 0;
		NVPTL_DEVICE_INFO *ptmpdevices = NULL;
		if (NVPTL_OK == nvptl_serial_enum(&tmptotal, &ptmpdevices))
		{
			if (tmptotal > 0)
			{
				resultdevices == NULL ? (resultdevices = ptmpdevices) : (resultdevices->next = ptmpdevices);
				total += tmptotal;
			}
		}

		/*	gettimeofday(&endtm, NULL);
		double netenumcost = (double)endtm.tv_sec - (double)starttm.tv_sec + ((double)endtm.tv_usec - (double)starttm.tv_usec)/1000000.0;
		printf("nvptl_net_enum timeout %.02f \n", netenumcost);*/
	}
#endif

	*ptotal = total;
	if (total == 0)
	{
		*ppdevices = NULL;
	}
	else
	{
		*ppdevices = (NVPTL_DEVICE_INFO *)calloc(total, sizeof(NVPTL_DEVICE_INFO));
		NVPTL_DEVICE_INFO *tmpinfo = resultdevices;
		int i = 0;
		while (tmpinfo != NULL)
		{
			(*ppdevices)[i] = *tmpinfo;
			if ((*ppdevices)[i].next != NULL)
				(*ppdevices)[i].next = &(*ppdevices)[i + 1];
			NVPTL_DEVICE_INFO *tobedelete = tmpinfo;
			tmpinfo = tmpinfo->next;
			free(tobedelete);
			i++;	
		}
	}

	return NVPTL_OK;
}
NVPTL_RESULT nvptl_enum(NVPTL_ENUM_CALLBACK callback, void *userdata) // int *ptotal, NVPTL_DEVICE_INFO **ppdevices)
{
	ENUMPARAM *param = (ENUMPARAM *)malloc(sizeof(ENUMPARAM));
	param->callback = callback;
	param->userdata = userdata;

	THREADHANDLE enumhandle;
#ifdef _WINDOWS
	DWORD threadID;
	enumhandle = (THREADHANDLE)_beginthreadex(NULL, 0, enum_falcon_thread, (LPVOID)param, 0, &threadID);
	CloseHandle(enumhandle);
#else
	pthread_create(&enumhandle, 0, enum_falcon_thread, (void *)param);
	pthread_detach(enumhandle);
#endif

	return NVPTL_OK;
}

NVPTL_DEVICE_HANDLE nvptl_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata)
{
	if (dev_info->type == NVPTL_USB_INTERFACE)
	{
		return nvptl_libusb_open(dev_info, callback, userdata);
	}
#ifdef _WINDOWS
	else if (dev_info->type == NVPTL_SERIAL_INTERFACE)
	{
		return nvptl_serial_open(dev_info, callback, userdata);
	}
#endif
	else if (dev_info->type == NVPTL_UVC_INTERFACE)
	{
#ifdef _WINDOWS
		return nvptl_mfuvc_open(dev_info, callback, userdata);
#else
		return nvptl_v4luvc_open(dev_info, callback, userdata);
#endif
	}
	else if (dev_info->type == NVPTL_NETWORK_INTERFACE)
	{
		return nvptl_net_open(dev_info, callback, userdata);
	}
	return NULL;
}
#ifdef _TEST
void readframecallback(NVPTL_DEVICE_HANDLE handle, void *framedata, size_t len, void *userdata)
{
	NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)framedata;
	printf("read whole frame ok!len:%u,type:%d,sub_type:%d\n", len, tmppack->type, tmppack->sub_type);
	/*NVPTL_INSTANCE* inst = (NVPTL_INSTANCE*)userdata;
	switch (inst->devinfo.type) {
	case NVPTL_USB_INTERFACE:
		printf("recv whole frame from usb camera\n");
		break;
	case NVPTL_UVC_INTERFACE:
		printf("recv whole frame from uvc camera\n");
		break;
	case NVPTL_NETWORK_INTERFACE:
		printf("recv whole frame from net camera\n");
		break;
	default:
		printf("recv while frame from unknown camera\n");
	}*/
}
#endif
// 4.nvptl_close
void nvptl_close(NVPTL_DEVICE_HANDLE handle)
{
	NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
	// todo judge if handle is usb device or net device
	if (inst->devinfo.type == NVPTL_USB_INTERFACE)
		nvptl_libusb_close(handle);
#ifdef _WINDOWS
	else if (inst->devinfo.type == NVPTL_SERIAL_INTERFACE)
	{
		nvptl_serial_close(handle);
	}
#endif
	else if (inst->devinfo.type == NVPTL_UVC_INTERFACE)
	{
#ifdef _WINDOWS
		nvptl_mfuvc_close(handle);
#else
		nvptl_v4luvc_close(handle);
#endif
	}
	else if (inst->devinfo.type == NVPTL_NETWORK_INTERFACE)
		nvptl_net_close(handle);
}
// 5.nvptl_send,nvptl_send_frame_callback�ص�

int nvptl_send(NVPTL_DEVICE_HANDLE handle, unsigned char *sendBuffer, size_t len)
{
	NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
	// todo judge if handle is usb or net device
	if (inst->devinfo.type == NVPTL_USB_INTERFACE)
		return nvptl_libusb_send(handle, sendBuffer, len);
#ifdef _WINDOWS
	else if (inst->devinfo.type == NVPTL_SERIAL_INTERFACE)
		return nvptl_serial_send(handle, sendBuffer, len);
#endif
	else if (inst->devinfo.type == NVPTL_NETWORK_INTERFACE)
		return nvptl_net_send(handle, sendBuffer, len);
	else if (inst->devinfo.type == NVPTL_UVC_INTERFACE)
	{
#ifdef _WINDOWS
		return nvptl_mfuvc_send(handle, sendBuffer, len);
#else
		return nvptl_v4luvc_send(handle, sendBuffer, len);
#endif
	}
	return NVPTL_UNKNOWN;
}
#ifdef _TEST
static void mycallback(int total, NVPTL_DEVICE_INFO *pinfo, void *userdata)
{
	NVPTL_DEVICE_INFO **ppdevice = (NVPTL_DEVICE_INFO **)userdata;
	*ppdevice = pinfo;
}
int main()
{
	printf("start!!!\n");
	nvptl_init();
	NVPTL_DEVICE_INFO *tmpdevice = NULL;
	nvptl_enum(mycallback, &tmpdevice);
	sleep(1);
	if (tmpdevice != NULL)
	{
		NVPTL_DEVICE_HANDLE handle = nvptl_open(tmpdevice, readframecallback, NULL);
		COMMONUSLEEP(50000 * 1000);
		printf("will close!!!\n");
		nvptl_close(handle);
	}

	nvptl_freedevices(tmpdevice);
	nvptl_deinit();

	printf("stop!!!\n");
	getchar();
	return 0;
}
#endif
