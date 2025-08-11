#ifdef _WINDOWS
#include <Windows.h>
#include <process.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "commondef.h"

#include "transferlayer/commondata.h"
#include "utils.h"

static libusb_context *s_context = NULL;
static int bhandleevent = 1;
static THREADHANDLE eventthreadid;

static const VIDPID VPS[] = {
    {0x4E56, 0x5055},
    {0x4E56, 0x5056},
    {0x2BDF, 0x0001}
#ifndef _WINDOWS
    ,
    {0x4E56, 0x594E} // serial on windows use nvptl_serial,on linux use nvptl_libusb
#endif
};
#ifdef _WINDOWS
static unsigned __stdcall eventhandle(void *param)
#else
static void *eventhandle(void *param)
#endif
{
  while (bhandleevent) {
    struct timeval tv = {0, 33000};                       // 10ms timeout
                                                          // libusb_lock_events(NULL);
    int r = libusb_handle_events_timeout(s_context, &tv); // �����κδ��Դ������¼�
                                                          // libusb_unlock_events(NULL);
                                                          // int r = libusb_handle_events(NULL);    //�����κδ��Դ������¼�
    if (r < 0) {
      if (r == -10)
        continue;
      printf("handle events failed!:%d\n", r);
      continue;
      // break;
    }
  }
  return 0;
}
void nvptl_libusb_init() {
  libusb_init(&s_context);
  bhandleevent = 1;

#ifdef _WINDOWS
  DWORD threadID;
  eventthreadid = (THREADHANDLE)_beginthreadex(NULL, 0, eventhandle, (LPVOID)NULL, 0, &threadID);
#else
  pthread_create(&eventthreadid, 0, eventhandle, (void *)NULL);
#endif
}

void nvptl_libusb_deinit() {
  bhandleevent = 0;
#ifdef _WINDOWS
  DWORD retfirst = WaitForSingleObject(eventthreadid, INFINITE);
#else
  pthread_join(eventthreadid, NULL);
#endif

  libusb_exit(s_context);
}

NVPTL_RESULT nvptl_libusb_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices) {
  int countdevices = 0;
  NVPTL_DEVICE_INFO *tmpdevice = NULL;

  libusb_device **devs;

  ssize_t cnt;

  cnt = libusb_get_device_list(s_context, &devs);
  if (cnt < 0) {
    printf("fail to get usb device list!\n");
    //	_LIBUSB_EXIT(NULL);
    return NVPTL_FAILED; // (int)cnt;
  }
  // nvptl_debug_printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
  libusb_device *dev;
  int i = 0, j = 0;
  uint8_t path[8];

  while ((dev = devs[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    int r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0) {
      fprintf(stderr, "failed to get device descriptor");
      libusb_free_device_list(devs, 1);
      //	_LIBUSB_EXIT(NULL);
      return NVPTL_FAILED;
    }
    int bus = libusb_get_bus_number(dev);

    char tmppath[64];
    memset(&tmppath[0], 0, sizeof(tmppath));
    r = libusb_get_port_numbers(dev, path, sizeof(path));
    if (r > 0) {
      sprintf(tmppath, "%d", path[0]);
      for (j = 1; j < r; j++) {
        char thepath[16];
        sprintf(thepath, ".%d", path[j]);
        strcat(tmppath, thepath);
      }
    }
    /*		nvptl_debug_printf("%04x:%04x (bus %d, path %s)\n",
    desc.idVendor, desc.idProduct, bus, tmppath);
    */
    int gotit = 0;
    for (unsigned int i = 0; i < sizeof(VPS) / sizeof(VIDPID); i++) {
      if (desc.idVendor == VPS[i].VID && desc.idProduct == VPS[i].PID) {
        gotit = 1;
        break;
      }
    }
    if (gotit) //(desc.idVendor == NVPTL_VID && desc.idProduct == NVPTL_PID)||(desc.idVendor==NVPFMHK_VID && desc.idProduct==NVPFMHK_PID))
    {
      NVPTL_DEVICE_INFO *thedevice = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
      thedevice->next = tmpdevice;
      sprintf((char *)thedevice->usb_camera_name, "falcon-%d-%s", bus, tmppath);
      thedevice->type = NVPTL_USB_INTERFACE;
      memcpy(&thedevice->usb_desc, &desc, sizeof(thedevice->usb_desc));
      tmpdevice = thedevice;
      countdevices++;

      //	callback((const char *)tmpbuf, userdata);
      //	_LIBUSB_CLOSE(dev_handle);
      //		_LIBUSB_CLOSE(handler);

      //	}
      //	else {
      //		nvptl_debug_printf("open usb ret:%d\n", ret);
      //	}
    }
  }
  //	nvptl_debug_printf("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
  libusb_free_device_list(devs, 1);

  //	_LIBUSB_EXIT(NULL);
  // #endif
  if (countdevices > 0) {
    *ptotal = countdevices;
    *ppdevices = tmpdevice;
    return NVPTL_OK;
  } else
    return NVPTL_FAILED;
}

static libusb_device_handle *my_open_device_with_vid_pid(
    libusb_context *ctx, int bus, char *path) {
  struct libusb_device **devs;
  struct libusb_device *found = NULL;
  struct libusb_device *dev;
  struct libusb_device_handle *dev_handle = NULL;
  size_t i = 0;
  int r, j;

  // nvptl_debug_printf("will get device list!\n");
  if (libusb_get_device_list(ctx, &devs) < 0) {
    nvptl_error_printf("fail to get device list!\n");
    return NULL;
  }

  // nvptl_debug_printf("will while device list!\n");
  while ((dev = devs[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
      goto out;
    int tmpbus = libusb_get_bus_number(dev);

    char tmppath[64];
    memset(&tmppath[0], 0, sizeof(tmppath));
    // nvptl_debug_printf("will get port numbers!\n");
    uint8_t pathcontains[8];
    r = libusb_get_port_numbers(dev, pathcontains, sizeof(pathcontains));
    if (r > 0) {
      sprintf(tmppath, "%d", pathcontains[0]);
      // printf(" path: %d", pathcontains[0]);
      for (j = 1; j < r; j++) {
        char subpath[16];
        sprintf(subpath, ".%d", pathcontains[j]);
        strcat(tmppath, subpath);
      }
    } else {
      nvptl_error_printf("fail to get port numbers!\n");
    }

    // nvptl_debug_printf("will get:path:%s!\n", tmppath);
    if (0 == strcmp(tmppath, path) && tmpbus == bus) //&& (desc.idVendor == NVPFM_VID||desc.idVendor==NVPFMHK_VID) && (desc.idProduct == NVPFM_PID||desc.idProduct==NVPFMHK_PID))
    {
      int gotit = 0;
      for (unsigned int i = 0; i < sizeof(VPS) / sizeof(VIDPID); i++) {
        if (desc.idVendor == VPS[i].VID && desc.idProduct == VPS[i].PID) {
          gotit = 1;
          break;
        }
      }
      if (gotit) {
        found = dev;
        break;
      }
    }
  }

  if (found) {
    r = libusb_open(found, &dev_handle);
    if (r < 0) {
      nvptl_error_printf("fail to open dev_handle!%d\n", r);
      dev_handle = NULL;
    }
  }

out:
  libusb_free_device_list(devs, 1);
  // nvptl_debug_printf("to the end,return dev_handle!\n");
  return dev_handle;
}

static int usb_hal_open(NVPTL_INSTANCE *inst, int bus, char *path) {
  int ret;
  char str[64];
  memset(str, 0, sizeof(str));

  // nvptl_debug_printf("will open bus:%d,path:%s\n", bus, path);
  inst->device_usb_handle = my_open_device_with_vid_pid(s_context, bus, path);
  if (inst->device_usb_handle == NULL) {
    nvptl_error_printf("open device failed\n");
    //_LIBUSB_EXIT(NULL);
    return -1;
  }
  if (libusb_kernel_driver_active(inst->device_usb_handle, 0)) {
    libusb_detach_kernel_driver(inst->device_usb_handle, 0);
  }
  ret = libusb_claim_interface(inst->device_usb_handle, 0);
  if (ret < 0) {
    nvptl_error_printf("claim interface failed:%d\n", ret);
    //_LIBUSB_EXIT(NULL);
    return -1;
  }
  return 0;
}

static void nvptl_recv_frame_callback(NVPTL_DEVICE_HANDLE handle, uint8_t *data, unsigned long len, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  if (inst->recvframecallback != NULL) {
    inst->recvframecallback(handle, data, len, userdata);
  }
}
static void LIBUSB_CALL transfercallback(struct libusb_transfer *transfer) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)transfer->user_data;
  if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
    if (transfer->actual_length > 0) {
      /*	static int count=0;
      count++;
      if(count%6==0)
      printf ("0x%lX,read %d bytes\n", transfer->user_data,transfer->actual_length);
      */

      ////////////////////here,we compose packets to ok frame
      long bytesTransffered = transfer->actual_length;
      inst->index += bytesTransffered;
      if ((long)inst->index >= (long)sizeof(NVPTL_USBHeaderDataPacket)) {
        if (0 == strncmp("NEXT_VPU", (const char *)inst->usb_buf, 8)) {
          inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
          if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
            //	printf("1userdata:0x%X\n", inst);
            nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);
            inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
            if (inst->index > 0) {
              memcpy(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
            } else {
              //	printf("no more data left:index=%d!\n", inst->index);
            }
          } else {
            //	printf("index too small tobe a complete frame:want:%d,has:%d\n", (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
          }
        } else {
          //	printf("first is not NEXT_VPU!will search\n");
          int searchednextvpu = 0;
          for (unsigned long i = 0; i < (inst->index - 8); i++) {
            if (0 == memcmp(&inst->usb_buf[i], "NEXT_VPU", 8)) {
              //	printf("searched NEXT_VPU!\n");
              searchednextvpu = 1;
              memcpy(inst->usb_buf, &inst->usb_buf[i], inst->index - i);
              inst->index = inst->index - i;

              inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
              if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
                //	printf("2userdata:0x%X\n", inst);
                nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);
                inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
                if (inst->index > 0) {
                  memcpy(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
                }
              }
              break;
            }
          }
          if (!searchednextvpu) {
            //	printf("has no nextvpu,so be it!!!\n");
            inst->index = 0;
          }
        }
      } else {
        //	printf("index too small tobe header:%d\n",inst->index);
      }
    }
    if (inst->status != STOPPED) {
      // �ٴ��ύ�������ڽ���
      //	printf("will submit userdata:0x%X\n", inst);
      libusb_fill_bulk_transfer(transfer, inst->device_usb_handle, NVPTL_ENDPOINT_IN, inst->usb_buf + inst->index, 64 * 1024,
                                &transfercallback, inst, 200);

      int rv = libusb_submit_transfer(transfer);
      if (rv < 0) {
        nvptl_warn_printf("just submit transfer failed,error libusb_submit_transfer : %d\n",rv);
        // libusb_strerror (libusb_error (rv)));
        libusb_free_transfer(inst->transfer);
        inst->transfer = NULL;
        if (inst->eventcallback != NULL) {
          inst->eventcallback(HASSTOP, inst->connectuserdata);
        }

      } else {
        /*	struct timeval tv;
          gettimeofday(&tv, NULL);
          int offset=(tv.tv_sec - inst->lastsubmittv.tv_sec) * 1000 + (tv.tv_usec - inst->lastsubmittv.tv_usec) / 1000;
          printf("read submit offset:%d\n", offset);
          inst->lastsubmittv = tv;
          */
        // nvptl_debug_printf("has submit transfer again!\n");
      }
    } else {
      libusb_free_transfer(inst->transfer);
      inst->transfer = NULL;
    }
  } else if (transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
    // nvptl_debug_printf("read transfer timeout!!!\n");
    if (inst->status != STOPPED) {
      // �ٴ��ύ�������ڽ���
      //	printf("will submit userdata:0x%X\n", inst);
      libusb_fill_bulk_transfer(transfer, inst->device_usb_handle, NVPTL_ENDPOINT_IN, inst->usb_buf + inst->index, 64 * 1024,
                                &transfercallback, inst, 200);

      int rv = libusb_submit_transfer(transfer);
      if (rv < 0) {
        nvptl_warn_printf("TIMEDOUT,error libusb_submit_transfer : %d\n",rv);
        // libusb_strerror (libusb_error (rv)));
        libusb_free_transfer(inst->transfer);
        inst->transfer = NULL;
        if (inst->eventcallback != NULL) {
          inst->eventcallback(HASSTOP, inst->connectuserdata);
        }

      } else {
        // nvptl_debug_printf("has submit transfer again!\n");
      }
    } else {
      if (inst->eventcallback != NULL) {
        inst->eventcallback(HASSTOP, inst->connectuserdata);
      }

      libusb_free_transfer(inst->transfer);
      inst->transfer = NULL;
    }
  } else if (transfer->status == LIBUSB_TRANSFER_CANCELLED) { // ȡ������
                                                              // �ͷŴ���ṹ
    if (inst->usb_buf != NULL) {
      free(inst->usb_buf);
      inst->usb_buf = NULL;
    }
    nvptl_error_printf("transfer cancelled!\n");

    if (inst->eventcallback != NULL) {
      inst->eventcallback(CANCELLED, inst->connectuserdata);
    }

    nvptl_error_printf("transfer free!\n");
    libusb_free_transfer(inst->transfer);
    inst->transfer = NULL;
  } else {
    nvptl_debug_printf("else status:%d\n", transfer->status);
    if ((transfer->status == LIBUSB_TRANSFER_NO_DEVICE || transfer->status == LIBUSB_TRANSFER_STALL|| transfer->status == LIBUSB_TRANSFER_ERROR) &&
        inst->eventcallback != NULL) {
      inst->eventcallback(PLUGOUT, inst->connectuserdata);
    }

    if (inst->usb_buf != NULL) {
      free(inst->usb_buf);
      inst->usb_buf = NULL;
    }
    libusb_free_transfer(inst->transfer);
    inst->transfer = NULL;
  }
}

static bool async_read(NVPTL_DEVICE_HANDLE nvpfmhandle) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)nvpfmhandle;

  if (inst->usb_buf == NULL)
    inst->usb_buf = (uint8_t *)malloc(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);

  struct libusb_device_handle *handle = inst->device_usb_handle;
  int rc = 0;

  // ѭ��100�� �ύ100�δ���ṹ���������ύ��100������ȴ����ܣ��������ᶪ��
  for (int i = 0; i < 1; i++) {
    // �����첽����ṹ
    inst->transfer = libusb_alloc_transfer(0);
    // ���ٽ����ڴ��ַ
    //      unsigned char *buf = (unsigned char*)malloc(128 * 1024);
    // ����ڴ�
    //      memset (buf, 0, 128*1024);
    inst->transfer->actual_length = 0;
    // ��䴫��ṹ
    // transfer   ����ṹ
    // dHand �豸���
    // 0x82 �豸�˵㣬��������ʹ�õĶ����ݶ˵�
    // buf ���ݽ��ܴ�Ż���
    // 64 �����С �˴��ҿ��ٵ���64���ֽڴ�С�Ļ���
    // callbackRevc ���ջص���������ɡ���ʱ��ʧ�ܵ�״̬������õڻص�
    // this �û����� ���˴����Է��������ݣ�
    // 0��ʱʱ��˴�д����0���������޵ȴ���ʱֱ�������ݴ�����
    //	nvptl_debug_printf("will fill bulk!\n");
    //	printf("async_read submit inst:0x%X\n", inst);
    libusb_fill_bulk_transfer(inst->transfer, handle, NVPTL_ENDPOINT_IN, inst->usb_buf, 64 * 1024,
                              &transfercallback, inst, 200);
    //	nvptl_debug_printf("will submit bulk!\n");

    // �ύ����
    rc = libusb_submit_transfer(inst->transfer);
    if (rc < 0) {
      nvptl_error_printf("fail submit bulk!:%d\n",rc);
      // ȡ������
      // libusb_cancel_transfer(inst->transfer);
      // �ͷŴ���ṹ
      libusb_free_transfer(inst->transfer);
      inst->transfer = NULL;
      free(inst->usb_buf); 
      inst->usb_buf=NULL; 
        if (inst->eventcallback != NULL) {
          inst->eventcallback(HASSTOP, inst->connectuserdata);
        }
      return false;
    }

    // qTransferList.append (transfer);
  }
  return true; 
  // nvptl_debug_printf("after submit bulk!\n");
}
static void hal_close(NVPTL_DEVICE_HANDLE handle) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;

  if (inst->device_usb_handle != NULL) {

    nvpfm_debug_printf("will close usb!\n");
    libusb_close(inst->device_usb_handle);
    if(inst->usb_buf!=NULL){
	    free(inst->usb_buf); 
	    inst->usb_buf=NULL;
    } 
  }
}

NVPTL_DEVICE_HANDLE nvptl_libusb_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata) {
  int bus;
  char path[256];
  sscanf(dev_info->usb_camera_name, "falcon-%d-%[^-]", &bus, path);

  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)calloc(1, sizeof(NVPTL_INSTANCE));
  inst->devinfo = *dev_info;
  if (usb_hal_open(inst, bus, path) >= 0) {
    inst->eventcallback = NULL;
    inst->connectuserdata = NULL;

    inst->recvframecallback = callback;
    inst->status = STARTED;
    inst->userdata = userdata;

    if(!async_read(inst)){
   	hal_close(inst); 
	free(inst);
	return NULL;	    
    }
    // nvptl_debug_printf("connect ok!!!!\n");
    return inst;
  }
  free(inst);
  return NULL;
}

void nvptl_libusb_close(NVPTL_DEVICE_HANDLE handle) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;

  inst->status = STOPPED;
  while (1) {
    nvpfm_debug_printf("test if transfer is null!\n");
    if (inst->transfer == NULL) {
      nvpfm_debug_printf("transfer is null,break\n");
      break;
    } else {
      nvpfm_debug_printf("transfer is not null,retest again sleep 1 second\n");
      COMMONUSLEEP(1000 * 1000);
    }
  }
  while (1) {
    nvpfm_debug_printf("test if sendtransfer is null!\n");
    if (inst->sendtransfer == NULL) {
      nvpfm_debug_printf("sendtransfer is null,break\n");
      break;
    } else {
      nvpfm_debug_printf("sendtransfer is not null,retest again sleep 1 second\n");
      COMMONUSLEEP(1000 * 1000);
    }
  }

  hal_close(inst);

  if (inst->tmpbuf != NULL) {
    free(inst->tmpbuf);
    inst->tmpbuf = NULL;
  }
  if (inst->usb_buf != NULL) {
    free(inst->usb_buf);
    inst->usb_buf = NULL;
  }

  free(inst);
}
static void LIBUSB_CALL callbackSend(struct libusb_transfer *transfer) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)transfer->user_data;
  if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
    // printf("something send ok!---%d\n",transfer->actual_length);
  } else {
    nvpfm_warn_printf("something send fail!\n");
  }
  if (inst->sendtransfer != NULL) {
    free(inst->sendtransfer->buffer);
    libusb_free_transfer(inst->sendtransfer);
    inst->sendtransfer = NULL;
  }
}
int nvptl_libusb_send(NVPTL_DEVICE_HANDLE handle, unsigned char *sendBuffer, size_t len) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  int rc = 0;

  // �����첽����ṹ
  if (inst->sendtransfer == NULL)
    inst->sendtransfer = libusb_alloc_transfer(0);
  else {
    for (int i = 0; i < 10; i++) {
      if (inst->sendtransfer != NULL)
        COMMONUSLEEP(100 * 1000);
    }
    if (inst->sendtransfer != NULL) {
      return 0;
    }
    inst->sendtransfer = libusb_alloc_transfer(0);
  }

  // ����첽����ṹ
  // transfer ����ṹ
  // dHand �豸���
  // sendBuffer Ҫ���͵�����
  // len ���ݳ���
  // callbackSend �ص�������������ɡ�ʧ�ܡ����߳�ʱ������ôʻص�
  // this �û����ݣ��˴����Է��������ݣ�
  // 10 ��ʱʱ��˴�д����10����
  libusb_fill_bulk_transfer(inst->sendtransfer,
                            inst->device_usb_handle, NVPTL_ENDPOINT_OUT,
                            sendBuffer, (int)len, &callbackSend, inst, 200);
  // �ύ����ṹ
  rc = libusb_submit_transfer(inst->sendtransfer);
  // �ж��Ƿ���ɹ�
  if (rc < 0) {
    // �ͷŴ���ṹ
    libusb_free_transfer(inst->sendtransfer);
    inst->sendtransfer = NULL;
    free(sendBuffer);
  }
  return 0;
}
