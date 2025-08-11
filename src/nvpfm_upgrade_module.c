/*#include "nvpfm_usb.h"
#include "nvpfm_usb_proto.h"

#include "nvpfm_cmd_module.h"
#include "nvpfm_upgrade_module.h"
*/
#include "nvpfm.h"
#include "utils.h"
#ifdef _WINDOWS
#include <io.h>
#include <windows.h>
#include <conio.h>
#include <tchar.h>
#else
#include <unistd.h>
#include <semaphore.h>
#endif

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
// #include <iostream>
#include <time.h>

#include "nvpfm_internal.h"

// #define UNUSED(A) (void *)(A);

#define UPGRADE_MAX_BLK_SIZE (1024 * 46)
// #pragma warning(disable : 4996)
// #define nvpfm_info_printf printf
// int update_result = 0;
// uint8_t upgrade_update_flag[256] = {0};

int32_t usb_send_upgrade_and_get_ack(NVPFM_DEVICE_HANDLE handle, NVPFM_UPGRADE_SUB_TYPE type, void *data, int32_t size, int isfirst);
int nvpfm_upgrade_withcontent(NVPFM_DEVICE_HANDLE handle, UPGRADECALLBACK callback, void *userdata, int upgrade_type, const char *dst_path, const char *dst_filename, const char *localcontent) {
  if (userdata != NULL) {
    nvpfm_info_printf("currently,user data is not used!\n");
  }
  int ret_code = -1;
  if (localcontent != NULL) {
    size_t file_size = 0, read_offset = 0, read_len = 0;
    unsigned int total_blk = 0, current_blk = 0;
    char *file_data = NULL;
    int updated = 0;

    file_size = strlen(localcontent);

    total_blk = (unsigned int)(file_size + UPGRADE_MAX_BLK_SIZE - 1) / UPGRADE_MAX_BLK_SIZE;

    file_data = (char *)malloc(sizeof(s_nvpfm_upgrade_data) + UPGRADE_MAX_BLK_SIZE);

    if (file_data) {
      s_nvpfm_upgrade_data *header = (s_nvpfm_upgrade_data *)file_data;
      ret_code = 0;
      int isfirst = 1;
      while (read_offset < file_size) {
        memset(file_data, 0, sizeof(s_nvpfm_upgrade_data) + UPGRADE_MAX_BLK_SIZE);

        read_len = (file_size - read_offset) > UPGRADE_MAX_BLK_SIZE ? UPGRADE_MAX_BLK_SIZE : file_size - read_offset;
        /*size_t readlen =*/
        memcpy(file_data + sizeof(s_nvpfm_upgrade_data), localcontent + read_offset, read_len);

        // UNUSED(readlen);
        header->packet_numbers = total_blk;
        header->curr_packet_numbers = (++current_blk);
        memset(header->path, 0, sizeof(header->path));
        memset(header->name, 0, sizeof(header->name));
        strcpy(header->path, dst_path);
        strcpy(header->name, dst_filename);
        header->data_len = (uint32_t)read_len;
        nvpfm_info_printf("upgrade : total_blk[%d], current_blk[%d], offset[%ld], data_len[%d]\n",
                          header->packet_numbers, header->curr_packet_numbers, read_offset, header->data_len);
        read_offset += read_len;

        if ((updated = usb_send_upgrade_and_get_ack(handle, (NVPFM_UPGRADE_SUB_TYPE)upgrade_type,
                                                    //   (upgrade_type==0) ? NVPFM_COMMAND_USB_UPGRADE_FEYNMAN : ((upgrade_type == 1) ? NVPFM_COMMAND_USB_UPGRADE_LIB : NVPFM_COMMAND_USB_UPGRADE_FILE),
                                                    header, (int32_t)(sizeof(s_nvpfm_upgrade_data) + read_len), isfirst)) == 1) {

          /*if (update_result == -1)
          {
              //need retransfer
              fseek(fp, 0 - read_len, SEEK_CUR);
              read_offset -= read_len;
              current_blk--;
          }
          else  */
          // if (update_result == 0)
          {
            // this package success, do nothing
            if (callback != NULL)
              callback(userdata, current_blk, total_blk);
            nvpfm_info_printf("current:%d,total:%d\n", current_blk, total_blk);
            if (current_blk == total_blk) {
              ret_code = 0;
              break;
            }
          }
          /*   else if (update_result == 1)
             {
                 // this file success, break
                 if (hascalllastcallback == 0&&callback!=NULL)
                     callback(total_blk, total_blk);

                 ret_code = 0;
                 break;
             }
             else
             {
                 // occr error ,stop
                 ret_code = -3;
                 break;
             }*/
        } else {
          // occur fatal error, stop
          ret_code = -4;
          break;
        }
        isfirst = 0;
      }
    } else {
      ret_code = -2;
    }
  }

  return ret_code;
}

int nvpfm_upgrade(NVPFM_DEVICE_HANDLE handle, UPGRADECALLBACK callback, void *userdata, int upgrade_type, const char *dst_path, const char *dst_filename, const char *local_filename) {
  if (userdata != NULL) {
    nvpfm_info_printf("currently,user data is not used!\n");
  }
  int ret_code = -1;
  FILE *fp = fopen(local_filename, "rb");
  nvpfm_info_printf("nvpfm_upgrade:upgrade_type[%d], dst_path[%s], dst_filename[%s], local_filename[%s]\r\n",
                    upgrade_type, dst_path, dst_filename, local_filename);

  if (fp) {
    size_t file_size = 0, read_offset = 0, read_len = 0;
    unsigned int total_blk = 0, current_blk = 0;
    char *file_data = NULL;
    int updated = 0;

    fseek(fp, 0, SEEK_END);
    file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    total_blk = (unsigned int)(file_size + UPGRADE_MAX_BLK_SIZE - 1) / UPGRADE_MAX_BLK_SIZE;

    file_data = (char *)malloc(sizeof(s_nvpfm_upgrade_data) + UPGRADE_MAX_BLK_SIZE);

    if (file_data) {
      s_nvpfm_upgrade_data *header = (s_nvpfm_upgrade_data *)file_data;
      ret_code = 0;
      int isfirst = 1;
      while (read_offset < file_size) {
        memset(file_data, 0, sizeof(s_nvpfm_upgrade_data) + UPGRADE_MAX_BLK_SIZE);

        read_len = (file_size - read_offset) > UPGRADE_MAX_BLK_SIZE ? UPGRADE_MAX_BLK_SIZE : file_size - read_offset;
        size_t readlen = fread(file_data + sizeof(s_nvpfm_upgrade_data), 1, read_len, fp);

        (void)readlen;
        header->packet_numbers = total_blk;
        header->curr_packet_numbers = (++current_blk);
        memset(header->path, 0, sizeof(header->path));
        memset(header->name, 0, sizeof(header->name));
        strcpy(header->path, dst_path);
        strcpy(header->name, dst_filename);
        header->data_len = (uint32_t)read_len;

        nvpfm_info_printf("upgrade : total_blk[%d], current_blk[%d], offset[%ld], data_len[%d]\n",
                          header->packet_numbers, header->curr_packet_numbers, read_offset, header->data_len);
        read_offset += read_len;

        if ((updated = usb_send_upgrade_and_get_ack(handle, (NVPFM_UPGRADE_SUB_TYPE)upgrade_type,
                                                    //   (upgrade_type==0) ? NVPFM_COMMAND_USB_UPGRADE_FEYNMAN : ((upgrade_type == 1) ? NVPFM_COMMAND_USB_UPGRADE_LIB : NVPFM_COMMAND_USB_UPGRADE_FILE),
                                                    header, (int32_t)(sizeof(s_nvpfm_upgrade_data) + read_len), isfirst)) == 1) {
          isfirst = 0;
          /*if (update_result == -1)
          {
              //need retransfer
              fseek(fp, 0 - read_len, SEEK_CUR);
              read_offset -= read_len;
              current_blk--;
          }
          else  */
          // if (update_result == 0)
          {
            // this package success, do nothing
            if (callback != NULL)
              callback(userdata, current_blk, total_blk);
            if (current_blk == total_blk) {
              ret_code = 0;
              break;
            }
          }
          /*else if (update_result == 1)
          {
              // this file success, break
              if (hascalllastcallback == 0&&callback!=NULL)
                  callback(total_blk, total_blk);

              ret_code = 0;
              break;
          }
          else
          {
              // occr error ,stop
              ret_code = -3;
              break;
          }*/
        } else {
          // occur fatal error, stop
          ret_code = -4;
          break;
        }
      }
    } else {
      ret_code = -2;
    }
    fclose(fp);
  }

  return ret_code;
}
/*
int32_t get_upgrade_update_flag_index(NVPFM_UPGRADE_SUB_TYPE type)
{
    int32_t index = -1;

    switch (type)
    {
    case NVPFM_COMMAND_USB_UPGRADE_USER_FILE:
    case NVPFM_COMMAND_USB_UPGRADE_USER_FILE_RETURN:
    {
        index = 0;
        break;
    }
    case NVPFM_COMMAND_USB_UPGRADE_USER_CONFIG:
    case NVPFM_COMMAND_USB_UPGRADE_USER_CONFIG_RETURN:
    {
        index = 1;
        break;
    }
    case NVPFM_COMMAND_USB_UPGRADE_NEXTVPU_SYSTEM:
    case NVPFM_COMMAND_USB_UPGRADE_NEXTVPU_SYSTEM_RETURN:
    {
        index = 2;
        break;
    }
    case NVPFM_COMMAND_USB_UPGRADE_NEXTVPU_CONFIG:
    case NVPFM_COMMAND_USB_UPGRADE_NEXTVPU_CONFIG_RETURN:
    {
        index = 3;
        break;
    }
    default:
  ;
    }

    return index;
}

void usb_upgrade_set_update_mask(NVPFM_UPGRADE_SUB_TYPE type)
{
    int32_t index = get_upgrade_update_flag_index(type);
    if (index >= 0)
    {
        upgrade_update_flag[index] = 0xA5;
    }
}
int32_t usb_upgrade_mask_is_updated(NVPFM_UPGRADE_SUB_TYPE type)
{
    int32_t index = get_upgrade_update_flag_index(type);
    if (index >= 0)
    {
        return (upgrade_update_flag[index] == 0x00) ? 1 : ((upgrade_update_flag[index] == 0xA5) ? 0 : (-1));
    }

    return 0;
}*/
static void nvpfm_usb_send_upgrade(NVPFM_DEVICE_HANDLE handle, uint16_t type, void *p_data, int data_length) {
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  char *tmp = (char *)malloc(sizeof(NVPTL_USBHeaderDataPacket) + data_length);
  NVPTL_USBHeaderDataPacket *p_cmd = (NVPTL_USBHeaderDataPacket *)tmp;
  memcpy((char *)p_cmd->magic, (char *)"NEXT_VPU", strlen("NEXT_VPU"));
  p_cmd->type = NVPFM_UPDATE_DATA;
  p_cmd->sub_type = type;
  p_cmd->checksum = 0xA5A5;
  p_cmd->len = data_length;

  if (data_length) {
    memcpy(p_cmd->data, p_data, data_length);
  }
  int transfered = 0;
  if ((NVPFM_UPGRADE_SUB_TYPE)type == NVPFM_COMMAND_USB_UPGRADE_NEXTVPU_SYSTEM)
    nvptl_send(inst->transferlayerhandle, (uint8_t *)p_cmd, sizeof(NVPTL_USBHeaderDataPacket) + data_length); // , &transfered, 4000);
  else
    nvptl_send(inst->transferlayerhandle, (uint8_t *)p_cmd, sizeof(NVPTL_USBHeaderDataPacket) + data_length); //, &transfered, 2000);

  /*	if (usb_para.m_dev_handle != NULL)
  {
      usb_bulk_write(usb_para.m_dev_handle, usb_para.usb_ep_out, (char*)p_cmd, sizeof(NVPTL_USBHeaderDataPacket) + data_length, 0);
  }*/

  // free(tmp);
}

#ifdef _WINDOWS
// typedef usb_dev_handle *USBHANDLE;
typedef HANDLE THREADHANDLE;
typedef HANDLE SEM_T;
typedef struct timeval2 MYTIMEVAL;
#else
typedef int SOCKET;
typedef pthread_t THREADHANDLE;
typedef sem_t SEM_T;
typedef struct timeval MYTIMEVAL;
#endif
void SEM_INIT(SEM_T *sem, int pshared, unsigned int value);
int SEM_TRYWAIT_MILLISECONDS(SEM_T *sem, int ms);
void SEM_POST(SEM_T *sem);

// s_nvpfm_upgrade_result g_upgraderesult;

int32_t usb_send_upgrade_and_get_ack(NVPFM_DEVICE_HANDLE handle, NVPFM_UPGRADE_SUB_TYPE type, void *data, int32_t size, int isfirst) {
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  for (int i = 0; i < 5; i++) {
    // int32_t updated = 0;
    // time_t trytimes = time(NULL);
    // int justcontinue = 0;

    //     usb_upgrade_set_update_mask(type);
    nvpfm_usb_send_upgrade(handle, type, data, size);
    // �̨���y?��?��????��|�㨹
    int maxtimes = (((type == NVPFM_COMMAND_USB_UPGRADE_NEXTVPU_SYSTEM) && isfirst) ? 10 : 10);
    for (int j = 0; j < maxtimes; j++) {
      int retvalue = SEM_TRYWAIT_MILLISECONDS(&inst->upgradesem, 1000);
      if (retvalue == -1 || retvalue == -2) {
        nvpfm_info_printf("upgrade packet still no answer!\n");
        COMMONUSLEEP(500 * 1000);
      } else {

        // nvpfm_info_printf("recv upgrade packet response:%d\n", inst->upgraderesult.result);
        if (inst->upgraderesult.result == 0 || inst->upgraderesult.result == 1) {
          return 1;
        } else {
          continue;
        }
      }
    }
  }
  return -1;
}
/*typedef struct
{
  int result; // -2can not upgrade this file��?-1upgrade failed��?0-current packet upgrade success��?  1-upgrade complete
} s_nvpfm_upgrade_result;*/
void usb_upgrade_callback(NVPFM_DEVICE_HANDLE handle, NVPFM_UPGRADE_SUB_TYPE type, void *data) {
  (void)type;
  NVPFM_INSTANCE *inst = (NVPFM_INSTANCE *)handle;
  // nvpfm_info_printf("call upgrade callback to proess upgrade response pkt!\n");
  inst->upgraderesult = *((s_nvpfm_upgrade_result *)data);
  SEM_POST(&inst->upgradesem);
}
