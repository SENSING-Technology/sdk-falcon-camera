#include "nvpfm.h"
#include "nvpfm.hpp"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#include <map>
using namespace std;

typedef struct
{
  pthread_t threadid;
  pthread_t depththreadid;

  NVPTL_DEVICE_INFO rawdevinfo;
  nvpfm *fm;
  char devicename[64];
  std::vector<s_nvpfm_camera_param> *camparam;

  bool willrun;
  char *romfile;
  s_nvpfm_dev_info devinfo;
} DEVICEINFO;
std::map<std::string, DEVICEINFO *> g_devicemap;
NVPTL_DEVICE_INFO *g_pdevice = NULL;

void *releasethread(void *userdata) {
  DEVICEINFO *info = (DEVICEINFO *)userdata;

  info->willrun = false;
  printf("in release thread,join threadid!\n");
  pthread_join(info->threadid, NULL);
  printf("join depththreadid!\n");
  pthread_join(info->depththreadid, NULL);

  printf("delete nvpfm!\n");
  ////////////////////////////delete instance
  delete info->fm;

  printf("erase instance in map:%s!\n", info->rawdevinfo.usb_camera_name);
  //////////////////////////////remove from map
  g_devicemap.erase(info->rawdevinfo.usb_camera_name);

  printf("delete camparam!\n");
  delete info->camparam;
  printf("free info!\n");
  free(info);
  return 0;
}

void eventcallback(EVENTREASON reason, void *userdata) {
  DEVICEINFO *info = (DEVICEINFO *)userdata;

  if (reason == INFORMREBOOT) {
    printf("will send reboot to camera!\n");
    info->fm->send_reboot();
    printf("after send reboot to camera!\n");
    return;
  } else {
    printf("plugout!\n");
  }

  pthread_t releasethreadid;
  pthread_create(&releasethreadid, NULL, releasethread, info);
  pthread_detach(releasethreadid);
}

void depthcallback(void *data, void *userdata) {
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
  NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));

  printf("recv depth frame!\n");
}

void rgbcallback(void *data, void *userdata) {
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
  NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));

  printf("recv rgb frame!\n");
}
void leftircallback(void *data, void *userdata) {
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;

  NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));

  printf("recv leftir frame!\n");
}
void rightircallback(void *data, void *userdata) {
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
  NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));

  printf("recv rightir frame!\n");
}

void groupcallback(void *data, void *userdata) {
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;

  grouppkt_info_custom_t *tmpimages = (grouppkt_info_custom_t *)data;

  NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmpimages->depthbuffer + sizeof(NVPTL_USBHeaderDataPacket));
  NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmpimages->rgbbuffer + sizeof(NVPTL_USBHeaderDataPacket));
  NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmpimages->leftirbuffer + sizeof(NVPTL_USBHeaderDataPacket));
  NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmpimages->rightirbuffer + sizeof(NVPTL_USBHeaderDataPacket));

  printf("got:depth:group_id:%d,timestamp:%lu\n", depthheader->group_id, depthheader->timestamp);
  printf("got:rgb:group_id:%d,timestamp:%lu\n", rgbheader->group_id, rgbheader->timestamp);
  printf("got:leftir:group_id:%d,timestamp:%lu\n", leftirheader->group_id, leftirheader->timestamp);
  printf("got:rightir:group_id:%d,timestamp:%lu\n", rightirheader->group_id, rightirheader->timestamp);
}
void imucallback(void *data, void *userdata) {
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;

  s_nvpfm_imu_data *tmpimudata = (s_nvpfm_imu_data *)tmppack->data;

  if (tmppack->type == NVPFM_IMU_DATA) {

    if (sizeof(s_nvpfm_imu_data) <= tmppack->len && tmpimudata->data_number > 0) {
      for (int i = 0; i < tmpimudata->data_number; i++) {
        static unsigned int countfps = 0, lastfps = 0;
        countfps++;
        static time_t last = time(NULL);
        time_t current = time(NULL);
        if ((current - last) > 5) {
          printf("imufps:%f\n", (float)(countfps - lastfps) / (float)(current - last));
          last = current;
          lastfps = countfps;
        }
      }
    }
  }
}
void init(DEVICEINFO *info) {
  if (info->fm->isplugged()) {
    printf("will connect usb falcon\n");
    ////////////////////here we config resolution and fps and fpsratio and start stream with nvpfm_start
    while (info->willrun) {
      if (NVPTL_OK == info->fm->get_devinfo(&info->devinfo)) {
        printf("got device info!\n");

        break;
      }
    }

    if (!info->willrun)
      return;

    printf("will set timesync cycle!\n");

    info->fm->set_timesynccycle(true, 5);

    if (!info->willrun)
      return;

    // info->fm->start_groupimages(groupcallback);

    /*   info->fm->start_leftir(leftircallback);

       info->fm->start_rightir(rightircallback);

       info->fm->start_depth(depthcallback);

       info->fm->start_rgb(rgbcallback);

       info->fm->start_imu(imucallback);*/

    if (!info->willrun)
      return;
  }
}

void upgradecallback(void *userdata, int index, int all) {
  // printf("packet:%d,all:%d\n",index,all);
}

void *devicethread(void *theparam) {
  DEVICEINFO *info = (DEVICEINFO *)theparam;

  if (info->fm == NULL) {
    printf("will new nvpfm!\n");
    info->fm = new nvpfm(&info->rawdevinfo, eventcallback, info);
  }

  // info->fm->set_global_time(true);

  if (info->willrun) {
    printf("will init nvpfm!\n");
    init(info);
  }

  if (0 == strcmp(info->romfile, "--query")) {
    for (int i = 0; i < 10; i++) {
      s_nvpfm_dev_info devinfo;
      if (NVPTL_OK == info->fm->get_devinfo(&devinfo)) {
        printf("firmware version:%s\n", devinfo.software_version);
        exit(0);
      } else {
        printf("fail to get firmware version,try again!\n");
        usleep(200 * 1000);
      }
    }
    exit(0);
  } else {
    s_nvpfm_efuse_status param;
    if (NVPTL_OK == info->fm->get_cypher_status(&param)) {
      if (nvpfm::is_encrypt_firmware_file(info->romfile)) {
        if (param.status == EFUSE_NO_KEY_NO_CIPER || param.status == EFUSE_STATUS_INVALID) {
          printf("firmware is encypt,but camera has no key,abort!\n");
          return 0;
        }

      } else {
        if (param.status == EFUSE_HAS_KEY_HAS_CIPER || param.status == EFUSE_STATUS_INVALID) {
          printf("firmware is not encypt,but camera has cypher or invalid status,abort!\n");
          return 0;
        }
      }

      s_nvpfm_get_app_run_config_ret cfg;
      if (NVPTL_OK == info->fm->get_runcfg(&cfg)) {
        cfg.config.app_run_mode = UPGRADE;
        if (NVPTL_OK == info->fm->set_runcfg(&cfg.config)) {

          if (NVPTL_OK == info->fm->upgrade(info->romfile, upgradecallback, info)) {
            printf("upgrade camera success!\n");
          } else {
            printf("upgrade camera failed!\n");
          }
        }
      }
    } else {
      printf("fail to get cypher status!\n");
    }
  }

  return 0;
}

void mergedevices(NVPTL_DEVICE_INFO **pps_device, NVPTL_DEVICE_INFO *tmpdevice) {
  // 测试已存在的device是否没有枚举到
  {
    NVPTL_DEVICE_INFO *s_device = *pps_device;
    NVPTL_DEVICE_INFO *s_tmp = s_device;
    while (s_tmp != NULL) {
      bool exist = false;
      NVPTL_DEVICE_INFO *newtmp = tmpdevice;
      while (newtmp != NULL) {
        if (0 == strcmp(newtmp->usb_camera_name, s_tmp->usb_camera_name)) {
          exist = true;
          break;
        }
        newtmp = newtmp->next;
      }
      if (!exist) { // 已存在的device是否没有枚举到，计数加1
        s_tmp->notexists++;
        nvpfm_info_printf("device:%s not enumerated this time,increase to %d!!!\n", s_tmp->usb_camera_name, s_tmp->notexists);
      } else { // 已存在的device没有枚举到的次数计数清0
        s_tmp->notexists = 0;
      }
      s_tmp = s_tmp->next;
    }
  }
  // 测试新枚举到的是否已经存在
  {
    NVPTL_DEVICE_INFO *newtmp = tmpdevice;
    while (newtmp != NULL) {
      NVPTL_DEVICE_INFO *s_tmp = *pps_device;
      bool exist = false;
      while (s_tmp != NULL) {
        if (0 == strcmp(newtmp->usb_camera_name, s_tmp->usb_camera_name)) {
          exist = true;
          break;
        }
        s_tmp = s_tmp->next;
      }
      if (!exist) { // 新枚举到的设备不存在列表中，加入列表
        NVPTL_DEVICE_INFO *tmp = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
        memcpy(tmp, newtmp, sizeof(NVPTL_DEVICE_INFO));
        tmp->next = *pps_device;
        *pps_device = tmp;

        nvpfm_info_printf("device:%s is new,add to list!!!\n", tmp->usb_camera_name);
      }
      newtmp = newtmp->next;
    }
  }
  // 干掉列表中枚举不到计数超过2的节点
  {
    NVPTL_DEVICE_INFO *s_tmp = *pps_device;
    NVPTL_DEVICE_INFO *prev = NULL;
    while (s_tmp != NULL) {
      if (s_tmp->notexists >= 2) { // 连续超过2次没有枚举到，干掉
        nvpfm_info_printf("device:%s not enumerated more than 2:%d!!!will delete\n",
                          s_tmp->usb_camera_name, s_tmp->notexists);
        if (prev == NULL) {
          *pps_device = s_tmp->next;
          free(s_tmp);
          s_tmp = *pps_device;
          continue;
        } else {
          prev->next = s_tmp->next;
          free(s_tmp);
          s_tmp = prev;
        }
      }
      s_tmp = s_tmp->next;
    }
  }
}
MUTEXHANDLE g_enummutexhandle = CREATEMUTEX();
void enum_falcon_callback(int total, NVPTL_DEVICE_INFO *pinfo, void *userdata) {
  NVPTL_DEVICE_INFO **ppdevice = (NVPTL_DEVICE_INFO **)userdata;
  MUTEXLOCK(g_enummutexhandle);
  mergedevices(ppdevice, pinfo);
  MUTEXUNLOCK(g_enummutexhandle);
  nvptl_freedevices(pinfo);
}

void *enumthread(void *param) {
  char *romfile = (char *)param;
  while (true) {

    int total = 0;
    if (NVPTL_OK == nvptl_enum(enum_falcon_callback, &g_pdevice)) {

      if (g_pdevice != NULL) {
        total = 0;
        NVPTL_DEVICE_INFO *tmpinfo = g_pdevice;
        while (tmpinfo != NULL) {
          total++;
          tmpinfo = tmpinfo->next;
        }
      }
      if (total > 0) {
        // printf("enum %d falcons!", total);
        NVPTL_DEVICE_INFO *tmpdevice = g_pdevice;
        for (int i = 0; i < total; i++) {
          // printf("falcon:%s", tmpdevice->usb_camera_name);
          //  if not connected
          if (g_devicemap.find(tmpdevice->usb_camera_name) == g_devicemap.end()) { // 没有找到，新的相机连接
            printf("no falcon:%s in map,so create new camera instance", tmpdevice->usb_camera_name);
            DEVICEINFO *info = (DEVICEINFO *)calloc(1, sizeof(DEVICEINFO));

            info->rawdevinfo = *tmpdevice;

            info->camparam = new std::vector<s_nvpfm_camera_param>;

            void *devicethread(void *param);
            info->willrun = true;
            info->romfile = romfile;
            pthread_create(&info->threadid, NULL, devicethread, info);
            g_devicemap[tmpdevice->usb_camera_name] = info;
            return 0;
          }
          tmpdevice = tmpdevice->next;
        }
      }
    }
    sleep(1);
  }
  return 0;
}
int main(int argc, const char *argv[]) {
  if (argc != 3) {
    printf("Upgrade firmware:\n./upgrade_falcon --upgrade {romfilepath}\n");
    printf("Query firmware version:\n./upgrade_falcon --query version\n");
    exit(0);
  }

  if (0 != strcmp(argv[1], "--upgrade") && 0 != strcmp(argv[1], "--query")) {
    printf("only support --upgrade and --query!\n");
  }
  if (0 == strcmp(argv[1], "--query") && 0 != strcmp(argv[2], "version")) {
    printf("only support --query version!\n");
  }

  nvpfm_init("./falcon.log", 50 * 1024 * 1024);

  pthread_t threadid;
  if (0 == strcmp(argv[1], "--upgrade")) {
    pthread_create(&threadid, NULL, enumthread, strdup(argv[2]));
  } else if (0 == strcmp(argv[1], "--query")) {
    pthread_create(&threadid, NULL, enumthread, strdup(argv[1]));
  }
  string str;
  char tmpstr[64];
  while (1) {
    cin.getline(tmpstr, 64);
    str = tmpstr;
    if (str == "cmd") {
    }
  }
  return 0;
}
