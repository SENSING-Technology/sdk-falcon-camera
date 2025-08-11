#include "nvpfm.h"
#include "nvpfm.hpp"
#include <stdio.h>
#include <string.h>
#include <math.h>
#ifdef _MSC_VER
#include <windows.h>
#include <time.h>
#include <process.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif // _MSC_VER
#include <iostream>
#include <vector>
#include <map>
#include <opencv2/opencv.hpp>

using namespace std;

// #define CHANGE_SENSOR_RES

#pragma comment(lib, "libusb-1.0.lib")
#pragma comment(lib, "yaml.lib")
int main(int argc, const char *argv[])
{
  if (argc !=3)
  {
    printf("usb camera usage:./changeip currentip newip\n");
    exit(0);
  }
  nvpfm_init("./falcon.log", 50 * 1024 * 1024);
  NVPTL_DEVICE_INFO tmpinfo;
  tmpinfo.type=NVPTL_NETWORK_INTERFACE;
  strcpy(tmpinfo.net_camera_ip,argv[1]);
  nvpfm* fm=new nvpfm(&tmpinfo,NULL, NULL);
  s_nvpfm_set_ip setip;
  memset(&setip,0,sizeof(s_nvpfm_set_ip));
  strncpy(setip.ip,argv[2],15);
  if(NVPTL_OK==fm->set_ip(&setip)){
    nvpfm_info_printf("set ip ok!\n");
    //reboot camera

  }else{
    nvpfm_info_printf("fail to set ip!\n");
  }
  fm->send_reboot();

  nvpfm_deinit();
  return 0;
}
