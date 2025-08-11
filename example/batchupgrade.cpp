#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include <string>
#include <map>
#include <fstream>
#include <iostream>

#include "nvpfm.h"
#include "nvpfm.hpp"
#define MAXPATHLEN 256

bool isencryptfirmwarefile=false;
char firmwarefilepath[MAXPATHLEN];
char firmwareversion[MAXPATHLEN];
int g_allupgrade=0;
int g_goodupgrade=0;

// typedef struct
// {


//   NVPTL_DEVICE_INFO rawdevinfo;
//   nvpfm *fm;
// //   char devicename[64];
// //   char* usb_camera_name
//   std::vector<s_nvpfm_camera_param> *camparam;

// //   bool willrun;
//   s_nvpfm_dev_info devinfo;
// } DEVICEINFO;

void upgradecallback(void* userdata,int index, int all)
{
	(void)userdata;
  //nvpfm_info_printf("current item index:%d,all:%d\n", index, all);
  if (index == all)
  {
    nvpfm_info_printf("all data transfered!\n");
  }
}
/*static void upgrade(NVPFM_DEVICE_HANDLE handle, NVPFM_UPGRADE_SUB_TYPE upgradetype, char *firmwarefilepath)
{
  int ret = nvpfm_upgrade(handle, upgradecallback, NULL, upgradetype, "", "", firmwarefilepath);

  if (ret < 0)
  {
    printf("upgrade failed!\n");
  }
  else
  {
    printf("upgrade success!\n");
  }
}
*/

int g_thread_num = 0;
typedef std::map<std::string, int> DATA_USBPORT;
DATA_USBPORT g_map_usbport;

int readusbport(const char* filename="./example/falcon.ini")
{
  std::ifstream inifile;
  inifile.open(filename, std::ios::in);
  if (inifile.is_open())
  {
	std::string str;
	long unsigned int pos;
	while (getline(inifile, str))
	{
		pos = str.find('=');
		if (pos != std::string::npos)
		{

		        std::string name = str.substr(pos + 1);
			name = name.substr(0, name.find('\r'));
			g_map_usbport[name] = atoi(str.substr(0, pos).c_str());
		}
	}
	inifile.close();
  }
  return 0;
}

int getusbport(char* usb_camera_name)
{
  if (usb_camera_name != NULL)
  {
	char* usbport = strstr(usb_camera_name, "falcon-");
	if (usbport != NULL)
	{
	  DATA_USBPORT::iterator itr = g_map_usbport.find(usbport+strlen("falcon-"));
	  if (itr != g_map_usbport.end())
	  {
		return itr->second;
	  }
	}
  }
  return 0;
}

void* upgradethread(void* param){
	// char* usb_camera_name=(char*)param;
    NVPTL_DEVICE_INFO *info = (NVPTL_DEVICE_INFO *)param;
    char* usb_camera_name = info->usb_camera_name;

    printf("will new nvpfm!\n");
    nvpfm* pfm = new nvpfm(info, NULL, info);
    if (pfm->isplugged())
    {
        s_nvpfm_dev_info devinfo;
		if(NVPTL_OK==pfm->get_devinfo(&devinfo)){
			if(0==strcmp(devinfo.software_version,firmwareversion)){
				nvpfm_info_printf("camera:%s from usb:%d has desired version firmware:%s\n",usb_camera_name,getusbport(usb_camera_name),firmwareversion);
				__sync_fetch_and_sub(&g_thread_num, 1);
				delete pfm;	
				// free(usb_camera_name);	
				return 0;
			}
		}else{
			nvpfm_info_printf("fail to get camera info of camera:%s from usb:%d\n",usb_camera_name,getusbport(usb_camera_name));
			__sync_fetch_and_sub(&g_thread_num, 1);
			delete pfm;	
			// free(usb_camera_name);	
			return 0;	
		}	
      nvpfm_info_printf("will change mode to upgrade!\n");
      s_nvpfm_get_app_run_config_ret cfg;
      if(NVPTL_OK==pfm->get_runcfg(&cfg)){
        cfg.config.app_run_mode=UPGRADE;
        if(NVPTL_OK==pfm->set_runcfg(&cfg.config))
        {
        //    nvpfm_info_printf("will reset pipeline!\n");
        //    if ( NVPTL_OK == pfm->set_pipeline(3)) // timeout 2000milliseconds
        //    { 
            s_nvpfm_efuse_status param;
            if( NVPTL_OK==pfm->get_cypher_status(&param)){	
                if(isencryptfirmwarefile){
                    if(param.status==EFUSE_NO_KEY_NO_CIPER||param.status==EFUSE_STATUS_INVALID){
                        nvpfm_info_printf("firmware is encypt,but camera has no key,abort!\n");
                            __sync_fetch_and_sub(&g_thread_num, 1);
                            delete pfm;	
                            // free(usb_camera_name);	
                        return 0;
                    }

                }else{
                    if (param.status == EFUSE_HAS_KEY_HAS_CIPER || param.status == EFUSE_STATUS_INVALID) {
                        nvpfm_info_printf("firmware is not encypt,but camera has cypher or invalid status,abort!\n");
                            __sync_fetch_and_sub(&g_thread_num, 1);
                            delete pfm;	
                            // free(usb_camera_name);	
                        return 0;
                    }
                }

                nvpfm_info_printf("will upgrade camera:%s from usb:%d\n",usb_camera_name,getusbport(usb_camera_name));	
                g_allupgrade++; 
                if( NVPTL_OK==pfm->upgrade(firmwarefilepath,upgradecallback,NULL)){
               		g_goodupgrade++; 
                    nvpfm_info_printf("upgrade camera:%s from usb:%d success!\n",usb_camera_name,getusbport(usb_camera_name));
                }else{
                    nvpfm_info_printf("upgrade camera:%s from usb:%d failed!\n",usb_camera_name,getusbport(usb_camera_name));
                }
               	printf("upgrade %d times,good:%d,fail:%d,success ratio:%f%%\n",g_allupgrade,g_goodupgrade,g_allupgrade-g_goodupgrade,(float)g_goodupgrade*100.0/(float)g_allupgrade); 
            }else{
                nvpfm_warn_printf("fail to get cypher status!\n");

            } 
            // } 
            // else
            // nvpfm_warn_printf("fail to reset pipeline!\n");
        }
        else
        {
            nvpfm_warn_printf("fail to set run mode to upgrade!\n");
        }
      }
    }
    else
    {
      nvpfm_warn_printf("connect falcon usb camera failed!\n");
    }

    __sync_fetch_and_sub(&g_thread_num, 1);
    delete pfm;	
    // free(usb_camera_name);	
    return 0;
}

//1.input romfile and firmware version
//2.enumerate all plugged cameras
//3.connect all cameras to retrieve firmware version
//4.if firmware version not equal input firmware version,upgrade the camera
//5.after upgrade,retrieve the firmware version,if the version equals input version,thread exit
//6.else reupgrade 
char* getnamebyindex(NVPTL_DEVICE_INFO* info,int index){
	int i=0;
	NVPTL_DEVICE_INFO* tmp=info;
	while(tmp!=NULL){
		if(i==index){
			return strdup(tmp->usb_camera_name);
		}
		i++;
		tmp=tmp->next;
	}
	return NULL;
}

int main(int argc, const char *argv[])
{
  if (argc != 3)
  {
    printf("usage:./upgrade_falcon romfile {version}\n version example:\"2023/06/08 16:24:56\"\n");
    return 0;
  }
  strcpy(firmwarefilepath, argv[1]);
  strcpy(firmwareversion, argv[2]);

  readusbport();
  
  isencryptfirmwarefile=nvpfm::is_encrypt_firmware_file(firmwarefilepath); 
  printf("firmware is encrypt:%s\n",isencryptfirmwarefile?"true":"false"); 

  nvpfm_init("./falcon.log",1024*1024);

  while (1)
  {
    NVPTL_DEVICE_INFO *deviceInfo = NULL;
    int total = 0;
    int max = 4;
    if (NVPTL_OK == nvptl_enum_sync(&total, &deviceInfo))
    {
        pthread_t* upgradethreadids=(pthread_t*)malloc(total*sizeof(pthread_t)); 
        NVPTL_DEVICE_INFO * tmpDevice = deviceInfo;
        for(int i=0;i<total;i++){//create multiple thread to upgrade
            // DEVICEINFO *info = (DEVICEINFO *)calloc(1, sizeof(DEVICEINFO));
            // info->rawdevinfo = *tmpdevice;
            // info->camparam = new std::vector<s_nvpfm_camera_param>;
            // info->usb_camera_name = getnamebyindex(deviceInfo,i);
            // info->usb_camera_name = info->rawdevinfo.usb_camera_name;
            // char* tmpcameraname=getnamebyindex(info,i);
            if(tmpDevice->usb_camera_name!=NULL){	
                nvpfm_info_printf("will create ugprade thread of camera:%s\n",tmpDevice->usb_camera_name);	
                pthread_create(&upgradethreadids[i],NULL,upgradethread,tmpDevice);
            }else{
                printf("this should not happen,there's no camera of index:%d\n",i);
                exit(0);	
            }	
            __sync_fetch_and_add(&g_thread_num, 1);
            int oldnum=0;
            while ((oldnum=__sync_fetch_and_add(&g_thread_num,0)) >= max)
            {
                sleep(1);	
            }
            tmpDevice = tmpDevice->next;
        }
        for(int i=0;i<total;i++){	
            pthread_join(upgradethreadids[i],NULL);	
        }
        nvptl_freedevices(deviceInfo);
    }
    else
    {
        nvpfm_info_printf("no falcon usb camera connected!\n");
    }
  }
  sleep(5);	
  return 0;
}
