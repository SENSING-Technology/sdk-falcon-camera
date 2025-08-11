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
typedef struct
{
  uint32_t countfps;
  uint32_t lastfps;
  time_t last;
} COUNTFPSDATA;
typedef struct
{
#ifdef _MSC_VER
  unsigned int threadid;
  unsigned int depththreadid;
  unsigned int depthalignthreadid;
#else
  pthread_t threadid;
  pthread_t depththreadid;
  pthread_t depthalignthreadid;
#endif

  NVPTL_DEVICE_INFO rawdevinfo;
  nvpfm *fm;
  char devicename[64];
  std::vector<s_nvpfm_camera_param> *camparam;

  bool willrun;
  s_nvpfm_dev_info devinfo;
  Ring_Queue* depthaligndataqueue;
  Ring_Queue* cnnqueue;

  COUNTFPSDATA rgbcountfps;
  COUNTFPSDATA depthcountfps;
  COUNTFPSDATA leftircountfps;
  COUNTFPSDATA rightircountfps;
  depthtransformer *depthtrans;
  int rgbwidth;
  int rgbheight;
} DEVICEINFO;

void countfps(COUNTFPSDATA *pdata, const char *streamtype, int width, int height)
{
  if (pdata->last == 0)
    pdata->last = time(NULL);
  pdata->countfps++;
  time_t current = time(NULL);
  if ((current - pdata->last) > 5)
  {
    printf("%s %dx%d fps:%f\n", streamtype, width, height, (float)(pdata->countfps - pdata->lastfps) / (float)(current - pdata->last));
    pdata->last = current;
    pdata->lastfps = pdata->countfps;
  }
}

std::map<std::string, DEVICEINFO *> g_devicemap;
NVPTL_DEVICE_INFO *g_pdevice = NULL;

#ifdef _MSC_VER
unsigned __stdcall releasethread(void *userdata)
#else
void *releasethread(void *userdata)
#endif
{
  DEVICEINFO *info = (DEVICEINFO *)userdata;

  info->willrun = false;
  printf("in release thread,join threadid!\n");
#ifdef _MSC_VER
  WaitForSingleObject((HANDLE)info->threadid, INFINITE);
#else
  pthread_join(info->threadid, NULL);
#endif
  printf("join depththreadid!\n");
#ifdef _MSC_VER
  WaitForSingleObject((HANDLE)info->depththreadid, INFINITE);
#else
  pthread_join(info->depththreadid, NULL);
#endif
  delete info->depthtrans;
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

void eventcallback(EVENTREASON reason, void *userdata)
{
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  if (reason == INFORMREBOOT)
  {
    printf("will send reboot to camera!\n");
    info->fm->send_reboot();
    printf("after send reboot to camera!\n");
    return;
  }
  else
  {
    printf("plugout!\n");
  }

#ifdef _MSC_VER
  unsigned int threadID = _beginthreadex(NULL, 0, releasethread, info, 0, NULL);
#else
  pthread_t releasethreadid;
  pthread_create(&releasethreadid, NULL, releasethread, info);
  pthread_detach(releasethreadid);
#endif
}

void othercallback(void *data, void *userdata)
{
  NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
  if (tmppack->type == NVPFM_CNN_DATA)
  {
    DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
    NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
    if (tmppack->type == NVPFM_CNN_DATA) {
      //nvpfm_info_printf("got cnn data!\n"); 
      s_nvpfm_cnn_data tmpcnndata = {0};
      nvpfm::get_cnn_data(tmppack, &tmpcnndata);

      s_nvpfm_cnn_data *p = (s_nvpfm_cnn_data *)SOLO_Write(ptmpinfo->cnnqueue);
      if (p) {
        memcpy(p, &tmpcnndata, sizeof(s_nvpfm_cnn_data));
        SOLO_Write_Over(ptmpinfo->cnnqueue);
      }
    }
  }
}
void *depthalignrgbthread(void *param) {
  s_nvpfm_cnn_data cnndata;
  uint16_t * depthaligndata=(uint16_t*)malloc(1280*800*2); 
  NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)malloc(sizeof(NVPFM_USB_IMAGE_HEADER) + 1280 * 800 * 2);
  uint16_t *depthdata = (uint16_t *)((uint8_t *)depthheader + sizeof(NVPFM_USB_IMAGE_HEADER));
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)param;
  
  while (ptmpinfo->willrun) {
    NVPFM_USB_IMAGE_HEADER *p = (NVPFM_USB_IMAGE_HEADER *)SOLO_Read(ptmpinfo->depthaligndataqueue);
    if (p) {
      memcpy(depthheader, p, sizeof(NVPFM_USB_IMAGE_HEADER) + 1280 * 800 * 2);
      SOLO_Read_Over(ptmpinfo->depthaligndataqueue);

     // nvpfm_info_printf("got depth data!!!!!!\n");

      if (ptmpinfo->camparam->size() == 1) {
  /*      s_nvpfm_cnn_data *p = (s_nvpfm_cnn_data *)SOLO_Read(ptmpinfo->cnnqueue);
        if (p) {
          memcpy(&cnndata, p, sizeof(s_nvpfm_cnn_data));
          SOLO_Read_Over(ptmpinfo->cnnqueue);
     // nvpfm_info_printf("got cnn data!!!!!!\n");
          if(cnndata.groups>0){
            */
            s_nvpfm_camera_param theparam = ptmpinfo->camparam->at(0);

            int width = ptmpinfo->rgbwidth;
            int height = ptmpinfo->rgbheight;

            float focus[2],photocenter[2];
            focus[0]=theparam.left_ir_focus[0]/((float)theparam.irwidth/(float)depthheader->width);
            focus[1]=theparam.left_ir_focus[1]/((float)theparam.irheight/(float)depthheader->height);
            photocenter[0]=theparam.left_ir_photocenter[0]/((float)theparam.irwidth/(float)depthheader->width);
            photocenter[1]=theparam.left_ir_photocenter[1]/((float)theparam.irheight/(float)depthheader->height);

           // nvpfm_info_printf("%dx%d,%dx%d\n",depthheader->width,depthheader->height,theparam.irwidth,theparam.irheight);

            ptmpinfo->depthtrans->compute_depth_rgb_align(depthdata, depthheader->width, depthheader->height,
                            (uint16_t *)depthaligndata, ptmpinfo->rgbwidth, ptmpinfo->rgbheight,
                            NULL, focus, photocenter,
                            theparam.color_focus, theparam.color_photocenter, theparam.left2color_matrix);
          //  nvpfm_info_printf("got cnn bbox:%d\n",cnndata.groups);	
            

                  static uint8_t* pseudo=NULL;
                  if(NULL==pseudo)pseudo=(uint8_t*)malloc(width*height*3);

                  ptmpinfo->depthtrans->compute_depth2pseudo(depthaligndata,width,height,pseudo);
                  cv::Mat srcmat=cv::Mat(height, width, CV_8UC3, pseudo);

                    static bool first=true;
                      if(first){
                        first=false;
                        cv::namedWindow("depthalignrgb", cv::WINDOW_NORMAL);
                      }

                  cv::imshow("depthalignrgb", srcmat);
                  cv::waitKey(1);

          /*  falcon_camera::cnninfo cnninfo;
            cnninfo.timestamp=depthheader->timestamp;
            cnninfo.cnnboxs.resize(cnndata.groups);
            for (int i = 0; i < cnndata.groups; i++) {
                  LABELINFO info = nvpfm::get_cnn_label_by_index(cnndata.group[i].label, (EM_CNN_TYPE)cnndata.type);
                  nvpfm_info_printf("############################\n");
                  nvpfm_info_printf("label:%s\nscore:%f\nxmin:%f,ymin:%f,\nxmax:%f,ymax:%f\n",
                  info.labelname, cnndata.group[i].score,
                  cnndata.group[i].xmin,
                  cnndata.group[i].ymin,
                  cnndata.group[i].xmax,
                  cnndata.group[i].ymax);
                  cnninfo.cnnboxs[i].xmin=(int)(cnndata.group[i].xmin*(float)width);
                  cnninfo.cnnboxs[i].ymin=(int)(cnndata.group[i].ymin*(float)height);
                  cnninfo.cnnboxs[i].xmax=(int)(cnndata.group[i].xmax*(float)width);
                  cnninfo.cnnboxs[i].ymax=(int)(cnndata.group[i].ymax*(float)height);
                  cnninfo.cnnboxs[i].label=cnndata.group[i].label;
                  cnninfo.cnnboxs[i].score=cnndata.group[i].score;
                  nvpfm_info_printf("############################\n");
                  int oxmin=(int)(cnndata.group[i].xmin*(float)ptmpinfo->rgbwidth); 
                  int oxmax=(int)(cnndata.group[i].xmax*(float)ptmpinfo->rgbwidth); 
                  int oymin=(int)(cnndata.group[i].ymin*(float)ptmpinfo->rgbheight); 
                  int oymax=(int)(cnndata.group[i].ymax*(float)ptmpinfo->rgbheight); 
                  int count=0; 
                  for(int row=oymin;row<oymax;row++){
                    for(int col=oxmin;col<oxmax;col++){
                      if(*(depthaligndata+row*ptmpinfo->rgbwidth+col)>0){
                        count++;	
                      }
                    }
                  }
                  cnninfo.cnnboxs[i].points.resize(count);
                  int index=0;
                  for(int row=oymin;row<oymax;row++){
                    for(int col=oxmin;col<oxmax;col++){
                      if(*(depthaligndata+row*ptmpinfo->rgbwidth+col)>0){
                        cnninfo.cnnboxs[i].points[index].z=(float)(*(depthaligndata+row*ptmpinfo->rgbwidth+col))/1000.0;	
                        index++;
                      }
                    }
                  }
                  nvpfm_info_printf("got points:%d\n",count); 

            }*/
           // ptmpinfo->cnnpublisher.publish(cnninfo);
        //  }
       // }
      }
      if (ptmpinfo->camparam->size() == 1) {
        s_nvpfm_camera_param theparam = ptmpinfo->camparam->at(0);

        int width = ptmpinfo->rgbwidth;
        int height = ptmpinfo->rgbheight;

       /* sensor_msgs::Image depthimage;
        depthimage.header.frame_id = "camera_color_optical_frame";
        ros::Time tmptime;
        tmptime.fromNSec(depthheader->timestamp * 1000);
        depthimage.header.stamp = tmptime;
        depthimage.width = width;
        depthimage.height = height;
        depthimage.is_bigendian = 0;
        depthimage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        depthimage.step = sizeof(unsigned short) * depthimage.width;
        int data_size = depthimage.step * depthimage.height;
        depthimage.data.resize(data_size);

        ptmpinfo->depthtrans->compute_depth_rgb_align(depthdata, depthheader->width, depthheader->height,
                                                      (uint16_t *)&depthimage.data[0], ptmpinfo->rgbwidth, ptmpinfo->rgbheight,
                                                      NULL, theparam.left_ir_focus, theparam.left_ir_photocenter,
                                                      theparam.color_focus, theparam.color_photocenter, theparam.left2color_matrix);
                                                      */

        {
        /*  sensor_msgs::Image pseudoimage;
          pseudoimage.header.frame_id = "camera_color_optical_frame";
          ros::Time tmptime;
          tmptime.fromNSec(depthheader->timestamp * 1000);
          pseudoimage.header.stamp = tmptime;
          pseudoimage.width = width;
          pseudoimage.height = height;
          pseudoimage.is_bigendian = 0;
          // pseudoimage.encoding = sensor_msgs::image_encodings::MONO8;
          pseudoimage.encoding = sensor_msgs::image_encodings::RGB8;
          // pseudoimage.step = pseudoimage.width;
          pseudoimage.step = 3 * pseudoimage.width;
          int data_size = pseudoimage.step * pseudoimage.height;
          pseudoimage.data.resize(data_size);
          uint8_t *depthalignpseudo = (uint8_t *)(&pseudoimage.data[0]);
          for (int i = 0; i < (width * height); i++) {
            uint16_t depth = *((uint16_t *)&depthimage.data[0] + i);
            MYRGB *color = g_colortable + depth;

            depthalignpseudo[i * 3] = color->r;
            depthalignpseudo[i * 3 + 1] = color->g;
            depthalignpseudo[i * 3 + 2] = color->b;
          }*/
        //  ptmpinfo->depthalignrgbviewpublisher.publish(pseudoimage);
        }

        {

          // depthimage.header.seq = depthseq;
          // ptmpinfo->depthseq++;

        //  ptmpinfo->depthalignrgbpublisher.publish(depthimage);
        }
      }
    }
    COMMONUSLEEP(33*1000);
  }
  free(depthaligndata);
  free(depthheader);
  return 0;
}
void depthcallback(void *data, void *userdata)
{
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
  NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));

  uint16_t *depthdata = (uint16_t *)((uint8_t *)depthheader + sizeof(NVPFM_USB_IMAGE_HEADER));

  countfps(&ptmpinfo->depthcountfps, "depth", depthheader->width, depthheader->height);

  if (ptmpinfo->depthtrans->m_smearfiltercfg.enable)
  {
    uint16_t *pic = (uint16_t *)((char *)depthheader + sizeof(NVPFM_USB_IMAGE_HEADER));
    ptmpinfo->depthtrans->m_smearfiltercfg.enable = 1;
    ptmpinfo->depthtrans->depth_smearfilter(
        pic, depthheader->width, depthheader->height,
        ptmpinfo->depthtrans->m_smearfiltercfg.radius,
        ptmpinfo->depthtrans->m_smearfiltercfg.threshold,
        ptmpinfo->depthtrans->m_smearfiltercfg.p);
  }

  if ( ptmpinfo->depthaligndataqueue == NULL) {
    ptmpinfo->depthaligndataqueue = Create_Ring_Queue(2, sizeof(NVPFM_USB_IMAGE_HEADER) + 1280 * 800 * 2);
    nvpfm_info_printf("will ceate dephthalignrgbthread!\n");
    pthread_create(&ptmpinfo->depthalignthreadid, NULL, depthalignrgbthread, ptmpinfo);
  }
  if ( ptmpinfo->camparam->size() == 1) {
    NVPFM_USB_IMAGE_HEADER *p = (NVPFM_USB_IMAGE_HEADER *)SOLO_Write(ptmpinfo->depthaligndataqueue);
    if (p) {
      memcpy(p, depthheader, sizeof(NVPFM_USB_IMAGE_HEADER) + 1280 * 800 * 2);
      SOLO_Write_Over(ptmpinfo->depthaligndataqueue);
    }
  }
   int width=depthheader->width;
   int height=depthheader->height;
   static uint8_t* pseudo=NULL;
   if(NULL==pseudo)pseudo=(uint8_t*)malloc(width*height*3);

   ptmpinfo->depthtrans->compute_depth2pseudo(depthdata,width,height,pseudo);
   cv::Mat srcmat=cv::Mat(height, width, CV_8UC3, pseudo);

    static bool first=true;
       if(first){
         first=false;
         cv::namedWindow("pseudo", cv::WINDOW_NORMAL);
       }

   cv::imshow("pseudo", srcmat);
   cv::waitKey(1);
}

void rgbcallback(void *data, void *userdata)
{
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
  NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));
  uint8_t *rgbdata = (uint8_t *)rgbheader + sizeof(NVPFM_USB_IMAGE_HEADER);
  countfps(&ptmpinfo->rgbcountfps, "rgb", rgbheader->width, rgbheader->height);

  ptmpinfo->rgbwidth=rgbheader->width; 
  ptmpinfo->rgbheight=rgbheader->height; 

   int width=rgbheader->width;
   int height=rgbheader->height;

    cv::Mat srcmat=cv::Mat(height*3/2, width, CV_8UC1, rgbdata);

   cv::Mat dstmat;
       cv::cvtColor(srcmat,dstmat,cv::COLOR_YUV2BGR_NV12);

    static bool first=true;
       if(first){
         first=false;
         cv::namedWindow("rgb", cv::WINDOW_NORMAL);
       }

   cv::imshow("rgb", dstmat);
   cv::waitKey(1);
}
void leftircallback(void *data, void *userdata)
{
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
  NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
  if (tmppack->sub_type == IMAGE_CHANNEL0_CALIBRATED ||
      tmppack->sub_type == IMAGE_CHANNEL0_ORIGNAL)
  {
    NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));
    countfps(&ptmpinfo->leftircountfps, "leftir", leftirheader->width, leftirheader->height);
  }
}
void rightircallback(void *data, void *userdata)
{
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;
  NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket));
  countfps(&ptmpinfo->rightircountfps, "rightir", rightirheader->width, rightirheader->height);
}

void groupcallback(void *depthframe, void *rgbframe, void *leftirframe, void *rightirframe, void *userdata)
{
  DEVICEINFO *ptmpinfo = (DEVICEINFO *)userdata;

  // grouppkt_info_custom_t *tmpimages = (grouppkt_info_custom_t *)data;

  NVPFM_USB_IMAGE_HEADER *depthheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket));
  NVPFM_USB_IMAGE_HEADER *rgbheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)rgbframe + sizeof(NVPTL_USBHeaderDataPacket));
  NVPFM_USB_IMAGE_HEADER *leftirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)leftirframe + sizeof(NVPTL_USBHeaderDataPacket));
  NVPFM_USB_IMAGE_HEADER *rightirheader = (NVPFM_USB_IMAGE_HEADER *)((unsigned char *)rightirframe + sizeof(NVPTL_USBHeaderDataPacket));

  printf("got:depth:group_id:%d,timestamp:%lu\n", depthheader->group_id, depthheader->timestamp);
  printf("got:rgb:group_id:%d,timestamp:%lu\n", rgbheader->group_id, rgbheader->timestamp);
  printf("got:leftir:group_id:%d,timestamp:%lu\n", leftirheader->group_id, leftirheader->timestamp);
  printf("got:rightir:group_id:%d,timestamp:%lu\n", rightirheader->group_id, rightirheader->timestamp);
}
void imucallback(void *data, void *userdata)
{
  DEVICEINFO *info = (DEVICEINFO *)userdata;
  NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;

  s_nvpfm_imu_data *tmpimudata = (s_nvpfm_imu_data *)tmppack->data;

  if (tmppack->type == NVPFM_IMU_DATA)
  {

    if (sizeof(s_nvpfm_imu_data) <= tmppack->len && tmpimudata->data_number > 0)
    {
      for (int i = 0; i < tmpimudata->data_number; i++)
      {
        static unsigned int countfps = 0, lastfps = 0;
        countfps++;
        static time_t last = time(NULL);
        time_t current = time(NULL);
        if ((current - last) > 5)
        {
          printf("imufps:%f\n", (float)(countfps - lastfps) / (float)(current - last));
          last = current;
          lastfps = countfps;
        }
      }
    }
  }
}
void init(DEVICEINFO *info)
{
  if (info->fm->isplugged())
  {
    printf("will connect usb falcon\n");
    ////////////////////here we config resolution and fps and fpsratio and start stream with nvpfm_start
    while (info->willrun)
    {
      if (NVPTL_OK == info->fm->get_devinfo(&info->devinfo))
      {
        printf("got device info!\n");

        break;
      }
      else
      {
        COMMONUSLEEP(1000 * 1000);
      }
    }

    if (!info->willrun)
      return;

    printf("will set timesync cycle!\n");

    info->fm->set_timesynccycle(true, 5);

    /*    struct timeval now;
        gettimeofday(&now, NULL);
        s_nvpfm_time tmpvalue;
        tmpvalue.local_time_alignto_pc_time = (NVP_U64)now.tv_sec * 1000000 + (NVP_U64)now.tv_usec;
        tmpvalue.pc_time = (NVP_U64)now.tv_sec * 1000000 + (NVP_U64)now.tv_usec;

        info->fm->set_timesync(&tmpvalue);
    */
    if (!info->willrun)
      return;
    info->depthtrans = new depthtransformer;
    // info->fm->start_groupimages(groupcallback);

    info->fm->start_leftir(leftircallback);

    info->fm->start_rightir(rightircallback);

    info->fm->start_depth(depthcallback);

    info->fm->start_rgb(rgbcallback);
    info->fm->start_other(othercallback);

    info->fm->start_imu(imucallback);

    if (!info->willrun)
      return;
  }
}

#ifdef _MSC_VER
static unsigned __stdcall devicethread(void *param)
#else
void *devicethread(void *param)
#endif // _MSC_VER
{
  DEVICEINFO *info = (DEVICEINFO *)param;

  if (info->fm == NULL)
  {
    printf("will new nvpfm!\n");
    info->fm = new nvpfm(&info->rawdevinfo, eventcallback, info);
  }

  // info->fm->set_global_time(true);

  if (info->willrun)
  {
    printf("will init nvpfm!\n");
    init(info);
  }
  /* s_get_transfer_config_ret ret;
   if (NVPTL_OK == info->fm->get_transfer_config(&ret)) {
     if (ret.ret == 0) {
       ret.config.image_transfer_frame_fps[IMAGE_CHANNEL0_CALIBRATED] = 15;
       ret.config.image_transfer_frame_fps[IMAGE_CHANNEL1_CALIBRATED] = 15;
       s_set_transfer_config cfg;
       cfg.config = ret.config;
       if (NVPTL_OK == info->fm->set_transfer_config(&cfg)) {
         printf("set ir fps to 15 ok!\n");
       }
     }
   }*/

  while (info->willrun)
  {
    s_nvpfm_get_sensor_config_ret mycfg;
    if (NVPTL_OK == info->fm->get_sensorcfg(&mycfg))
    {
      if (mycfg.ret == 0)
      {
#if 0
        printf("got sensor cfg\nir:%s %d\nrgb:%s %d\n",
               nvpfm::framesize2str(mycfg.config.isp_frame_size[0]),
               mycfg.config.fps[0],
               nvpfm::framesize2str(mycfg.config.isp_frame_size[2]),
               mycfg.config.fps[2]);
#endif
/*    if(mycfg.config.isp_frame_size[0]!=info->config.resolution||
        mycfg.config.isp_frame_size[2]!=info->config.rgbresolution||
        mycfg.config.fps[0]!=info->config.fps){
          printf("res or fps not as launch config,will test if can change!\n");
          if(IMAGE_UNKNOWN!=nvpfm::findsensorresbyisp(info->devinfo,info->config.resolution,CHANNEL0)&&
             IMAGE_UNKNOWN!=nvpfm::findsensorresbyisp(info->devinfo,info->config.rgbresolution,CHANNEL2)){
                mycfg.config.isp_frame_size[0]=info->config.resolution;
                mycfg.config.isp_frame_size[1]=info->config.resolution;
                mycfg.config.isp_frame_size[2]=info->config.rgbresolution;

                mycfg.config.frame_size[0]=nvpfm::findsensorresbyisp(info->devinfo,info->config.resolution,CHANNEL0);
                mycfg.config.frame_size[1]=nvpfm::findsensorresbyisp(info->devinfo,info->config.resolution,CHANNEL1);
                mycfg.config.frame_size[2]=nvpfm::findsensorresbyisp(info->devinfo,info->config.rgbresolution,CHANNEL2);

                mycfg.config.fps[0]=info->config.fps;
                mycfg.config.fps[1]=info->config.fps;
                mycfg.config.fps[2]=info->config.fps;

                info->fm->set_sensorcfg(&mycfg.config);
                printf("ok to change res to:%s->%s,%d\n",
                nvpfm::framesize2str(mycfg.config.frame_size[0]),
                nvpfm::framesize2str(mycfg.config.isp_frame_size[0]),info->config.fps);
             }else{
                printf("launch config resolution not supported!\n");
             }
        }else{
         // printf("ok,res and fps is as launch set!\n");
        }*/
// change sensor resolution
#ifdef CHANGE_SENSOR_RES
        s_nvpfm_dev_info devinfo;
        if (NVPTL_OK == info->fm->get_devinfo(&devinfo))
        {
          if (IMAGE_1280_800 != mycfg.config.frame_size[devinfo.image_dev_info.depth[0].left_channel] ||
              IMAGE_1280_800 != mycfg.config.frame_size[devinfo.image_dev_info.depth[0].right_channel] ||
              IMAGE_320_200 != mycfg.config.isp_frame_size[devinfo.image_dev_info.depth[0].left_channel] ||
              IMAGE_320_200 != mycfg.config.isp_frame_size[devinfo.image_dev_info.depth[0].right_channel])
          {
            s_nvpfm_app_sensor_config scfg;
            memcpy(&scfg, &mycfg.config, sizeof(scfg));
            scfg.frame_size[devinfo.image_dev_info.depth[0].left_channel] = IMAGE_1280_800;
            scfg.frame_size[devinfo.image_dev_info.depth[0].right_channel] = IMAGE_1280_800;
            scfg.isp_frame_size[devinfo.image_dev_info.depth[0].left_channel] = IMAGE_320_200;
            scfg.isp_frame_size[devinfo.image_dev_info.depth[0].right_channel] = IMAGE_320_200;
            if (NVPTL_OK == info->fm->set_sensorcfg(&scfg))
            {
              printf("setsensor cfg \n");
            }
          }
        }
#endif
      }
    }

    while (info->willrun && info->camparam->size() == 0)
    {
      s_nvpfm_camera_param param;
      if (NVPTL_OK == info->fm->get_camera_param(&param))
      {
        printf("got camera param!!!!\n");
        info->camparam->push_back(param);
        break;
      }
      else
      {
        printf("fail to get camera param!\n");
        COMMONUSLEEP(1000 * 1000);
      }
    }
    /*
        while (info->willrun)
        {
          s_nvpfm_imu_external_reference* param;
          if((param=info->fm->get_imu_externalref())!=NULL)
          {
              printf("got imu extern param!!!!\n");
              for(int i=0;i<16;i++){
                printf("t_cam_imu[%d]=%f\n",i,param->t_cam_imu[i]);
              }
              printf("timeshift_cam_imu=%f\n",param->timeshift_cam_imu);
              break;
          }
          else
          {
            printf("fail to get imu extern param!\n");
            COMMONUSLEEP(1000 * 1000);
          }
        }

        while (info->willrun)
        {
          s_nvpfm_imu_internal_reference* param;
          if((param=info->fm->get_imu_internalref())!=NULL)
          {
              printf("got imu intern param!!!!\n");
              for(int i=0;i<9;i++){
                printf("accel_m[%d]=%f\n",i,param->accel_m[i]);
              }
              for(int i=0;i<3;i++){
                printf("accel_b[%d]=%f\n",i,param->accel_b[i]);
              }
              for(int i=0;i<3;i++){
                printf("gyro_b[%d]=%f\n",i,param->gyro_b[i]);
              }
              break;
          }
          else
          {
            printf("fail to get imu intern param!\n");
            COMMONUSLEEP(1000 * 1000);
          }
        }
    */
    COMMONUSLEEP(1000 * 1000);
  }
  return 0;
}
bool g_willrun = true;
#ifdef _MSC_VER
unsigned __stdcall enumthread(void *param)
#else
void *enumthread(void *param)
#endif // _VS_VER
{
  while (g_willrun)
  {

    int total = 0;
    g_pdevice = NULL;
    if (NVPTL_OK == nvptl_enum_sync(&total, &g_pdevice))
    {
      if (total > 0)
      {
        // printf("enum %d falcons!", total);
        NVPTL_DEVICE_INFO *tmpdevice = g_pdevice;
        while (tmpdevice != NULL)
        {
          // printf("falcon:%s", tmpdevice->usb_camera_name);
          //  if not connected
          if (g_devicemap.find(tmpdevice->usb_camera_name) == g_devicemap.end())
          { // 没有找到，新的相机连接
            printf("no falcon:%s in map,so create new camera instance", tmpdevice->usb_camera_name);
            DEVICEINFO *info = (DEVICEINFO *)calloc(1, sizeof(DEVICEINFO));

            info->rawdevinfo = *tmpdevice;
            info->cnnqueue = Create_Ring_Queue(1, sizeof(s_nvpfm_cnn_data));

            info->camparam = new std::vector<s_nvpfm_camera_param>;
#ifdef _MSC_VER
            unsigned __stdcall devicethread(void *param);
#else
            void *devicethread(void *param);
#endif // _MSC_VER

            printf("set will run true!\n");
            info->willrun = true;

#ifdef _MSC_VER
            info->threadid = _beginthreadex(NULL, 0, devicethread, info, 0, NULL);
#else
            pthread_create(&info->threadid, NULL, devicethread, info);
#endif
            g_devicemap[tmpdevice->usb_camera_name] = info;
          }
          tmpdevice = tmpdevice->next;
        }
        nvptl_freedevices(g_pdevice);
      }
    }
    COMMONUSLEEP(1000 * 1000);
  }
  return 0;
}
int main(int argc, const char *argv[])
{
  if (argc > 1)
  {
    printf("usb camera usage:./test_cpp\n");
    exit(0);
  }

  nvpfm_init("./falcon.log", 50 * 1024 * 1024);

#ifdef _MSC_VER
  _beginthreadex(NULL, 0, enumthread, NULL, 0, NULL);
  // CloseHandle(enumhandle);
#else
  pthread_t threadid;

  pthread_create(&threadid, NULL, enumthread, NULL);
#endif

  string str;
  char tmpstr[64];
  while (1)
  {
    cin.getline(tmpstr, 64);
    str = tmpstr;
    if (str == "exit")
    {
      g_willrun = false;
      pthread_join(threadid, NULL);

      for (auto it = g_devicemap.begin(); it != g_devicemap.end(); ++it)
      {
        DEVICEINFO *tmpinfo = it->second;
        tmpinfo->willrun = false;
        pthread_join(tmpinfo->threadid, NULL);
        delete tmpinfo->fm;
        delete tmpinfo->depthtrans;
        delete tmpinfo->camparam;
        free(tmpinfo);
      }
      nvpfm_deinit();
      exit(0);
    }
  }
  return 0;
}
