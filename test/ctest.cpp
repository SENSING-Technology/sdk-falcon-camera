#include <gtest/gtest.h>
#include <iostream>
#include <chrono>
#include <sys/time.h>
#include <memory>
#include <thread>

#include "nvpfm.h"

using namespace std;

namespace FalconCTestNameSpace {
class FalconCTestBase : public testing::Test {
   protected:
    virtual void SetUp() override {
    }

    virtual void TearDown() override {}

   public:
    
};

/**
 * @brief Initialize
 */
TEST_F(FalconCTestBase, Initialize) {
    ASSERT_EQ(0,nvpfm_init());
    nvpfm_deinit(); 
}

/**
 * @brief Get SDK version
 */
TEST_F(FalconCTestBase, GetSDKVersion) {
    ASSERT_EQ(0,nvpfm_init());
    ASSERT_NE((const char*)0,nvpfm_getsdkversion());
    cout<<"get sdk version:"<<nvpfm_getsdkversion()<<endl; 
    nvpfm_deinit(); 
}

/**
 * @brief Enumerate falcon
 */
TEST_F(FalconCTestBase, Enumerate) {
    ASSERT_EQ(0,nvpfm_init());
    int total=0;
    NVPFM_DEVICE_INFO* pdevices=NULL;
    NVPTL_RESULT result=nvpfm_enum_device(&total,&pdevices);	
    ASSERT_EQ(NVPTL_OK,result);
    cout<<"has enumerate:"<<total<<" falcons"<<endl;	

    nvpfm_deinit(); 
}

/**
 * @brief Open falcon
 */
TEST_F(FalconCTestBase, OpenUSBCamera) {
    ASSERT_EQ(0,nvpfm_init());
    int total=0;
    NVPFM_DEVICE_INFO* pdevices=NULL;
    NVPTL_RESULT result=nvpfm_enum_device(&total,&pdevices);	
    ASSERT_EQ(NVPTL_OK,result);
    cout<<"has enumerate:"<<total<<" falcons"<<endl;	
   
    NVPFM_DEVICE_INFO* thedevice=NULL; 
    NVPFM_DEVICE_INFO* tmpdevice=pdevices;
    while(tmpdevice!=NULL){
	if(tmpdevice->type==NVPFM_USB_INTERFACE){
		thedevice=tmpdevice;
		break;	
	}
	tmpdevice=tmpdevice->next;	
    }
    ASSERT_NE((void*)NULL,(void*)thedevice);
    NVPFM_DEVICE_HANDLE handle=nvpfm_open(thedevice);
    ASSERT_NE((void*)handle,(void*)NULL);
    nvpfm_close(handle); 
    nvpfm_deinit(); 
}
void depthcallback(void *data, void *userdata)
{

}

/**
 * @brief Time sync
 */
TEST_F(FalconCTestBase, Timesync) {
    ASSERT_EQ(0,nvpfm_init());
    int total=0;
    NVPFM_DEVICE_INFO* pdevices=NULL;
    NVPTL_RESULT result=nvpfm_enum_device(&total,&pdevices);	
    ASSERT_EQ(NVPTL_OK,result);
    cout<<"has enumerate:"<<total<<" falcons"<<endl;	
   
    NVPFM_DEVICE_INFO* thedevice=NULL; 
    NVPFM_DEVICE_INFO* tmpdevice=pdevices;
    while(tmpdevice!=NULL){
	if(tmpdevice->type==NVPFM_USB_INTERFACE){
		thedevice=tmpdevice;
		break;	
	}
	tmpdevice=tmpdevice->next;	
    }
    ASSERT_NE((void*)NULL,(void*)thedevice);
    NVPFM_DEVICE_HANDLE handle=nvpfm_open(thedevice);
    ASSERT_NE((void*)handle,(void*)NULL);

  NVPFM_CONFIG config;
  config.irres = NVPFM_RESOLUTION_640_400;
  config.rgbres = NVPFM_RESOLUTION_640_400;
  config.fps = FPS_30;
  config.fpsratio = 1;

  NVPTL_RESULT ret = nvpfm_config(handle, &config);
    ASSERT_EQ(ret,NVPTL_OK);
    
  nvpfm_start(handle, NVPFM_STREAM_DEPTH, depthcallback, NULL);
    sleep(3);

	s_nvpfm_time_ret lastinfo;
	lastinfo.pc_time=0;	
      for(int i=0;i<2;i++){
       NVP_U64 param = 0;
        struct timeval tv;
        gettimeofday(&tv, NULL);
        param = tv.tv_sec * 1000000 + tv.tv_usec;
        printf("will timesync to:%llu\n", param);
        s_nvpfm_time_ret tmpparam;
	tmpparam.pc_time = param;
        
        NVPTL_RESULT ret = nvpfm_set(handle, NVPFM_TIME_SYNC, &tmpparam, sizeof(tmpparam), 2000);
        ASSERT_EQ(NVPTL_OK , ret);
         
        if(lastinfo.pc_time!=0){
		long offset=param-(lastinfo.local_time_calib+tmpparam.local_time_no_calib-lastinfo.local_time_no_calib);
		printf("offset:%ld\n",offset);
          	ASSERT_LT(labs(offset),10000); 
        } 
        //printf("-----------------------time sync,sent:%llu,camera primitive timestamp:%llu,after calib:%lld\n",param, tmpparam.local_time_no_calib, tmpparam.local_time_calib);
        lastinfo=tmpparam;
 
        sleep(5); 
    }

    nvpfm_close(handle); 
    nvpfm_deinit(); 
}



}
