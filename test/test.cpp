#include "gtest/gtest.h"

#include <chrono>
#include <memory>
#include <thread>
#include <iostream>
#include <stdio.h>

#include <sys/time.h>

#include "nvpfm.hpp"
#include "nvpfm.h"
#include "test_process.h"

using namespace std;

#define LOG_FILE "./falcon.log"
#define LOG_FILE_SIZE 512 * 1024

#define MAX_PROJECTOR_NUM 1

#define MAX_GET_TRY_TIMES 10

#define FEYNMAN_TEST_BASE
#define FEYNMAN_TEST_CAMERA
// #define FEYNMAN_TEST_REBOOT

typedef enum {
    EM_CAMERA_PLUGGED,
    EM_CAMERA_REBOOT,
    EM_CAMERA_PLUGOUT
}E_CAMERA_STATUS;

void *releasethread(void *camera)
{
//   DEVICEINFO *info = (DEVICEINFO *)userdata;

//   info->willrun = false;
//   printf("in release thread,join threadid!\n");
// #ifdef _MSC_VER
//   WaitForSingleObject((HANDLE)info->threadid, INFINITE);
// #else
//   pthread_join(info->threadid, NULL);
// #endif
//   printf("join depththreadid!\n");
// #ifdef _MSC_VER
//   WaitForSingleObject((HANDLE)info->depththreadid, INFINITE);
// #else
//   pthread_join(info->depththreadid, NULL);
// #endif

  printf("delete nvpfm!\n");
  ////////////////////////////delete instance
  delete camera;
  camera = NULL;
//   delete info->fm;

//   printf("erase instance in map:%s!\n", info->rawdevinfo.usb_camera_name);
//   //////////////////////////////remove from map
//   g_devicemap.erase(info->rawdevinfo.usb_camera_name);

//   printf("delete camparam!\n");
//   delete info->camparam;
//   printf("free info!\n");
//   free(info);
  return 0;
}


namespace FalconTestNamespace {

    class FalconTestBase: public testing::Test 
    {
        protected:
            virtual void SetUp() override {
                // cout << "FalconTestBase setup" << endl;
            }

            virtual void TearDown() override {
                //  cout << "FalconTestBase teardown" << endl;
            }
        public:
            shared_ptr<nvpfm> basecamera_;
    };

    class FalconTestCamera : public testing::Test
    {
    protected:
        virtual void SetUp() override {
            // cout << "FalconTestCamera setup" << endl;
        }

        virtual void TearDown() override {
            // cout << "FalconTestCamera teardown" << endl;
        }

        // Per-test-suite set-up.
        // Called before the first test in this test suite.
        // Can be omitted if not needed.
        static void SetUpTestSuite() {
            cout << "SetUpTestSuite" << endl;
            nvpfm_init(LOG_FILE, LOG_FILE_SIZE);
            NVPTL_DEVICE_INFO *deviceInfo;
            int total = 0;
            nvptl_enum_sync(&total, &deviceInfo);
            if(total > 0) {
                camera_ = make_shared<nvpfm>(deviceInfo, nullptr, nullptr);
            }
            COMMONUSLEEP(500);
            nvptl_freedevices(deviceInfo);
        }

        // Per-test-suite tear-down.
        // Called after the last test in this test suite.
        // Can be omitted if not needed.
        static void TearDownTestSuite() {
            cout << "TearDownTestSuite" << endl;
            camera_ = nullptr;
            nvpfm_deinit();
        }

        // static void eventcallback(EVENTREASON reason,void* userdata) {
        //     // device_model* model = (device_model*)userdata;
        //     if (reason == INFORMREBOOT) {
        //         nvpfm_info_printf("will send reboot to camera!\n");
        //         // FalconTestCamera::camera_->send_reboot();
        //         nvpfm_info_printf("after send reboot to camera!\n");
        //     }
        //     else {
        //         nvpfm_info_printf("plugout!\n");
        //     }
        //     // g_closefalcon.push_back(model->getdevicehandle());
        // }

    public:
        static shared_ptr<nvpfm> camera_;
    };

    class FalconTestReboot: public testing::Test {
        protected:
            virtual void SetUp() override {
                // cout << "FalconTestBase setup" << endl;
                // status_ = EM_CAMERA_PLUGOUT;
            }

            virtual void TearDown() override {
                //  cout << "FalconTestBase teardown" << endl;
                // camera_ = nullptr;
                // nvpfm_deinit();
            }

              // Per-test-suite set-up.
            // Called before the first test in this test suite.
            // Can be omitted if not needed.
            static void SetUpTestSuite() {
                cout << "SetUpTestSuite" << endl;
                nvpfm_init(LOG_FILE, LOG_FILE_SIZE);
                // NVPTL_DEVICE_INFO *deviceInfo;
                // int total = 0;
                // nvptl_enum_sync(&total, &deviceInfo);
                // if(total > 0) {
                //     camera_ = make_shared<nvpfm>(deviceInfo, eventcallback, nullptr);
                // }
                // COMMONUSLEEP(500);
                // nvptl_freedevices(deviceInfo);
                // status_ = EM_CAMERA_PLUGGED;
            }

            // Per-test-suite tear-down.
            // Called after the last test in this test suite.
            // Can be omitted if not needed.
            static void TearDownTestSuite() {
                cout << "TearDownTestSuite" << endl;
                // camera_ = nullptr;
                // nvpfm_deinit();
                // status_ = EM_CAMERA_PLUGOUT;
            }

            static void eventcallback(EVENTREASON reason,void* userdata) {
                // device_model* model = (device_model*)userdata;
                if (reason == INFORMREBOOT) {
                    nvpfm_info_printf("will send reboot to camera!\n");
                    status_ = EM_CAMERA_REBOOT;
                    camera_->send_reboot();
                    nvpfm_info_printf("after send reboot to camera!\n");
                }
                else {
                   
                    status_ = EM_CAMERA_PLUGOUT;
                    // camera_ = nullptr;
                    // nvpfm_deinit();
                    

                    // NVPTL_DEVICE_INFO *deviceInfo;
                    // int total = 0;
                     nvpfm_info_printf("plugout 111!\n");
                    //while (total == 0)
                    // {
                    //     nvptl_enum_sync(&total, &deviceInfo);
                    //     nvpfm_info_printf("reboot total:%d!\n", total);
                    //     if(total > 0) {
                    //         camera_ = make_shared<nvpfm>(deviceInfo, eventcallback, nullptr);
                    //         status_ = EM_CAMERA_PLUGGED;
                    //         nvpfm_info_printf("reboot succeed!\n");
                    //         // break;
                    //     }
                    //     COMMONUSLEEP(500);
                    //     nvptl_freedevices(deviceInfo);
                        
                    // }

                }
                // g_closefalcon.push_back(model->getdevicehandle());
            }

        // static void eventcallback(EVENTREASON reason,void *userdata)
        // {
        //     // DEVICEINFO *info = (DEVICEINFO *)userdata;
        //     if (reason == INFORMREBOOT) {
        //         nvpfm_info_printf("will send reboot to camera!\n");
        //         camera_->send_reboot();
        //         nvpfm_info_printf("after send reboot to camera!\n");
        //     }
        //     else {
        //         nvpfm_info_printf("plugout!\n");
        //     }

        // // #ifdef _MSC_VER
        // //     unsigned int threadID = _beginthreadex(NULL, 0, releasethread, camera_, 0, NULL);
        // // #else
        // //     pthread_t releasethreadid;
        // //     pthread_create(&releasethreadid, NULL, releasethread, camera_);
        // //     pthread_detach(releasethreadid);
        // // #endif
        // }
        public:
           static nvpfm* camera_;
           static E_CAMERA_STATUS status_;

    };
    shared_ptr<nvpfm> FalconTestCamera::camera_ = nullptr;
    nvpfm * FalconTestReboot::camera_ = NULL;
    E_CAMERA_STATUS FalconTestReboot::status_ = EM_CAMERA_PLUGOUT;

    #ifdef FEYNMAN_TEST_BASE
    // /**
    //  * @brief Initialize
    //  */
    TEST_F(FalconTestBase, Initialize) {
        cout << "start test init" << endl;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        nvpfm_deinit(); 
    }

    /**
     * @brief get sdk version
     */
    TEST_F(FalconTestBase, GetSDKVersion) {
        cout << "start test get sdk version" << endl;
        EXPECT_NE((const char *)0, nvpfm_getsdkversion());
        cout << "get sdk version: " << nvpfm_getsdkversion() << endl;
    }

    /**
     * @brief get all the devices
     */
    TEST_F(FalconTestBase, EnumDevice) {
        int total = 0;
        NVPTL_DEVICE_INFO *deviceInfo;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceInfo));
        cout << "enum device total: " << total << endl;
        nvptl_freedevices(deviceInfo);
        nvpfm_deinit();
    }

    /**
     * @brief open device and close device
     */
    TEST_F(FalconTestBase, OpenCamaer) {
        int total = 0;
        NVPTL_DEVICE_INFO *deviceInfo;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceInfo));
        cout << "OpenCamaer enum device total: " << total << endl;
        ASSERT_GT(total, 0);

        NVPTL_DEVICE_INFO *pDevice = deviceInfo;
        while (pDevice)
        {
            shared_ptr <nvpfm> fm =make_shared<nvpfm>(pDevice, nullptr, nullptr);
            ASSERT_NE(nullptr, fm) << "open device failed" << endl;
            EXPECT_TRUE(fm->isplugged()) << "camera is not plugged" << endl;
            cout << "open device successfully!" << endl;
            fm = nullptr;
            pDevice = deviceInfo->next;
        }
        cout << "free device!" << endl;
        nvptl_freedevices(deviceInfo);
        nvpfm_deinit();
    }

    /**
     * @brief  get device info
     */
    TEST_F(FalconTestBase, GetDevInfo) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GT(total, 0);
        shared_ptr <nvpfm> fm = make_shared<nvpfm>(deviceinfo, nullptr, nullptr);
        ASSERT_NE(nullptr, fm) << "open camera failed" << endl;
        EXPECT_TRUE(fm->isplugged()) << "camera is not plugged" << endl;;

        s_nvpfm_dev_info info;
        memset(&info, 0, sizeof(info));
        // NVPTL_RESULT ret;
        // for (int i = 0; i < MAX_GET_TRY_TIMES; i ++) {
        //     ret = fm->get_devinfo(&info);
        //     if(NVPTL_OK == ret) {
        //         break;
        //     } else {
        //         COMMONUSLEEP(500);
        //     }
        // }
        // ASSERT_EQ(NVPTL_OK, ret) << "get dev info failed" << endl;
        ASSERT_EQ(NVPTL_OK, fm->get_devinfo(&info)) << "get dev info failed" << endl;
        printf("dev info sn:%s, fireware ver: %s\n", info.sn, info.software_version);
        fm = nullptr;
        nvptl_freedevices(deviceinfo);
        nvpfm_deinit();
    }

     /**
     * @brief  get depth
     */

    TEST_F(FalconTestBase, GetDepth) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GE(total, 0);
        basecamera_ = make_shared<nvpfm>(deviceinfo, nullptr, nullptr);
        ASSERT_NE(nullptr, basecamera_) << "open camera failed" << endl;
        EXPECT_TRUE(basecamera_->isplugged()) << "camera is not plugged" << endl;
        EXPECT_EQ(NVPTL_OK, basecamera_->start_depth(depthcallback)) << "start depth failed" << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        basecamera_ = nullptr;
        nvpfm_freedevices(deviceinfo);
        nvpfm_deinit();
    }

     /**
     * @brief  get other
     */
    TEST_F(FalconTestBase, GetOther) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GE(total, 0);
        basecamera_ = make_shared<nvpfm>(deviceinfo, nullptr, nullptr);
        ASSERT_NE(nullptr, basecamera_) << "open camera failed" << endl;
        EXPECT_TRUE(basecamera_->isplugged()) << "camera is not plugged" << endl;
        EXPECT_EQ(NVPTL_OK, basecamera_->start_other(othercallback));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        basecamera_ = nullptr;
        nvptl_freedevices(deviceinfo);
        nvpfm_deinit();
    }

     /**
     * @brief  get imu 
     */
    TEST_F(FalconTestBase, GetIMU) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GE(total, 0);
        basecamera_ = make_shared<nvpfm>(deviceinfo, nullptr, nullptr);
        ASSERT_NE(nullptr, basecamera_) << "open camera failed" << endl;
        EXPECT_TRUE(basecamera_->isplugged()) << "camera is not plugged" << endl;
        EXPECT_EQ(NVPTL_OK, basecamera_->start_imu(imucallback));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        basecamera_ = nullptr;
        nvptl_freedevices(deviceinfo);
        nvpfm_deinit();
    }

     /**
     * @brief  get rgb
     */
    TEST_F(FalconTestBase, GetRGB) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GE(total, 0);
        basecamera_ = make_shared<nvpfm>(deviceinfo, nullptr, nullptr);
        ASSERT_NE(nullptr, basecamera_) << "open camera failed" << endl;
        EXPECT_TRUE(basecamera_->isplugged()) << "camera is not plugged" << endl;
        EXPECT_EQ(NVPTL_OK, basecamera_->start_rgb(rgbcallback));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        basecamera_ = nullptr;
        nvptl_freedevices(deviceinfo);
        nvpfm_deinit();
    }

     /**
     * @brief  get left IR
     */
    TEST_F(FalconTestBase, GetLeftIR) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GT(total, 0);
        basecamera_ = make_shared<nvpfm>(deviceinfo, nullptr, nullptr);
        ASSERT_NE(nullptr, basecamera_) << "open camera failed" << endl;
        EXPECT_TRUE(basecamera_->isplugged()) << "camera is not plugged" << endl;
        EXPECT_EQ(NVPTL_OK, basecamera_->start_leftir(leftircallback));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        basecamera_ = nullptr;
        nvptl_freedevices(deviceinfo);
        nvpfm_deinit();
    }

     /**
     * @brief  get right IR
     */
    TEST_F(FalconTestBase, GetRightIR) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GT(total, 0);
        basecamera_ = make_shared<nvpfm>(deviceinfo, nullptr, nullptr);
        ASSERT_NE(nullptr, basecamera_) << "open camera failed" << endl;
        EXPECT_EQ(NVPTL_OK, basecamera_->start_leftir(rightircallback));
        EXPECT_TRUE(basecamera_->isplugged()) << "camera is not plugged" << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        basecamera_ = nullptr;
        nvptl_freedevices(deviceinfo);
        nvpfm_deinit();
    }
    #endif


    #ifdef FEYNMAN_TEST_CAMERA
    /**
     * @brief  get sensor config to be synchronized
     */
    TEST_F(FalconTestCamera, GetSensorCfgSync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
        s_nvpfm_get_sensor_config_ret sensorCfg;
        memset(&sensorCfg, 0, sizeof(sensorCfg));
        NVPTL_RESULT ret;
        for (int i = 0; i < MAX_GET_TRY_TIMES; i++)
        {
            ret = camera_->get_sensorcfg(&sensorCfg);
            // EXPECT_EQ(NVPTL_OK, ret) << "get no sensor the " << i << " times." << endl;
            if (NVPTL_OK == ret) {
                ASSERT_EQ(0, sensorCfg.ret) << "the sensor config ret is not 0 " << endl;
                if (0 == sensorCfg.ret) {
                    cout << "get sensor config successfully!" << endl;
                    break;
                }
            } else {
                COMMONUSLEEP(500);
            }
        }
        ASSERT_EQ(NVPTL_OK, ret) << "get the sensor config failed " << endl;
    }

     /**
     * @brief  set sensor config to be synchronized
     */
    TEST_F(FalconTestCamera, SetSensorCfgSync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        ASSERT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
        s_nvpfm_get_sensor_config_ret sensorCfg;
        memset(&sensorCfg, 0, sizeof(sensorCfg));
        NVPTL_RESULT ret;
        for (int i = 0; i < MAX_GET_TRY_TIMES; i++)
        {
            ret = camera_->get_sensorcfg(&sensorCfg);
            // EXPECT_EQ(NVPTL_OK, ret) << "get no sensor the " << i << " times." << endl;
            if (NVPTL_OK == ret) {
                EXPECT_EQ(0, sensorCfg.ret) << "the sensor config ret is not 0 " << endl;
                if (0 == sensorCfg.ret) {
                    cout << "get sensor config successfully!" << endl;
                    break;
                }
            } else {
                COMMONUSLEEP(500);
            }
        }
        ASSERT_EQ(NVPTL_OK, ret) << "get the sensor config failed " << endl;

        for(int i = 0; i < MAX_SENSOR_NUMBER; i ++) {
            nvptl_debug_printf("[before set ] get sensor config[%d], framesize:%d\n",
            i, sensorCfg.config.fps[i]);
        }

        s_nvpfm_app_sensor_config cfg;
        memcpy(&cfg, &sensorCfg.config, sizeof(cfg));
        cfg.fps[0] = 15;
        ASSERT_EQ(NVPTL_OK, camera_->set_sensorcfg(&cfg));

        s_nvpfm_get_sensor_config_ret newcfg;
        ASSERT_EQ(NVPTL_OK, camera_->get_sensorcfg(&newcfg));
        ASSERT_EQ(0, newcfg.ret);
        for(int i = 0; i < MAX_SENSOR_NUMBER; i ++) {
            nvptl_debug_printf("[after set ] get sensor config[%d], framesize:%d\n",
            i, newcfg.config.fps[i]);
        }

        //restore setting
        memcpy(&cfg, &sensorCfg.config, sizeof(sensorCfg));
        ASSERT_EQ(NVPTL_OK, camera_->set_sensorcfg(&cfg));
        ASSERT_EQ(NVPTL_OK, camera_->get_sensorcfg(&newcfg));
        ASSERT_EQ(0, newcfg.ret);
        for(int i = 0; i < MAX_SENSOR_NUMBER; i ++) {
            nvptl_debug_printf("[restore set ] get sensor config[%d], framesize:%d\n",
            i, newcfg.config.fps[i]);
        }

    }


    /**
     * @brief  get sensor config to be asynchronized
     */
    // TEST_F(FalconTestCamera, GetSensorCfgAsync) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
    //     camera_->get_sensorcfg_async(sensorcfgcallback, nullptr);
    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    /**
     * @brief  get device info
     */
    TEST_F(FalconTestCamera, GetDevInfo) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
        s_nvpfm_dev_info info;
        memset(&info, 0, sizeof(info));
        ASSERT_EQ(NVPTL_OK, camera_->get_devinfo(&info)) << "get dev info failed" << endl;
        nvptl_debug_printf("dev info sn:%s, fireware ver: %s\n", info.sn, info.software_version);
    }
    // /**
    //  * @brief  get depth
    //  */

    // TEST_F(FalconTestCamera, GetDepth) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
    //     EXPECT_EQ(NVPTL_OK, camera_->start_depth(depthcallback)) << "start depth failed" << endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // }

    //  /**
    //  * @brief  get other
    //  */
    // TEST_F(FalconTestCamera, GetOther) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
    //     EXPECT_EQ(NVPTL_OK, camera_->start_other(othercallback));
    //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }

    //  /**
    //  * @brief  get imu 
    //  */
    // TEST_F(FalconTestCamera, GetIMU) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
    //     EXPECT_EQ(NVPTL_OK, camera_->start_imu(imucallback));
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }

    /**
     * @brief  stop imu 
     */
    // TEST_F(FalconTestCamera, StopIMU) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

    //     EXPECT_EQ(NVPTL_OK, camera_->stop_imu());
    //     nvptl_debug_printf("stop imu\n");
    // }

     /**
     * @brief  get rgb
     */
    // TEST_F(FalconTestCamera, GetRGB) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
    //     EXPECT_EQ(NVPTL_OK, camera_->start_rgb(rgbcallback));
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }

    //  /**
    //  * @brief  get left IR
    //  */
    // TEST_F(FalconTestCamera, GetLeftIR) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
    //     EXPECT_EQ(NVPTL_OK, camera_->start_leftir(leftircallback));
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }

    //  /**
    //  * @brief  get right IR
    //  */
    // TEST_F(FalconTestCamera, GetRightIR) {
    //     ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
    //     EXPECT_EQ(NVPTL_OK, camera_->start_leftir(rightircallback));
    //     EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
    //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
    // }

      /**
     * @brief  get camera param
     */
    TEST_F(FalconTestCamera, GetCameraParam) {
        s_nvpfm_camera_param param;
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
        EXPECT_EQ(NVPTL_OK, camera_->get_camera_param(&param));
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));
        cout << "get camera param, width: " << param.irwidth << ", height: " << param.irheight << endl;
    }

     /**
     * @brief  get IR exposure to be synchronized
     */
    TEST_F(FalconTestCamera, GetIRExposureSync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_get_sensor_exposure_ret exposure;
        EXPECT_EQ(NVPTL_OK, camera_->get_irexposure(&exposure));
        ASSERT_EQ(0, exposure.ret);
        nvptl_debug_printf("ir exposure time:%d, \n", exposure.exposure.exposure_time);
    }

     /**
     * @brief  set IR exposure to be synchronized
     */
    TEST_F(FalconTestCamera, SetIRExposureSync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_get_sensor_exposure_ret exposure;
        EXPECT_EQ(NVPTL_OK, camera_->get_irexposure(&exposure));
        ASSERT_EQ(0, exposure.ret);
        nvptl_debug_printf("[before set] ir exposure time:%d, mode:%d\n", exposure.exposure.exposure_time, exposure.exposure.exposure_mode);

        s_nvpfm_set_sensor_exposure exposurecfg;
        exposurecfg.channel = CHANNEL0;
        memcpy(&exposurecfg.config, &exposure.exposure, sizeof(exposurecfg.config));
        exposurecfg.config.exposure_mode = 1;
        exposurecfg.config.exposure_time = 1;
        EXPECT_EQ(NVPTL_OK, camera_->set_irexposure(&exposurecfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        s_nvpfm_get_sensor_exposure_ret newcfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_irexposure(&newcfg));
        ASSERT_EQ(0, newcfg.ret);
        nvptl_debug_printf("[after set] ir exposure time:%d, mode:%d\n", 
        newcfg.exposure.exposure_time, newcfg.exposure.exposure_mode);

        //restore the setting
        exposurecfg.channel = CHANNEL0;
        memcpy(&exposurecfg.config, &exposure.exposure, sizeof(exposurecfg.config));
        EXPECT_EQ(NVPTL_OK, camera_->set_irexposure(&exposurecfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

         EXPECT_EQ(NVPTL_OK, camera_->get_irexposure(&newcfg));
        ASSERT_EQ(0, newcfg.ret);
        nvptl_debug_printf("[restore set] ir exposure time:%d, mode:%d\n", 
        newcfg.exposure.exposure_time, newcfg.exposure.exposure_mode);
    }

    /**
     * @brief  get IR exposure to be synchronized
     */
    TEST_F(FalconTestCamera, GetRGBExposureSync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_get_sensor_exposure_ret exposure;
        EXPECT_EQ(NVPTL_OK, camera_->get_rgbexposure(&exposure));
        ASSERT_EQ(0, exposure.ret);
        nvptl_debug_printf("rgb exposure time:%d\n", exposure.exposure.exposure_time);
    }

     /**
     * @brief  set RGB exposure to be synchronized
     */
    TEST_F(FalconTestCamera, SetRGBExposureSync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_get_sensor_exposure_ret exposure;
        EXPECT_EQ(NVPTL_OK, camera_->get_rgbexposure(&exposure));
        ASSERT_EQ(0, exposure.ret);
        nvptl_debug_printf("[before set] RGB exposure time:%d, mode:%d\n", exposure.exposure.exposure_time, exposure.exposure.exposure_mode);

        s_nvpfm_set_sensor_exposure exposurecfg; 
        exposurecfg.channel = CHANNEL2;
        memcpy(&exposurecfg.config, &exposure.exposure, sizeof(exposurecfg.config));
        exposurecfg.config.exposure_mode = 1;
        exposurecfg.config.exposure_time = 2;
        EXPECT_EQ(NVPTL_OK, camera_->set_rgbexposure(&exposurecfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        s_nvpfm_get_sensor_exposure_ret newcfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_rgbexposure(&newcfg));
        ASSERT_EQ(0, newcfg.ret);
        nvptl_debug_printf("[after set] rgb exposure time:%d, mode:%d\n",
         newcfg.exposure.exposure_time, newcfg.exposure.exposure_mode);

        //restore the setting
        exposurecfg.channel = CHANNEL2;
        memcpy(&exposurecfg.config, &exposure.exposure, sizeof(exposurecfg.config));
        EXPECT_EQ(NVPTL_OK, camera_->set_rgbexposure(&exposurecfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        EXPECT_EQ(NVPTL_OK, camera_->get_rgbexposure(&newcfg));
        ASSERT_EQ(0, newcfg.ret);
        nvptl_debug_printf("[restore set] rgb exposure time:%d, mode:%d\n",
         newcfg.exposure.exposure_time, newcfg.exposure.exposure_mode);
    }

     /**
     * @brief  get run config
     */
    TEST_F(FalconTestCamera, GetRunCfg) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_get_app_run_config_ret cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_runcfg(&cfg));
        ASSERT_EQ(0, cfg.ret);
        nvptl_debug_printf("run config, run mode:%d, calibrated:%d\n", 
        cfg.config.app_run_mode, cfg.config.channel_calibrated);
    }

    /**
     * @brief  get confidence
     */
    TEST_F(FalconTestCamera, GetConfidence) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_confidence confidence;
        EXPECT_EQ(NVPTL_OK, camera_->get_confidence(&confidence));
        nvptl_debug_printf("confidence enable:%d, sigma:%f\n", confidence.enable, confidence.sigma);
    }

     /**
     * @brief  set confidence
     */
    TEST_F(FalconTestCamera, SetConfidence) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_confidence confidence;
        EXPECT_EQ(NVPTL_OK, camera_->get_confidence(&confidence));
        nvptl_debug_printf("[before set] confidence enable:%d, sigma:%f\n", confidence.enable, confidence.sigma);

        s_nvpfm_set_confidence cfg;
        cfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&cfg.info, &confidence, sizeof(confidence));
        cfg.info.enable = !confidence.enable;
        EXPECT_EQ(NVPTL_OK, camera_->set_confidence(&cfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        EXPECT_EQ(NVPTL_OK, camera_->get_confidence(&confidence));
        nvptl_debug_printf("[after set] confidence enable:%d, sigma:%f\n", confidence.enable, confidence.sigma);

        //restore setting
        cfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&cfg.info, &confidence, sizeof(confidence));
        cfg.info.enable = !confidence.enable;
        EXPECT_EQ(NVPTL_OK, camera_->set_confidence(&cfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        EXPECT_EQ(NVPTL_OK, camera_->get_confidence(&confidence));
        nvptl_debug_printf("[restore set] confidence enable:%d, sigma:%f\n", confidence.enable, confidence.sigma);

    }

    /**
     * @brief  get high precision
     */
    TEST_F(FalconTestCamera, GetHighprecision) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_high_precision cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_high_precision(cfg));
        nvptl_debug_printf("get high precision %d\n", cfg.enable);
    }

    /**
     * @brief  get repeat texture filter
     */
    TEST_F(FalconTestCamera, GetRepeatfilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_repeated_texture_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_repeat_texture_filter(&cfg));
        nvptl_debug_printf("get repeat texture filter enable: %d, threshold:%f, winSize:%d\n", cfg.enable, cfg.threshold, cfg.winSize);
    }

    /**
     * @brief  set repeat texture filter
     */
    TEST_F(FalconTestCamera, SetRepeatfilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

       
        s_nvpfm_repeated_texture_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_repeat_texture_filter(&cfg));
        nvptl_debug_printf("[before set] repeat texture filter enable: %d, threshold:%f, winSize:%d\n", cfg.enable, cfg.threshold, cfg.winSize);

         std::this_thread::sleep_for(std::chrono::milliseconds(100));
        s_nvpfm_set_repeated_texture_filter setcfg;
        setcfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&setcfg.info, &cfg, sizeof(cfg));
        setcfg.info.enable = !cfg.enable;
        setcfg.info.threshold = 0.5;
        EXPECT_EQ(NVPTL_OK, camera_->set_repeat_texture_filter(&setcfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        s_nvpfm_repeated_texture_filter newcfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_repeat_texture_filter(&newcfg));
        nvptl_debug_printf("[after set] repeat texture filter enable: %d, threshold:%f, winSize:%d\n",
         newcfg.enable, newcfg.threshold, newcfg.winSize);

        //reset setting
        setcfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&setcfg.info, &cfg, sizeof(cfg));
        EXPECT_EQ(NVPTL_OK, camera_->set_repeat_texture_filter(&setcfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        EXPECT_EQ(NVPTL_OK, camera_->get_repeat_texture_filter(&newcfg));
        nvptl_debug_printf("[restore set] repeat texture filter enable: %d, threshold:%f, winSize:%d\n",
         newcfg.enable, newcfg.threshold, newcfg.winSize);

    }

    /**
     * @brief  get depth config
     */
    TEST_F(FalconTestCamera, GetDepthConfig) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_app_dsp_process_static_config cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_depth_config(&cfg));
        nvptl_debug_printf("get depth config mode: %d, denoise:%d\n", cfg.dsp_depth_mode, cfg.dsp_depth_denoise);
    }

    /**
     * @brief  get smear filter
     */
    TEST_F(FalconTestCamera, GetSmearFilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_smear_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_smear_filter(&cfg));
        nvptl_debug_printf("get smear filter enable: %d, dispThreshold:%f, winsize:%d\n", 
        cfg.enable, cfg.dispThreshold, cfg.winSize);
    }

     /**
     * @brief  set smear filter
     */
    TEST_F(FalconTestCamera, SetSmearFilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_smear_filter filter;
        EXPECT_EQ(NVPTL_OK, camera_->get_smear_filter(&filter));
        nvptl_debug_printf("[before set] smear filter enable: %d, dispThreshold:%f, winsize:%d\n",
         filter.enable, filter.dispThreshold, filter.winSize);

        s_nvpfm_set_smear_filter cfg;
        cfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&cfg.info, &filter, sizeof(filter));
        cfg.info.enable = !filter.enable;
        cfg.info.dispThreshold = 0.5;
        EXPECT_EQ(NVPTL_OK, camera_->set_smear_filter(&cfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        s_nvpfm_smear_filter newcfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_smear_filter(&newcfg));
        nvptl_debug_printf("[after set] smear filter enable: %d, dispThreshold:%f, winsize:%d\n",
         newcfg.enable, newcfg.dispThreshold, newcfg.winSize);

        //restore setting 
        cfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&cfg.info, &filter, sizeof(filter));
        EXPECT_EQ(NVPTL_OK, camera_->set_smear_filter(&cfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        EXPECT_EQ(NVPTL_OK, camera_->get_smear_filter(&newcfg));
        nvptl_debug_printf("[restore set] smear filter enable: %d, dispThreshold:%f, winsize:%d\n",
         newcfg.enable, newcfg.dispThreshold, newcfg.winSize);
    }

    /**
     * @brief  get spacial config to be asynchornized
     */
    TEST_F(FalconTestCamera, GetSpatialAsync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        camera_->get_spatial_async(spatialfiltercallback, nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    /**
     * @brief  get speckle config
     */
    TEST_F(FalconTestCamera, GetSpeckle) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_speckle_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_speckle(&cfg));
        nvptl_debug_printf("get speckle enable: %d\n", cfg.enable);
    }

    /**
     * @brief  set speckle config
     */
    TEST_F(FalconTestCamera, SetSpeckle) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

       s_nvpfm_speckle_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_speckle(&cfg));
        nvptl_debug_printf("[before set] speckle enable: %d\n", cfg.enable);
    
        s_nvpfm_set_speckle_filter filtercfg;
        filtercfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&filtercfg.info, &cfg, sizeof(cfg));
        filtercfg.info.enable = !cfg.enable;
        EXPECT_EQ(NVPTL_OK, camera_->set_speckle(&filtercfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        EXPECT_EQ(NVPTL_OK, camera_->get_speckle(&cfg));
        nvptl_debug_printf("[after set] speckle enable: %d\n", cfg.enable);

        //restore setting
        filtercfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&filtercfg.info, &cfg, sizeof(cfg));
        filtercfg.info.enable = !cfg.enable;
        EXPECT_EQ(NVPTL_OK, camera_->set_speckle(&filtercfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        EXPECT_EQ(NVPTL_OK, camera_->get_speckle(&cfg));
        nvptl_debug_printf("[restore set] speckle enable: %d\n", cfg.enable);
    }

     /**
     * @brief  get hightlight filter to be asynchornized
     */
    TEST_F(FalconTestCamera, GetHightlightAsync) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        camera_->get_highlight_async(highlightfiltercallback, nullptr);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

     /**
     * @brief  get projectors
     */
    TEST_F(FalconTestCamera, GetProjectors) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
        s_nvpfm_get_projector_ret projectors[MAX_PROJECTOR_NUM];
        memset(projectors, 0, sizeof(projectors));
        for (int i = 0; i < MAX_PROJECTOR_NUM; i ++) {
            s_nvpfm_get_projector cfg;
            cfg.channel = (E_NVPFM_PROJECTOR_CHANNEL)i;
            EXPECT_EQ(NVPTL_OK, camera_->get_projector(&cfg, &projectors[i]));
            nvptl_debug_printf("get projector[%d], open:%d\n", i, projectors[i].config.open);
        }
    }

    /**
     * @brief  set projectors
     */
    TEST_F(FalconTestCamera, SetProjectors) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_get_projector_ret projectors[MAX_PROJECTOR_NUM];
        memset(projectors, 0, sizeof(projectors));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        for (int i = 0; i < MAX_PROJECTOR_NUM; i ++) {
            s_nvpfm_get_projector cfg;
            cfg.channel = (E_NVPFM_PROJECTOR_CHANNEL)i;
            EXPECT_EQ(NVPTL_OK, camera_->get_projector(&cfg, &projectors[i]));
            nvptl_debug_printf("[befor set] projector[%d], open:%d, mA:%f\n", 
            i, projectors[i].config.open, projectors[i].config.projector_mA);

            s_nvpfm_set_projector procfg;
            procfg.channel = (E_NVPFM_PROJECTOR_CHANNEL)i;
            memcpy(&procfg.config, &projectors[i].config, sizeof(procfg));
            procfg.config.open = !projectors[i].config.open;
            procfg.config.projector_mA = projectors[i].config.projector_mA == 0 ?  1 : 0;
            nvpfm_info_printf("before set projector\n");
            // EXPECT_EQ(NVPTL_OK, camera_->set_projector(&procfg));

            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            cfg.channel = (E_NVPFM_PROJECTOR_CHANNEL)i;
            EXPECT_EQ(NVPTL_OK, camera_->get_projector(&cfg, &projectors[i]));
            nvpfm_info_printf("[after set] projector[%d], open:%d, mA:%f\n", 
            i, projectors[i].config.open, projectors[i].config.projector_mA);
        }
    }

    /**
     * @brief get bad filter
     */
    TEST_F(FalconTestCamera, GetBadFilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_bad_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_bad_filter(&cfg));
        nvptl_debug_printf("get bad filter enable:%d, kernel:%d\n", cfg.enable, cfg.kernel);
    }

    /**
     * @brief set bad filter
     */
    TEST_F(FalconTestCamera, SetBadFilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_bad_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_bad_filter(&cfg));
        nvptl_debug_printf("[before set] bad filter enable:%d, kernel:%d\n", cfg.enable, cfg.kernel);

        s_nvpfm_set_bad_filter setcfg;
        setcfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&setcfg.info, &cfg, sizeof(cfg));
        setcfg.info.enable = !cfg.enable;
        // EXPECT_EQ(NVPTL_OK, camera_->set_bad_filter(&setcfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        s_nvpfm_bad_filter newcfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_bad_filter(&newcfg));
        nvptl_debug_printf("[after set] bad filter enable:%d, kernel:%d\n", newcfg.enable, newcfg.kernel);

        //restore setting
        setcfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&setcfg.info, &cfg, sizeof(cfg));
        // EXPECT_EQ(NVPTL_OK, camera_->set_bad_filter(&setcfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        EXPECT_EQ(NVPTL_OK, camera_->get_bad_filter(&newcfg));
        nvptl_debug_printf("[restore set] bad filter enable:%d, kernel:%d\n", newcfg.enable, newcfg.kernel);
    }

    /**
     * @brief get light filter
     */
    TEST_F(FalconTestCamera, GetLightFilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

        s_nvpfm_light_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_light_filter(&cfg));
        nvptl_debug_printf("get light filter enable:%d, sigma:%f, threshold:%d\n", 
        cfg.enable, cfg.sigma, cfg.pixel_threshold);
    }

    /**
     * @brief set light filter
     */
    TEST_F(FalconTestCamera, SetLightFilter) {
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;

         std::this_thread::sleep_for(std::chrono::milliseconds(200));
        s_nvpfm_light_filter cfg;
        EXPECT_EQ(NVPTL_OK, camera_->get_light_filter(&cfg));
        nvptl_debug_printf("[before set] light filter enable:%d, sigma:%f, threshold:%d\n", 
        cfg.enable, cfg.sigma, cfg.pixel_threshold);

        s_nvpfm_set_light_filter setcfg;
        setcfg.channel = NVPFM_DEPTH_CHANNEL0;
        memcpy(&setcfg.info, &cfg, sizeof(cfg));
        setcfg.info.enable = !cfg.enable;
        setcfg.info.sigma = cfg.sigma == 0.5 ? 0 : 0.5;
        // EXPECT_EQ(NVPTL_OK, camera_->set_light_filter(&setcfg));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        EXPECT_EQ(NVPTL_OK, camera_->get_light_filter(&cfg));
        nvptl_debug_printf("[after set] light filter enable:%d, sigma:%f, threshold:%d\n", 
        cfg.enable, cfg.sigma, cfg.pixel_threshold);
    }
    #endif

    /////////////////////////////////////////////////////////////////////////////////
    #ifdef FEYNMAN_TEST_REBOOT
     TEST_F(FalconTestReboot, SetProjectors) {
        NVPTL_DEVICE_INFO *deviceinfo = NULL;
        int total = 0;
        // EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
        EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
        ASSERT_GE(total, 0);
        camera_ = new nvpfm(deviceinfo, eventcallback, NULL);
        ASSERT_NE(nullptr, camera_) << "open camera failed" << endl;
        EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
        status_ = EM_CAMERA_PLUGGED;
        nvptl_freedevices(deviceinfo);
        deviceinfo = NULL;

        s_nvpfm_get_projector_ret projectors[MAX_PROJECTOR_NUM];
        memset(projectors, 0, sizeof(projectors));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        for (int i = 0; i < MAX_PROJECTOR_NUM; i ++) {
            s_nvpfm_get_projector cfg;
            cfg.channel = (E_NVPFM_PROJECTOR_CHANNEL)i;
            EXPECT_EQ(NVPTL_OK, camera_->get_projector(&cfg, &projectors[i]));
            nvptl_debug_printf("[befor set] projector[%d], open:%d, mA:%f\n", 
            i, projectors[i].config.open, projectors[i].config.projector_mA);

            s_nvpfm_set_projector procfg;
            procfg.channel = (E_NVPFM_PROJECTOR_CHANNEL)i;
            memcpy(&procfg.config, &projectors[i].config, sizeof(procfg));
            procfg.config.open = !projectors[i].config.open;
            // procfg.config.projector_mA = projectors[i].config.projector_mA == 0 ?  1 : 0;
            procfg.config.projector_duty_auto = !projectors[i].config.projector_duty_auto;
            nvpfm_info_printf("before set projector\n");
            SCOPED_TRACE("A");
            EXPECT_EQ(NVPTL_OK, camera_->set_projector(&procfg));
            // camera_->send_reboot();
            // camera_ = nullptr;
            std::this_thread::sleep_for(std::chrono::seconds(10));
            // int total = 0;
            // NVPTL_DEVICE_INFO *deviceinfo = NULL;
            // nvpfm_info_printf("start reboot, total:%d!\n", total);
            // do {
            //     // EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
            //     nvptl_enum_sync(&total, &deviceinfo);
            //     if(total > 0) {
            //         nvpfm_info_printf("after set projector plugged!\n");
            //         std::this_thread::sleep_for(std::chrono::seconds(1));
            //         break;
            //     } else {
            //         nvpfm_info_printf("after set projector rebooting\n");
            //         COMMONUSLEEP(500);
            //     }
            // }while (status_ != EM_CAMERA_PLUGGED);

            // do {
            //     // EXPECT_EQ(0, nvpfm_init(LOG_FILE, LOG_FILE_SIZE));
            //     // nvptl_enum_sync(&total, &deviceinfo);
            //     // if(total > 0 ) {
            //     //     nvpfm_info_printf("after set projector plugged!\n");
            //     //     nvptl_freedevices(deviceinfo);
                    
            //     //     if(status_ == EM_CAMERA_PLUGGED) {
            //     //         nvpfm_info_printf("after set projector break!\n");
            //     //          break;
            //     //     }
            //     //     COMMONUSLEEP(500);
                   
            //     // } else {
            //     //     nvpfm_info_printf("after set projector rebooting\n");
            //     //     COMMONUSLEEP(500);
            //     // }
            //     nvpfm_info_printf("after set projector rebooting status_:%d\n", status_);
            //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
            // }while (status_ != EM_CAMERA_PLUGGED);
            
            

            
            // nvpfm_info_printf("after set projector\n");
            // // int total = 0; 
            // // NVPTL_DEVICE_INFO *deviceinfo = NULL;
            // // EXPECT_EQ(NVPTL_OK, nvptl_enum_sync(&total, &deviceinfo));
            // ASSERT_GT(total, 0);
            // camera_ = make_shared<nvpfm>(deviceinfo, eventcallback, nullptr);
            // ASSERT_NE(nullptr, camera_) << "open camera failed after reboot" << endl;
            // EXPECT_TRUE(camera_->isplugged()) << "camera is not plugged" << endl;
            // // status_ = EM_CAMERA_PLUGGED;
            // // nvptl_freedevices(deviceinfo);
            // std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // cfg.channel = (E_NVPFM_PROJECTOR_CHANNEL)i;
            // EXPECT_EQ(NVPTL_OK, camera_->get_projector(&cfg, &projectors[i]));
            // nvpfm_info_printf("[after set] projector[%d], open:%d, mA:%f\n", 
            // i, projectors[i].config.open, projectors[i].config.projector_mA);
        }
        
        // nvpfm_deinit();
    }
    #endif

}

int main(int argc,char* argv[]){
	testing::InitGoogleTest(&argc,argv);
    return RUN_ALL_TESTS();
}
