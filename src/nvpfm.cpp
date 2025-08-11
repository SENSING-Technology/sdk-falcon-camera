
#include <stdio.h>
#include "commondef.h"
#include "nvpfm.hpp"
#include <string.h>
#include <iostream>
#ifdef _WINDOWS
#include "json/json.h"
#else
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "jsoncpp/json/json.h"
#endif
#ifndef _WINDOWS
#include <unistd.h>
#endif
#include <math.h>
#include <sstream>
#include <iostream>
#include <string>
#include <regex>
#if defined(_WINDOWS)
#include "opencv2/opencv.hpp"
#elif defined(__LINUX__)
#include <opencv2/opencv.hpp>
#endif
#ifndef _WINDOWS
#include <libgen.h>
#endif

#ifdef USEYAMLCPP
#include <yaml-cpp/yaml.h>
#endif

#define CHECKPLUGRETURNNULL()      \
  do {                             \
    bool checkpass = false;        \
    for (int i = 0; i < 10; i++) { \
      if (!isplugged()) {          \
        COMMONUSLEEP(1000 * 1000); \
        refresh();                 \
      } else {                     \
        checkpass = true;          \
        break;                     \
      }                            \
    }                              \
    if (!checkpass) {              \
      return NULL;                 \
    }                              \
  } while (0)

#define CHECKPLUG()                \
  do {                             \
    bool checkpass = false;        \
    for (int i = 0; i < 10; i++) { \
      if (!isplugged()) {          \
        COMMONUSLEEP(1000 * 1000); \
        refresh();                 \
      } else {                     \
        checkpass = true;          \
        break;                     \
      }                            \
    }                              \
    if (!checkpass) {              \
      return NVPTL_FAILED;         \
    }                              \
  } while (0)

typedef struct
{
  NVPFM_IMAGE_SIZE res;
  const char *resstr;
  int width;
  int height;
} RESINFO;

static RESINFO resmapper[] = {
    {IMAGE_1280_800, "1280x800", 1280, 800},
    {IMAGE_1280_720, "1280x720", 1280, 720},
    {IMAGE_640_480, "640x480", 640, 480},
    {IMAGE_640_400, "640x400", 640, 400},
    {IMAGE_320_200, "320x200", 320, 200},
    {IMAGE_640_360, "640x360", 640, 360},
    {IMAGE_320_240, "320x240", 320, 240},
    {IMAGE_960_600, "960x600", 960, 600},
    {IMAGE_480_300, "480x300", 480, 300},
    {IMAGE_1600_1200, "1600x1200", 1600, 1200},
    {IMAGE_1280_1080, "1280x1080", 1280, 1080},
    {IMAGE_1280_960, "1280x960", 1280, 960},
    {IMAGE_800_600, "800x600", 800, 600},
    {IMAGE_848_480, "848x480", 848, 480},
    {IMAGE_768_480, "768x480", 768, 480},
    {IMAGE_1280_480, "1280x480", 1280, 480},
    {IMAGE_1920_1080, "1920x1080", 1920, 1080},
    {IMAGE_960_1280, "960x1280", 960, 1280},
    {IMAGE_480_640, "480x640", 480, 640},
    {IMAGE_960_720, "960x720", 960, 720},
    {IMAGE_640_800, "640x800", 640, 800},
    {IMAGE_1280_400, "1280x400", 1280, 400},
    {IMAGE_640_200, "640x200", 640, 200},
    {IMAGE_720_1280, "720x1280", 720, 1280},
    {IMAGE_1024_768, "1024x768", 1024, 768},
    {IMAGE_512_384, "512x384", 512, 384},
    {IMAGE_1024_480, "1024x480", 1024, 480},
    {IMAGE_512_240, "512x240", 512, 240},
    {IMAGE_896_672, "896x672", 896, 672},
    {IMAGE_768_576, "768x576", 768, 576},
    {IMAGE_864_1152, "864x1152", 864, 1152},
    {IMAGE_1152_720, "1152x720", 1152, 720},
    {IMAGE_1152_400, "1152x400", 1152, 400},
    {IMAGE_1152_864, "1152x864", 1152, 864},
    {IMAGE_1152_480, "1152x480", 1152, 480},
    {IMAGE_UNKNOWN, "?x?", -1, -1},
};

depth_postproc_desc postproc_descs[] = {
    {CONFIDENCE_E, "confidence", "confidence", "Confidence", false},
    {LIGHTFILTER_E, "light_filter", "highlight filter", "Highlight Filter", false},
    {GOODFEATURE_E, "good_feature", "good feature", "Good Feature", false},
    {BADFILTER_E, "bad_filter", "bad filter", "Bad Filter", false},
    {LKPARAM_E, "lk_param", "lk param", "LK Param", false},
    {HIGHPRECISION_E, "high_precision", "high precision", "High Precision", true},
    {REPEATEDFILTER_E, "repeated_texture_filter", "repeated texture filter", "Repeated Texture Filter", false},
    {SPATIALFILTER_E, "spatial_filter", "spatial filter", "Spatial Filter", false},
    {SMEARFILTER_E, "smear_filter", "smear filter", "Smear Filter", false},
    {SPECKLEFILTER_E, "speckle_filter", "speckle filter", "Speckle Filter", true},
    {TEMPORALFILTER_E, "temporal_filter", "temporal filter", "Temporal Filter", true},
    {AGGPROCESS_E, "agg_process", "agg process", "Agg Process", true},
    {HIGHLIGHTFILTER_E, "depth_high_light", "high light", "High Light", true},
    {LIGHTSTRIPFILTER_E, "depth_light_strip", "light strip", "Light Strip", true},
    {TEXTLIGHTFILTER_E, "textlight_filter", "text light filter", "Textlight Filter", false},
    {DEPTHRESIZE_E, "depth_resize", "depth resize", "Depth Resize", false},
    {GOLDENDIFF_E, "golden_diff", "golden diff", "Golden Diff", true},
    {RIGHTUPDOWN_E, "depth_right_up_down_info", "depth right up down info", "RightUpDown Info", false},
    {STATICCONFIG_E, "depth_static_config", "depth static config", "Static Config", false},
    {CONVERTTODEPTH_E, "convert_to_depth", "convert to depth", ""},
    {HIGH_LUMA_REMOVAL_E, "highluma_removal", "highluma removal", "Highluma Removal", false},
    {HIGHPRESION_E, "highpresion", "high presion", "High Presion", true},
    {OUTLIERS_REMOVAL_E, "outliers_removal", "outliers removal", "Outliers Removal", false},
    {DEPTH_MAP_EXPANSION_E, "depth_map_expansion", "depth map expansion", "Depth Map Expansion", false},
    {LOWTEXTURE_REMOVAL_E, "lowtexture_removal", "lowtexture removal", "Lowtexture Removal", false},
    {AMBIGUITY_REMOVAL_E, "ambiguity_removal", "ambiguity removal", "Ambiguity Removal", false},
    {STEREOCALI_CORRECTION_E, "stereo_calibration_correction", "stereo calibration correction", "Stereo Calibration Correction", false},
    {DEPTH_RANGE_JUSTMENT_1_E, "range_adjustment1", "range adjustment1", "Range Adjustment1", false},
    {DEPTH_RANGE_JUSTMENT_2_E, "range_adjustment2", "range adjustment2", "Range Adjustment2", false},
    {REPEATED_TEXTURE_E, "repeated_texture", "repeated texture", "Repeated Texture", true},
    {FUSION2_E, "fusion2", "fusion2", "Fusion2", true},
    {FUSION_1_E, "fusion_1", "fusion_1", "Fusion_1", true},
    {DP_MODE_1_1_E, "dp_mode_1_1", "dp mode1_1", "DP mode1_1", false},
    {DP_MODE_1_2_E, "dp_mode_1_2", "dp mode1_2", "DP mode1_2", false},
    {DP_MODE_2_1_E, "dp_mode_2_1", "dp mode2_1", "DP mode2_1", false},
    {DP_MODE_2_2_E, "dp_mode_2_2", "dp mode2_2", "DP mode2_2", false},
    {RANGE_ADJUSTMENT3, "range_adjustment3", "range adjustment3", "Range Adjustment3", false},
    {SMALL_DISP_MASK, "small_disp_mask", "small disp mask", "Small Disp Mask", true}};

depth_postproc_desc *nvpfm::get_depthpostdesc(POSTPROCESS_E type) {
  return &postproc_descs[type];
}

bool nvpfm::is_encrypt_firmware_file(const char *filepath) {
  char buffer[8];
  FILE *fp = fopen(filepath, "rb");
  fseek(fp, 0x448, 0);
  fread(&buffer, sizeof(int), 1, fp);
  int tmpoffset = *(int *)buffer;

  fseek(fp, 0x400 + tmpoffset, 0);
  fread(&buffer, sizeof(unsigned char), 8, fp);
  fclose(fp);
  if (0 == strncmp(buffer, "NextVPU", 7)) {
    if (buffer[7] == 0) {
      return false;
    } else {
      return true;
    }
  }
  return false;
}

NVPFM_IMAGE_SIZE nvpfm::parsewxh(const char *wxh) {
  int width, height;
  sscanf(wxh, "%dx%d", &width, &height);
  for (int i = 0; i < IMAGE_UNKNOWN; i++) {
    if (resmapper[i].width == width && resmapper[i].height == height) {
      return (NVPFM_IMAGE_SIZE)i;
    }
  }
  return IMAGE_UNKNOWN;
}

NVPFM_IMAGE_SIZE nvpfm::findsensorresbyisp(s_nvpfm_dev_info info, NVPFM_IMAGE_SIZE res, E_NVPFM_SENSOR_CHANNEL chan) {
  printf("will search channel:%d,res:%s,in supported\n", chan, resmapper[res].resstr);
  for (int i = 0; i < MAXSUPPORTEDRES; i++) {
    if (chan == CHANNEL0 || chan == CHANNEL1) {
      if (info.irsupported[i].valid) {
        printf("support:%s\n", resmapper[info.irsupported[i].ispres].resstr);
        if (info.irsupported[i].ispres == res) {
          return info.irsupported[i].sensorres;
        }
      }
    } else {
      if (info.colorsupported[i].valid) {
        if (info.colorsupported[i].ispres == res) {
          return info.colorsupported[i].sensorres;
        }
      }
    }
  }
  return IMAGE_UNKNOWN;
}

void nvpfm::framesize2int(NVPFM_IMAGE_SIZE res, int *pwidth, int *pheight) {
  *pwidth = resmapper[res].width;
  *pheight = resmapper[res].height;
}
const char *nvpfm::framesize2str(NVPFM_IMAGE_SIZE res) {
  return resmapper[res].resstr;
}
s_nvpfm_imu_external_reference *nvpfm::get_imu_externalref() {
#ifdef USEYAMLCPP
  if (!nvpfm_hasconnect(m_handle)) {
    return NULL;
  }

  BOOL gotimuyaml = FALSE;
  BOOL gotcamchain = FALSE;
  // 1.get imu.yaml
  s_get_imu_param senddata1;
  senddata1.channel = NVPFM_IMU_CHANNEL0;
  senddata1.type = 1;
  s_get_imu_param_result *recvdata1 = (s_get_imu_param_result *)calloc(1, sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE);
  int recvlen1 = sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE;
  if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_IMU_PARAM, &senddata1, sizeof(s_get_imu_param), recvdata1, &recvlen1, 2000)) {
    if (recvdata1->result == 0)
      gotimuyaml = TRUE;
    else
      printf("got response of imu param,but resuilt is not 0!\n");
  } else {
    printf("fail to get imu param of type 1 in nvpfm.cpp!\n");
  }

  s_get_imu_param senddata2;
  senddata2.channel = NVPFM_IMU_CHANNEL0;
  senddata2.type = 2;
  s_get_imu_param_result *recvdata2 = (s_get_imu_param_result *)calloc(1, sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE);
  int recvlen2 = sizeof(s_get_imu_param_result) + MAXDOWNLOADFILESIZE;
  if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_IMU_PARAM, &senddata2, sizeof(s_get_imu_param), recvdata2, &recvlen2, 2000)) {
    // 2.get camchain-imucam.yaml
    if (recvdata2->result == 0)
      gotcamchain = TRUE;
    else
      printf("got response of type 2 of imu param,but resuilt is not 0!\n");
  } else {
    printf("fail to get imu param of type 2 in nvpfm.cpp!\n");
  }

  if (gotimuyaml && gotcamchain) {
    // float camimus[16];
    // parsecamimuconfig(recvdata2->data, strlen(recvdata2->data), &camimus[0]);

    YAML::Node camchainconfig, imuconfig;
    try {
      // printf("camchain:%s\n",recvdata2->data);
      camchainconfig = YAML::Load(recvdata2->data);

      // float imus[4];
      // parseimuconfig(recvdata1->data, strlen(recvdata1->data), &imus[0]);
      imuconfig = YAML::Load(recvdata1->data);

      for (int row = 0; row < 4; row++)
        for (int col = 0; col < 4; col++) {
          m_imuexternalref.t_cam_imu[row * 4 + col] = camchainconfig["cam0"]["T_cam_imu"].as<std::vector<std::vector<float>>>().at(row).at(col);
        }
      // printf("will parse acc and gyro!\n");
      m_imuexternalref.acc_noise_density = imuconfig["imu0"]["accelerometer_noise_density"].as<float>();
      m_imuexternalref.acc_random_walk = imuconfig["imu0"]["accelerometer_random_walk"].as<float>();
      m_imuexternalref.gyro_noise_density = imuconfig["imu0"]["gyroscope_noise_density"].as<float>();
      m_imuexternalref.gyro_random_walk = imuconfig["imu0"]["gyroscope_random_walk"].as<float>();
      m_imuexternalref.timeshift_cam_imu = (int)(camchainconfig["cam0"]["timeshift_cam_imu"].as<float>() * 1000.0);
      // imuconfig["imu0"]["gyroscope_random_walk"].as<float>();

      // printf("got imuexternalref:\n");
      free(recvdata1);
      free(recvdata2);
      return &m_imuexternalref;
    } catch (YAML::BadFile &e) {
      std::cout << "read error!" << std::endl;

      free(recvdata1);
      free(recvdata2);
      nvpfm_warn_printf("read imuexternalref failed!!!\n");
      return NULL;
    } catch (YAML::ParserException &e) {
      std::cout << "parse error:" << e.what();

      free(recvdata1);
      free(recvdata2);
      nvpfm_warn_printf("parse imuexternalref failed!!!\n");
      return NULL;
    }
  }
  free(recvdata1);
  free(recvdata2);
  // 1.download two yaml file
  return NULL;
#else
  return nvpfm_getimuexternalref(m_handle, &m_imuexternalref);
#endif
}
s_nvpfm_imu_internal_reference *nvpfm::get_imu_internalref() {
  return nvpfm_getimuinternalref(m_handle, &m_imuinternalref);
}
/*typedef struct
    {
        int enable;
        int win_width;
        int win_height;
        int point_limt;
        int maxCorners;
        float qualityLevel;
        float minDistance;
    } s_nvpfm_lk;*/
s_nvpfm_lk *nvpfm::get_lk_param() {
  for (int i = 0; i < 5; i++) {
    int responselen = sizeof(s_nvpfm_lk);
    if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_LK_PARAM, NULL, 0, &m_lkparam, &responselen, 2000)) // timeout 2000milliseconds
    {
      nvpfm_debug_printf("got lk param:\n");

      nvpfm_debug_printf("lk enable:%d\n", m_lkparam.enable);
      nvpfm_debug_printf("win_width:%d\n", m_lkparam.win_width);
      nvpfm_debug_printf("win_height:%d\n", m_lkparam.win_height);
      return &m_lkparam;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NULL;
}
s_nvpfm_good_feature *nvpfm::get_goodfeature_param() {
  for (int i = 0; i < 5; i++) {
    int responselen = sizeof(s_nvpfm_good_feature);
    if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_GOODFEATURE_PARAM, NULL, 0, &m_goodfeatureparam, &responselen, 2000)) // timeout 2000milliseconds
    {
      nvpfm_debug_printf("got lk param:\n");

      nvpfm_debug_printf("goodfeature enable:%d\n", m_goodfeatureparam.enable);
      nvpfm_debug_printf("goodfeature maxCorners:%d\n", m_goodfeatureparam.maxCorners);
      nvpfm_debug_printf("qualityLevel:%f\n", m_goodfeatureparam.qualityLevel);
      nvpfm_debug_printf("minDistance:%f\n", m_goodfeatureparam.minDistance);
      return &m_goodfeatureparam;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NULL;
}

static void c_split(char *src, const char *separator, int maxlen, char **dest, int *num) {
  char *pNext;
  int count = 0;
  if (src == NULL || strlen(src) == 0)
    return;
  if (separator == NULL || strlen(separator) == 0)
    return;
  pNext = strtok(src, separator);
  while (pNext != NULL && count < maxlen) {
    *dest++ = pNext;
    ++count;
    pNext = strtok(NULL, separator);
  }
  *num = count;
}
static char *nvpfm_getportid(const char *devicename) {
  //	int ret = -1;
  char *msg = strdup(devicename);
  int num = 0;
  int bus;
  char *split_buf[512] = {0};
  //	int i = 0;
  c_split((char *)msg, "-", 512, split_buf, &num);
  if (num < 3) {
    nvpfm_error_printf("fail to split devicename!\n");
    free(msg);
    return strdup("");
  }
  bus = atoi(split_buf[1]);

  char tmpresult[256];
  sprintf(tmpresult, "%d-%s", bus, split_buf[2]);
  free(msg);
  //////////////////////////////////////////////////////////////////////
  return strdup(tmpresult);
}

void nvpfm::send_echo() {
  nvpfm_sendecho(m_handle);
}
void nvpfm::send_reboot() {
  nvpfm_sendreboot(m_handle);
}
// cfy add
NVPTL_RESULT nvpfm::request_iframe(uint16_t sensor_channel) {
  s_nvpfm_request_h265_i_frame req;
  req.channel = (E_NVPFM_SENSOR_CHANNEL)(sensor_channel);
  // printf("request channel :%d\n", sensor_channel);
  s_nvpfm_cmd_set_ret ret;
  int ret_len = sizeof(ret);
  NVPTL_RESULT res =
      nvpfm_set(m_handle, NVPFM_REQ_IFRAME, &req, sizeof(req), &ret, &ret_len, 500);
  if (ret.ret != 0) {
    // printf("Request iframe failed!ret:%d\n", ret.ret);
  }
  return res;
}

// save get get_all delete load_user
NVPTL_RESULT nvpfm::save_user_config(const std::string &config_name) {
  s_nvpfm_save_user_custom_config req;
  s_nvpfm_save_user_custom_config_ret ret;

  int ret_len = sizeof(ret);
  memset(&req, 0x0, sizeof(req));
  memset(&ret, 0x0, sizeof(ret));
  strcpy(req.file_name, config_name.c_str());
  NVPTL_RESULT res = nvpfm_set(m_handle, NVPFM_SAVE_USER_CFG, &req, sizeof(req), &ret, &ret_len, 2000);
  if (ret.ret != 0) {
    // some error;
    printf("Save file error!\n");
  }
  if (res != NVPTL_OK) {
    printf("Save user config error code %d. ret.ret %d\n", res, ret.ret);
  }
  return res;
}

NVPTL_RESULT nvpfm::get_user_config(const std::string &config_name, std::vector<uint8_t> *out_data) {
  s_nvpfm_get_user_custom_config req;
  // s_nvpfm_get_user_custom_config_ret ret;
  // # MAXDOWNLOADFILESIZE
  std::vector<uint8_t> buffer(sizeof(s_nvpfm_get_user_custom_config_ret) + MAXDOWNLOADFILESIZE);
  memset(&req, 0x0, sizeof(req));
  strcpy(req.file_name, config_name.c_str());
  // TBD: 可变长度  if (ret.ret != 0)
  {
    // some error;
  }
  int len = buffer.size();
  NVPTL_RESULT res = nvpfm_set(m_handle, NVPFM_GET_USER_CFG, &req, sizeof(req), buffer.data(), &len, 500);
  if (res == NVPTL_OK && out_data) {
    out_data->swap(buffer);
  }
  return res;
}

NVPTL_RESULT nvpfm::get_all_user_config_names(std::vector<std::string> *names) {
  // s_nvpfm_get_user_all_custom_config_name req;
  s_nvpfm_get_user_all_custom_config_name_ret ret;
  int len = sizeof(ret);
  NVPTL_RESULT res = nvpfm_set(m_handle, NVPFM_GET_ALL_USER_CFG, NULL, 0, &ret, &len, 10000);
  if (res == NVPTL_OK && names) {
    names->resize(ret.size);
    for (int idx = 0; idx < ret.size; ++idx) {
      (*names)[idx] = std::string(&(ret.file_name[idx][0]));
    }
  } else if (res != NVPTL_OK) {
    printf("get config names failed, res:%d, res size :%d\n", res, ret.size);
  }
  return res;
}

NVPTL_RESULT nvpfm::delete_user_config(const std::string &config_name) {
  s_nvpfm_delete_user_custom_config req;
  s_nvpfm_delete_user_custom_config_ret ret;
  int len = sizeof(ret);
  strcpy(req.file_name, config_name.c_str());
  NVPTL_RESULT res = nvpfm_set(m_handle, NVPFM_DELETE_USER_CFG, &req, sizeof(req), &ret, &len, 2500);
  if (ret.ret != 0) {
  }
  return res;
}

NVPTL_RESULT nvpfm::load_user_config(const std::string &config_name) {
  s_nvpfm_load_user_custom_config req;
  s_nvpfm_load_user_custom_config_ret ret;
  int len = sizeof(ret);
  strcpy(req.file_name, config_name.c_str());
  NVPTL_RESULT res = nvpfm_set(m_handle, NVPFM_LOAD_USER_CFG, &req, sizeof(req), &ret, &len, 500);
  if (ret.ret != 0) {
  }
  return res;
}
// pwr mode comes from device info
NVPTL_RESULT nvpfm::get_pwr_clk_config(const std::string &pwr_mode, s_nvpfm_get_clk_by_power_mode_info *result) {
  s_nvpfm_get_clk_by_power_mode_info tmp_ret;
  int len = sizeof(tmp_ret);
  if (!result) {
    result = &tmp_ret;
  }
  s_nvpfm_get_clk_by_power_mode req;
  memset(&req, 0x0, sizeof(req));
  strcpy(req.power_mode, pwr_mode.c_str());
  NVPTL_RESULT res = nvpfm_set(m_handle, NVPFM_GET_PWR_MODE_DESC, &req, sizeof(req), result, &len, 500);
  if (result->ret != 0) {
    // error;
  }
  return res;
}

nvpfm::nvpfm(NVPTL_DEVICE_INFO *pinfo, EVENTCALLBACK callback, void *userdata) {
  m_depthconfig.dsp_depth_mode = 0;
  m_depthconfig.dsp_depth_zoom = 1;
  m_depthconfig.dsp_depth_fusion = 1;
  m_depthconfig.dsp_depth_stitch = 0;
  m_depthconfig.dsp_depth_denoise = 0;

  memset(&m_irexposure, 0, sizeof(s_nvpfm_app_sensor_exposure_config));
  memset(&m_runcfg, 0, sizeof(s_nvpfm_app_run_config));
  //	memset(&m_cnnmodel, 0, sizeof(s_nvpfm_depth_cnn_model));
  memset(&m_rgbexposure, 0, sizeof(s_nvpfm_app_sensor_exposure_config));
  memset(&m_devinfo, 0, sizeof(s_nvpfm_dev_info));
  /*  m_config.irres = NVPFM_RESOLUTION_640_400;
    m_config.rgbres = NVPFM_RESOLUTION_640_400;
    m_config.fps = FPS_30;
    m_config.fpsratio = 1;*/

  m_handle = nvpfm_open(pinfo, callback, userdata);
  if (m_handle != NULL) {
    m_isplugged = true;
  }
}

nvpfm::~nvpfm() {
  nvpfm_debug_printf("destructor nvpfm\n");
  if (m_handle != NULL) {
    nvpfm_debug_printf("call nvpfm_close\n");
    nvpfm_close(m_handle);
    //  nvpfm_releasehandle(m_handle);
  }
}
bool nvpfm::isplugged() {
  return m_isplugged && m_handle != NULL && nvpfm_hasconnect(m_handle);
}
std::string nvpfm::get_usb_speed() {
  return m_usb_speed;
}
std::string nvpfm::get_producttype() {
  return m_producttype;
}
std::string nvpfm::get_sn() {
  return m_sn;
}
std::string nvpfm::get_sdk_version() {
  return std::string(nvpfm_getsdkversion());
}

NVPTL_RESULT nvpfm::set_timesynccycle(bool enable, int seconds) {
  return nvpfm_settimesynccycle(m_handle, (enable ? 1 : 0), seconds);
}
NVPTL_RESULT nvpfm::set_timesync(s_nvpfm_time *timestamp) {
  s_nvpfm_time_ret timeret;
  int retlen = sizeof(s_nvpfm_time_ret);
  NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_TIME_SYNC, timestamp, sizeof(s_nvpfm_time), &timeret, &retlen, 2000);
  return ret;
}
NVPTL_RESULT nvpfm::set_cypher_key(s_nvpfm_efuse_key *pparam) {
  s_nvpfm_efuse_key retparam;
  int retlen = sizeof(s_nvpfm_efuse_key);
  return nvpfm_set(m_handle, NVPFM_WRITE_KEY, pparam, sizeof(s_nvpfm_efuse_key), &retparam, &retlen, 2000);
}

NVPTL_RESULT nvpfm::set_high_precision(bool on) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_set_high_precision tmpprecision;
    tmpprecision.info.enable = (on ? 1 : 0);
    tmpprecision.channel = NVPFM_DEPTH_CHANNEL0;
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_HIGH_PRECISION, &tmpprecision, sizeof(s_nvpfm_set_high_precision), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_projector(s_nvpfm_set_projector *pcfg) {
  s_nvpfm_cmd_set_ret setret;
  int retlen = sizeof(s_nvpfm_cmd_set_ret);
  if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_PROJECTOR_STATUS, pcfg, sizeof(s_nvpfm_set_projector), &setret, &retlen, 2000)) {
    if (setret.ret == 0) {
      return NVPTL_OK;
    }
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::get_depth_calculate(s_nvpfm_get_depth_calculate *pdepthchn, s_nvpfm_get_depth_calculate_ret *pdata) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_get_depth_calculate_ret param;
    int reslen = sizeof(s_nvpfm_get_depth_calculate_ret);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_DEPTH_CALCULATE, pdepthchn, sizeof(s_nvpfm_get_depth_calculate), &pdata, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      // nvpfm_debug_printf("got lightfilter:\n");

      // nvpfm_debug_printf("lightfilter:enable:%d\n", param.enable);
      *pdata = param;
      return ret;
    }
    COMMONUSLEEP(100 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::get_smear_filter(s_nvpfm_smear_filter *pfilter) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_smear_filter param;
    int reslen = sizeof(s_nvpfm_smear_filter);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_SMEAR_FILTER, NULL, 0, &param, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      // nvpfm_debug_printf("got lightfilter:\n");

      // nvpfm_debug_printf("lightfilter:enable:%d\n", param.enable);
      *pfilter = param;
      return ret;
    }
    COMMONUSLEEP(100 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_light_filter(s_nvpfm_light_filter *pfilter) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_light_filter param;
    int reslen = sizeof(s_nvpfm_light_filter);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_LIGHT_FILTER, NULL, 0, &param, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      // nvpfm_debug_printf("got lightfilter:\n");

      // nvpfm_debug_printf("lightfilter:enable:%d\n", param.enable);
      *pfilter = param;
      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_repeat_texture_filter(s_nvpfm_repeated_texture_filter *pfilter) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_repeated_texture_filter param;
    int reslen = sizeof(s_nvpfm_repeated_texture_filter);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_REPEAT_TEXTURE_FILTER, NULL, 0, &param, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      // nvpfm_debug_printf("got repeat_texturefilter:\n");

      // nvpfm_debug_printf("repeat_texturefilter:enable:%d\n", param.enable);
      *pfilter = param;
      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_confidence(s_nvpfm_confidence *pdata) {
  //    CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    s_nvpfm_confidence tmpparam;
    int reslen = sizeof(s_nvpfm_confidence);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_CONFIDENCE, NULL, 0, &tmpparam, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      nvpfm_debug_printf("got confidence:\n");

      nvpfm_debug_printf("confidence:%d,:%f\n", tmpparam.enable, tmpparam.sigma);
      *pdata = tmpparam;

      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_speckle(s_nvpfm_get_dynamic_config *pcfg, s_nvpfm_speckle_filter *pdata) {
  for (int i = 0; i < 2; i++) {
    int reslen = sizeof(s_nvpfm_speckle_filter);
    //  s_nvpfm_get_dynamic_config config;
    // config.channel = NVPFM_DEPTH_CHANNEL0;
    int configlen = sizeof(s_nvpfm_get_dynamic_config);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_SPECKLE_FILTER, pcfg, configlen, pdata, &reslen, 2000);
    if (NVPTL_OK == ret) {
      return ret;
    }
    COMMONUSLEEP(100 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_speckle(s_nvpfm_set_speckle_filter *pdata) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_dsp_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_dsp_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_SPECKLE_FILTER, pdata, sizeof(s_nvpfm_set_speckle_filter), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK && retvalue.ret == 0)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::get_bad_filter(s_nvpfm_bad_filter *tmpparam) {
  //   CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    int reslen = sizeof(s_nvpfm_bad_filter);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_BAD_FILTER, NULL, 0, tmpparam, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      // nvpfm_debug_printf("got badfilter:\n");

      // nvpfm_debug_printf("badfilter:%d\n", tmpparam->enable);
      return ret;
    }
    COMMONUSLEEP(100 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::get_cypher_status(s_nvpfm_efuse_status *pparam) {
  for (int i = 0; i < 5; i++) {
    int reslen = sizeof(s_nvpfm_efuse_status);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_CYPHER_STATUS, NULL, 0, pparam, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      nvpfm_debug_printf("got cypher status:0x%X\n", pparam->status);
      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::get_rgb_else_param(s_nvpfm_rgb_else_param *pparam) {
  for (int i = 0; i < 5; i++) {
    int reslen = sizeof(s_nvpfm_rgb_else_param);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_RGB_ELSE_PARAM, NULL, 0, pparam, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      nvpfm_debug_printf("got rgb else param:\n");

      nvpfm_debug_printf("rgb_rectify_mode:%d\n", pparam->rgb_rectify_mode);
      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::get_high_precision(s_nvpfm_high_precision &status) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_high_precision tmpparam;
    int reslen = sizeof(s_nvpfm_high_precision);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_HIGH_PRECISION, NULL, 0, &tmpparam, &reslen, 2000); // timeout 2000milliseconds
    if (ret == NVPTL_OK) {
      nvpfm_debug_printf("got highprecision:\n");

      nvpfm_debug_printf("highprecision:%d\n", tmpparam.enable);
      status = tmpparam;
      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_projector(s_nvpfm_get_projector *pcfg, s_nvpfm_get_projector_ret *retvalue) {
  int retlen = sizeof(s_nvpfm_get_projector_ret);
  return nvpfm_get(m_handle, NVPFM_PROJECTOR_STATUS, pcfg, sizeof(s_nvpfm_get_projector), retvalue, &retlen, 2000); // timeout 2000milliseconds
}

void nvpfm::set_global_time(bool enable) {
  nvpfm_globaltimeenable(m_handle, (enable ? 1 : 0));
}

NVPTL_RESULT nvpfm::config_goodfeature(s_nvpfm_set_good_feature *feature) {
  if (m_handle == NULL)
    return NVPTL_NOTCONNECT;
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_GOODFEATURE_PARAM, feature, sizeof(s_nvpfm_set_good_feature), &retvalue, &retlen, 2000))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::config_lk(s_nvpfm_set_lk *lk) {
  if (m_handle == NULL)
    return NVPTL_NOTCONNECT;

  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_LK_PARAM, lk, sizeof(s_nvpfm_set_lk), &retvalue, &retlen, 2000))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::start_save(FRAMECALLBACK savecallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_SAVE, (void *)savecallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::start_groupimages(GROUPFRAMECALLBACK groupcallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_GROUPIMAGES, (void *)groupcallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::start_leftir(FRAMECALLBACK leftircallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_DEPTH_LEFTIR, (void *)leftircallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::start_rightir(FRAMECALLBACK rightircallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_DEPTH_RIGHTIR, (void *)rightircallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::start_other(FRAMECALLBACK othercallback) {
  //   CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_OTHER, (void *)othercallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::start_depth(FRAMECALLBACK depthcallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_DEPTH, (void *)depthcallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
/*
NVPTL_RESULT nvpfm::save_images(const char* cameraname,const char* path,int groups)
{
    return nvpfm_saveimages(m_handle,cameraname,path,groups);
}
*/
NVPTL_RESULT nvpfm::start_rgb(FRAMECALLBACK rgbcallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_RGB, (void *)rgbcallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::start_goodfeature(FRAMECALLBACK goodfeaturecallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_GOODFEATURE, (void *)goodfeaturecallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::start_imu(FRAMECALLBACK imucallback) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_start(m_handle, NVPFM_STREAM_IMU, (void *)imucallback)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
/*
typedef struct
    {
        uint32_t exposure_mode;		   //< 0-自动曝光  1-手动曝光
int32_t exposure_time[2];      //< us, 大于等于0时有效，小于0表示不操作曝光时间
int32_t digital_gain[2];       //< 大于等于0时有效，小于0表示不操作数字增益
int32_t exp_ratio;             // 10~25
int32_t AE_compensation_id[2]; // 128 96~160
}
s_nvpfm_sensor_exposure;

NVPTL_RESULT nvpfm::set_pointcloud_transfer(bool status)
{
    m_depthconfig.dsp_depth_denoise = (status ? 1 : 0);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_DEPTH_CONFIG, &m_depthconfig, sizeof(s_nvpfm_app_dsp_process_static_config), 2000);
    return ret;
}*/

NVPTL_RESULT nvpfm::save_config() {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_SAVE_CONFIG, NULL, 0, &retvalue, &retlen, 2000)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::senduserdata(void *data, int size) {
  return nvpfm_senduserdata(m_handle, data, size);
}
NVPTL_RESULT nvpfm::uploadsensorcalib(std::string localfilepath, NVPFM_IMAGE_SIZE frame_size, E_NVPFM_SENSOR_CHANNEL channel, NVPFM_CALIBRATION_SENSOR_TYPE calitype) {
  return uploadcalib(localfilepath, CALIBRATION_SENSOR, frame_size, channel, calitype);
}
NVPTL_RESULT nvpfm::uploadimucalib(std::string localfilepath, const std::string &file_name, E_NVPFM_IMU_CHANNEL channel, NVPFM_CALIBRATION_IMU_TYPE calitype) {
  return uploadcalib(localfilepath, CALIBRATION_IMU, IMAGE_UNKNOWN, channel, calitype, file_name);
}

NVPTL_RESULT nvpfm::factorystartimage() {
  int retlen = sizeof(s_factory_set_ret);
  s_factory_set_ret retvalue;
  if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_FACTORYSTARTIMAGE, NULL, 0, &retvalue, &retlen, 2000)) {

    if (retvalue.ret == 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::factorystopimage() {
  int retlen = sizeof(s_factory_set_ret);
  s_factory_set_ret retvalue;
  if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_FACTORYSTOPIMAGE, NULL, 0, &retvalue, &retlen, 2000)) {

    if (retvalue.ret == 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::factorystartimu() {
  int retlen = sizeof(s_factory_set_ret);
  s_factory_set_ret retvalue;
  if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_FACTORYSTARTIMU, NULL, 0, &retvalue, &retlen, 2000)) {

    if (retvalue.ret == 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::factorystopimu() {
  int retlen = sizeof(s_factory_set_ret);
  s_factory_set_ret retvalue;
  if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_FACTORYSTOPIMU, NULL, 0, &retvalue, &retlen, 2000)) {

    if (retvalue.ret == 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::uploadcalibstart() {
  int retlen = sizeof(s_nvpfm_cmd_set_ret);
  s_nvpfm_cmd_set_ret retvalue;
  if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_UPLOAD_CALIB_START, NULL, 0, &retvalue, &retlen, 2000)) {

    if (retvalue.ret == 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::uploadcalibend() {
  int retlen = sizeof(s_nvpfm_cmd_set_ret);
  s_nvpfm_cmd_set_ret retvalue;
  if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_UPLOAD_CALIB_END, NULL, 0, &retvalue, &retlen, 2000)) {

    if (retvalue.ret == 0)
      return NVPTL_OK;
    else
      return NVPTL_FAILED;
  }
  return NVPTL_FAILED;
}
#define UPGRADE_MAX_BLK_SIZE (1024 * 46)
// 46k max/file
// curr_packet_numbers从1开始
NVPTL_RESULT nvpfm::uploadcalib(std::string localfilepath, NVPFM_CALIBRATION_TYPE type, NVPFM_IMAGE_SIZE frame_size, uint16_t channel, uint16_t calitype, const std::string &file_name) {
  FILE *fp = fopen(localfilepath.c_str(), "rb");
  if (fp != NULL) {
    struct stat tmpstat;
    if (0 == fstat(fileno(fp), &tmpstat)) {

      uint32_t total_blk = (uint32_t)(tmpstat.st_size + UPGRADE_MAX_BLK_SIZE - 1) / UPGRADE_MAX_BLK_SIZE;

      s_nvpfm_calibration_data *frame = (s_nvpfm_calibration_data *)calloc(1, sizeof(s_nvpfm_calibration_data) + UPGRADE_MAX_BLK_SIZE);

      size_t read_offset = 0;
      uint32_t frameindex = 1;
      while (read_offset < tmpstat.st_size) {
        bool uploadok = false;
        size_t read_len = (tmpstat.st_size - read_offset) > UPGRADE_MAX_BLK_SIZE ? UPGRADE_MAX_BLK_SIZE : (tmpstat.st_size - read_offset);
        size_t readlen = fread(frame->data, 1, read_len, fp);
        read_offset += read_len;
        char *filebase = nullptr;
        if (file_name == "") {
          filebase = mybasename(localfilepath.c_str());
        }
        frame->packet_numbers = total_blk;
        frame->curr_packet_numbers = frameindex++;
        frame->data_len = read_len;
        strcpy(frame->name, filebase ? filebase : file_name.c_str());
        frame->type = type;
        frame->frame_size = frame_size;
        frame->channel = channel;
        frame->sub_type = calitype;
        if (filebase) {
          free(filebase);
        }
        s_nvpfm_calibration_result retvalue;
        int reponselen = sizeof(s_nvpfm_calibration_result);
        for (int i = 0; i < 5; i++) {
          if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_UPLOAD_CALIB_DATA, frame, sizeof(s_nvpfm_calibration_data) + tmpstat.st_size, &retvalue, &reponselen, 2000)) {
            if (retvalue.result == 0 || retvalue.result == 1) {
              uploadok = true;
              break;
            } else {
              printf("upload ok,but result is not 0 or 1,try again!\n");
            }
          } else {
            printf("fail to upload calib data,try again!\n");
          }
          COMMONUSLEEP(500 * 1000);
        }
        if (!uploadok) {
          free(frame);
          fclose(fp);
          return NVPTL_FAILED;
        }
      }
      free(frame);
    } else {
      printf("fail to get file size!\n");
      fclose(fp);
      return NVPTL_FAILED;
    }
    fclose(fp);
  }
  return NVPTL_OK;
}

NVPTL_RESULT nvpfm::downloadcalib(std::string localfilepath) {
  int retlen = sizeof(NVPTL_USBHeaderDataPacket) + MAXDOWNLOADFILESIZE;
  s_nvpfm_calibration_download_result *content = (s_nvpfm_calibration_download_result *)malloc(retlen);
  NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_DOWNLOAD_CALIB, NULL, 0, &content, &retlen, 2000);
  if (ret == NVPTL_OK && content->result == 0) {
    FILE *fp = fopen(localfilepath.c_str(), "wb");
    fwrite(content->data, 1, retlen - sizeof(s_nvpfm_calibration_download_result), fp);
    fclose(fp);
    return NVPTL_OK;
  }
  return NVPTL_FAILED;
}

NVPTL_RESULT nvpfm::downloadfile2mem(std::string remotefilepath, char** ppfilecontent)
{
   // 1.download two yaml file
  // s_nvpfm_download_bidata *pdata1 = (s_nvpfm_download_bidata *)malloc(sizeof(s_nvpfm_download_bidata) + MAXDOWNLOADFILESIZE);
  // strcpy(pdata1->downloaddata.file_path, remotefilepath.c_str());
  s_nvpfm_download_data wantdata;
  strcpy(wantdata.file_path, remotefilepath.c_str());

  s_nvpfm_download_result *pdata = (s_nvpfm_download_result *)malloc(sizeof(s_nvpfm_download_result) + MAXDOWNLOADFILESIZE);

  int responselen = sizeof(s_nvpfm_download_result) + MAXDOWNLOADFILESIZE;
  if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_DOWNLOAD_FILE, &wantdata, sizeof(s_nvpfm_download_data), pdata, &responselen, 2000)) // timeout 2000milliseconds
  {
      nvpfm_info_printf("ok to download file,len:%d\n",responselen - sizeof(s_nvpfm_download_result));
      *ppfilecontent=(char*)calloc(1,responselen - sizeof(s_nvpfm_download_result)+1);
      strncpy(*ppfilecontent,pdata->data,responselen - sizeof(s_nvpfm_download_result));
      free(pdata);
      return NVPTL_OK;
  }else{
      nvpfm_info_printf("fail to download file!\n");
  }
  free(pdata);
  return NVPTL_FAILED;
}


NVPTL_RESULT nvpfm::uploadmem2file(const char* content, std::string remotefilepath)
{
    char *tmpstrfordir = strdup(remotefilepath.c_str());
    char *tmpdirname = dirname(tmpstrfordir);
    char *tmpstrbase = strdup(remotefilepath.c_str());
    char *tmpbasename = basename(tmpstrbase);
/*int nvpfm_upgrade_withcontent(NVPFM_DEVICE_HANDLE handle,
                                UPGRADECALLBACK callback,
                                void *userdata, 
                                int upgrade_type,
                                const char *dst_path,
                                const char *dst_filename, 
                                const char *localcontent)
    */
    int ret = nvpfm_upgrade_withcontent(m_handle, NULL, NULL, NVPFM_COMMAND_USB_UPGRADE_USER_FILE, tmpdirname, tmpbasename,content);
    free(tmpstrfordir);
    free(tmpstrbase);
    if (ret < 0)
    {
        nvpfm_info_printf("upload failed!\n");
        return NVPTL_FAILED;
    }
    else
    {
        nvpfm_info_printf("upload success!\n");
        return NVPTL_OK;
    }
}
NVPTL_RESULT nvpfm::downloadfile(std::string remotefilepath, std::string localfilepath) {
  // 1.download two yaml file
  // s_nvpfm_download_bidata *pdata1 = (s_nvpfm_download_bidata *)malloc(sizeof(s_nvpfm_download_bidata) + MAXDOWNLOADFILESIZE);
  // strcpy(pdata1->downloaddata.file_path, remotefilepath.c_str());
  s_nvpfm_download_data wantdata;
  strcpy(wantdata.file_path, remotefilepath.c_str());

  s_nvpfm_download_result *pdata = (s_nvpfm_download_result *)malloc(sizeof(s_nvpfm_download_result) + MAXDOWNLOADFILESIZE);

  int responselen = sizeof(s_nvpfm_download_result) + MAXDOWNLOADFILESIZE;
  if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_DOWNLOAD_FILE, &wantdata, sizeof(s_nvpfm_download_data), pdata, &responselen, 2000)) // timeout 2000milliseconds
  {
    FILE *fp = fopen(localfilepath.c_str(), "wb");
    if (fp != NULL) {
      fwrite(pdata->data, 1, responselen - sizeof(s_nvpfm_download_result), fp);
      fclose(fp);
      return NVPTL_OK;
    }
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::upgrade(std::string localfirmwarefilepath, UPGRADECALLBACK upgradecallback, void *userdata) {
  int nvpfm_upgrade(NVPFM_DEVICE_HANDLE handle, UPGRADECALLBACK callback, void *userdata, int upgrade_type, const char *dst_path, const char *dst_filename, const char *local_filename);
  int ret = nvpfm_upgrade(m_handle, upgradecallback, userdata, NVPFM_COMMAND_USB_UPGRADE_NEXTVPU_SYSTEM, "", "", localfirmwarefilepath.c_str());
  if (ret < 0) {
    nvpfm_error_printf("upload failed!\n");
    return NVPTL_FAILED;
  } else {
    nvpfm_debug_printf("upload success!\n");
    return NVPTL_OK;
  }
}
NVPTL_RESULT nvpfm::uploadfile(std::string localfilepath, std::string remotefilepath) {
  char *tmpdirname = mydirname(remotefilepath.c_str());
  char *tmpbasename = mybasename(remotefilepath.c_str());

  int ret = nvpfm_upgrade(m_handle, NULL, NULL, NVPFM_COMMAND_USB_UPGRADE_USER_FILE, tmpdirname, tmpbasename, localfilepath.c_str());
  free(tmpdirname);
  free(tmpbasename);
  if (ret < 0) {
    nvpfm_error_printf("upload failed!\n");
    return NVPTL_FAILED;
  } else {
    nvpfm_debug_printf("upload success!\n");
    return NVPTL_OK;
  }
}
NVPTL_RESULT nvpfm::set_ip(s_nvpfm_set_ip* setip){
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_SET_CAMERA_IP, setip, sizeof(s_nvpfm_set_ip), &retvalue, &retlen, 2000)) {
      return NVPTL_OK;
    }
    return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::set_efuse_user(s_nvpfm_efuse_user *param) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_efuse_user_ret retvalue;
    int retlen = sizeof(s_nvpfm_efuse_user_ret);
    if (NVPTL_OK == nvpfm_set(m_handle, NVPFM_EFUSE_USER, param, sizeof(s_nvpfm_efuse_user), &retvalue, &retlen, 2000)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

/*
NVPTL_RESULT nvpfm::set_rgb_exposure(s_nvpfm_sensor_exposure *exposure)
{
    for(int i=0;i<5;i++){
      if(NVPTL_OK==nvpfm_set(m_handle, NVPFM_RGB_EXPOSURE, exposure, sizeof(s_nvpfm_sensor_exposure), 2000))
  {
        return NVPTL_OK;
  }
    COMMONUSLEEP(50*1000);
    }
   return NVPTL_TIMEOUT;
}*/

NVPTL_RESULT nvpfm::get_efuse_user(s_nvpfm_efuse_user *param) {
  for (int i = 0; i < 5; i++) {
    int reslen = 0;
    if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_EFUSE_USER, NULL, 0, param, &reslen, 2000)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_camera_time(s_nvpfm_time_info *timeinfo) {
  for (int i = 0; i < 5; i++) {
    int reslen = 0;
    if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_CAMERA_TIME, NULL, 0, timeinfo, &reslen, 2000)) {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_imu_rgb_offset(float *ms) {
  s_nvpfm_imu_external_reference *ref = get_imu_externalref();
  *ms = ref->timeshift_cam_imu;
  return NVPTL_OK;
}
/*
NVPTL_RESULT nvpfm::get_rgb_exposure(s_nvpfm_sensor_exposure *exposure)
{
    for(int i=0;i<5;i++){
      if(NVPTL_OK==nvpfm_get(m_handle, NVPFM_RGB_EXPOSURE, exposure, 2000))
  {
        return NVPTL_OK;
  }
    COMMONUSLEEP(50*1000);
    }
   return NVPTL_TIMEOUT;
}*/
NVPTL_RESULT nvpfm::get_depth_config(s_nvpfm_get_dsp_static_config_ret *depthconfig) {
  for (int i = 0; i < 5; i++) {
    int reslen = sizeof(s_nvpfm_get_dsp_static_config_ret);
    s_nvpfm_get_dsp_static_config cfg;
    cfg.channel = NVPFM_DEPTH_CHANNEL0;
    if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_DEPTH_CONFIG, (uint8_t *)&cfg, sizeof(s_nvpfm_get_dsp_static_config), depthconfig, &reslen, 2000)) {
      if (depthconfig->ret == 0) {
        memcpy(&m_depthconfig, &depthconfig->config, sizeof(m_depthconfig));
        return NVPTL_OK;
      }
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_usb_cc_info(s_nvpfm_usb_cc_info *pinfo) {
  for (int i = 0; i < 5; i++) {
    int reslen = 0;
    if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_USB_CC_INFO, NULL, 0, pinfo, &reslen, 2000))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_mode(s_nvpfm_get_app_run_config_ret *pconfig) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_get_app_run_config_ret src;
    int reslen = 0;
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_RUN_MODE, NULL, 0, &src, &reslen, 2000);
    if (ret == NVPTL_OK) {
      *pconfig = src;
      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
/*
NVPTL_RESULT nvpfm::set_pipeline(int mode){
  for(int i=0;i<5;i++){
        s_nvpfm_pipeline_cmd param;
        param.status = mode;
        NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_PIPELINE, &param, sizeof(param), 2000);
    if(ret==NVPTL_OK)
      return ret;
    COMMONUSLEEP(50*1000);
    }
    return NVPTL_TIMEOUT;
}*/
NVPTL_RESULT nvpfm::set_mode(s_nvpfm_app_run_config config) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_RUN_MODE, &config, sizeof(s_nvpfm_app_run_config), &retvalue, &retlen, 2000);
    if (NVPTL_OK == ret) // timeout 2000milliseconds
    {
      return NVPTL_OK;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

/*
NVPTL_RESULT nvpfm::set_edgefilter_async(s_nvpfm_edge_filter_cmd* pdata)
{
  nvpfm_set_async(m_handle, NVPFM_EDGE_FILTER, (uint8_t*)pdata, sizeof(s_nvpfm_edge_filter_cmd), NULL, NULL);
  return NVPTL_OK;
}*/
NVPTL_RESULT nvpfm::set_badfilter_async(s_nvpfm_set_bad_filter *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_BAD_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_bad_filter), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_lightfilter_async(s_nvpfm_set_light_filter *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_LIGHT_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_light_filter), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_confidence_async(s_nvpfm_set_confidence *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_CONFIDENCE, (uint8_t *)pdata, sizeof(s_nvpfm_set_confidence), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_sensorcfg_async(s_nvpfm_app_sensor_config *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_SENSOR_CONFIG, (uint8_t *)pdata, sizeof(s_nvpfm_app_sensor_config), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_sensorcfg(s_nvpfm_app_sensor_config *pdata) {
  s_nvpfm_cmd_set_ret retvalue;
  int reslen = sizeof(s_nvpfm_cmd_set_ret);
  return nvpfm_set(m_handle, NVPFM_SENSOR_CONFIG, (uint8_t *)pdata, sizeof(s_nvpfm_app_sensor_config), &retvalue, &reslen, 2000);
}
NVPTL_RESULT nvpfm::set_spatial_async(s_nvpfm_set_spatial_filter *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_SPATIAL_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_spatial_filter), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_temporal_async(s_nvpfm_set_temporal_filter *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_TEMPORAL_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_temporal_filter), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_speckle_async(s_nvpfm_set_speckle_filter *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_SPECKLE_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_speckle_filter), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_highlight_async(s_nvpfm_set_depth_high_light *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_HIGHLIGHT_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_depth_high_light), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_repeatfilter_async(s_nvpfm_set_repeated_texture_filter *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_REPEAT_TEXTURE_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_repeated_texture_filter), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_depthconfig_async(s_nvpfm_set_dsp_static_config *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_DEPTH_CONFIG, (uint8_t *)pdata, sizeof(s_nvpfm_set_dsp_static_config), callback, userdata);

  //	Sleep(100);
  //	set_pipeline(3);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_irexposure(s_nvpfm_set_sensor_exposure *pdata) {
  s_nvpfm_image_set_ret ret;
  int retlen = sizeof(s_nvpfm_image_set_ret);
  return nvpfm_set(m_handle, NVPFM_IR_EXPOSURE, (uint8_t *)pdata, sizeof(s_nvpfm_set_sensor_exposure), &ret, &retlen, 2000);
}
NVPTL_RESULT nvpfm::set_rgbexposure(s_nvpfm_set_sensor_exposure *pdata) {
  s_nvpfm_image_set_ret ret;
  int retlen = sizeof(s_nvpfm_image_set_ret);

  return nvpfm_set(m_handle, NVPFM_RGB_EXPOSURE, (uint8_t *)pdata, sizeof(s_nvpfm_set_sensor_exposure), &ret, &retlen, 2000);
}
NVPTL_RESULT nvpfm::set_irexposure_async(s_nvpfm_set_sensor_exposure *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_IR_EXPOSURE, (uint8_t *)pdata, sizeof(s_nvpfm_set_sensor_exposure), callback, userdata);

  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_rgbexposure_async(s_nvpfm_set_sensor_exposure *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_RGB_EXPOSURE, (uint8_t *)pdata, sizeof(s_nvpfm_set_sensor_exposure), callback, userdata);

  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_runcfg(s_nvpfm_app_run_config *pdata) {
  s_nvpfm_cmd_set_ret retvalue;
  int retlen = sizeof(s_nvpfm_cmd_set_ret);
  return nvpfm_set(m_handle, NVPFM_RUN_MODE, pdata, sizeof(s_nvpfm_app_run_config), &retvalue, &retlen, 2000);
}

NVPTL_RESULT nvpfm::set_triggercfg_async(s_nvpfm_app_trigger_config *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_TRIGGER_CONFIG, (uint8_t *)pdata, sizeof(s_nvpfm_app_trigger_config), callback, userdata);

  // Sleep(100);
  // set_pipeline(3);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_runcfg_async(s_nvpfm_app_run_config *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_RUN_MODE, (uint8_t *)pdata, sizeof(s_nvpfm_app_run_config), callback, userdata);

  // Sleep(100);
  // set_pipeline(3);
  return NVPTL_OK;
}
/*NVPTL_RESULT nvpfm::set_cnnmodel_async(s_nvpfm_depth_cnn_model* pdata)
{
  nvpfm_set_async(m_handle, NVPFM_CNN_MODEL, (uint8_t*)pdata, sizeof(s_nvpfm_depth_cnn_model), NULL, NULL);

  Sleep(100);
  set_pipeline(3);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_rgbexposure_async(s_nvpfm_sensor_exposure* pdata)
{
  nvpfm_set_async(m_handle, NVPFM_RGB_EXPOSURE, (uint8_t*)pdata, sizeof(s_nvpfm_sensor_exposure), NULL, NULL);

  //	Sleep(100);
  //set_pipeline(3);
  return NVPTL_OK;
}*/
NVPTL_RESULT nvpfm::set_smearfilter_async(s_nvpfm_set_smear_filter *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_SMEAR_FILTER, (uint8_t *)pdata, sizeof(s_nvpfm_set_smear_filter), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_highprecision_async(s_nvpfm_set_high_precision *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_HIGH_PRECISION, (uint8_t *)pdata, sizeof(s_nvpfm_set_high_precision), callback, userdata);
  return NVPTL_OK;
}
NVPTL_RESULT nvpfm::set_sensor_status(s_set_sensor_status *pdata, bool basync, ASYNCCALLBACK callback, void *userdata) {
  // CHECKPLUG();
  if (basync) {
    nvpfm_set_async(m_handle, NVPFM_SENSOR_STATUS, (uint8_t *)pdata, sizeof(s_set_sensor_status), callback, userdata);
    return NVPTL_OK;
  } else {
    for (int i = 0; i < 5; i++) {
      s_nvpfm_cmd_set_ret retvalue;
      int retlen = sizeof(s_nvpfm_cmd_set_ret);
      NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_SENSOR_STATUS, pdata, sizeof(s_set_sensor_status), &retvalue, &retlen, 2000);
      if (NVPTL_OK == ret)
        return ret;
      COMMONUSLEEP(50 * 1000);
    }
    return NVPTL_TIMEOUT;
  }
}
NVPTL_RESULT nvpfm::set_imu_status(s_set_imu_status *pdata) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_IMU_STATUS, pdata, sizeof(s_set_imu_status), &retvalue, &retlen, 2000);
    if (NVPTL_OK == ret)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::set_enter_ready() {
  int len = 0;
  return nvpfm_set(m_handle, NVPFM_ENTER_READY, NULL, 0, NULL, &len, 2000);
}
NVPTL_RESULT nvpfm::set_exit_ready() {
  int len = 0;
  return nvpfm_set(m_handle, NVPFM_EXIT_READY, NULL, 0, NULL, &len, 2000);
}

NVPTL_RESULT nvpfm::get_run_at_ready_mode_info(s_nvpfm_run_at_ready_mode_info *info) {
  int len = sizeof(s_nvpfm_run_at_ready_mode_info);
  return nvpfm_get(m_handle, NVPFM_RUN_READY_MODE, NULL, 0, info, &len, 2000);
}
NVPTL_RESULT nvpfm::set_trigger_config(s_nvpfm_app_trigger_config *triggercfg, s_nvpfm_cmd_set_ret *retcfg) {
  int retlen = sizeof(s_nvpfm_cmd_set_ret);
  return nvpfm_set(m_handle, NVPFM_TRIGGER_CONFIG, triggercfg, sizeof(s_nvpfm_app_trigger_config), retcfg, &retlen, 2000);
}
NVPTL_RESULT nvpfm::get_trigger_config(s_nvpfm_get_app_trigger_config_ret *triggercfg) {
  int retlen = sizeof(s_nvpfm_get_app_trigger_config_ret);
  return nvpfm_get(m_handle, NVPFM_TRIGGER_CONFIG, NULL, 0, triggercfg, &retlen, 2000);
}
NVPTL_RESULT nvpfm::set_imu_config(s_set_imu_config *imucfg) {
  for (int i = 0; i < 5; i++) {

    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_IMU_CONFIG, imucfg, sizeof(s_set_imu_config), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_imu_config(s_get_imu_config_ret *cfg) {
  for (int i = 0; i < 5; i++) {
    int retlen = sizeof(s_get_imu_config_ret);
    s_get_imu_config imucfg;
    imucfg.channel = NVPFM_IMU_CHANNEL0;
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_IMU_CONFIG, &imucfg, sizeof(s_get_imu_config), cfg, &retlen, 2000);
    if (ret == NVPTL_OK) {
      if (cfg->ret == 0) {
        return ret;
      }
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_imu_config_async(s_set_imu_config *pdata, ASYNCCALLBACK callback, void *userdata) {
  nvpfm_set_async(m_handle, NVPFM_IMU_CONFIG, (uint8_t *)pdata, sizeof(s_set_imu_config), callback, userdata);
  return NVPTL_OK;
}

void nvpfm::get_imu_config_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_IMU_CONFIG, callback, userdata);
}

NVPTL_RESULT nvpfm::set_can_status(s_set_can_status *pdata) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_CAN_STATUS, pdata, sizeof(s_set_can_status), &retvalue, &retlen, 2000);
    if (NVPTL_OK == ret)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::set_confidence(s_nvpfm_set_confidence *pdata) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_CONFIDENCE, pdata, sizeof(s_nvpfm_set_confidence), &retvalue, &retlen, 2000);
    if (NVPTL_OK == ret)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::set_bad_filter(s_nvpfm_set_bad_filter *pfilter) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_BAD_FILTER, pfilter, sizeof(s_nvpfm_set_bad_filter), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_depth_calculate(s_nvpfm_set_depth_calculate *pdata) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    s_nvpfm_set_depth_calculate_ret retvalue;
    int retlen = sizeof(s_nvpfm_set_depth_calculate_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_DEPTH_CALCULATE, pdata, sizeof(s_nvpfm_set_depth_calculate), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::set_smear_filter(s_nvpfm_set_smear_filter *pdata) {
  // CHECKPLUG();
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_SMEAR_FILTER, pdata, sizeof(s_nvpfm_set_smear_filter), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_light_filter(s_nvpfm_set_light_filter *light) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_LIGHT_FILTER, light, sizeof(s_nvpfm_set_light_filter), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_repeat_texture_filter(s_nvpfm_set_repeated_texture_filter *pfilter) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_REPEAT_TEXTURE_FILTER, pfilter, sizeof(s_nvpfm_set_repeated_texture_filter), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK)
      return ret;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
/*
NVPTL_RESULT nvpfm::set_edge_filter(s_nvpfm_edge_filter_cmd *edge)
{
    for(int i=0;i<5;i++){
        NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_EDGE_FILTER, edge, sizeof(s_nvpfm_edge_filter_cmd), 2000);
    if(ret==NVPTL_OK)
        return ret;
    COMMONUSLEEP(50*1000);
    }
    return NVPTL_TIMEOUT;
}*/
#ifdef _DEPTHEVAL
float nvpfm::get_plane_median(NVPTL_USBHeaderDataPacket *depthframe, double *degree) { // compute median distance of plane
  camera_param *camparam = get_camera_param(false);
  if (camparam == NULL) {
    return 0.0;
  }
  double fx, fy, b, u0, v0;
  fx = camparam->getleftirfx();
  fy = camparam->getleftirfy();
  b = camparam->getbaseline();

  u0 = camparam->getleftirphotocenterx();
  v0 = camparam->getleftirphotocentery();
  ROI tmproi = get_roi(depthframe);
  int tmpH1 = tmproi.top;
  int tmpW1 = tmproi.left;
  int tmpH2 = tmproi.bottom;
  int tmpW2 = tmproi.right;

  int width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  int height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->height;
  uint16_t *depthraw = (uint16_t *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket) + sizeof(NVPFM_USB_IMAGE_HEADER));

  cv::Mat img = cv::Mat(height, width, CV_16UC1, (void *)depthraw);
  cv::Mat depth_roi = img(cv::Range(tmpH1, tmpH2 + 1), cv::Range(tmpW1, tmpW2 + 1));
  //	cout << "origin(roi):" << depth_roi << endl;
  int h = tmpH2 - tmpH1 + 1;
  int w = tmpW2 - tmpW1 + 1;

  cv::Mat MatTemp2, tmpmask, tmpsort, sort_disp_roi, ind1, mask;
  depth_roi.convertTo(MatTemp2, CV_64F);

  cv::Mat disp_roi = (fx * b) / MatTemp2;
  // cout << "disparity(roi):" << disp_roi << endl;

  depth_roi.convertTo(tmpmask, CV_8U);

  cv::threshold(tmpmask, tmpmask, 0, 1, cv::THRESH_BINARY_INV);

  disp_roi.setTo(0, tmpmask);

  //	cout << "disparity(roi)inf->0:" << disp_roi << endl;

  tmpsort = disp_roi.clone();

  tmpsort = tmpsort.reshape(0, 1);
  cv::sort(tmpsort, sort_disp_roi, 0);
  cv::sortIdx(tmpsort, ind1, 0);
  //	cout << "sort to one dim:" << sort_disp_roi << endl;

  mask = cv::Mat::zeros(disp_roi.size(), CV_8UC1);
  // tmpmask.reshape(0, 1);
  mask.setTo(1, tmpmask);
  //	cout << "mask 0 to 1:" << mask << endl;
  int find_0_num = sum(mask)[0];
  //	cout << "find 0 num:" << find_0_num << endl;
  int sortsize = sort_disp_roi.size().width * sort_disp_roi.size().height;
  //	cout << "sortsize:" << sortsize << endl;
  int colfrom1 = find_0_num;
  int colto1 = find_0_num + round((sortsize - find_0_num) * 0.005) + 1;
  if (find_0_num < sort_disp_roi.cols)
    sort_disp_roi(cv::Range(0, 1), cv::Range(colfrom1, colto1)) = 0;
  //	cout << "set 0.005 to 0:" << sort_disp_roi << endl;

  int colfrom = find_0_num + round((sortsize - find_0_num) * 0.995) - 1;
  int colto = sortsize - 1 + 1;
  sort_disp_roi(cv::Range(0, 1), cv::Range(colfrom, colto)) = 0;
  //	cout << "set 0.995 to 0:" << sort_disp_roi << endl;

  cv::Mat tmpmask0;
  cv::threshold(sort_disp_roi, tmpmask0, 0, 1, cv::THRESH_BINARY_INV);

  int find_afterprocess0_num = sum(tmpmask0)[0];
  //	cout << "0 num:" << find_afterprocess0_num << endl;

  cv::Mat value0_ind = cv::Mat::zeros(1, find_afterprocess0_num, CV_32SC1);
  int oindex = 0;
  for (int ii = 0; ii < sort_disp_roi.rows; ii++) {
    for (int j = 0; j < sort_disp_roi.cols; j++) {
      if (sort_disp_roi.at<double>(cv::Point(j, ii)) == 0) {
        value0_ind.at<int>(cv::Point(oindex, 0)) = ind1.at<int>(cv::Point(j, ii));
        oindex++;
      }
    }
  }
  //	cout << "got 0 indexs:" << value0_ind << endl;

  cv::Mat removeoutL_disp = disp_roi.clone();
  removeoutL_disp = removeoutL_disp.reshape(0, 1);

  for (int ii = 0; ii < value0_ind.rows; ii++) {
    for (int j = 0; j < value0_ind.cols; j++) {
      removeoutL_disp.at<double>(cv::Point(value0_ind.at<int>(cv::Point(j, ii)), 0)) = 0;
    }
  }
  removeoutL_disp = removeoutL_disp.reshape(0, disp_roi.size().height);
  //	cout << "set zero to zero:" << removeoutL_disp << endl;
  cv::Mat dispDatafit = removeoutL_disp.clone();
  //---- - begin::fit Piane disp---- -
  h = dispDatafit.size().height;
  w = dispDatafit.size().width;

  int nonzeros = countNonZero(dispDatafit);
  //	cout << "nonzeros:" << nonzeros << endl;
  cv::Mat x = cv::Mat::zeros(nonzeros, 1, CV_64FC1);
  cv::Mat y = cv::Mat::zeros(nonzeros, 1, CV_64FC1);
  cv::Mat z = cv::Mat::zeros(nonzeros, 1, CV_64FC1);
  cv::Mat d = cv::Mat::zeros(nonzeros, 1, CV_64FC1);
  int ind = 0;

  for (int ii = 0; ii < dispDatafit.rows; ii++) {
    for (int j = 0; j < dispDatafit.cols; j++) {
      if (dispDatafit.at<double>(cv::Point(j, ii)) != 0) {
        z.at<double>(cv::Point(0, ind)) = fx * b / dispDatafit.at<double>(cv::Point(j, ii));
        d.at<double>(cv::Point(0, ind)) = 1;
        x.at<double>(cv::Point(0, ind)) = fx * b / dispDatafit.at<double>(cv::Point(j, ii)) * (j + tmpW1 - u0) / fx;
        y.at<double>(cv::Point(0, ind)) = fx * b / dispDatafit.at<double>(cv::Point(j, ii)) * (ii + tmpH1 - v0) / fy;
        ind++;
      }
    }
  }

  cv::Mat P;
  cv::hconcat(x, y, P);
  cv::hconcat(P, d, P);
  //	cout << "P:" <<P << endl;
  //	cout << "z:" << z << endl;
  cv::Mat param = (P.t() * P).inv() * P.t() * z;
  param = param.reshape(0, 1);
  //	cout << "x:" << x<<endl;
  //	cout << "y:" << y << endl;
  //	cout << "param:" << param << endl;
  double param1 = param.at<double>(0, 0);
  double param2 = param.at<double>(0, 1);
  double param3 = param.at<double>(0, 2);

  cv::Mat predict_norotate = param1 * x + param2 * y + param3;

  cv::Mat predict;

  if (1) {
    cv::Mat PcaData;
    cv::hconcat(x, y, PcaData);
    cv::hconcat(PcaData, z, PcaData);
    PcaData = PcaData.t();

    int k = PcaData.cols; // 计算PcaData列数
    // p_mean = (1 / k) * sum(PcaData, 2);//行求和
    cv::Mat mb(PcaData.rows, 1, CV_64FC1, cv::Scalar(0));
    cv::reduce(PcaData, mb, 1, cv::REDUCE_SUM);
    cv::Mat p_mean = ((double)1 / (double)k) * mb;

    cv::Mat CovP = (PcaData - cv::repeat(p_mean, 1, k)) * ((PcaData - cv::repeat(p_mean, 1, k)).t());
    // CovP = (PcaData - repmat(p_mean, 1, k)) * (PcaData - repmat(p_mean, 1, k))';

    cv::Mat S, u, vt;
    cv::SVD::compute(CovP, S, u, vt, cv::SVD::FULL_UV);
    cv::Mat V = vt.t();
    //[V S] = svd(CovP);
    cv::Mat n = V.colRange(V.cols - 1, V.cols).clone();
    // n = V(:, end);//最后一列
    n = n / norm(n);
    // n = n/ norm(n);
    cv::Mat flag(1, 3, CV_64F);
    flag.at<double>(0, 0) = 0;
    flag.at<double>(0, 1) = 0;
    flag.at<double>(0, 2) = 1;
    // flag = [0, 0, 1];
    double cos1 = n.dot(flag.t()) / (norm(n) * norm(flag));
    // cos = dot(n, flag) / (norm(n, 2) * norm(flag, 2));
    *degree = acos(fabs(cos1)) * 180.0 / 3.1415926;
    /* if (index == 0 && i == 0) {
         char tmpdegree[32];
         sprintf(tmpdegree, "%.2f°", angle);
         //	this->depthquality->rotatedegreelabel->setText(QString::fromLocal8Bit(tmpdegree));
     }*/
    cv::Mat r3;
    if (*degree < 0.05)
      predict = predict_norotate.clone();
    else {
      r3 = n.clone();

      cv::Mat tmpconst(1, 3, CV_64F);
      tmpconst.at<double>(0, 0) = 1;
      tmpconst.at<double>(0, 1) = 0;
      tmpconst.at<double>(0, 2) = 0;

      cv::Mat r2 = r3.t().cross(tmpconst);
      cv::Mat r1 = r2.cross(r3.t());
      cv::Mat R;
      cv::hconcat(r1.t(), r2.t(), R);
      cv::hconcat(R, r3, R);

      cv::hconcat(x, y, P);
      cv::hconcat(P, predict_norotate, P);

      cv::Mat predict_rotate = R.inv() * P.t();
      cv::Mat zP;
      cv::hconcat(x, y, zP);
      cv::hconcat(zP, z, zP);
      cv::Mat z_rotate = R.inv() * zP.t();
      int k1 = predict_rotate.cols;

      cv::Mat tmpsum(predict_rotate.rows, 1, CV_64FC1, cv::Scalar(0));
      cv::reduce(predict_rotate, tmpsum, 1, cv::REDUCE_SUM);

      cv::Mat P_rotate_mean = ((double)1 / (double)k1) * tmpsum;

      cv::Mat CovP1 = (predict_rotate - cv::repeat(P_rotate_mean, 1, k1)) * ((predict_rotate - cv::repeat(P_rotate_mean, 1, k1)).t());

      cv::SVD::compute(CovP1, S, u, vt, cv::SVD::FULL_UV);
      cv::Mat V1 = vt.t();
      cv::Mat n1 = V1.colRange(V1.cols - 1, V1.cols).clone();

      double cos1 = n1.dot(flag.t()) / (norm(n1) * norm(flag));
      double degree1 = acos(fabs(cos1)) * 180.0 / 3.1415926;
      /*  if (index == 0 && i == 0) {
            char tmpdegree[32];
            sprintf(tmpdegree, "%.2f°", angle1);
            //this->depthquality->afterrotatedegreelabel->setText(QString::fromLocal8Bit(tmpdegree));
        }*/

      predict = predict_rotate.rowRange(2, 3);
      predict = predict.t();
      z = z_rotate.rowRange(2, 3);
      z = z.t();

      /*  if (index == 0 && i == 0) {
            cv::Mat rotatemat;
            cv::hconcat(x, y, rotatemat);
            cv::hconcat(rotatemat, predict, rotatemat);
            //showrotatemats(zP, rotatemat);
        }*/
    }
  } else
    predict = predict_norotate.clone();

  // ------RMSE------ -
  cv::Mat z_depth = z;

  cv::Mat tmpzdepth;
  cv::sort(z_depth.reshape(0, 1), tmpzdepth, 0);
  return tmpzdepth.at<double>(0, tmpzdepth.cols / 2);
}
#endif
float nvpfm::get_distance(NVPTL_USBHeaderDataPacket *depthframe, int x, int y) {
  uint32_t width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  // uint32_t height=((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->height;
  uint16_t *depthraw = (uint16_t *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket) + sizeof(NVPFM_USB_IMAGE_HEADER));
  uint16_t thedepth = *((uint16_t *)(depthraw + y * width + x));
  return ((float)thedepth) / 1000.0;
}
float nvpfm::get_distance_min(NVPTL_USBHeaderDataPacket *depthframe, bool ignorezero) {
  uint32_t width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  // uint32_t height=((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->height;
  uint16_t *depthraw = (uint16_t *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket) + sizeof(NVPFM_USB_IMAGE_HEADER));
  uint16_t min = 0xffff;
  ROI theroi = get_roi(depthframe);
  for (int32_t row = theroi.top; row <= theroi.bottom; row++)
    for (int32_t col = theroi.left; col <= theroi.right; col++) {
      uint16_t tmpdepth = *((uint16_t *)(depthraw + row * width + col));
      if (tmpdepth <= min) {
        if (ignorezero) {
          if (tmpdepth != 0) {
            min = tmpdepth;
          }
        } else {
          min = tmpdepth;
        }
      }
    }
  return ((float)min) / 1000.0;
}
float nvpfm::get_distance_max(NVPTL_USBHeaderDataPacket *depthframe) {
  uint32_t width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  // uint32_t height=((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->height;
  uint16_t *depthraw = (uint16_t *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket) + sizeof(NVPFM_USB_IMAGE_HEADER));
  uint16_t max = 0;
  ROI theroi = get_roi(depthframe);
  for (int32_t row = theroi.top; row <= theroi.bottom; row++)
    for (int32_t col = theroi.left; col <= theroi.right; col++) {
      uint16_t tmpdepth = *((uint16_t *)(depthraw + row * width + col));
      if (tmpdepth >= max) {
        max = tmpdepth;
      }
    }
  return ((float)max) / 1000.0;
}
float nvpfm::get_fillrate(NVPTL_USBHeaderDataPacket *depthframe) {
  uint32_t width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  // uint32_t height=((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->height;
  uint16_t *depthraw = (uint16_t *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket) + sizeof(NVPFM_USB_IMAGE_HEADER));

  ROI theroi = get_roi(depthframe);
  uint32_t count = 0;
  for (int32_t row = theroi.top; row <= theroi.bottom; row++)
    for (int32_t col = theroi.left; col <= theroi.right; col++) {
      uint16_t thedepth = *((uint16_t *)(depthraw + row * width + col));
      if (thedepth != 0) {
        count++;
      }
    }

  return ((float)count) / ((float)((theroi.bottom - theroi.top + 1) * (theroi.right - theroi.left + 1)));
}

ROI nvpfm::get_roi(NVPTL_USBHeaderDataPacket *depthframe) {
  int32_t width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  int32_t height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->height;
  if (m_roi.left >= 0 && m_roi.left <= (width - 1) &&
      m_roi.top >= 0 && m_roi.top <= (height - 1) &&
      m_roi.right >= 0 && m_roi.right <= (width - 1) &&
      m_roi.bottom >= 0 && m_roi.bottom <= (height - 1) &&
      m_roi.left <= m_roi.right && m_roi.top <= m_roi.bottom) {
  } else {
    m_roi.left = 0;
    m_roi.right = width - 1;
    m_roi.top = 0;
    m_roi.bottom = height - 1;
  }
  return m_roi;
}
void nvpfm::set_roi(NVPTL_USBHeaderDataPacket *depthframe, float ratio) {
  uint32_t width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->width;
  uint32_t height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)depthframe + sizeof(NVPTL_USBHeaderDataPacket)))->height;
  if (ratio <= 1.0) {
    double offset = 2.0 * ((double)width + (double)height) -
                    sqrt(4.0 * ((double)width + (double)height) * ((double)width + (double)height) -
                         4.0 * 4.0 * (1.0 - ratio) * (double)width * (double)height);
    offset = offset / 8.0;
    m_roi.left = (int)offset;
    m_roi.top = (int)offset;
    m_roi.right = width - offset - 1;
    m_roi.bottom = height - offset - 1;
  }
}

void nvpfm::get_device_info_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_DEVICE_INFO, callback, userdata);
}
void nvpfm::get_depth_param_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_DEPTH_PARAM, callback, userdata);
}
void nvpfm::get_confidence_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_CONFIDENCE, callback, userdata);
}
void nvpfm::get_sensorcfg_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_SENSOR_CONFIG, callback, userdata);
}
NVPTL_RESULT nvpfm::get_factory_eeprom_sn(s_factory_get_eeprom_sn *pcfg,
                                          s_factory_get_eeprom_sn_ret *pret) {
  int reslen = sizeof(s_factory_get_eeprom_sn_ret);
  return nvpfm_get(m_handle, NVPFM_EEPROM, pcfg, sizeof(s_factory_get_eeprom_sn), pret, &reslen, 2000);
}

NVPTL_RESULT nvpfm::set_factory_eeprom_sn(s_factory_set_eeprom_sn *pcfg,
                                          s_factory_set_eeprom_sn_ret *pret) {
  int reslen = sizeof(s_factory_set_eeprom_sn_ret);
  return nvpfm_set(m_handle, NVPFM_EEPROM, pcfg, sizeof(s_factory_set_eeprom_sn), pret, &reslen, 2000);
}

NVPTL_RESULT nvpfm::set_factory_clear_calib(s_factory_clear_calib *pcfg, s_factory_clear_calib_return *pret) {
  int reslen = sizeof(s_factory_clear_calib_return);
  return nvpfm_set(m_handle, NVPFM_CLEAR_CALIB, pcfg, sizeof(s_factory_clear_calib), pret, &reslen, 2000);
}
NVPTL_RESULT nvpfm::get_sensorcfg(s_nvpfm_get_sensor_config_ret *pcfg) {
  int reslen = sizeof(s_nvpfm_get_sensor_config_ret);
  return nvpfm_get(m_handle, NVPFM_SENSOR_CONFIG, NULL, 0, pcfg, &reslen, 2000);
}

/*
typedef struct{
    char sn[32];
    char product[32];
    char cpu_net_type[64];
    char comm_type[32];
    char projector_type[32];
    char imu_type0[32];
    char imu_type1[32];
    char software_version[32];
    char usb_speed[32];
    float cpu_temperature;
  }s_nvpfm_dev_info;
*/
static bool parseoldres(std::string res, NVPFM_IMAGE_SIZE *imagesize, int *fps) {
  int width, height, tmpfps;
  sscanf(res.c_str(), "%d_%d_%d", &width, &height, &tmpfps);
  *fps = tmpfps;
  for (int i = 0; i < IMAGE_UNKNOWN; i++) {
    if (resmapper[i].width == width && resmapper[i].height == height) {
      *imagesize = (NVPFM_IMAGE_SIZE)i;
      return TRUE;
    }
  }
  return FALSE;
}
/*
static const char *getstrbyres(NVPFM_IMAGE_SIZE res, bool isx) {
  switch (res) {
  case IMAGE_1280_800:
    return (isx ? "1280x800" : "1280_800");
  case IMAGE_1280_720:
    return (isx ? "1280x720" : "1280_720");
  case IMAGE_640_480:
    return (isx ? "640x480" : "640_480");
  case IMAGE_640_400:
    return (isx ? "640x400" : "640_400");
  case IMAGE_320_200:
    return (isx ? "320x200" : "320_200");
  case IMAGE_640_360:
    return (isx ? "640x360" : "640_360");
  case IMAGE_320_240:
    return (isx ? "320x240" : "320_240");
  case IMAGE_960_600:
    return (isx ? "960x600" : "960_600");
  case IMAGE_480_300:
    return (isx ? "480x300" : "480_300");
  case IMAGE_1600_1200:
    return (isx ? "1600x1200" : "1600_1200");
  case IMAGE_1280_1080:
    return (isx ? "1280x1080" : "1280_1080");
  case IMAGE_1280_960:
    return (isx ? "1280x960" : "1280_960");
  case IMAGE_800_600:
    return (isx ? "800x600" : "800_600");
  case IMAGE_848_480:
    return (isx ? "848x480" : "848_480");
  case IMAGE_768_480:
    return (isx ? "768x480" : "768_480");
  case IMAGE_1280_480:
    return (isx ? "1280x480" : "1280_480");
  case IMAGE_1920_1080:
    return (isx ? "1920x1080" : "1920_1080");
  case IMAGE_960_1280:
    return (isx ? "960x1280" : "960_1280");
  case IMAGE_480_640:
    return (isx ? "480x640" : "480_640");
  case IMAGE_960_720:
    return (isx ? "960x720" : "960_720");
  }
  return "";
}*/
bool nvpfm::parseres(const char *res, NVPFM_IMAGE_SIZE *sensorres, NVPFM_IMAGE_SIZE *ispres, int *osensorfps, int *oispfps) {
  int sensorwidth, sensorheight, sensorfps, ispwidth, ispheight, ispfps;
  char type[16];
  if (7 == sscanf(res, "%dx%d_%d %s %dx%d_%d", &sensorwidth, &sensorheight, &sensorfps, type, &ispwidth, &ispheight, &ispfps)) {
    for (int i = 0; i < IMAGE_UNKNOWN; i++) {
      for (int j = 0; j < IMAGE_UNKNOWN; j++) {
        char tmpres[64];
        sprintf(tmpres, "%s_%d %s %s_%d", nvpfm::framesize2str((NVPFM_IMAGE_SIZE)i), sensorfps, type, nvpfm::framesize2str((NVPFM_IMAGE_SIZE)j), ispfps);
        if (0 == strcmp(tmpres, res)) {
          *sensorres = (NVPFM_IMAGE_SIZE)i;
          *ispres = (NVPFM_IMAGE_SIZE)j;
          *osensorfps = sensorfps;
          *oispfps = ispfps;
          return true;
        }
      }
    }
  }
  return false;
   /*else if (6 == sscanf(res, "%d_%d_%d->%d_%d_%d", &sensorwidth, &sensorheight, &sensorfps, &ispwidth, &ispheight, &ispfps)) {
    for (int i = 0; i < IMAGE_UNKNOWN; i++) {
      for (int j = 0; j < IMAGE_UNKNOWN; j++) {
        char tmpres[64];
        sprintf(tmpres, "%s_%d->%s_%d", getstrbyres((NVPFM_IMAGE_SIZE)i, false), sensorfps, getstrbyres((NVPFM_IMAGE_SIZE)j, false), ispfps);
        if (0 == strcmp(tmpres, res)) {
          *sensorres = (NVPFM_IMAGE_SIZE)i;
          *ispres = (NVPFM_IMAGE_SIZE)j;
          *osensorfps = sensorfps;
          *oispfps = ispfps;
          return true;
        }
      }
    }
    return false;
  } else {
    return false;
  }*/
}
/*"dsp_post_process_support":	[{
      "confidence":	"no",
      "light_filter":	"no",
      "good_feature":	"no",
      "bad_filter":	"no",
      "lk_param":	"no",
      "high_precision":	"yes",
      "repeated_texture_filter":	"no",
      "spatial_filter":	"no",
      "smear_filter":	"no",
      "speckle_filter":	"yes",
      "temporal_filter":	"no",
      "agg_process":	"no",
      "depth_high_light":	"no",
      "depth_light_strip":	"no",
      "textlight_filter":	"no",
      "depth_resize":	"no",
      "golden_diff":	"no",
      "depth_right_up_down_info":	"no"
    }]*/
NVPTL_RESULT nvpfm::get_devinfo(s_nvpfm_dev_info *pinfo) {
  memset(pinfo, 0, sizeof(s_nvpfm_dev_info));
  char *infostr = (char *)calloc(1, 64 * 1024);
  int reslen = 64 * 1024;
  if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_DEVICE_INFO, NULL, 0, infostr, &reslen, 2000)) {
  //  printf("deviceinfo:%s\n",infostr);
    Json::Value node;
    Json::Reader reader;
    reader.parse((const char *)infostr, node);
    strcpy(pinfo->sn, node["sn"].asString().c_str());
    strcpy(pinfo->product, node["product"].asString().c_str());
    strcpy(pinfo->cpu_net_type, node["cpu_net_type"].asString().c_str());
    strcpy(pinfo->comm_type, node["comm_type"].asString().c_str());
    memset(pinfo->postprocess, 0, sizeof(pinfo->postprocess));
    if (node.isMember("dsp_post_process_support")) {
      // cfy note: modify to multi channels!
      // cfy note: 2 depth product provide only one post-process.
      int len = node["dsp_post_process_support"].size();
      for (int i = 0; i < POSTPROCESSNUM; i++) {
        if (node["dsp_post_process_support"][0].isMember(postproc_descs[i].typestr)) {
          if (node["dsp_post_process_support"][0][postproc_descs[i].typestr] == "yes") {
            pinfo->postprocess[i] = TRUE;
          }
        }
      }
      if (!node["dsp_post_process_support"][0].isMember(postproc_descs[STATICCONFIG_E].typestr)) {
        pinfo->postprocess[STATICCONFIG_E] = TRUE;
      }
    }

    if (node.isMember("projectors")) {
      pinfo->projector_num = node["projectors"].size();
      for (int i = 0; i < node["projectors"].size(); i++) {
        strcpy(pinfo->projectors[i].type, node["projectors"][i]["type"].asString().c_str());
        pinfo->projectors[i].temperature[0] = node["projectors"][i]["temperature"][0].asFloat();
        pinfo->projectors[i].temperature[1] = node["projectors"][i]["temperature"][1].asFloat();
      }
      // for old version
      strcpy(pinfo->projector_type, node["projectors"][0]["type"].asString().c_str());
      pinfo->projector_temperature[0] = node["projectors"][0]["temperature"][0].asFloat();
      pinfo->projector_temperature[1] = node["projectors"][0]["temperature"][1].asFloat();
    } else if (node.isMember("projector_type")) {
      pinfo->projector_num = 1;
      strcpy(pinfo->projector_type, node["projector_type"].asString().c_str());
    }
    if (node.isMember("imus")) {
      pinfo->imu_num = node["imus"].size();
      for (int i = 0; i < node["imus"].size(); i++) {
        if (i == 0) {
          strcpy(pinfo->imu_type0, node["imus"][i]["type"].asString().c_str());
        } else if (i == 1) {
          strcpy(pinfo->imu_type1, node["imus"][i]["type"].asString().c_str());
        }
        pinfo->imus[i].channel = (E_NVPFM_IMU_CHANNEL)node["imus"][i]["channel"].asInt();
        strcpy(pinfo->imus[i].type, node["imus"][i]["type"].asString().c_str());
        strcpy(pinfo->imus[i].calib_image_channel, node["imus"][i]["calib_image_channel"].asString().c_str());
      }
    }
    // strcpy(pinfo->imu_type0, node["imu_type0"].asString().c_str());
    //   strcpy(pinfo->imu_type1, node["imu_type1"].asString().c_str());
    strcpy(pinfo->software_version, node["software_version"].asString().c_str());
    strcpy(pinfo->usb_speed, node["usb_speed"].asString().c_str());
    pinfo->cpu_temperature = node["cpu_temperature"].asFloat();

    if (node.isMember("projector0_temperature")) {
      pinfo->projector_temperature[0] = node["projector0_temperature"][0].asFloat();
      pinfo->projector_temperature[1] = node["projector0_temperature"][1].asFloat();
    } else if (node.isMember("ntc_temperature")) {
      pinfo->projector_temperature[0] = node["ntc_temperature"][0].asFloat();
      pinfo->projector_temperature[1] = node["ntc_temperature"][1].asFloat();
    }

    if (node.isMember("depth0")) {
      pinfo->depth[0] = TRUE;
      int index = 0;
      if (1 == sscanf(node["depth0"]["left"].asString().c_str(), "channel%d", &index)) {
        pinfo->sensor[index] = TRUE;
        pinfo->depth0[index] = TRUE;
        pinfo->image_dev_info.depth[0].left_channel = (E_NVPFM_SENSOR_CHANNEL)index;
      }

      if (1 == sscanf(node["depth0"]["right"].asString().c_str(), "channel%d", &index)) {
        pinfo->sensor[index] = TRUE;
        pinfo->depth0[index] = TRUE;
        pinfo->image_dev_info.depth[0].right_channel = (E_NVPFM_SENSOR_CHANNEL)index;
      }
    }
    if (node.isMember("depth1")) {
      pinfo->depth[1] = TRUE;
      int index = 0;
      if (1 == sscanf(node["depth1"]["left"].asString().c_str(), "channel%d", &index)) {
        pinfo->sensor[index] = TRUE;
        pinfo->depth1[index] = TRUE;
        pinfo->image_dev_info.depth[1].left_channel = (E_NVPFM_SENSOR_CHANNEL)index;
      }

      if (1 == sscanf(node["depth1"]["right"].asString().c_str(), "channel%d", &index)) {
        pinfo->sensor[index] = TRUE;
        pinfo->depth1[index] = TRUE;
        pinfo->image_dev_info.depth[1].right_channel = (E_NVPFM_SENSOR_CHANNEL)index;
      }
    }
    if (node.isMember("rgb0")) {
      pinfo->rgb[0] = TRUE;
      int index = 0;
      if (1 == sscanf(node["rgb0"]["channel"].asString().c_str(), "channel%d", &index)) {
        pinfo->sensor[index] = TRUE;
        pinfo->rgb0[index] = TRUE;
        pinfo->image_dev_info.rgb[0].channel = (E_NVPFM_SENSOR_CHANNEL)index;
      }
    }
    if (node.isMember("rgb1")) {
      pinfo->rgb[1] = TRUE;
      int index = 0;
      if (1 == sscanf(node["rgb1"]["channel"].asString().c_str(), "channel%d", &index)) {
        pinfo->sensor[index] = TRUE;
        pinfo->rgb1[index] = TRUE;
        pinfo->image_dev_info.rgb[1].channel = (E_NVPFM_SENSOR_CHANNEL)index;
      }
    }

    int irindex = 0;
    if (node[node["depth0"]["left"].asString() + "_support"][0].isString()) {
      for (int i = 0; i < node[node["depth0"]["left"].asString() + "_support"].size(); i++) {
        NVPFM_IMAGE_SIZE sensorimagesize, ispimagesize;
        int sensorfps = 30, ispfps = 30;
        if (nvpfm::parseres(node[node["depth0"]["left"].asString() + "_support"][i].asString().c_str(),
                            &sensorimagesize, &ispimagesize, &sensorfps, &ispfps)) {

          if (irindex < MAXSUPPORTEDRES) {
            pinfo->irsupported[irindex].valid = 1;
            pinfo->irsupported[irindex].sensorres = sensorimagesize;
            pinfo->irsupported[irindex].ispres = ispimagesize;
            pinfo->irsupported[irindex].fps = ispfps;

            irindex++;
          }
        }
      }
    } else {
      for (int i = 0; i < node[node["depth0"]["left"].asString() + "_support"].size(); i++) {
        Json::Value tmpnode = node[node["depth0"]["left"].asString() + "_support"][i];

        Json::Value::Members member = tmpnode.getMemberNames();
        for (Json::Value::Members::iterator iter = member.begin(); iter != member.end(); ++iter) {
          Json::Value mytmparray = tmpnode[*iter];
          for (int j = 0; j < mytmparray.size(); j++) {
            NVPFM_IMAGE_SIZE sensorimagesize, ispimagesize;
            int sensorfps, ispfps;
            if (parseoldres(*iter, &sensorimagesize, &sensorfps) &&
                parseoldres(mytmparray[j].asString(), &ispimagesize, &ispfps)) {

              if (irindex < MAXSUPPORTEDRES) {
                pinfo->irsupported[irindex].valid = 1;
                pinfo->irsupported[irindex].sensorres = sensorimagesize;
                pinfo->irsupported[irindex].ispres = ispimagesize;
                pinfo->irsupported[irindex].fps = ispfps;

                irindex++;
              }
            }
          }
        }
      }
    }

    int colorindex = 0;
    if (node[node["rgb0"]["channel"].asString() + "_support"][0].isString()) {
      for (int i = 0; i < node[node["rgb0"]["channel"].asString() + "_support"].size(); i++) {
        NVPFM_IMAGE_SIZE sensorimagesize, ispimagesize;
        int sensorfps = 30, ispfps = 30;
        if (nvpfm::parseres(node[node["rgb0"]["channel"].asString() + "_support"][i].asString().c_str(), &sensorimagesize, &ispimagesize, &sensorfps, &ispfps)) {

          if (colorindex < MAXSUPPORTEDRES) {
            pinfo->colorsupported[colorindex].valid = 1;
            pinfo->colorsupported[colorindex].sensorres = sensorimagesize;
            pinfo->colorsupported[colorindex].ispres = ispimagesize;
            pinfo->colorsupported[colorindex].fps = ispfps;

            colorindex++;
          }
        }
      }
    } else {
      for (int i = 0; i < node[node["rgb0"]["channel"].asString() + "_support"].size(); i++) {
        Json::Value tmpnode = node[node["rgb0"]["channel"].asString() + "_support"][i];

        Json::Value::Members member = tmpnode.getMemberNames();
        for (Json::Value::Members::iterator iter = member.begin(); iter != member.end(); ++iter) {
          Json::Value mytmparray = tmpnode[*iter];
          for (int j = 0; j < mytmparray.size(); j++) {
            NVPFM_IMAGE_SIZE sensorimagesize, ispimagesize;
            int sensorfps, ispfps;
            if (parseoldres(*iter, &sensorimagesize, &sensorfps) &&
                parseoldres(mytmparray[j].asString(), &ispimagesize, &ispfps)) {

              if (colorindex < MAXSUPPORTEDRES) {
                pinfo->colorsupported[colorindex].valid = 1;
                pinfo->colorsupported[colorindex].sensorres = sensorimagesize;
                pinfo->colorsupported[colorindex].ispres = ispimagesize;
                pinfo->colorsupported[colorindex].fps = ispfps;

                colorindex++;
              }
            }
          }
        }
      }
    }

    if (node.isMember("leds")) {
      pinfo->led_num = node["leds"].size();
      for (int i = 0; i < pinfo->led_num; i++) {
        pinfo->leds[i].channel = (E_NVPFM_LED_CHANNEL)node["leds"][i]["channel"].asInt();
        strcpy(pinfo->leds[i].type, node["leds"][i]["type"].asString().c_str());
      }
    }

    if (node.isMember("support_slave_mode")) {
      if (0 == strcmp(node["support_slave_mode"].asString().c_str(), "yes")) {
        pinfo->issupportslavemode = true;
      }
    }
    for (int index = 0; index < SENSOR_CHANNEL_UNKNOWN; index++) {
      E_NVPFM_SENSOR_CHANNEL channel = (E_NVPFM_SENSOR_CHANNEL)index;
      char channelsrt[64];
      sprintf(channelsrt, "channel%d_format_support", index);
      if (node.isMember(channelsrt)) {
        for (int i = 0; i < node[channelsrt].size(); i++) {
          strcpy(pinfo->channelforamt[index][i], node[channelsrt][i].asString().c_str());
        }
      }
    }

    free(infostr);
    memcpy(&m_devinfo, pinfo, sizeof(s_nvpfm_dev_info));
    return NVPTL_OK;
  }
  free(infostr);
  return NVPTL_FAILED;
}

E_NVPFM_SENSOR_CHANNEL nvpfm::get_sensor_channel_by_stream(EM_AVAILABE_STREAM_TYPE streamtype) {
  E_NVPFM_SENSOR_CHANNEL channel = SENSOR_CHANNEL_UNKNOWN;
  s_nvpfm_dev_info devinfo;
  if (0 == strcmp(m_devinfo.product, "") && NVPTL_OK != get_devinfo(&devinfo)) {
    return channel;
  }
  switch (streamtype) {
  case STREAM_LEFT0:
    channel = m_devinfo.image_dev_info.depth[0].left_channel;
    break;
  case STREAM_LEFT1:
    channel = m_devinfo.image_dev_info.depth[1].left_channel;
    break;
  case STREAM_RIGHT0:
    channel = m_devinfo.image_dev_info.depth[0].right_channel;
    break;
  case STREAM_RIGHT1:
    channel = m_devinfo.image_dev_info.depth[1].right_channel;
    break;
  case STREAM_RGB0:
    channel = m_devinfo.image_dev_info.rgb[0].channel;
    break;
  case STREAM_RGB1:
    channel = m_devinfo.image_dev_info.rgb[1].channel;
    break;
  defatu:
    break;
  }
  return channel;
}

NVPFM_IMAGE_TYPE nvpfm::get_image_channel_by_sensor_channel(E_NVPFM_SENSOR_CHANNEL channel, bool iscalibrated) {
  NVPFM_IMAGE_TYPE type = IMAGE_TYPE_UNKNOWN;
  switch (channel) {
  case CHANNEL0:
    type = iscalibrated ? IMAGE_CHANNEL0_CALIBRATED : IMAGE_CHANNEL0_ORIGNAL;
    break;
  case CHANNEL1:
    type = iscalibrated ? IMAGE_CHANNEL1_CALIBRATED : IMAGE_CHANNEL1_ORIGNAL;
    break;
  case CHANNEL2:
    type = iscalibrated ? IMAGE_CHANNEL2_CALIBRATED : IMAGE_CHANNEL2_ORIGNAL;
    break;
  case CHANNEL3:
    type = iscalibrated ? IMAGE_CHANNEL3_CALIBRATED : IMAGE_CHANNEL3_ORIGNAL;
    break;
  default:
    break;
  }
  return type;
}

NVPFM_IMAGE_TYPE nvpfm::get_image_channel_by_stream(s_nvpfm_app_run_config runcfg, EM_AVAILABE_STREAM_TYPE streamtype) {
  if (STREAM_DEPTH0 == streamtype) {
    return IMAGE_DEPTH0;
  } else if (STREAM_DEPTH1 == streamtype) {
    return IMAGE_DEPTH1;
  }
  NVPFM_IMAGE_TYPE type = IMAGE_TYPE_UNKNOWN;
  E_NVPFM_SENSOR_CHANNEL channel = get_sensor_channel_by_stream(streamtype);
  if (SENSOR_CHANNEL_UNKNOWN == channel) {
    return type;
  }

  bool iscalibrated = false;
  switch (streamtype) {
  case STREAM_LEFT0:
    iscalibrated = (runcfg.depth_mode[0] == DEPTH_L_R_CALIBRATED ||
                    runcfg.depth_mode[0] == DEPTH_L_CALIBRATED ||
                    runcfg.depth_mode[0] == L_R_CALIBRATED);
    type = get_image_channel_by_sensor_channel(channel, iscalibrated);
    break;
  case STREAM_LEFT1:
    iscalibrated = (runcfg.depth_mode[1] == DEPTH_L_R_CALIBRATED ||
                    runcfg.depth_mode[1] == DEPTH_L_CALIBRATED ||
                    runcfg.depth_mode[1] == L_R_CALIBRATED);
    type = get_image_channel_by_sensor_channel(channel, iscalibrated);
    break;
  case STREAM_RIGHT0:
    iscalibrated = (runcfg.depth_mode[0] == DEPTH_L_R_CALIBRATED ||
                    runcfg.depth_mode[0] == L_R_CALIBRATED);
    type = get_image_channel_by_sensor_channel(channel, iscalibrated);
    break;
  case STREAM_RIGHT1:
    iscalibrated = (runcfg.depth_mode[1] == DEPTH_L_R_CALIBRATED ||
                    runcfg.depth_mode[1] == L_R_CALIBRATED);
    type = get_image_channel_by_sensor_channel(channel, iscalibrated);
    break;
  case STREAM_RGB0:
    iscalibrated = runcfg.rgb_calibrated[0];
    type = get_image_channel_by_sensor_channel(channel, iscalibrated);
    break;
  case STREAM_RGB1:
    iscalibrated = runcfg.rgb_calibrated[1];
    type = get_image_channel_by_sensor_channel(channel, iscalibrated);
    break;
  defatu:
    break;
  }
  return type;
}

void nvpfm::get_spatial_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_SPATIAL_FILTER, callback, userdata);
}
void nvpfm::get_temporal_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_TEMPORAL_FILTER, callback, userdata);
}
void nvpfm::get_speckle_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_SPECKLE_FILTER, callback, userdata);
}
void nvpfm::get_highlight_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_HIGHLIGHT_FILTER, callback, userdata);
}
void nvpfm::get_lightfilter_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_LIGHT_FILTER, callback, userdata);
}
void nvpfm::get_badfilter_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_BAD_FILTER, callback, userdata);
}
/*
void nvpfm::get_edgefilter_async(ASYNCCALLBACK callback, void* userdata)
{
  nvpfm_get_async(m_handle, NVPFM_EDGE_FILTER, callback, userdata);
}*/
void nvpfm::get_highprecision_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_HIGH_PRECISION, callback, userdata);
}
void nvpfm::get_repeatfilter_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_REPEAT_TEXTURE_FILTER, callback, userdata);
}
void nvpfm::get_depthconfig_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_DEPTH_CONFIG, callback, userdata);
}
NVPTL_RESULT nvpfm::get_irexposure(s_nvpfm_get_sensor_exposure_ret *pret) {
  s_nvpfm_get_sensor_exposure getdata;
  getdata.channel = CHANNEL0;
  int retlen = sizeof(s_nvpfm_get_sensor_exposure_ret);
  return nvpfm_get(m_handle, NVPFM_IR_EXPOSURE, &getdata, sizeof(s_nvpfm_get_sensor_exposure), pret, &retlen, 2000);
}
NVPTL_RESULT nvpfm::get_rgbexposure(s_nvpfm_get_sensor_exposure_ret *pret) {
  s_nvpfm_get_sensor_exposure getdata;
  getdata.channel = CHANNEL2;
  int retlen = sizeof(s_nvpfm_get_sensor_exposure_ret);
  return nvpfm_get(m_handle, NVPFM_RGB_EXPOSURE, &getdata, sizeof(s_nvpfm_get_sensor_exposure), pret, &retlen, 2000);
}
NVPTL_RESULT nvpfm::get_lowtextureremoval(s_nvpfm_lowtexture_removal *pcfg){
  int retlen = sizeof(s_nvpfm_depth_infer_process_param);
  s_nvpfm_depth_infer_process_param infoparam;
  NVPTL_RESULT ret=nvpfm_get(m_handle, NVPFM_INFER_DEPTH_PROCESS, NULL, 0, &infoparam, &retlen, 2000);
  if(ret==NVPTL_OK){
    *pcfg=infoparam.lowtexture_removal;
  }
  return ret;
}
NVPTL_RESULT nvpfm::set_lowtextureremoval(s_nvpfm_set_lowtexture_removal *pcfg){
  int retlen = sizeof(s_nvpfm_depth_infer_process_param);
  s_nvpfm_depth_infer_process_param infoparam;
  NVPTL_RESULT ret=nvpfm_get(m_handle, NVPFM_INFER_DEPTH_PROCESS, NULL, 0, &infoparam, &retlen, 2000);
  if(ret==NVPTL_OK){
	  s_nvpfm_set_depth_infer_process_param_return retvalue;
	  int retlen = sizeof(s_nvpfm_set_depth_infer_process_param_return);
	  s_nvpfm_set_depth_infer_process_param setinfoparam;
	  setinfoparam.info=infoparam; 
	  setinfoparam.channel=pcfg->channel;
	  setinfoparam.info.lowtexture_removal=pcfg->info;
	  setinfoparam.changeflag=(((NVP_U64)0x1)<<E_LOWTEXTURE_REMOVAL);
	  NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_INFER_DEPTH_PROCESS, &setinfoparam, sizeof(s_nvpfm_set_depth_infer_process_param), &retvalue, &retlen, 2000);
	  if (ret == NVPTL_OK&&retvalue.ret.ret==0) {
	    return ret;
	  }
  }
  return NVPTL_FAILED;
}
void nvpfm::get_irexposure_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_IR_EXPOSURE, callback, userdata);
}
void nvpfm::get_rgbexposure_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_RGB_EXPOSURE, callback, userdata);
}
NVPTL_RESULT nvpfm::get_runcfg(s_nvpfm_get_app_run_config_ret *pdata) {
  int retlen = sizeof(s_nvpfm_get_app_run_config_ret);
  return nvpfm_get(m_handle, NVPFM_RUN_MODE, NULL, 0, pdata, &retlen, 2000);
}
void nvpfm::get_runcfg_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_RUN_MODE, callback, userdata);
}
/*void nvpfm::get_cnnmodel_async(ASYNCCALLBACK callback, void* userdata)
{
  nvpfm_get_async(m_handle, NVPFM_CNN_MODEL, callback, userdata);
}
void nvpfm::get_rgbexposure_async(ASYNCCALLBACK callback, void* userdata)
{
  nvpfm_get_async(m_handle, NVPFM_RGB_EXPOSURE, callback, userdata);
}*/
void nvpfm::get_smearfilter_async(ASYNCCALLBACK callback, void *userdata) {
  nvpfm_get_async(m_handle, NVPFM_SMEAR_FILTER, callback, userdata);
}
void nvpfm::set_roi(NVPTL_USBHeaderDataPacket *depthframe, ROI roi) {
  (void)depthframe;
  m_roi = roi;
}

/*
typedef struct{
    float left_ir_focus[2];//0:fx,1:fy
    float left_ir_photocenter[2];//0:px,1:py
    float right_ir_focus[2];
    float right_ir_photocenter[2];
    float color_focus[2];
    float color_photocenter[2];
    float left2right_matrix[12];
    float left2color_matrix[12];
  }s_nvpfm_camera_param;
  typedef struct
  {
    int result;       //-1获取失败   0-获取成功
    float param[5];  //fx fy u0 v0 baseline
    E_NVPFM_ROTATE rotate;
    float param_after_rotate[5];  //fx fy u0 v0 baseline after rotate
  }s_get_depth_param_result;
*/
NVPTL_RESULT nvpfm::get_camera_param(s_nvpfm_camera_param *param) {
  s_nvpfm_dev_info devinfo;
  if (NVPTL_OK != get_devinfo(&devinfo)) {
    nvpfm_warn_printf("fail to get device info!\n");
    return NVPTL_FAILED;
  }
  s_nvpfm_get_sensor_config_ret mycfg;
  if (NVPTL_OK == get_sensorcfg(&mycfg)) {
    if (mycfg.ret == 0) {
      s_get_depth_param depthparam;

      depthparam.channel = NVPFM_DEPTH_CHANNEL0;
      depthparam.frame_size = mycfg.config.isp_frame_size[0];

      framesize2int(depthparam.frame_size, &param->irwidth, &param->irheight);
      s_get_depth_param_result result;
      int reslen = sizeof(s_get_depth_param_result);
      if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_DEPTH_PARAM, &depthparam, sizeof(s_get_depth_param), &result, &reslen, 2000)) {
        if (result.result == 0) {
          param->depthparam = result;
          /*if (param->irwidth > param->irheight) {
            param->left_ir_focus[0] = result.param[0];
            param->left_ir_focus[1] = result.param[1];
            param->left_ir_photocenter[0] = result.param[2];
            param->left_ir_photocenter[1] = result.param[3];
            param->left2right_matrix[9] = -result.param[4];
          } else {
            param->left_ir_focus[0] = result.param_after_rotate[0];
            param->left_ir_focus[1] = result.param_after_rotate[1];
            param->left_ir_photocenter[0] = result.param_after_rotate[2];
            param->left_ir_photocenter[1] = result.param_after_rotate[3];
            param->left2right_matrix[9] = -result.param_after_rotate[4];
            int tmpwidth = param->irwidth;
            param->irwidth = param->irheight;
            param->irheight = tmpwidth;
          }*/

          nvpfm_debug_printf("ok to get depth param!\n");
        }
      }

      s_get_sensor_channel_inner_param leftparam;
      leftparam.channel = CHANNEL0;
      leftparam.frame_size = mycfg.config.isp_frame_size[0];
      s_get_sensor_channel_inner_param_result leftresult;
      int leftreslen = sizeof(s_get_sensor_channel_inner_param_result);
      if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_INNER_PARAM, &leftparam, sizeof(s_get_sensor_channel_inner_param), &leftresult, &leftreslen, 2000)) {
        if (leftresult.result == 0) {
          param->left_ir_focus[0] = leftresult.param[0];
          param->left_ir_focus[1] = leftresult.param[1];
          param->left_ir_photocenter[0] = leftresult.param[2];
          param->left_ir_photocenter[1] = leftresult.param[3];
          nvpfm_debug_printf("ok to get left inner param!\n");
        } else {
          nvpfm_debug_printf("fail to get left innner param!\n");
        }
      }

      s_get_sensor_channel_inner_param rightparam;
      rightparam.channel = CHANNEL1;
      rightparam.frame_size = mycfg.config.isp_frame_size[1];
      s_get_sensor_channel_inner_param_result rightresult;
      int rightreslen = sizeof(s_get_sensor_channel_inner_param_result);
      if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_INNER_PARAM, &rightparam, sizeof(s_get_sensor_channel_inner_param), &rightresult, &rightreslen, 2000)) {
        if (rightresult.result == 0) {
          param->right_ir_focus[0] = rightresult.param[0];
          param->right_ir_focus[1] = rightresult.param[1];
          param->right_ir_photocenter[0] = rightresult.param[2];
          param->right_ir_photocenter[1] = rightresult.param[3];
          nvpfm_debug_printf("ok to get right inner param!\n");
        } else {
          nvpfm_debug_printf("fail to get right innner param!\n");
        }
      }

      s_get_sensor_channel_inner_param colorparam;
      colorparam.channel = CHANNEL2;
      colorparam.frame_size = mycfg.config.isp_frame_size[2];
      s_get_sensor_channel_inner_param_result colorresult;
      int colorreslen = sizeof(s_get_sensor_channel_inner_param_result);
      if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_INNER_PARAM, &colorparam, sizeof(s_get_sensor_channel_inner_param), &colorresult, &colorreslen, 2000)) {
        if (colorresult.result == 0) {
          param->color_focus[0] = colorresult.param[0];
          param->color_focus[1] = colorresult.param[1];
          param->color_photocenter[0] = colorresult.param[2];
          param->color_photocenter[1] = colorresult.param[3];
          framesize2int(colorparam.frame_size, &param->rgbwidth, &param->rgbheight);

          nvpfm_debug_printf("ok to get color inner param!");
        } else {
          nvpfm_debug_printf("fail to get color innner param!");
        }
      }

      s_get_channel_pos_param left2rightparam;
      left2rightparam.channel_x = CHANNEL0;
      left2rightparam.channel_y = CHANNEL1;
      s_get_channel_pos_param_result left2rightresult;
      int left2rightreslen = sizeof(s_get_channel_pos_param_result);
      if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_OUTER_PARAM, &left2rightparam, sizeof(s_get_channel_pos_param), &left2rightresult, &left2rightreslen, 2000)) {
        if (left2rightresult.result == 0) {
          memcpy(&param->left2right_matrix[0], &left2rightresult.param[0], 12 * sizeof(float));
          nvpfm_debug_printf("ok to get left2right outer param!\n");
        }
      }

      if (devinfo.rgb[0]) {
        s_get_channel_pos_param left2colorparam;
        left2colorparam.channel_x = CHANNEL0;
        left2colorparam.channel_y = CHANNEL2;
        s_get_channel_pos_param_result left2colorresult;
        int left2colorreslen = sizeof(s_get_channel_pos_param_result);
        if (NVPTL_OK == nvpfm_get(m_handle, NVPFM_OUTER_PARAM, &left2colorparam, sizeof(s_get_channel_pos_param), &left2colorresult, &left2colorreslen, 2000)) {
          if (left2colorresult.result == 0) {
            memcpy(&param->left2color_matrix[0], &left2colorresult.param[0], 12 * sizeof(float));
            nvpfm_info_printf("ok to get left2color outer param!\n");
          } else {
            nvpfm_warn_printf("fail to get left2color outer param!\n");
          //  return NVPTL_FAILED;
          }
        } else {
          nvpfm_warn_printf("fail to call get outer param of left2color!\n");
        //  return NVPTL_FAILED;
        }
      }

      return NVPTL_OK;
    }
  }
  return NVPTL_FAILED;
}
NVPTL_RESULT nvpfm::send_update_tftp() {
  return nvpfm_send_update_tftp(m_handle);
}
NVPTL_RESULT nvpfm::stop_leftir() {
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_stop(m_handle, NVPFM_STREAM_DEPTH_LEFTIR))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::stop_rightir() {
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_stop(m_handle, NVPFM_STREAM_DEPTH_RIGHTIR))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::stop_depth() {
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_stop(m_handle, NVPFM_STREAM_DEPTH))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::stop_rgb() {
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_stop(m_handle, NVPFM_STREAM_RGB))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}
NVPTL_RESULT nvpfm::stop_imu() {
  for (int i = 0; i < 5; i++) {
    if (NVPTL_OK == nvpfm_stop(m_handle, NVPFM_STREAM_IMU))
      return NVPTL_OK;
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_transfer_config(s_get_transfer_config_ret *pdata) {
  for (int i = 0; i < 2; i++) {
    int retlen = sizeof(s_get_transfer_config_ret);
    NVPTL_RESULT ret = nvpfm_get(m_handle, NVPFM_TRANSFER_CONFIG, NULL, 0, pdata, &retlen, 2000);
    if (ret == NVPTL_OK) {
      if (pdata->ret == 0) {
        return ret;
      }
    }
    COMMONUSLEEP(500 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_transfer_config(s_set_transfer_config *pdata) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(m_handle, NVPFM_TRANSFER_CONFIG, pdata, sizeof(s_set_transfer_config), &retvalue, &retlen, 2000);
    if (ret == NVPTL_OK) {
      return ret;
    }
    COMMONUSLEEP(50 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::get_led_config(s_nvpfm_get_led_ret *pdata, E_NVPFM_LED_CHANNEL channel) {
  for (int i = 0; i < 5; i++) {
    int retlen = sizeof(s_nvpfm_get_led_ret);
    s_nvpfm_get_led getcfg;
    getcfg.channel = channel;
    NVPTL_RESULT ret = nvpfm_get(
        m_handle,
        NVPFM_LED_CONFIG,
        &getcfg,
        sizeof(s_nvpfm_get_led),
        pdata,
        &retlen,
        2000);
    if (NVPTL_OK == ret) {
      if (0 == pdata->ret) {
        return ret;
      }
    }
    COMMONUSLEEP(500 * 1000);
  }
  return NVPTL_TIMEOUT;
}

NVPTL_RESULT nvpfm::set_led_config(s_nvpfm_set_led *pdata) {
  for (int i = 0; i < 5; i++) {
    s_nvpfm_cmd_set_ret retvalue;
    int retlen = sizeof(s_nvpfm_cmd_set_ret);
    NVPTL_RESULT ret = nvpfm_set(
        m_handle,
        NVPFM_LED_CONFIG,
        pdata,
        sizeof(s_nvpfm_set_led),
        &retvalue,
        &retlen,
        2000);
    if (NVPTL_OK == ret) {
      return ret;
    }
    COMMONUSLEEP(500 * 1000);
  }
  return NVPTL_TIMEOUT;
}

#include <stdlib.h>
#include "libyuv/cpu_id.h"
#include "libyuv/rotate.h"
using namespace libyuv;
static __inline int Abs(int v) {
  return v >= 0 ? v : -v;
}
#define align_buffer_page_end(var, size)                                 \
  uint8_t *var##_mem =                                                   \
      reinterpret_cast<uint8_t *>(malloc(((size) + 4095 + 63) & ~4095)); \
  uint8_t *var = reinterpret_cast<uint8_t *>(                            \
      (intptr_t)(var##_mem + (((size) + 4095 + 63) & ~4095) - (size)) & ~63)

#define free_aligned_buffer_page_end(var) \
  free(var##_mem);                        \
  var = 0

static void NV12TestRotate(int src_width,
                           int src_height,
                           int dst_width,
                           int dst_height,
                           libyuv::RotationMode mode, uint8_t *src_buffer, uint8_t *dst_buffer) {
  if (src_width < 1) {
    src_width = 1;
  }
  if (src_height == 0) { // allow negative for inversion test.
    src_height = 1;
  }
  if (dst_width < 1) {
    dst_width = 1;
  }
  if (dst_height < 1) {
    dst_height = 1;
  }
  int src_nv12_y_size = src_width * Abs(src_height);
  int src_nv12_uv_size =
      ((src_width + 1) / 2) * ((Abs(src_height) + 1) / 2) * 2;
  int src_nv12_size = src_nv12_y_size + src_nv12_uv_size;
  align_buffer_page_end(src_nv12, src_nv12_size);
  memcpy(src_nv12, src_buffer, src_nv12_y_size + src_nv12_uv_size);
  /*for (int i = 0; i < src_nv12_size; ++i) {
    src_nv12[i] = fastrand() & 0xff;
  }*/

  int dst_i420_y_size = dst_width * dst_height;
  int dst_i420_uv_size = ((dst_width + 1) / 2) * ((dst_height + 1) / 2);
  int dst_i420_size = dst_i420_y_size + dst_i420_uv_size * 2;
  align_buffer_page_end(dst_i420_c, dst_i420_size);
  align_buffer_page_end(dst_i420_opt, dst_i420_size);
  memset(dst_i420_c, 2, dst_i420_size);
  memset(dst_i420_opt, 3, dst_i420_size);
  // int disable_cpu_flags=1;
  int enable_cpu_flags = -1;
  /*  MaskCpuFlags(disable_cpu_flags);  // Disable all CPU optimization.
    NV12ToI420Rotate(src_nv12, src_width, src_nv12 + src_nv12_y_size,
                     (src_width + 1) & ~1, dst_i420_c, dst_width,
                     dst_i420_c + dst_i420_y_size, (dst_width + 1) / 2,
                     dst_i420_c + dst_i420_y_size + dst_i420_uv_size,
                     (dst_width + 1) / 2, src_width, src_height, mode);
  */
  MaskCpuFlags(enable_cpu_flags); // Enable all CPU optimization.
  // for (int i = 0; i < benchmark_iterations; ++i) {
  NV12ToI420Rotate(src_nv12, src_width, src_nv12 + src_nv12_y_size,
                   (src_width + 1) & ~1, dst_i420_opt, dst_width,
                   dst_i420_opt + dst_i420_y_size, (dst_width + 1) / 2,
                   dst_i420_opt + dst_i420_y_size + dst_i420_uv_size,
                   (dst_width + 1) / 2, src_width, src_height, mode);
  //}
  memcpy(dst_buffer, dst_i420_opt, dst_i420_y_size + dst_i420_uv_size * 2);
  // Rotation should be exact.
  /* for (int i = 0; i < dst_i420_size; ++i) {
     EXPECT_EQ(dst_i420_c[i], dst_i420_opt[i]);
   }
  */
  free_aligned_buffer_page_end(dst_i420_c);
  free_aligned_buffer_page_end(dst_i420_opt);
  free_aligned_buffer_page_end(src_nv12);
}

void nvpfm::grayscale_rotate_90(NVPFM_USB_IMAGE_HEADER *pheader) {
  // NVPFM_USB_IMAGE_HEADER* pheader=(NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
  int height = pheader->height;
  int width = pheader->width;
  uint8_t *raw = (uint8_t *)((uint8_t *)pheader + sizeof(NVPFM_USB_IMAGE_HEADER));
  Map<Matrix<uint8_t, Dynamic, Dynamic, RowMajor>> mymat(raw, height, width);
  Matrix<uint8_t, Dynamic, Dynamic, RowMajor> tmpmat = mymat.transpose();
  tmpmat.rowwise().reverseInPlace();
  memcpy(raw, tmpmat.data(), width * height);
  pheader->width = height;
  pheader->height = width;
}
void nvpfm::grayscale_rotate_270(NVPFM_USB_IMAGE_HEADER *pheader) {
  // NVPFM_USB_IMAGE_HEADER* header=(NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
  int height = pheader->height;
  int width = pheader->width;
  uint8_t *raw = (uint8_t *)((uint8_t *)pheader + sizeof(NVPFM_USB_IMAGE_HEADER));
  Map<Matrix<uint8_t, Dynamic, Dynamic, RowMajor>> mymat(raw, height, width);
  mymat.rowwise().reverseInPlace();
  Matrix<uint8_t, Dynamic, Dynamic, RowMajor> tmpmat = mymat.transpose();
  memcpy(raw, tmpmat.data(), width * height);
  // mymat.rowwise().reverse().transpose();
  // memcpy(raw,mymat.data(),width*height);
  pheader->width = height;
  pheader->height = width;
}

void nvpfm::nv12_rotate_90_i420(NVPFM_USB_IMAGE_HEADER *pheader) {
  // NVPFM_USB_IMAGE_HEADER* pheader=(NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
  uint8_t *pdata = (uint8_t *)((uint8_t *)pheader + sizeof(NVPFM_USB_IMAGE_HEADER));
  uint32_t width = pheader->width;
  uint32_t height = pheader->height;
  pheader->width = height;
  pheader->height = width;

  NV12TestRotate(width, height, height, width,
                 kRotate90, pdata, pdata);
}

void nvpfm::nv12_rotate_270_i420(NVPFM_USB_IMAGE_HEADER *pheader) {
  // NVPFM_USB_IMAGE_HEADER* pheader=(NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
  uint8_t *pdata = (uint8_t *)((uint8_t *)pheader + sizeof(NVPFM_USB_IMAGE_HEADER));
  uint32_t width = pheader->width;
  uint32_t height = pheader->height;
  pheader->width = height;
  pheader->height = width;

  NV12TestRotate(width, height, height, width,
                 kRotate270, pdata, pdata);
}

void nvpfm::get_cnn_data(NVPTL_USBHeaderDataPacket *tmppack, s_nvpfm_cnn_data *pdata) {
  Json::Value node;
  Json::Reader reader;
  reader.parse((const char *)tmppack->data, node);

  int16_t *buffers[3];
  buffers[0] = (int16_t *)(((uint8_t *)tmppack->data) + node["output0"]["offset"].asInt());
  buffers[1] = (int16_t *)(((uint8_t *)tmppack->data) + node["output1"]["offset"].asInt());
  buffers[2] = (int16_t *)(((uint8_t *)tmppack->data) + node["output2"]["offset"].asInt());
  uint64_t timestamp = node["timestamp"].asInt64();
  pdata->timestamp = timestamp;
  if (node.isMember("model_type")) {
    if (node["model_type"].asInt() == 0) {
      pdata->type = CNN_YOLOX_TYPE;
      yolox_post_process(buffers, pdata);
    } else if (node["model_type"].asInt() == 1) {
      pdata->type = CNN_HAND_TYPE;
      hand_post_process(buffers, pdata);
    }
  }
}

LABELINFO nvpfm::get_cnn_label_by_index(int index, EM_CNN_TYPE type) {
  switch (type) {
  case CNN_YOLOX_TYPE:
    return get_yolox_label_by_index(index);
  case CNN_HAND_TYPE:
    return get_hand_label_by_index(index);
  default:
    break;
  }
}

std::string nvpfm::get_usbtype_by_spec(EM_NVPTL_USB_SPEC spectype) {
  std::map<EM_NVPTL_USB_SPEC, std::string> usb_spec_names = {
      {USB_UNDEFINED, "Undefined"},
      {USB1_TYPE, "1.0"},
      {USB1_1_TYPE, "1.1"},
      {USB2_TYPE, "2.0"},
      {USB2_01_TYPE, "2.01"},
      {USB2_1_TYPE, "2.1"},
      {USB3_TYPE, "3.0"},
      {USB3_1_TYPE, "3.1"},
      {USB3_2_TYPE, "3.2"}};
  return usb_spec_names.at(spectype);
}

transform_matrix *depthtransformer::get_trans_matrix(int w, int h) {
  if (tm.width != w || tm.height != h) {
    tm.width = w;
    tm.height = h;
    tm.col_mat.resize(h, w);
    tm.row_mat.resize(h, w);
    for (uint32_t i = 0; i < h; ++i) {
      for (uint32_t j = 0; j < w; ++j) {
        // 记住每个元素的列信息
        tm.col_mat(i, j) = float(j);
        // 记住每个元素的行信息
        tm.row_mat(i, j) = float(i);
      }
    }
  }
  return &tm;
}

void depthtransformer::calculatecolortable() {
  if (NULL == m_colortable) {
    m_colortable = (MYRGB *)calloc(1, sizeof(MYRGB) * 65536);
  }
  memset(m_colortable, 0, sizeof(MYRGB) * 65536);

  for (int i = (int)m_depthDistanceInfo.minDepth; i < 65536; i++) { // 0.2m-5m
    if (i > (int)m_depthDistanceInfo.maxDepth) {
      unsigned char y = 0;
      unsigned char u = 0;
      unsigned char v = 0;
      int result = nvpfm_getyuvfromindex(254, &y, &u, &v);
      float fr = y + 1.4075 * (v - 128);

      float fg = y - 0.3455 * (u - 128) - 0.7169 * (v - 128);

      float fb = y + 1.779 * (u - 128);
      /*		static FILE* fp = fopen("table.txt", "wt");
      fprintf(fp,"table:%f,%f,%f\n", fr, fg, fb);
      fflush(fp);
      static FILE* fp = fopen("table.txt", "wt");
      fprintf(fp, ">max,:%f,%f,%f\n", fr, fg, fb);
      fflush(fp);*/
      m_colortable[i].r = (unsigned char)fr;
      m_colortable[i].g = (unsigned char)fg;
      m_colortable[i].b = (unsigned char)fb;
      continue;
    }
    int effectindex = (int)((float)(i - (int)m_depthDistanceInfo.minDepth) * 255.0 / (float)(m_depthDistanceInfo.maxDepth - m_depthDistanceInfo.minDepth));
    //	if (effectindex > 255)effectindex = 255;
    if (effectindex == 255)
      effectindex = 254;

    unsigned char y = 0;
    unsigned char u = 0;
    unsigned char v = 0;
    int result = nvpfm_getyuvfromindex(effectindex, &y, &u, &v);
    // y = 26; u = 170; v = 122;
    if (result >= 0) {
      float fr = y + 1.4075 * (v - 128);

      float fg = y - 0.3455 * (u - 128) - 0.7169 * (v - 128);

      float fb = y + 1.779 * (u - 128);
      /*		static FILE* fp = fopen("table.txt", "wt");
      fprintf(fp, "index:%d,:%f,%f,%f\n", effectindex, fr, fg, fb);
      fflush(fp);
      */
      m_colortable[i].r = (unsigned char)fr;
      m_colortable[i].g = (unsigned char)fg;
      m_colortable[i].b = (unsigned char)fb;
    }
  }
}
void depthtransformer::compute_depth2pseudof(uint16_t *depthdata, int width, int height, float *pseudo, uint16_t max_d, uint16_t min_d, bool rgborbgr) {
  if (max_d == 0 && min_d == 0) {
    if (m_colortable == NULL) {
      calculatecolortable();
    }
  } else {
    if (m_colortable == NULL || (m_depthDistanceInfo.maxDepth != max_d || m_depthDistanceInfo.minDepth != min_d)) {
      m_depthDistanceInfo.maxDepth = max_d;
      m_depthDistanceInfo.minDepth = min_d;
      calculatecolortable();
    }
  }

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      int index = row * width + col;
      uint16_t distance = *(depthdata + row * width + col);
      MYRGB *color = (m_colortable + distance);

      pseudo[index * 3] = (rgborbgr ? ((float)color->r / 255.0) : ((float)color->b / 255.0));     // info->cr;
      pseudo[index * 3 + 1] = (float)color->g / 255.0;                                            // info->cg;
      pseudo[index * 3 + 2] = (rgborbgr ? ((float)color->b / 255.0) : ((float)color->r / 255.0)); // info->cb;
    }
  }
}
void depthtransformer::compute_depth2pseudo(uint16_t *depthdata, int width, int height, uint8_t *pseudo, uint16_t max_d, uint16_t min_d, bool rgborbgr) {
  if (max_d == 0 && min_d == 0) {
    if (m_colortable == NULL) {
      calculatecolortable();
    }
  } else {
    if (m_colortable == NULL || (m_depthDistanceInfo.maxDepth != max_d || m_depthDistanceInfo.minDepth != min_d)) {
      m_depthDistanceInfo.maxDepth = max_d;
      m_depthDistanceInfo.minDepth = min_d;
      calculatecolortable();
    }
  }

  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      int index = row * width + col;
      uint16_t distance = *(depthdata + row * width + col);
      MYRGB *color = (m_colortable + distance);

      pseudo[index * 3] = (rgborbgr ? color->r : color->b);     // info->cr;
      pseudo[index * 3 + 1] = color->g;                         // info->cg;
      pseudo[index * 3 + 2] = (rgborbgr ? color->b : color->r); // info->cb;
    }
  }
}

void depthtransformer::compute_depth2pointcloud(uint16_t *depthdata, int depthwidth, int depthheight,
                                                float *pointcloud, int stride,
                                                float leftfocus[2], float leftpc[2], float clipdistance, int edgecut) {
  Eigen::Map<MatType<uint16_t>::Type> depth_mat(depthdata, depthheight, depthwidth);
  MatType<float>::Type mat_depth_f = depth_mat.cast<float>() / 1000.0;

  if (NULL == m_ptm) {
    m_ptm = get_trans_matrix(depthwidth, depthheight);

    MatType<float>::Type fx_map(m_ptm->col_mat.rows(), m_ptm->col_mat.cols());
    fx_map.fill(leftpc[0]);
    MatType<float>::Type fy_map(m_ptm->row_mat.rows(), m_ptm->row_mat.cols());
    fy_map.fill(leftpc[1]);

    pre_x_mat = (m_ptm->col_mat - fx_map) / leftfocus[0];
    pre_y_mat = (m_ptm->row_mat - fy_map) / leftfocus[1];
  }

  MatType<float>::Type x_mat = pre_x_mat.cwiseProduct(mat_depth_f);
  MatType<float>::Type y_mat = pre_y_mat.cwiseProduct(mat_depth_f);

  for (int i = edgecut; i < (depthheight - edgecut); ++i) {
    for (int j = edgecut; j < (depthwidth - edgecut); ++j) {
      float *tmpx = (pointcloud + (i * depthwidth + j) * stride / sizeof(float));
      *tmpx = x_mat(i, j);
      *(tmpx + 1) = y_mat(i, j);
      float tmpz = mat_depth_f(i, j);
      if (clipdistance > 0.0 && tmpz > clipdistance) {
        *tmpx = 0.0;
        *(tmpx + 1) = 0.0;
        *(tmpx + 2) = 0.0;
      } else {
        *(tmpx + 2) = tmpz;
      }
    }
  }
}
void depthtransformer::compute_depth_rgb_align(
    uint16_t *depthdata, int depthwidth, int depthheight,
    uint16_t *align_data,
    int rgbwidth, int rgbheight,
    RGBPOS *align_pos, // depthwidth,depthheight
    float leftfocus[2], float leftpc[2],
    float rgbfocus[2], float rgbpc[2], float left2rgb[12]) {
  memset(align_data, 0, rgbwidth * rgbheight * sizeof(uint16_t));
  Eigen::Map<MatType<uint16_t>::Type> depth_mat(depthdata, depthheight, depthwidth);
  MatType<float>::Type mat_depth_f = depth_mat.cast<float>();

  if (NULL == align_tm) {
    align_tm = get_trans_matrix(depthwidth, depthheight);

    MatType<float>::Type fx_map(align_tm->col_mat.rows(), align_tm->col_mat.cols());
    fx_map.fill(leftpc[0]);
    MatType<float>::Type fy_map(align_tm->row_mat.rows(), align_tm->row_mat.cols());
    fy_map.fill(leftpc[1]);

    align_pre_x_mat = (align_tm->col_mat - fx_map) / leftfocus[0];
    align_pre_y_mat = (align_tm->row_mat - fy_map) / leftfocus[1];

    align_prea_mat = (rgbfocus[0] * left2rgb[0] * align_pre_x_mat + rgbfocus[0] * left2rgb[1] * align_pre_y_mat).array() + rgbfocus[0] * left2rgb[2];
    align_preb_mat = (rgbfocus[1] * left2rgb[3] * align_pre_x_mat + rgbfocus[1] * left2rgb[4] * align_pre_y_mat).array() + rgbfocus[1] * left2rgb[5];
    align_prec_mat = (left2rgb[6] * align_pre_x_mat + left2rgb[7] * align_pre_y_mat).array() + left2rgb[8];

    align_prek = rgbfocus[0] * left2rgb[9];
    align_prel = rgbfocus[1] * left2rgb[10];
  }

  MatType<float>::Type rgb_x_mat = align_prea_mat.cwiseProduct(mat_depth_f).array() + align_prek;

  MatType<float>::Type rgb_y_mat = align_preb_mat.cwiseProduct(mat_depth_f).array() + align_prel;

  MatType<float>::Type rgb_z_mat = align_prec_mat.cwiseProduct(mat_depth_f).array() + left2rgb[11];

  MatType<float>::Type fu = (rgb_x_mat.array() / rgb_z_mat.array() + rgbpc[0] + 0.5);
  MatType<float>::Type fv = (rgb_y_mat.array() / rgb_z_mat.array() + rgbpc[1] + 0.5);

  MatType<int>::Type iu = fu.cast<int>();
  MatType<int>::Type iv = fv.cast<int>();
  for (int i = 0; i < depthheight; ++i) {
    for (int j = 0; j < depthwidth; ++j) {
      float depth = rgb_z_mat(i, j); /// ???

      if (depth > 0.00001) {
        int riu = iu(i, j);
        int riv = iv(i, j);
        if (riu >= 0 && riu < rgbwidth && riv >= 0 && riv < rgbheight) {
          if (align_pos != NULL) { // 得到i行j列深度值对应的rgb行、列值
            RGBPOS *tmppos = align_pos + i * depthwidth + j;
            tmppos->row = riv;
            tmppos->col = riu;
          }
          if (align_data != NULL) {
            *(align_data + riv * rgbwidth + riu) = (uint16_t)depth;
          }
        }
      }
    }
  }
}

void depthtransformer::cal_depth_hist(uint16_t *depthdata, int width, int height) {
#ifdef _DEPTHEVAL
  // remove 0
  uint16_t *ptmpdata = (uint16_t *)malloc(width * height * sizeof(uint16_t));
  memset(ptmpdata, 0, width * height * sizeof(uint16_t));
  int total = 0;
  for (int i = 0; i < width * height; i++) {
    if (depthdata[i] != 0 && depthdata[i] <= m_maxAutoDepth) {
      ptmpdata[total++] = depthdata[i];
    }
  }
  if (0 == total) {
    return;
  }

  // sort
  cv::Mat gray = cv::Mat(1, total, CV_16UC1, (void *)ptmpdata);
  cv::Mat sortgray;
  cv::sort(gray, sortgray, cv::SORT_EVERY_ROW + cv::SORT_ASCENDING);

  // clip
  int roiIdx = total * DEPTH_ADJUST_CLIP_RATE;
  /*x：左上角的列坐标
    y：左上角的行坐标
    width：裁剪几列
    height：裁剪几行
  */
  cv::Mat roigray = sortgray(cv::Rect(roiIdx, 0, total - 2 * roiIdx, 1));
  double minvalue = 0;
  double maxvalue = 0;
  // 获取最大值和最小值
  cv::minMaxLoc(roigray, &minvalue, &maxvalue, 0, 0);
  m_autoDistanceInfo.maxDepth = minvalue;
  m_autoDistanceInfo.maxDepth = maxvalue;
  calculatecolortable();
  nvpfm_debug_printf("cal depth hist value(min:%f, max:%f)\n", minvalue, maxvalue);
#endif
}

void depthtransformer::depth_smearfilter(uint16_t *depthdata, int width, int height, int radius, float threshold, int p) {
  cv::Mat depthmap = cv::Mat(height, width, CV_16UC1, (void *)depthdata);
  cv::Mat depthfiltermap;
  smearfilter(depthmap, depthfiltermap, radius, threshold, p);
  memcpy(depthdata, (uint16_t *)depthfiltermap.data, depthfiltermap.rows * depthfiltermap.cols * sizeof(uint16_t));
  // printf("w:%d, h:%d, d(w:%d, h:%d)\n", width, height, depthfiltermap.cols, depthfiltermap.rows);
}

void depthtransformer::clip_depth(uint16_t *depthdata, int width, int height) {
  cv::Mat depthmap = cv::Mat(height, width, CV_16UC1, (void *)depthdata);
  depthmap.setTo(0, depthmap > m_clipdepthcfg.threshold);
  memcpy(depthdata, (uint16_t *)depthmap.data, width * height * sizeof(uint16_t));
}

void depthtransformer::set_smearfiltercfg(s_nvpfm_smear_filter_cfg cfg) {
  memcpy(&m_smearfiltercfg, &cfg, sizeof(m_smearfiltercfg));
}

s_nvpfm_smear_filter_cfg *depthtransformer::get_smearfiltercfg() {
  return &m_smearfiltercfg;
}

void depthtransformer::set_clipdepthcfg(s_nvpfm_clip_depht_cfg cfg) {
  memcpy(&m_clipdepthcfg, &cfg, sizeof(m_clipdepthcfg));
}
