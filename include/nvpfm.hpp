#ifndef __NVPFMHPP__
#define __NVPFMHPP__

#include <string>
#include <cstring>
#include "commondef.h"
#include "nvpfm.h"

#include "utils.h"
#include "cnn/nvp_cnn.h"
#include "depthfilter/smearfilter.h"
#include <vector>
#include <Eigen>
using namespace Eigen;
/// \brief camera param
///
/// Store camera param received from camera

typedef struct
{
  int left;
  int top;
  int right;
  int bottom;
} ROI;

typedef struct {
	POSTPROCESS_E type;
	const char* typestr; //define in device info json
	const char* name;
	const char* showname; //show name in the ui
	bool isenable; // only enable
}depth_postproc_desc;
/// \brief instance of opening camera
///
/// Open camera and get and set or start or stop something
class nvpfm {
public:
  /// \brief default constructor

  /// open first usb camera plugged in
  ///
  //  nvpfm();
  ~nvpfm();

  /// \brief binding constructor

  /// open binding usb or network camera

  /// \param bindingstr usb interface camera bindingstr format:usbport:bus-path,usbport and bus-path can be seen with command: lsusb -t,network interface camera bindingstr format:ip address
  nvpfm(NVPTL_DEVICE_INFO *pinfo, EVENTCALLBACK callback, void *userdata);
#ifdef _DEPTHEVAL
  float get_plane_median(NVPTL_USBHeaderDataPacket *depthframe, double *degree);
#endif
  void send_echo();
  void send_reboot();
  depth_postproc_desc *get_depthpostdesc(POSTPROCESS_E type);
  NVPTL_RESULT request_iframe(uint16_t sensor_channel);
  // save get get_all delete load_user
  NVPTL_RESULT save_user_config(const std::string &config_name);
  // reinterpret_cast out_data.data()
  NVPTL_RESULT get_user_config(const std::string &config_name, std::vector<uint8_t> *out_data);
  NVPTL_RESULT get_all_user_config_names(std::vector<std::string> *names);
  NVPTL_RESULT delete_user_config(const std::string &config_name);
  NVPTL_RESULT load_user_config(const std::string &config_name);
  // pwr mode comes from device info
  NVPTL_RESULT get_pwr_clk_config(const std::string &pwr_mode, s_nvpfm_get_clk_by_power_mode_info *result);

  float get_distance(NVPTL_USBHeaderDataPacket *depthframe, int x, int y);
  float get_fillrate(NVPTL_USBHeaderDataPacket *depthframe);
  float get_distance_min(NVPTL_USBHeaderDataPacket *depthframe, bool ignorezero = true);
  float get_distance_max(NVPTL_USBHeaderDataPacket *depthframe);
  static bool is_encrypt_firmware_file(const char *filepath);

  ROI get_roi(NVPTL_USBHeaderDataPacket *depthframe);
  void set_roi(NVPTL_USBHeaderDataPacket *depthframe, float ratio);
  void set_roi(NVPTL_USBHeaderDataPacket *depthframe, ROI roi);
  NVPTL_RESULT get_sensorcfg(s_nvpfm_get_sensor_config_ret *pcfg);
  NVPTL_RESULT get_lowtextureremoval(s_nvpfm_lowtexture_removal *pcfg);
  NVPTL_RESULT set_lowtextureremoval(s_nvpfm_set_lowtexture_removal *pcfg);
  NVPTL_RESULT get_factory_eeprom_sn(s_factory_get_eeprom_sn *pcfg, s_factory_get_eeprom_sn_ret *pret);
  NVPTL_RESULT get_devinfo(s_nvpfm_dev_info *pinfo);
  NVPTL_RESULT set_sensorcfg(s_nvpfm_app_sensor_config *pdata);
  NVPTL_RESULT set_factory_eeprom_sn(s_factory_set_eeprom_sn *pcfg, s_factory_set_eeprom_sn_ret *pret);
  NVPTL_RESULT set_factory_clear_calib(s_factory_clear_calib *pcfg, s_factory_clear_calib_return *pret);
  NVPTL_RESULT get_speckle(s_nvpfm_get_dynamic_config *pcfg, s_nvpfm_speckle_filter *pdata);
  NVPTL_RESULT set_speckle(s_nvpfm_set_speckle_filter *pdata);
  void get_device_info_async(ASYNCCALLBACK callback, void *userdata);
  void get_depth_param_async(ASYNCCALLBACK callback, void *userdata);
  void get_confidence_async(ASYNCCALLBACK callback, void *userdata);
  void get_sensorcfg_async(ASYNCCALLBACK callback, void *userdata);
  void get_spatial_async(ASYNCCALLBACK callback, void *userdata);
  void get_temporal_async(ASYNCCALLBACK callback, void *userdata);
  void get_speckle_async(ASYNCCALLBACK callback, void *userdata);
  void get_highlight_async(ASYNCCALLBACK callback, void *userdata);
  void get_lightfilter_async(ASYNCCALLBACK callback, void *userdata);
  void get_badfilter_async(ASYNCCALLBACK callback, void *userdata);
  // void get_edgefilter_async(ASYNCCALLBACK callback, void* userdata);
  void get_highprecision_async(ASYNCCALLBACK callback, void *userdata);
  void get_repeatfilter_async(ASYNCCALLBACK callback, void *userdata);
  void get_depthconfig_async(ASYNCCALLBACK callback, void *userdata);
  void get_irexposure_async(ASYNCCALLBACK callback, void *userdata);
  void get_rgbexposure_async(ASYNCCALLBACK callback, void *userdata);
  NVPTL_RESULT get_irexposure(s_nvpfm_get_sensor_exposure_ret *pret);
  NVPTL_RESULT get_rgbexposure(s_nvpfm_get_sensor_exposure_ret *pret);
  void get_runcfg_async(ASYNCCALLBACK callback, void *userdata);
  NVPTL_RESULT get_runcfg(s_nvpfm_get_app_run_config_ret *pdata);
  //	void get_cnnmodel_async(ASYNCCALLBACK callback, void* userdata);
  // void get_rgbexposure_async(ASYNCCALLBACK callback, void* userdata);
  void get_smearfilter_async(ASYNCCALLBACK callback, void *userdata);
  E_NVPFM_SENSOR_CHANNEL get_sensor_channel_by_stream(EM_AVAILABE_STREAM_TYPE streamtype);
  NVPFM_IMAGE_TYPE get_image_channel_by_stream(s_nvpfm_app_run_config runcfg, EM_AVAILABE_STREAM_TYPE streamtype);
  NVPFM_IMAGE_TYPE get_image_channel_by_sensor_channel(E_NVPFM_SENSOR_CHANNEL channel, bool iscalibrated);
  static std::string get_usbtype_by_spec(EM_NVPTL_USB_SPEC spectype);
  static NVPFM_IMAGE_SIZE parsewxh(const char *wxh);
  static NVPFM_IMAGE_SIZE findsensorresbyisp(s_nvpfm_dev_info info, NVPFM_IMAGE_SIZE res, E_NVPFM_SENSOR_CHANNEL chan);
  static const char *framesize2str(NVPFM_IMAGE_SIZE res);
  static void framesize2int(NVPFM_IMAGE_SIZE res, int *pwidth, int *pheight);
  // void close();
  /// \brief attempt to reopen camera

  /// reopen camera when ispluged return false
  // void refresh();

  /// \brief test if camera is already opened

  /// test if camera is already opened

  /// \return true:camera is opened,false:camera closed
  bool isplugged();

  /// \brief config ir and depth stream

  /// config ir and depth stream's resolution and fps and fps ratio,real fps=fps/fpsratio,fps=FPS_15,ratio=4,real fps=15/4=3.75fps

  /// \param res resolution enum
  /// \param fps fps enum
  /// \param fpsratio ratio of fps

  /// \return NVPTL_OK:config ir ok,else:config ir failed
  // NVPTL_RESULT config_ir(NVPFM_SENSOR_RESOLUTION_TYPE res, NVPFM_FPS_E fps, int fpsratio);

  void set_global_time(bool enable);
  /// \brief config rgb stream

  /// config rgb stream's resolution and fps and fps ratio,real fps=fps/fpsratio,fps=FPS_15,ratio=4,real fps=15/4=3.75fps

  /// \param res resolution enum
  /// \param fps fps enum
  /// \param fpsratio ratio of fps

  /// \return NVPTL_OK:config rgb ok,else:config rgb failed
  // NVPTL_RESULT config_rgb(NVPFM_SENSOR_RESOLUTION_TYPE res, NVPFM_FPS_E fps, int fpsratio);

  /// \brief config good feature param

  /// config good feature param,enable or disable good feature,you can not enable good feature and lk at same time

  /// \param feature s_nvpfm_good_feature type pointer,enable or disable good feature,config good feature's param

  /// \return NVPTL_OK:config goodfeature ok,else:config good feature failed
  NVPTL_RESULT config_goodfeature(s_nvpfm_set_good_feature *feature);
  /// \brief config lk optical flow param

  /// config lk optical flow param,enable or disable lk optical flow,you can not enable good feature and lk optical flow at same time

  /// \param feature s_nvpfm_lk type pointer,enable or disable lk optical flow,config lk optical flow's param

  /// \return NVPTL_OK:config lk ok,else:config lk failed
  NVPTL_RESULT config_lk(s_nvpfm_set_lk *lk);
  //  NVPTL_RESULT set_pipeline(int mode);
  // NVPTL_RESULT save_images(const char* cameraname,const char* path,int groups);
  /// \brief start left ir stream

  /// start left ir stream

  /// \param leftircallback user define callback function,data can be retrieved in leftircallback function
  /// \param userdata user can pass pointer to leftircallback callback function
  NVPTL_RESULT start_leftir(FRAMECALLBACK leftircallback);
  NVPTL_RESULT start_groupimages(GROUPFRAMECALLBACK groupcallback);
  NVPTL_RESULT start_save(FRAMECALLBACK savecallback);

  /// \brief start right ir stream

  /// start right ir stream

  /// \param leftircallback user define callback function,data can be retrieved in rightircallback function
  /// \param userdata user can pass pointer to rightircallback callback function
  NVPTL_RESULT start_rightir(FRAMECALLBACK rightircallback);

  /// \brief start depth stream

  /// start depth stream

  /// \param depthcallback user define callback function,data can be retrieved in depthcallback function
  /// \param userdata user can pass pointer to depthcallback callback function
  NVPTL_RESULT start_depth(FRAMECALLBACK depthcallback);

  /// \brief start other stream

  /// start other stream

  /// \param othercallback user define callback function,data can be retrieved in othercallback function
  /// \param userdata user can pass pointer to othercallback callback function
  NVPTL_RESULT start_other(FRAMECALLBACK depthcallback);

  /// \brief start rgb stream

  /// start rgb stream

  /// \param rgbcallback user define callback function,data can be retrieved in rgbcallback function
  /// \param userdata user can pass pointer to rgbcallback callback function
  NVPTL_RESULT start_rgb(FRAMECALLBACK rgbcallback);

  /// \brief start imu stream

  /// start imu stream

  /// \param imucallback user define callback function,data can be retrieved in imucallback function
  /// \param userdata user can pass pointer to imucallback callback function
  NVPTL_RESULT start_imu(FRAMECALLBACK imucallback);

  /// \brief start good feature/lk stream

  /// start good feature/lk stream

  /// \param goodfeaturecallback user define callback function,data can be retrieved in goodfeaturecallback function
  /// \param userdata user can pass pointer to goodfeaturecallback callback function
  NVPTL_RESULT start_goodfeature(FRAMECALLBACK goodfeaturecallback);

  /// \brief stop left ir stream

  /// stop left ir stream
  NVPTL_RESULT stop_leftir();
  NVPTL_RESULT send_update_tftp();

  /// \brief stop right ir stream

  /// stop right ir stream
  NVPTL_RESULT stop_rightir();

  /// \brief stop depth stream

  /// stop depth stream
  NVPTL_RESULT stop_depth();

  /// \brief stop rgb stream

  /// stop rgb stream
  NVPTL_RESULT stop_rgb();

  /// \brief stop imu stream

  /// stop imu stream
  NVPTL_RESULT stop_imu();

  /// \brief save current config in camera

  /// save current config in camera

  /// \return NVPTL_OK:save config successfully,else:save config failed
  NVPTL_RESULT save_config();

  /// \brief upload local file to camera

  /// upload local file localfilepath to camera's remotefilepath

  /// \param localfilepath,local file path,which will be upload to camera
  /// \param remotefilepath,remote file path in camera,include file name
  /// \return NVPTL_OK:upload file ok,else:upload file failed
  NVPTL_RESULT upgrade(std::string localfirmwarefilepath, UPGRADECALLBACK upgradecallback, void *userdata);
  NVPTL_RESULT uploadfile(std::string localfilepath, std::string remotefilepath);

  /// \brief download file from camera

  /// download remotefilepath in camera to local localfilepath,only small size file download support,do not exceed 512KB or less

  /// \param remotefilepath,remote file path in camera,which will be download
  /// \param localfilepath,local file path,which will be saved
  /// \return NVPTL_OK:download file ok,else:download file failed
  
  NVPTL_RESULT downloadfile2mem(std::string remotefilepath, char** ppfilecontent);
  NVPTL_RESULT uploadmem2file(const char* content, std::string remotefilepath);
  NVPTL_RESULT downloadfile(std::string remotefilepath, std::string localfilepath);
  NVPTL_RESULT downloadcalib(std::string localfilepath);
  NVPTL_RESULT factorystartimage();
  NVPTL_RESULT factorystopimage();
  NVPTL_RESULT factorystartimu();
  NVPTL_RESULT factorystopimu();

  NVPTL_RESULT uploadcalibstart();
  NVPTL_RESULT uploadcalibend();
  NVPTL_RESULT uploadcalib(std::string localfilepath,  NVPFM_CALIBRATION_TYPE type, NVPFM_IMAGE_SIZE frame_size, uint16_t channel, uint16_t calitype, const std::string& file_name = "");
  NVPTL_RESULT uploadsensorcalib(std::string localfilepath, NVPFM_IMAGE_SIZE frame_size, E_NVPFM_SENSOR_CHANNEL channel, NVPFM_CALIBRATION_SENSOR_TYPE calitype);
  // cfy modify.
  NVPTL_RESULT uploadimucalib(std::string localfilepath, const std::string& imu_cali_file_name, E_NVPFM_IMU_CHANNEL channel, NVPFM_CALIBRATION_IMU_TYPE calitype);
  NVPTL_RESULT senduserdata(void *data, int size);

  /// \brief set rgb exposure param

  /// set rgb exposure param

  /// \param exposure,s_nvpfm_sensor_exposure type pointer,exposure param of rgb
  /// \return NVPTL_OK:set rgb exposure ok,else:set rgb exposure failed
  //  NVPTL_RESULT set_rgb_exposure(s_nvpfm_sensor_exposure *exposure);

  NVPTL_RESULT get_imu_rgb_offset(float *ms); // milliseconds
  NVPTL_RESULT get_camera_time(s_nvpfm_time_info *timeinfo);

  NVPTL_RESULT get_efuse_user(s_nvpfm_efuse_user *param);
  NVPTL_RESULT set_efuse_user(s_nvpfm_efuse_user *param);

  NVPTL_RESULT set_ip(s_nvpfm_set_ip* setip);
  /// \brief get rgb exposure param

  /// get rgb exposure param,data will be get in exposure param

  /// \param exposure,s_nvpfm_sensor_exposure type pointer,valid when function return NVPTL_OK
  /// \return NVPTL_OK:get rgb exposure ok,else:get rgb exposure failed
  //   NVPTL_RESULT get_rgb_exposure(s_nvpfm_sensor_exposure *exposure);

  /// \brief get depth config param

  /// get depth config param,data will be get in depth config param

  /// \param depthconfig,s_nvpfm_get_dsp_static_config_ret type pointer,valid when function return NVPTL_OK
  /// \return NVPTL_OK:get depth config ok,else:get depth config failed
  NVPTL_RESULT get_depth_config(s_nvpfm_get_dsp_static_config_ret *depthconfig);

  /// \brief get ir exposure param

  /// get ir exposure param,data will be get in exposure param

  /// \param exposure,s_nvpfm_sensor_exposure type pointer,valid when function return NVPTL_OK
  /// \return NVPTL_OK:get ir exposure ok,else:get ir exposure failed
  //   NVPTL_RESULT get_ir_exposure(s_nvpfm_sensor_exposure *exposure);

  /// \brief set ir sensor auto exposure

  /// set ir sensor auto exposure

  /// \param exp_ratio,exposure ratio,which will affect max exposure time
  /// \param ae_compensation_id,auto exposure compensation value
  /// \return NVPTL_OK:set ir auto exposure ok,else:set ir auto exposure failed
  // NVPTL_RESULT set_ir_auto_exposure(int exp_ratio, int ae_compensation_id, int maxexposuretime, int maxagain);

  /// \brief set ir sensor manual exposure

  /// set ir sensor manual exposure

  /// \param leftexposuretime,left ir sensor's exposure time in microseconds
  /// \param left_gain,left ir sensor gain
  /// \param rightexposuretime,right ir sensor's exposure time in microseconds
  /// \param right_gain,rigth ir sensor gain
  /// \param exp_ratio,exposure ratio,which will affect max exposure time
  /// \param ae_compensation_id,auto exposure compensation value
  /// \return NVPTL_OK:set ir manual exposure ok,else:set ir manual exposure failed
  // NVPTL_RESULT set_ir_manual_exposure(int leftexposuretime, int left_gain, int rightexposuretime, int right_gain, int exp_ratio, int ae_compensation_id);

  /// \brief set camera's run mode

  /// set camera's run mode

  /// \param mode,run mode,0:sensor,1:sensor+rectify,2:depth,3:depth+cnn,4:upgrade
  /// \return NVPTL_OK:set run mode ok,else:set run mode failed
  NVPTL_RESULT set_mode(s_nvpfm_app_run_config config);
  NVPTL_RESULT get_mode(s_nvpfm_get_app_run_config_ret *pconfig);
  NVPTL_RESULT get_usb_cc_info(s_nvpfm_usb_cc_info *pinfo);

  /// \brief switch stream to vpss(rectify) in run mode 1:sensor+rectify

  /// switch stream to vpss(rectify) in run mode 1:sensor+rectify

  /// \return NVPTL_OK:switch vpss ok,else:switch vpss failed
  NVPTL_RESULT switch_vpss();

  /// \brief switch stream to vi(sensor) in run mode 1:sensor+rectify

  /// switch stream to vi(sensor) in run mode 1:sensor+rectify

  /// \return NVPTL_OK:switch vi ok,else:switch vi failed
  NVPTL_RESULT switch_vi();

  NVPTL_RESULT get_transfer_config(s_get_transfer_config_ret *pdata);
  NVPTL_RESULT set_transfer_config(s_set_transfer_config *pdata);
  NVPTL_RESULT get_led_config(s_nvpfm_get_led_ret *pdata, E_NVPFM_LED_CHANNEL channel);
  NVPTL_RESULT set_led_config(s_nvpfm_set_led *pdata);

  /// \brief set confidence

  /// set confidence,enable or disable,confidence's sigma to sigma,this is depth data post process in camera

  /// \param enable,true:enable confidence,sigma will be effective,false:disable confidence,sigma ignored
  /// \param sigma,sigma value of confidence,[0.0-1.0],0.0392 is recommended
  /// \return NVPTL_OK:set confidence ok,else:set confidence failed
  NVPTL_RESULT set_sensor_status(s_set_sensor_status *pdata, bool basync = false, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_imu_status(s_set_imu_status *pdata);
  NVPTL_RESULT set_can_status(s_set_can_status *pdata);
  NVPTL_RESULT set_confidence(s_nvpfm_set_confidence *pdata);
  NVPTL_RESULT set_confidence_async(s_nvpfm_set_confidence *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_sensorcfg_async(s_nvpfm_app_sensor_config *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_spatial_async(s_nvpfm_set_spatial_filter *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_temporal_async(s_nvpfm_set_temporal_filter *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_speckle_async(s_nvpfm_set_speckle_filter *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_highlight_async(s_nvpfm_set_depth_high_light *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_repeatfilter_async(s_nvpfm_set_repeated_texture_filter *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_depthconfig_async(s_nvpfm_set_dsp_static_config *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_irexposure_async(s_nvpfm_set_sensor_exposure *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_rgbexposure_async(s_nvpfm_set_sensor_exposure *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_irexposure(s_nvpfm_set_sensor_exposure *pdata);
  NVPTL_RESULT set_rgbexposure(s_nvpfm_set_sensor_exposure *pdata);
  NVPTL_RESULT set_runcfg(s_nvpfm_app_run_config *pdata);

  NVPTL_RESULT set_triggercfg_async(s_nvpfm_app_trigger_config *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_runcfg_async(s_nvpfm_app_run_config *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  //	NVPTL_RESULT set_cnnmodel_async(s_nvpfm_depth_cnn_model* pdata);
  NVPTL_RESULT set_smearfilter_async(s_nvpfm_set_smear_filter *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_lightfilter_async(s_nvpfm_set_light_filter *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  NVPTL_RESULT set_badfilter_async(s_nvpfm_set_bad_filter *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  // NVPTL_RESULT set_edgefilter_async(s_nvpfm_edge_filter_cmd* pdata);
  NVPTL_RESULT set_highprecision_async(s_nvpfm_set_high_precision *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
	
	template <typename T>
	NVPTL_RESULT set_depth_postproc(T *param, const NVPFM_VALUE_TYPE type, bool async = false, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
	template <typename T1, typename T2>
	NVPTL_RESULT set_depth_postprocwithret(T1 *param, T2 *retcfg, const NVPFM_VALUE_TYPE type, bool async = false, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
	template <typename T>
	NVPTL_RESULT get_depth_postproc(T *param, const E_NVPFM_DEPTH_CHANNEL channel, const NVPFM_VALUE_TYPE type);
	void get_depth_postproc_async(const NVPFM_VALUE_TYPE type, const E_NVPFM_DEPTH_CHANNEL channel, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
	/// \brief set bad filter

  /// set bad filter,enable or disable,bad filter's kernel value to kernel,this is depth data post process in camera

  /// \param enable,true:enable bad filter,kernel will be effective,false:disable bad filter,kernel ignored
  /// \param kernel,kernel value of bad filter,only 15 or 19 is supported
  /// \return NVPTL_OK:set bad filter ok,else:set bad filter failed
  NVPTL_RESULT set_bad_filter(s_nvpfm_set_bad_filter *pdata);
  NVPTL_RESULT set_smear_filter(s_nvpfm_set_smear_filter *pdata);
  NVPTL_RESULT set_depth_calculate(s_nvpfm_set_depth_calculate *pdata);

  /// \brief set light filter

  /// set light filter

  /// \param lightfilter,s_nvpfm_light_filter type pointer,set light filter param
  /// \return NVPTL_OK:set light filter ok,else:set light filter failed
  NVPTL_RESULT set_light_filter(s_nvpfm_set_light_filter *lightfilter);
  /// \brief set repeat texture filter

  /// set repeat texture filter

  /// \param pfilter,s_nvpfm_repeated_texture_filter type pointer
  /// \return NVPTL_OK:set repeat texture filter ok,else:set repeat texture filter failed
  NVPTL_RESULT set_repeat_texture_filter(s_nvpfm_set_repeated_texture_filter *pfilter);

  /// \brief set edge filter

  /// set edge filter

  /// \param edgefilter,s_nvpfm_edge_filter_cmd type pointer,set left top point and right bottom point of edge
  /// \return NVPTL_OK:set edge filter ok,else:set edge filter failed
  // NVPTL_RESULT set_edge_filter(s_nvpfm_edge_filter_cmd *edgefilter);

  /// \brief get camera param

  /// get camera param

  /// \return not NULL:get camera_param type pointer camera param ok,NULL:get camera param failed
  NVPTL_RESULT get_camera_param(s_nvpfm_camera_param *param);

  /// \brief get good feature param

  /// get good feature param

  /// \return not NULL:get s_nvpfm_good_feature type pointer param ok,NULL:get good feature param failed
  s_nvpfm_good_feature *get_goodfeature_param();

  /// \brief get lk optical flow param

  /// get lk optical flow param

  /// \return not NULL:get s_nvpfm_lk type pointer param ok,NULL:get lk optical flow param failed
  s_nvpfm_lk *get_lk_param();

  /// \brief set imu config

  /// set imu config

  /// \param imuinfo,s_nvpfm_imu_config type pointer
  /// \return NVPTL_OK:set imu config ok,else:set imu config failed
  NVPTL_RESULT set_enter_ready();
  NVPTL_RESULT set_exit_ready();

  NVPTL_RESULT get_run_at_ready_mode_info(s_nvpfm_run_at_ready_mode_info *info);

  NVPTL_RESULT set_trigger_config(s_nvpfm_app_trigger_config *triggercfg, s_nvpfm_cmd_set_ret *retcfg);
  NVPTL_RESULT get_trigger_config(s_nvpfm_get_app_trigger_config_ret *triggercfg);

  NVPTL_RESULT set_imu_config(s_set_imu_config *imucfg);
  NVPTL_RESULT set_imu_config_async(s_set_imu_config *pdata, ASYNCCALLBACK callback = NULL, void *userdata = NULL);
  // NVPTL_RESULT set_tare_calibration(int width,int height,float measuredistancemm,float groundtruemm);

  /// \brief get imu config

  /// get imu config

  /// \return not NULL:get imu config ok,NULL:get imu config failed
  NVPTL_RESULT get_imu_config(s_get_imu_config_ret *cfg);
  void get_imu_config_async(ASYNCCALLBACK callback, void *userdata);

  /// \brief get imu internal reference

  /// get imu internal reference

  /// \return not NULL:get imu internal reference ok,NULL:get imu internal reference failed
  s_nvpfm_imu_internal_reference *get_imu_internalref();

  /// \brief get imu external reference

  /// get imu external reference

  /// \return not NULL:get imu external reference ok,NULL:get imu external reference failed
  s_nvpfm_imu_external_reference *get_imu_externalref();
  /*#ifdef USEYAMLCPP
      s_nvpfm_imu_external_reference *_get_imu_externalref(const char* chainyaml,const char* imuyaml);
      s_nvpfm_imu_internal_reference *_get_imu_internalref(const char* internaltxt);
  #endif*/

  /// \brief get device info of camera

  /// get device info of camera

  /// \return not NULL:get device info ok,NULL:get device info failed
  // s_nvpfm_device_info *get_device_info(bool read);

  /// \brief get sn of camera

  /// get sn of camera

  /// \return not "":get camera sn ok,"":get camera sn failed
  std::string get_sn();
  std::string get_producttype();

  /// \brief get firmware version of camera

  /// get firmware version of camera

  /// \return not "":get camera firmware version ok,"":get camera firmware version failed
  std::string get_software_version();
  std::string get_sdk_version();

  /// \brief get usb speed of camera

  /// get usb speed of camera

  /// \return not "":get camera usb speed ok,"":get camera usb speed failed
  std::string get_usb_speed();

  /// \brief timesync camera with upper computer

  /// timesync camera with upper computer

  /// \param timestamp,s_nvpfm_time_ret type pointer,pc_time member variable is the timestamp tobe sent to camera
  /// \return NVPTL_OK:time sync ok,else:time sync failed
  NVPTL_RESULT set_timesync(s_nvpfm_time *timestamp);
  NVPTL_RESULT set_timesynccycle(bool enable, int seconds);

  /// \brief set projector on or off

  /// set projector on or off

  /// \param on,true:set projector on,false:set projector off
  /// \return NVPTL_OK:set projector ok,else:set projector failed
  NVPTL_RESULT set_projector(s_nvpfm_set_projector *pcfg);
  NVPTL_RESULT set_high_precision(bool on);
  // NVPTL_RESULT set_rgb_else_param(s_nvpfm_rgb_else_param *pparam);
  NVPTL_RESULT set_cypher_key(s_nvpfm_efuse_key *pparam);
  NVPTL_RESULT get_rgb_else_param(s_nvpfm_rgb_else_param *pparam);
  NVPTL_RESULT get_cypher_status(s_nvpfm_efuse_status *pparam);

  /// \brief get projector status

  /// get projector status,status will be set to true or false when function return

  /// \param status,true:projector on,false:projector off
  /// \return NVPTL_OK:get projector status ok,else:get projector status failed
  NVPTL_RESULT get_projector(s_nvpfm_get_projector *pcfg, s_nvpfm_get_projector_ret *retvalue);
  NVPTL_RESULT get_high_precision(s_nvpfm_high_precision &status);

  /// \brief get confidence info

  /// get confidence info,status will be set to true or false,sigma will be set when function return

  /// \param status,true:confidence on,false:confidence off
  /// \param sigma,confidence param,[0.0,1.0]
  /// \return NVPTL_OK:get confidence ok,else:get confidence failed
  NVPTL_RESULT get_confidence(s_nvpfm_confidence *pdata);

  /// \brief get repeat texture filter info

  /// get repeat texture filter info

  /// \param pfilter,s_nvpfm_repeated_texture_filter type pointer,value is repeat texture filter's param
  /// \return NVPTL_OK:get repeat texture filter ok,else:get repeat texture filter failed
  NVPTL_RESULT get_repeat_texture_filter(s_nvpfm_repeated_texture_filter *pfilter);

  /// \brief get edge filter info

  /// get edge filter info

  /// \param pfilter,s_nvpfm_edge_filter_cmd type pointer,value is edge filter's param
  /// \return NVPTL_OK:get edge filter ok,else:get edge filter failed
  // NVPTL_RESULT get_edge_filter(s_nvpfm_edge_filter_cmd *pfilter);

  /// \brief get light filter info

  /// get light filter info

  /// \param plightfilter,s_nvpfm_light_filter type pointer,point to variable which save light filter param
  /// \return NVPTL_OK:get light filter ok,else:get light filter failed
  NVPTL_RESULT get_light_filter(s_nvpfm_light_filter *plightfilter);
  /// \brief get bad filter info

  /// get bad filter info

  /// \param tmpparam,s_nvpfm_bad_filter type pointer,point to variable save bad filter param
  /// \return NVPTL_OK:get bad filter ok,else:get bad filter failed
  NVPTL_RESULT get_bad_filter(s_nvpfm_bad_filter *tmpparam);
  NVPTL_RESULT get_smear_filter(s_nvpfm_smear_filter *tmpparam);
  NVPTL_RESULT get_depth_calculate(s_nvpfm_get_depth_calculate *pdepthchn, s_nvpfm_get_depth_calculate_ret *pdata);
  static bool parseres(const char *res, NVPFM_IMAGE_SIZE *sensorres, NVPFM_IMAGE_SIZE *ispres, int *osensorfps, int *oispfps);
  static void nv12_rotate_90_i420(NVPFM_USB_IMAGE_HEADER *pheader);
  static void nv12_rotate_270_i420(NVPFM_USB_IMAGE_HEADER *pheader);
  static void get_cnn_data(NVPTL_USBHeaderDataPacket *tmppack, s_nvpfm_cnn_data *pdata);
  static LABELINFO get_cnn_label_by_index(int index, EM_CNN_TYPE type);

  static void grayscale_rotate_90(NVPFM_USB_IMAGE_HEADER *pheader);
  static void grayscale_rotate_270(NVPFM_USB_IMAGE_HEADER *pheader);

  // bool get_camera_info();
  //  void set_camera_info(s_nvpfm_device_info* pinfo);

  std::string get_bindingstr() {
    return m_bindingstr;
  };

  std::string get_port() {
    if (m_bindingstr.length() > strlen("falcon-"))
      return m_bindingstr.substr(8);
    else
      return std::string("unknown");
  }

private:
  /*  void construct_binding(std::string bindingstr);
    void construct_usb_port_binding(std::string port);
  void construct_usb_uvc_binding(std::string path);
    void construct_net_ip_binding(std::string ip);
    void construct_usb_deviceid_binding(std::string deviceid);*/
  ROI m_roi;

  bool m_isplugged;
  s_nvpfm_imu_config m_imuconfig;
  s_nvpfm_lk m_lkparam;
  s_nvpfm_good_feature m_goodfeatureparam;
  s_nvpfm_imu_internal_reference m_imuinternalref;
  s_nvpfm_imu_external_reference m_imuexternalref;
  s_nvpfm_app_dsp_process_static_config m_depthconfig;
  s_nvpfm_app_sensor_exposure_config m_irexposure;
  s_nvpfm_app_run_config m_runcfg;
  //	s_nvpfm_depth_cnn_model m_cnnmodel;
  s_nvpfm_app_sensor_exposure_config m_rgbexposure;
  s_nvpfm_dev_info m_devinfo;
  std::string m_sn;
  std::string m_producttype;
  std::string m_usb_speed;
  // s_nvpfm_device_info* m_pdeviceinfo;
  std::string m_bindingstr;
  NVPFM_DEVICE_HANDLE m_handle;

  // NVPFM_CONFIG m_config;
};

template<typename T>
NVPTL_RESULT nvpfm::set_depth_postproc(T *param, const NVPFM_VALUE_TYPE type, bool async, ASYNCCALLBACK callback, void *userdata) {
	if (async) {
		nvpfm_set_async(m_handle, type, (uint8_t *)param, sizeof(T), callback, userdata);
		return NVPTL_OK;
	}
	s_nvpfm_cmd_set_ret setcfg;
	int retlen = sizeof(s_nvpfm_cmd_set_ret);
	return nvpfm_set(m_handle, type, param, sizeof(T), &setcfg, &retlen, 2000);
}

template <typename T1, typename T2>
NVPTL_RESULT nvpfm::set_depth_postprocwithret(T1 *param, T2 *retcfg, const NVPFM_VALUE_TYPE type, bool async, ASYNCCALLBACK callback, void *userdata) {
	if (async) {
		nvpfm_set_async(m_handle, type, (uint8_t *)param, sizeof(T1), callback, userdata);
		return NVPTL_OK;
	}
	T2 cfg;
	int retlen = sizeof(T2);
	NVPTL_RESULT ret = nvpfm_set(m_handle, type, param, sizeof(T1), &cfg, &retlen, 2000);
	if (NVPTL_OK == ret) {
		if (0 == cfg.ret.ret) {
			memcpy(retcfg, &cfg, sizeof(T2));
			return NVPTL_OK;
		}
		return NVPTL_FAILED;
	}
	return ret;
}

template<typename T>
NVPTL_RESULT nvpfm::get_depth_postproc(T *param, const E_NVPFM_DEPTH_CHANNEL channel, const NVPFM_VALUE_TYPE type) {
	int len = sizeof(T);
	s_nvpfm_get_dynamic_config cfg;
	cfg.channel = channel;
	return nvpfm_get(m_handle, type, &cfg, sizeof(cfg), param, &len, 2000);
}

typedef struct _depth_distance_info {
  uint16_t minDepth;
  uint16_t maxDepth;
  _depth_distance_info() {
    minDepth = 100;
    maxDepth = 5000;
  }
} depth_distance_info;
/*
typedef struct {
  unsigned char r;
  unsigned char g;
  unsigned char b;
}MYRGB;*/

template <typename T>
struct MatType {
  using Type = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
};
struct transform_matrix {
  MatType<float>::Type col_mat;
  MatType<float>::Type row_mat;
  int width = 0;
  int height = 0;
};
class depthtransformer {
public:
  depthtransformer() : m_colortable(NULL),
                       m_ptm(NULL),
                       align_tm(NULL),
                       m_isAutoDepthCal(true),
                       m_maxAutoDepth(5000),
                       m_maxAutoGap(5) {
    m_smearfiltercfg.enable = 0;
    m_smearfiltercfg.radius = 1;
    m_smearfiltercfg.threshold = 0.000012f;
    m_smearfiltercfg.p = 2;

    m_clipdepthcfg.enable = 0;
    m_clipdepthcfg.threshold = 5000;
  };
  ~depthtransformer() {
    if (NULL != m_colortable) {
      delete m_colortable;
    }
  };
  void compute_depth2pointcloud(uint16_t *depthdata, int depthwidth, int depthheight, float *pointcloud, int stride, float leftfocus[2], float leftpc[2], float clipdistance = 0.0, int edgecut = 0);
  void compute_depth2pseudof(uint16_t *depthdata, int width, int height, float *pseudo, uint16_t max_d = 0, uint16_t min_d = 0, bool rgborbgr = true);
  void compute_depth2pseudo(uint16_t *depthdata, int width, int height, uint8_t *pseudo, uint16_t max_d = 0, uint16_t min_d = 0, bool rgborbgr = true);
  void compute_depth_rgb_align(
      uint16_t *depthdata, int depthwidth, int depthheight,
      uint16_t *align_data,
      int rgbwidth, int rgbheight,
      RGBPOS *align_pos, // depthwidth,depthheight
      float leftfocus[2], float leftpc[2],
      float rgbfocus[2], float rgbpc[2], float left2rgb[12]);
  void calculatecolortable();
  void cal_depth_hist(uint16_t *depthdata, int width, int height);
  void depth_smearfilter(uint16_t *depthdata, int width, int height, int radius, float threshold, int p);
  void clip_depth(uint16_t *depthdata, int width, int height);
  void set_smearfiltercfg(s_nvpfm_smear_filter_cfg cfg);
  s_nvpfm_smear_filter_cfg *get_smearfiltercfg();
  void set_clipdepthcfg(s_nvpfm_clip_depht_cfg cfg);

  depth_distance_info m_depthDistanceInfo;
  depth_distance_info m_autoDistanceInfo;
  uint8_t m_isAutoDepthCal; // 1:auto, 0:manual
  double m_maxAutoDepth;    // ��λmm
  double m_maxAutoGap;      // ��λ��s
  s_nvpfm_smear_filter_cfg m_smearfiltercfg;
  s_nvpfm_clip_depht_cfg m_clipdepthcfg; // ��λmm

private:
  struct transform_matrix *m_ptm;
  MatType<float>::Type pre_x_mat, prea_mat, preb_mat, prec_mat;
  MatType<float>::Type pre_y_mat;
  float prek, prel;

  transform_matrix *get_trans_matrix(int w, int h);
  transform_matrix tm;
  MYRGB *m_colortable;

  struct transform_matrix *align_tm;
  MatType<float>::Type align_pre_x_mat, align_prea_mat, align_preb_mat, align_prec_mat;
  MatType<float>::Type align_pre_y_mat;
  float align_prek, align_prel;
};
#endif
