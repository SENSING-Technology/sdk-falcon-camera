#include "test_process.h"

/*********************************callback*******************************/
void depthcallback(void *data, void *userdata)
{
    printf("call back\n");

    NVPTL_USBHeaderDataPacket *pPack = (NVPTL_USBHeaderDataPacket*) data;
    printf(" magic:%s, type:%d, sub_type:%d, checksum:%d, len:%d\n ", pPack->magic, pPack->type, pPack->sub_type, pPack->checksum, pPack->len);
    NVPFM_USB_IMAGE_HEADER *pDepthHeader = (NVPFM_USB_IMAGE_HEADER *)((uint8_t *)pPack + sizeof(NVPTL_USBHeaderDataPacket));
    printf("*******************image hearder *******************\n");
    printf("group_id:%d\n", pDepthHeader->group_id);
    printf("resolution: %d * %d\n", pDepthHeader->width, pDepthHeader->height);
    printf("timestamp:%d\n", pDepthHeader->timestamp);
    printf("exposure:%d\n", pDepthHeader->exposure);
    printf("===============================================\n");
}

void sensorcfgcallback(void* pdata, int len, void* userdata) {
	s_nvpfm_get_sensor_config_ret* pinfo = (s_nvpfm_get_sensor_config_ret*)pdata;
	if (pinfo->ret != 0)return;
    cout << "sensor fps: " << pinfo->config.fps << ", frame size: " << pinfo->config.frame_size << endl;
}

void othercallback(void* data, void* userdata) {
	NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
	if (tmppack->type == NVPFM_LOG_DATA)
	{
        cout << "get log data" << endl;
    }
    else if (tmppack->type == NVPFM_CNN_DATA) {
		// Json::Value node;
		// Json::Reader reader;
		// reader.parse((const char*)tmppack->data, node);

		// int16_t* buffers[3];
		// buffers[0] = (int16_t*)(((uint8_t*)tmppack->data) + node["output0"]["offset"].asInt());
		// buffers[1] = (int16_t*)(((uint8_t*)tmppack->data) + node["output1"]["offset"].asInt());
		// buffers[2] = (int16_t*)(((uint8_t*)tmppack->data) + node["output2"]["offset"].asInt());
		// uint64_t timestamp = node["timestamp"].asInt64();
        cout << "get cnn data" << endl;
    }
}

void imucallback(void* data, void* userdata) {
	NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
	s_nvpfm_imu_data* pimudata = (s_nvpfm_imu_data*)((uint8_t*)tmppack + sizeof(NVPTL_USBHeaderDataPacket));
	if (pimudata->factory_data == 0) {
        cout << "imu call back, depth mode, data_number: " << pimudata->data_number << endl;
    }
	else {
		cout << "imu call back, factory mode, data_number: " << pimudata->data_number << endl;
	}
}

void rgbcallback(void* data, void* userdata) {
    NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
    int width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket)))->width;
    int height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)tmppack + sizeof(NVPTL_USBHeaderDataPacket)))->height;

    cout << "rgb call back width: " << width << ", height: " << height << endl;
    if (tmppack->sub_type == IMAGE_CHANNEL2_ORIGNAL ||
        tmppack->sub_type == IMAGE_CHANNEL2_CALIBRATED)
    {
        cout << "rgb call back rgb0" << endl;
    }
    else if(tmppack->sub_type == IMAGE_CHANNEL3_ORIGNAL ||
    tmppack->sub_type == IMAGE_CHANNEL3_CALIBRATED) {
         cout << "rgb call back rgb1" << endl;
    }
   
}

void leftircallback(void *data, void *userdata) {
    NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
    int width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket)))->width;
    int height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket)))->height;
    nvptl_debug_printf("left ir callback width:%d, height:%d\n", width, height);
    if (tmppack->sub_type == IMAGE_CHANNEL0_ORIGNAL ||
        tmppack->sub_type == IMAGE_CHANNEL0_CALIBRATED)
    {
        nvptl_debug_printf("leftri0\n");
    }
    else if (tmppack->sub_type == IMAGE_CHANNEL2_ORIGNAL ||
             tmppack->sub_type == IMAGE_CHANNEL2_CALIBRATED)
    {
        nvptl_debug_printf("leftir1\n");
    }
}

void rightircallback(void *data, void *userdata) {
    NVPTL_USBHeaderDataPacket *tmppack = (NVPTL_USBHeaderDataPacket *)data;
    int width = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket)))->width;
    int height = ((NVPFM_USB_IMAGE_HEADER *)((unsigned char *)data + sizeof(NVPTL_USBHeaderDataPacket)))->height;
    nvptl_debug_printf("right ir callback width:%d, height:%d\n", width, height);
    if (tmppack->sub_type == IMAGE_CHANNEL1_ORIGNAL ||
        tmppack->sub_type == IMAGE_CHANNEL1_CALIBRATED)
    {
        nvptl_debug_printf("rightir0\n");
    }
    else if (tmppack->sub_type == IMAGE_CHANNEL3_ORIGNAL ||
             tmppack->sub_type == IMAGE_CHANNEL3_CALIBRATED)
    {
        nvptl_debug_printf("rightir1\n");
    }
}

// void exposurecallback(void* pdata,int len, void* userdata) {
// 	s_nvpfm_get_sensor_exposure_ret* pinfo = (s_nvpfm_get_sensor_exposure_ret*)pdata;
// 	if (pinfo->ret != 0)return;


// 	if (pinfo->exposure == CHANNEL0 || pinfo->exposure.channel == CHANNEL1) {
//         nvpfm_debug_printf("exposure callback ir expousure:%d\n", pinfo->exposure);
//     }
//     else {
// 		nvpfm_debug_printf("exposure callback rgb expousure:%d\n", pinfo->exposure);
// 	}
// }

void highprecisioncallback(void* pdata,int len, void* userdata) {
	s_nvpfm_high_precision* pinfo = (s_nvpfm_high_precision*)pdata;
    nvpfm_debug_printf("highprecisioncallback enable:%d\n", pinfo->enable);
}

void repeatfiltercallback(void* pdata,int len, void* userdata) {
	s_nvpfm_repeated_texture_filter* pinfo = (s_nvpfm_repeated_texture_filter*)pdata;
    nvpfm_debug_printf("repeatfiltercallback enable:%d, threshold:%d, winSize:%f\n",
     pinfo->enable, pinfo->threshold, pinfo->winSize);
}

void spatialfiltercallback(void* pdata,int len, void* userdata) {
	s_nvpfm_spatial_filter* pinfo = (s_nvpfm_spatial_filter*)pdata;
    nvpfm_debug_printf("spatialfiltercallback enable:%d, sigmaColor:%f, sigmaSpace:%f\n",
     pinfo->enable, pinfo->sigmaColor, pinfo->sigmaSpace);
}

void highlightfiltercallback(void* pdata,int len, void* userdata) {
	s_nvpfm_depth_high_light* pinfo = (s_nvpfm_depth_high_light*)pdata;
    nvpfm_debug_printf("highlightfiltercallback enable:%d\n", pinfo->enable);
}
