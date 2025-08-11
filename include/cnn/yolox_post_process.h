#ifndef __YOLOX_POST_PROCESS__
#define __YOLOX_POST_PROCESS__

#ifdef __cplusplus

extern "C" {
#endif


LABELINFO get_yolox_label_by_index(int index);

void yolox_post_process(short* bufs[], s_nvpfm_cnn_data* pdata);


#ifdef __cplusplus
}
#endif
#endif
