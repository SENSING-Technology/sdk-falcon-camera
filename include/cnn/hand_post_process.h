#ifndef __HAND_POST_PROCESS__
#define __HAND_POST_PROCESS__

#ifdef __cplusplus

extern "C" {
#endif

LABELINFO get_hand_label_by_index(int index);

void hand_post_process(short *bufs[], s_nvpfm_cnn_data *pdata);

#ifdef __cplusplus
}
#endif
#endif
