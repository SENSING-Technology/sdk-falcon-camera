#ifndef __CNN_PROCESS__
#define __CNN_PROCESS__

#ifdef __cplusplus
extern "C" {
#endif
typedef struct
{
  float x1;
  float y1;
  float x2;
  float y2;
} Box_xyxy;

typedef struct
{
  int cls_id;
  float conf;
  Box_xyxy box2d;
} Object;

typedef struct {
  const char *labelname;
  int red;
  int green;
  int blue;
} LABELINFO;

typedef enum {
  CNN_YOLOX_TYPE,
  CNN_HAND_TYPE,
  CNN_UNKNOWN_TYPE
} EM_CNN_TYPE;

float APP_FP16_To_FP32(short inp);

#ifdef __cplusplus
}
#endif
#endif
