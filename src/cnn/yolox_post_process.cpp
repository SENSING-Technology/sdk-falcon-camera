#include "nvpfm.hpp"
#include "cnn_process.h"
#include "yolox_post_process.h"
#include <math.h>
#define DEFAULT_DSP_DEV 1 // cnn forward
#define cnnwidth 416
#define cnnheight 416

/*for yolox post process*/
#define CONF_THRESH 0
#define NMS_THRESH 0.4
#define OBJ_THRESH 0
#define RESIZE_HEIGHT 416
#define RESIZE_WIDTH 416
#define CHANNEL 7
#define CLASSES 2
#define N 3
#define FLT_MAX_TEST 3.402823466e+38F

static LABELINFO label_info[] = {
    {"person", 125, 0, 0},
    {"bird", 0, 125, 0},
    {"cat", 0, 0, 125},
    {"cow", 0, 125, 125},
    {"dog", 125, 0, 125},
    {"horse", 125, 125, 0},
    {"sheep", 255, 0, 0},
    {"aeroplane", 0, 255, 0},
    {"bicycle", 0, 0, 255},
    {"boat", 0, 255, 255},
    {"bus", 255, 0, 255},
    {"car", 255, 255, 0},
    {"motorbike", 255, 255, 255},
    {"train", 255, 125, 125},
    {"bottle", 125, 255, 125},
    {"chair", 125, 125, 255},
    {"dining table", 125, 255, 255},
    {"potted plant", 255, 125, 255},
    {"sofa", 255, 255, 125},
    {"tv_monitor", 125, 125, 125}};

LABELINFO get_yolox_label_by_index(int index) {
  return label_info[index];
}

/*for yolox post process*/
static float f32_sigmoid(float x) {
  return (1 / (1 + exp(-x)));
}

float APP_FP16_To_FP32(short inp) {
  int mant;
  int exp;
  float val = 0;
  int *pval;
  pval = (int *)&val;
  mant = (inp & 0x3FF) << 13;
  exp = (inp >> 10) & 0x1F;
  if (exp > 15) {
    exp = 127 + (exp - 15);
  } else if (exp < 15) {
    exp = 127 - (15 - exp);
  } else {
    exp = 127;
  }
  //-
  if (inp == 0) {
    *pval = 0;
  } else if (inp & (1 << 15)) {
    *pval = 0x80000000 | (exp << 23) | mant;
  } else {
    *pval = (exp << 23) | mant;
  }
  return val;
}
static void proposals_sort(Object *proposals_cls, const int size) {
  for (int i = 0; i < (int)(size - 1); i++) {
    for (int j = i + 1; j < size; j++) {
      if (proposals_cls[i].conf < proposals_cls[j].conf) {
        Object tmp = proposals_cls[j];
        proposals_cls[j] = proposals_cls[i];
        proposals_cls[i] = tmp;
      }
    }
  }
}

static float box2d_iou(Box_xyxy box1, Box_xyxy box2) {
  float left = box1.x1 > box2.x1 ? box1.x1 : box2.x1;
  float right = box1.x2 > box2.x2 ? box2.x2 : box1.x2;
  float w = right - left;
  float top = box1.y1 > box2.y1 ? box1.y1 : box2.y1;
  float bottom = box1.y2 > box2.y2 ? box2.y2 : box1.y2;
  float h = bottom - top;

  if (w <= 0 || h <= 0)
    return 0;

  float inter_area = w * h;
  float union_area = (box1.x2 - box1.x1) * (box1.y2 - box1.y1) +
                     (box2.x2 - box2.x1) * (box2.y2 - box2.y1) - inter_area;
  if (inter_area == 0 || union_area == 0)
    return 0;

  return inter_area / union_area;
}

static int nms_process(std::vector<Object> &objects, std::vector<Object> &filterObjects, const float nms_thres, const int num_cls, int size) {
  Object *proposal_cls = (Object *)malloc(sizeof(Object) * size);
  int filterObjects_num = 0;
  for (int i = 0; i < num_cls; i++) {
    int num_proposal_cls = 0;
    // Object proposal_cls[size];
    memset(proposal_cls, 0, sizeof(Object) * size);

    for (int j = 0; j < size; j++) {
      if (objects[j].cls_id == i) {
        proposal_cls[num_proposal_cls++] = objects[j];
      }
    }

    if (num_proposal_cls > 1) {
      proposals_sort(proposal_cls, num_proposal_cls);

      for (int j = 0; j < num_proposal_cls - 1; j++) {
        if (proposal_cls[j].conf == 0)
          continue;

        Box_xyxy box_a = proposal_cls[j].box2d;
        for (int k = j + 1; k < num_proposal_cls; k++) {
          Box_xyxy box_b = proposal_cls[k].box2d;
          if (box2d_iou(box_a, box_b) > nms_thres)
            proposal_cls[k].conf = 0;
        }
      }

      for (int j = 0; j < num_proposal_cls; j++) {
        if (proposal_cls[j].conf != 0) {
          filterObjects.push_back(proposal_cls[j]);
        }
      }
    }
  }
  free(proposal_cls);
  return filterObjects.size();
}

void yolox_post_process(short *bufs[], s_nvpfm_cnn_data *pdata) {
  int stride[N] = {8, 16, 32};
  float conf_thres = CONF_THRESH;
  float nms_thres = NMS_THRESH;
  float obj_thres = OBJ_THRESH;
  int resize_height = RESIZE_HEIGHT;
  int resize_width = RESIZE_WIDTH;
  int number_classes = 20;
  int number_channel = CHANNEL;
  int num_proposal = 0;

  int num = 0;
	std::vector<Object> objects;
	std::vector<Object> filterObjects;
  for (int n = 0; n < 3; n++) {
    int num_grid_x = resize_width / stride[n];
    int num_grid_y = resize_height / stride[n];
    short *buf = bufs[n];
    for (int i = 0; i < num_grid_y; i++) {
      for (int j = 0; j < num_grid_x; j++) {
        short *bufLine = buf + i * 25 * num_grid_x + j * 25;
        float box_score = APP_FP16_To_FP32(bufLine[4]);
        if (box_score <= obj_thres)
          continue;

        int class_index = 0;
        float class_score = -FLT_MAX_TEST;
        for (int m = 0; m < number_classes; m++) {
          float score = APP_FP16_To_FP32(bufLine[(5 + m)]);
          if (score > class_score) {
            class_index = m;
            class_score = score;
          }
        }
        if (class_score > conf_thres) {
          Object tmp_det;
          float c_x = (APP_FP16_To_FP32(bufLine[0]) + j) * stride[n];
          float c_y = (APP_FP16_To_FP32(bufLine[1]) + i) * stride[n];
          float w = exp(APP_FP16_To_FP32(bufLine[2])) * stride[n];
          float h = exp(APP_FP16_To_FP32(bufLine[3])) * stride[n];
          tmp_det.box2d.x1 = c_x - w * 0.5f;
          tmp_det.box2d.y1 = c_y - h * 0.5f;
          tmp_det.box2d.x2 = c_x + w * 0.5f;
          tmp_det.box2d.y2 = c_y + h * 0.5f;
          tmp_det.conf = f32_sigmoid(box_score) * f32_sigmoid(class_score);
          tmp_det.cls_id = class_index;
          objects.push_back(tmp_det);
        }
      } // j
    }
  } // n

  int object_num = nms_process(objects, filterObjects, nms_thres, number_classes, objects.size());
  int index = 0;
  for (int i = 0; i < object_num && index < MAX_CNN_GROUP_NUM; i++) {
    if (filterObjects[i].box2d.x1 >= 0.0 && filterObjects[i].box2d.x1 <= (float)resize_width &&
        filterObjects[i].box2d.x2 >= 0.0 && filterObjects[i].box2d.x2 <= (float)resize_width &&
        filterObjects[i].box2d.y1 >= 0.0 && filterObjects[i].box2d.y1 <= (float)resize_height &&
        filterObjects[i].box2d.y1 >= 0.0 && filterObjects[i].box2d.y2 <= (float)resize_height) {
      pdata->group[index].label = filterObjects[i].cls_id;
      pdata->group[index].xmin = filterObjects[i].box2d.x1 / (float)resize_width;
      pdata->group[index].xmax = filterObjects[i].box2d.x2 / (float)resize_width;
      pdata->group[index].ymin = filterObjects[i].box2d.y1 / (float)resize_height;
      pdata->group[index].ymax = filterObjects[i].box2d.y2 / (float)resize_height;
      pdata->group[index].score = filterObjects[i].conf;
      index++;
    }
  }

  pdata->groups = index;

  return;
}