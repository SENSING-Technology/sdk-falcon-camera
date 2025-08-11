#define _CRT_SECURE_NO_DEPRECATE

#include <vector>
#include <math.h>

#include "nvpfm.hpp"
#include "cnn_process.h"
#include "hand_post_process.h"
#define NVP_USE_CACHE 1
#define CONF_THRESH -1.5 //-0.8473
#define NMS_THRESH 0.45
#define OBJ_THRESH -1.5
#define RESIZE_HEIGHT 416
#define RESIZE_WIDTH 416
#define CHANNEL 39
#define CLASSES 8
#define N 3
#define FLT_MAX_TEST 3.402823466e+38F

static LABELINFO labelinfo[] = {
    {"four", 125, 0, 0},
    {"mute", 0, 125, 0},
    {"ok", 0, 0, 125},
    {"one", 0, 125, 125},
    {"palm", 125, 0, 125},
    {"peace", 125, 125, 0},
    {"three", 255, 0, 0},
    {"no_gesture", 0, 255, 0}};

LABELINFO get_hand_label_by_index(int index) {
  return labelinfo[index];
}

static float aSigmoid(float x) {
  return (log(x / (1.f - x)));
}

static float f32_sigmoid(float x) {
  return (1 / (1 + exp(-x)));
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

static void nms_process(std::vector<Object> &objects, std::vector<Object> &filterObjects, const float nms_thres, const int num_cls) {
  for (int i = 0; i < num_cls; i++) {
    int num_proposal_cls = 0;
    Object proposal_cls[1000];

    for (int j = 0; j < objects.size(); j++) {
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
}

void hand_post_process(short *bufs[], s_nvpfm_cnn_data *pdata) {
  static int stride[N] = {8, 16, 32};
  static int mask[3][6] = {
      {10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}};

  float conf_thres = CONF_THRESH;
  float nms_thres = NMS_THRESH;
  float obj_thres = OBJ_THRESH;
  int channel = CHANNEL;
  int resize_height = RESIZE_HEIGHT;
  int resize_width = RESIZE_WIDTH;
  int number_classes = channel / N - 5; // 4
  int number_channel = channel / N;     // 9
  int num_proposal = 0;
  std::vector<Object> objects;
  std::vector<Object> filterObjects;

  for (int n = 0; n < 3; n++) {
    int num_grid_x = resize_width / stride[n];
    int num_grid_y = resize_height / stride[n];
    short *buf = bufs[n];

    for (int i = 0; i < num_grid_y; i++) {
      for (int j = 0; j < num_grid_x; j++) {

        short *bufLine = buf + i * 39 * num_grid_x + j * 39;

        for (int k = 0; k < N; k++) {
          float box_score = APP_FP16_To_FP32(bufLine[(4 + number_channel * k)]);
          if (box_score <= obj_thres)
            continue;

          int class_index = 0;
          float class_score = -FLT_MAX_TEST;

          for (int m = 0; m < number_classes; m++) {
            float score = APP_FP16_To_FP32(bufLine[(5 + m + number_channel * k)]);
            if (score > class_score) {
              class_index = m;
              class_score = score;
            }
          }

          if (class_score > conf_thres) {
            Object tmp_det;
            float anchor_w = mask[n][k * 2];
            float anchor_h = mask[n][k * 2 + 1];

            float dx = f32_sigmoid(APP_FP16_To_FP32(bufLine[(0 + number_channel * k)]));
            float dy = f32_sigmoid(APP_FP16_To_FP32(bufLine[(1 + number_channel * k)]));
            float dw = f32_sigmoid(APP_FP16_To_FP32(bufLine[(2 + number_channel * k)]));
            float dh = f32_sigmoid(APP_FP16_To_FP32(bufLine[(3 + number_channel * k)]));

            float pb_cx = (dx * 2.f - 0.5f + j) * stride[n];
            float pb_cy = (dy * 2.f - 0.5f + i) * stride[n];
            float pb_w = pow(dw * 2.f, 2) * anchor_w;
            float pb_h = pow(dh * 2.f, 2) * anchor_h;

            tmp_det.box2d.x1 = pb_cx - pb_w * 0.5f;
            tmp_det.box2d.y1 = pb_cy - pb_h * 0.5f;
            tmp_det.box2d.x2 = pb_cx + pb_w * 0.5f;
            tmp_det.box2d.y2 = pb_cy + pb_h * 0.5f;

            // tmp_det.conf = box_score * class_score;
            // tmp_det.conf = f32_sigmoid(box_score) * f32_sigmoid(class_score);

            tmp_det.conf = f32_sigmoid(class_score);
            tmp_det.cls_id = class_index;
            objects.push_back(tmp_det);
          }
        }
      }
    }
  }

  nms_process(objects, filterObjects, nms_thres, number_classes);
  int index = 0;
  for (int i = 0; i < filterObjects.size() && i < MAX_CNN_GROUP_NUM; i++) {
    Box_xyxy box = filterObjects.at(i).box2d;
    if (box.x1 >= 0.0 && box.x1 <= (float)resize_width &&
        box.x2 >= 0.0 && box.x2 <= (float)resize_width &&
        box.y1 >= 0.0 && box.y1 <= (float)resize_height &&
        box.y2 >= 0.0 && box.y2 <= (float)resize_height) {
      pdata->group[index].label = filterObjects[i].cls_id;
      pdata->group[index].xmin = box.x1 / (float)resize_width;
      pdata->group[index].xmax = box.x2 / (float)resize_width;
      pdata->group[index].ymin = box.y1 / (float)resize_height;
      pdata->group[index].ymax = box.y2 / (float)resize_height;
      pdata->group[index].score = filterObjects[i].conf;
      index++;
    }
  }
  pdata->groups = index;

  return;
}
