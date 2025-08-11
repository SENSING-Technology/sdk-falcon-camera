#include "../include/depthfilter/smearfilter.h"
void fillzeros(const cv::Mat src, cv::Mat &dst, int radius) {
  radius = std::max(radius, 1);
  dst = src.clone();
  int h = src.rows;
  int w = src.cols;
  const uint16_t *srcData = (uint16_t *)src.data;
  uint16_t *dstData = (uint16_t *)dst.data;
  for (int y = 0; y < h; y++) {
    int rowId = y * w;
    for (int x = 0; x < w; x++) {
      if (srcData[rowId + x] > 0) {
        continue;
      }
      uint16_t &dstDepth = dstData[rowId + x];
      float depth = 0.0f;
      int count = 0;
      for (int i = -radius; i <= radius; i++) {
        int r = y + i;
        if (r < 0 || r >= h) {
          continue;
        }
        int winRowId = r * w;
        for (int j = -radius; j <= radius; j++) {
          int c = x + j;
          if (c < 0 || c >= w) {
            continue;
          }
          const uint16_t &srcDepth = srcData[winRowId + c];
          if (srcDepth == 0) {
            continue;
          }
          depth += srcDepth;
          count++;
        }
      }
      if (count == 0) {
        continue;
      }
      depth = depth / count;
      dstDepth = uint16_t(depth + 0.5f);
    }
  }
}

void smearfilter(const cv::Mat src, cv::Mat &dst, int radius, float threshold, int p) {
  threshold *= 2.0f;
  p = std::min(std::max(p, 1), 2);
  cv::Mat tempDepthMap;
  fillzeros(src, tempDepthMap, radius);
  cv::Mat Gx, Gy;
  Sobel(tempDepthMap, Gx, CV_16SC1, 1, 0, 1);
  Sobel(tempDepthMap, Gy, CV_16SC1, 0, 1, 1);
  dst = src.clone();
  int h = src.rows;
  int w = src.cols;
  int imgSize = h * w;
  const uint16_t *srcData = (uint16_t *)src.data;
  uint16_t *dstData = (uint16_t *)dst.data;
  const int16_t *GxData = (int16_t *)Gx.data;
  const int16_t *GyData = (int16_t *)Gy.data;
  for (int i = 0; i < imgSize; i++) {
    if (srcData[i] == 0) {
      continue;
    }
    // float GradNorm = sqrtf(GxData[i] * GxData[i] + GyData[i] * GyData[i]);
    float GradNorm = (p == 1) ? (abs(GxData[i]) + abs(GyData[i])) : (sqrtf(GxData[i] * GxData[i] + GyData[i] * GyData[i]));
    if (GradNorm >= srcData[i] * srcData[i] * threshold) {
      dstData[i] = 0;
    }
  }
}
