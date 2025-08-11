
#include <stdlib.h>
#include "libyuv/cpu_id.h"
#include "libyuv/rotate.h"
#include <string.h>
#include <sys/time.h>
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
#include <Eigen>
using namespace Eigen;

void nv12_rotate_90_i420(uint8_t *pdata, int width, int height) {
  NV12TestRotate(width, height, height, width,
                 kRotate90, pdata, pdata);
}

void nv12_rotate_270_i420(uint8_t *pdata, int width, int height) {
  NV12TestRotate(width, height, height, width,
                 kRotate270, pdata, pdata);
}
void grayscale_rotate_90(uint8_t *pdata, int width, int height) {
  Map<Matrix<uint8_t, Dynamic, Dynamic, RowMajor>> mymat(pdata, height, width);
  Matrix<uint8_t, Dynamic, Dynamic, RowMajor> tmpmat = mymat.transpose();
  tmpmat.rowwise().reverseInPlace();
  memcpy(pdata, tmpmat.data(), width * height);
}
void grayscale_rotate_270(uint8_t *pdata, int width, int height) {
  Map<Matrix<uint8_t, Dynamic, Dynamic, RowMajor>> mymat(pdata, height, width);
  mymat.rowwise().reverseInPlace();
  Matrix<uint8_t, Dynamic, Dynamic, RowMajor> tmpmat = mymat.transpose();
  memcpy(pdata, tmpmat.data(), width * height);
}

int main(int argc, const char *argv[]) {
  uint8_t *pdata = (uint8_t *)malloc(640 * 480 * 3 / 2);
  printf("nv12 rotate 90 degree 10000 times:\n");
  struct timeval tv1, tv2;
  gettimeofday(&tv1, NULL);
  for (int i = 0; i < 10000; i++) {
    nv12_rotate_90_i420(pdata, 480, 640);
  }
  gettimeofday(&tv2, NULL);
  double offset = (double)(tv2.tv_sec - tv1.tv_sec) + (double)(tv2.tv_usec - tv2.tv_usec) / 1000000.0;
  printf("cost:%f seconds!\n", offset);

  printf("nv12 rotate 270 degree 10000 times:\n");
  gettimeofday(&tv1, NULL);
  for (int i = 0; i < 10000; i++) {
    nv12_rotate_270_i420(pdata, 480, 640);
  }
  gettimeofday(&tv2, NULL);
  offset = (double)(tv2.tv_sec - tv1.tv_sec) + (double)(tv2.tv_usec - tv2.tv_usec) / 1000000.0;
  printf("cost:%f seconds!\n", offset);
  printf("grayscale rotate 90 degree 10000 times:\n");
  gettimeofday(&tv1, NULL);
  for (int i = 0; i < 10000; i++) {
    grayscale_rotate_90(pdata, 480, 640);
  }
  gettimeofday(&tv2, NULL);
  offset = (double)(tv2.tv_sec - tv1.tv_sec) + (double)(tv2.tv_usec - tv2.tv_usec) / 1000000.0;
  printf("cost:%f seconds!\n", offset);
  printf("grayscale rotate 90 degree 10000 times:\n");
  gettimeofday(&tv1, NULL);
  for (int i = 0; i < 10000; i++) {
    grayscale_rotate_270(pdata, 480, 640);
  }
  gettimeofday(&tv2, NULL);
  offset = (double)(tv2.tv_sec - tv1.tv_sec) + (double)(tv2.tv_usec - tv2.tv_usec) / 1000000.0;
  printf("cost:%f seconds!\n", offset);
  return 0;
}