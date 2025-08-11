#include "ring_queue.h"
#include <stdlib.h>
#include <string.h>
Ring_Queue *Create_Ring_Queue(int nmemb, int size) {
  if (nmemb <= 0 || size <= 0) {
    return NULL;
  }
  Ring_Queue *tmpqueue = (Ring_Queue *)calloc(1, sizeof(Ring_Queue));
  tmpqueue->_nmemb = nmemb;
  tmpqueue->_size = size;
  tmpqueue->_read_now = 0;
  tmpqueue->_write_now = 0;
  tmpqueue->_queue_p = (u_char *)malloc(nmemb * (sizeof(TAG) + size));
  memset(tmpqueue->_queue_p, 0, nmemb * (sizeof(TAG) + size));

  return tmpqueue;
}
void Destroy_Ring_Queue(Ring_Queue *ring) {
  if (ring->_queue_p)
    free(ring->_queue_p);
  free(ring);
}
u_char *SOLO_Read(Ring_Queue *ring) {
  u_char *g_p = 0;
  TAG *tag_p = 0;
  u_char *user_data = 0;

  g_p = queue_peek_nth(ring, ring->_queue_p, ring->_read_now);
  tag_p = (TAG *)g_p;
  if (tag_p->tag_value == CAN_READ) {
    user_data = (u_char *)g_p + sizeof(TAG);
    tag_p->tag_value = READING;
  }
  return user_data;
}
void SOLO_Read_Over(Ring_Queue *ring) {
  u_char *g_p = 0;
  TAG *tag_p = 0;

  g_p = queue_peek_nth(ring, ring->_queue_p, ring->_read_now);
  tag_p = (TAG *)g_p;
  if (tag_p->tag_value == READING) {
    tag_p->tag_value = CAN_WRITE;
    ring->_read_now = (ring->_read_now + 1) % ring->_nmemb;
  }
}
u_char *SOLO_Write(Ring_Queue *ring) {
  u_char *g_p = 0;
  TAG *tag_p = 0;
  u_char *user_data = 0;

  g_p = queue_peek_nth(ring, ring->_queue_p, ring->_write_now);
  tag_p = (TAG *)g_p;
  if (tag_p->tag_value == CAN_WRITE) {
    user_data = (u_char *)g_p + sizeof(TAG);
    tag_p->tag_value = WRITING;
  }
  return user_data;
}
void SOLO_Write_Over(Ring_Queue *ring) {
  u_char *g_p = 0;
  TAG *tag_p = 0;

  g_p = queue_peek_nth(ring, ring->_queue_p, ring->_write_now);
  tag_p = (TAG *)g_p;
  if (tag_p->tag_value == WRITING) {
    tag_p->tag_value = CAN_READ;
    ring->_write_now = (ring->_write_now + 1) % ring->_nmemb;
  }
}
u_char *queue_peek_nth(Ring_Queue *ring, u_char *queue_p, int pos) {
  u_char *rst = 0;
  if (queue_p && pos < ring->_nmemb) {
    rst = queue_p + pos * (sizeof(TAG) + ring->_size);
  }
  return rst;
}
