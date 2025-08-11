#ifndef __RING_QUEUE__
#define __RING_QUEUE__
#ifdef __cplusplus
extern "C" {
#endif
// #include <assert.h>
// #include <string.h>

typedef unsigned char u_char;

#define CAN_WRITE 0x00
#define CAN_READ 0x01
#define READING 0x02
#define WRITING 0x03

typedef struct tag {
  u_char tag_value;
} TAG;

typedef struct
{
  u_char *_queue_p;
  int _nmemb;
  int _size;
  volatile int _read_now;
  volatile int _write_now;
} Ring_Queue;

Ring_Queue *Create_Ring_Queue(int nmemb, int size);
void Destroy_Ring_Queue(Ring_Queue *);
u_char *SOLO_Read(Ring_Queue *);
void SOLO_Read_Over(Ring_Queue *);
u_char *SOLO_Write(Ring_Queue *);
void SOLO_Write_Over(Ring_Queue *);

u_char *queue_peek_nth(Ring_Queue *, u_char *queue_p, int pos);
#ifdef __cplusplus
}
#endif
#endif