#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/syscall.h> /* For SYS_xxx definitions */
#include <stdlib.h>
#include <time.h>
#include "error.h"
#include "log_file.h"
#include "log.h"

uint32_t GET_MODE(uint32_t filelevel, uint32_t streamlevel, uint32_t loglevel) {
  return (filelevel <= loglevel ? (streamlevel <= loglevel ? F_S_MODE : F_MODE) : S_MODE);
}
int g_is_open = 1;
msg_list *g_plist = NULL;
pthread_t write_thread_id;
static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
static const char *const severity[] = {"[DEBUG]", "[INFO]", "[WARN]", "[ERROR]"};
#define START_LIST

log_handle_t *stream_handle_create(uint8_t streams) {
  log_handle_t *lh = calloc(1, S_LOG_HANDLE_T);
  if (!lh)
    exit_throw("failed to calloc!");
  lh->tag = S_MODE;
  lh->hld = _stream_handle_create(streams);
  return lh;
}

static void *loop_write(void *param) {
  log_t *lhs = (log_t *)param;
  assert(lhs != NULL);
  log_t *lhs_bk = lhs;
  while (g_is_open) {
    pthread_mutex_lock(&g_mutex);
    while (g_plist) {
      data_msg *pdata = (data_msg *)(g_plist->data);
      while (lhs) {
        switch ((((log_handle_t *)(lhs->data))->tag) & (pdata->mode)) {
        case F_MODE:
          write_file((handle_file_t *)(((log_handle_t *)(lhs->data))->hld), pdata->log_msg, pdata->datalen);
          break;
        case S_MODE:
          write_stream((handle_stream_t *)(((log_handle_t *)(lhs->data))->hld), pdata->level, pdata->log_msg);
          break;
        default:
          break;
        }
        lhs = lhs->next;
      }
      free(g_plist->data);
      void *tmplist = g_plist;
      g_plist = g_plist->next;
      free(tmplist);
      lhs = lhs_bk;
    }
    // delelte
    g_plist = NULL;
    pthread_mutex_unlock(&g_mutex);
    usleep(500 * 1000);
  }
  pthread_mutex_lock(&g_mutex);
  while (g_plist) {
    data_msg *pdata = (data_msg *)(g_plist->data);
    while (lhs) {
      switch ((((log_handle_t *)(lhs->data))->tag) & (pdata->mode)) {
      case F_MODE:
        write_file((handle_file_t *)(((log_handle_t *)(lhs->data))->hld), pdata->log_msg, pdata->datalen);
        break;
      case S_MODE:
        write_stream((handle_stream_t *)(((log_handle_t *)(lhs->data))->hld), pdata->level, pdata->log_msg);
        break;
      default:
        break;
      }
      lhs = lhs->next;
    }
    free(g_plist->data);
    void *tmplist = g_plist;
    g_plist = g_plist->next;
    free(tmplist);
    lhs = lhs_bk;
  }
  // delelte
  g_plist = NULL;
  pthread_mutex_unlock(&g_mutex);

  return NULL;
}

void _log_start(log_t *lhs) {
  (void)lhs;
  (void)loop_write;
  g_is_open = 1;
#ifdef START_LIST
  // create write thread
  pthread_create(&write_thread_id, 0, loop_write, (void *)lhs);
#endif
}

void _log_close() {
  g_is_open = 0;
}

log_handle_t *create_log_file(const char *log_filename, size_t max_file_size, size_t max_file_num, size_t max_iobuf_size) {
  log_handle_t *lh = calloc(1, S_LOG_HANDLE_T);
  if (!lh)
    exit_throw("failed to calloc!");
  lh->tag = F_MODE;
  lh->hld = _file_handle_create(log_filename, max_file_size, max_file_num, max_iobuf_size);
  return lh;
}

log_t *add_to_handle_list(log_t *lhs, void *hld) {
  return list_insert_beginning(lhs, hld);
}

msg_list *add_to_msg_list(msg_list *plist, void *data) {
  return list_insert_end(plist, data);
}

void _log_write(uint32_t mode, log_t *lhs, log_level_t level, const char *format, ...) {
  // printf("mode:%u\n",mode);
  if (!g_is_open)
    return;

  if (!lhs) {
    error_display("Log handle is null!\n");
    return;
  }

  if (!format) {
    error_display("Log format is null!\n");
    return;
  }

  va_list args;
  va_start(args, format);
#ifdef START_LIST
  data_msg *pmsg_data = malloc(sizeof(data_msg));
  memset(pmsg_data, 0x00, sizeof(data_msg));
#else
  char log_msg[EX_SINGLE_LOG_SIZE] = {0};
#endif
/* debug message */
#if (LOG_DEBUG_MESSAGE != 0)
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_t rawtime = tv.tv_sec; // time(NULL);
  struct tm timeinfo;
  localtime_r(&rawtime, &timeinfo);
  pid_t tid = syscall(SYS_gettid);
#ifdef START_LIST
  int info_len = snprintf(pmsg_data->log_msg, EX_SINGLE_LOG_SIZE, "%s%04d-%02d-%02d %02d:%02d:%02d,%ld %s<%s> %d: ", severity[level], timeinfo.tm_year + 1900,
                          timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000, va_arg(args, char *), va_arg(args, char *), tid);
#else
  int info_len = snprintf(log_msg, EX_SINGLE_LOG_SIZE, "%s%04d-%02d-%02d %02d:%02d:%02d,%ld %s<%s> %d: ", severity[level], timeinfo.tm_year + 1900,
                          timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, tv.tv_usec / 1000, va_arg(args, char *), va_arg(args, char *), tid);
#endif
#else
  int info_len = strlen(severity[level]);
  strncpy(log_msg, severity[level], info_len);
#endif
#ifdef START_LIST
  int total_len = vsnprintf(pmsg_data->log_msg + info_len, SINGLE_LOG_SIZE - info_len, format, args);
#else
  int total_len = vsnprintf(log_msg + info_len, SINGLE_LOG_SIZE - info_len, format, args);
#endif
  va_end(args);

  if (total_len <= 0 || info_len <= 0 || total_len > SINGLE_LOG_SIZE - info_len) {
    error_display("Failed to vsnprintf a text entry: (total_len) %d\n", total_len);
    return;
  }
  total_len += info_len;
#ifdef START_LIST
  pmsg_data->level = level;
  pmsg_data->datalen = total_len;
  pmsg_data->mode = mode;
  pthread_mutex_lock(&g_mutex);
  g_plist = add_to_msg_list(g_plist, (void *)pmsg_data);
  pthread_mutex_unlock(&g_mutex);
#else
  while (lhs) {
    switch ((((log_handle_t *)(lhs->data))->tag) & mode) {
    case F_MODE:
      write_file((handle_file_t *)(((log_handle_t *)(lhs->data))->hld), log_msg, total_len);
      break;
    case S_MODE:
      write_stream((handle_stream_t *)(((log_handle_t *)(lhs->data))->hld), level, log_msg);
      break;
    case F_S_MODE:
      printf("not process of F_S_MODE!!!\n");
      break;
    default:
      break;
    }
    lhs = lhs->next;
  }
#endif
}

void log_flush(log_t *lhs) {
  stream_handle_flush(); // stdout flush only once
  while (lhs && (((log_handle_t *)(lhs->data))->tag) == F_MODE) {
    file_handle_flush((handle_file_t *)(((log_handle_t *)(lhs->data))->hld));
    lhs = lhs->next;
  }
}

void log_destory(log_t *lhs) {
  log_t *tmp = lhs;
  g_is_open = 0;
#ifdef START_LIST
  pthread_join(write_thread_id, NULL);
#endif
  while (lhs) {
    switch ((((log_handle_t *)(lhs->data))->tag)) {
    case F_MODE:
      file_handle_destory((handle_file_t *)(((log_handle_t *)(lhs->data))->hld));
      break;
    case S_MODE:

      stream_handle_destory((handle_stream_t *)(((log_handle_t *)(lhs->data))->hld));
      break;
    default:
      break;
    }
    free(lhs->data);
    tmp = lhs;
    lhs = lhs->next;
    free(tmp);
  }
}
