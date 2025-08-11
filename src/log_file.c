#include <sys/syscall.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <fcntl.h>
#include <time.h>

#include "error.h"
#include "log_file.h"

#define PATH_MAX 4096

static void _update_file(handle_file_t *fh);

typedef struct
{
  uint8_t *wkey;
  int cflag;
  char *file_path;
  FILE *f_log;
  char *io_buf;
  size_t io_cap;
  size_t max_file_size;
  size_t cur_file_size;
  size_t cur_bak_num;
  size_t max_bak_num;
  pthread_mutex_t mutex;
} file_handle_t;

#if !defined(S_ISREG)
#define S_ISREG(x) (((x) & S_IFMT) == S_IFREG)
#endif

static uint64_t _get_file_size(const char *fullfilepath) {
  int ret;
  struct stat sbuf;
  ret = stat(fullfilepath, &sbuf);
  if (ret || !S_ISREG(sbuf.st_mode))
    return 0;
  return (uint64_t)sbuf.st_size;
}

static void _log_lock(handle_file_t *l) {
  pthread_mutex_lock(&l->mutex);
}

static void _log_unlock(handle_file_t *l) {
  pthread_mutex_unlock(&l->mutex);
}

static int _iobuf_init(handle_file_t *fh, size_t io_ms) {
  fh->io_cap = io_ms;

  fh->io_buf = calloc(1, fh->io_cap);

  if (!fh->io_buf)
    return 1;

  return 0;
}

static int _file_init(handle_file_t *fh, const char *lp, size_t ms, size_t mb) {
  fh->max_file_size = ms;
  fh->file_path = strdup(lp);
  if (!fh->file_path)
    return 1;
  fh->cur_file_size = _get_file_size(lp);
  fh->cur_bak_num = 0;
  fh->max_bak_num = mb;
  return 0;
}

void *_file_handle_create(const char *log_filename, size_t max_file_size, size_t max_file_bak, size_t max_iobuf_size) {
  if (!log_filename)
    return NULL;
  handle_file_t *l = calloc(1, S_LOG_FILE_SIZE);
  if (!l)
    exit_throw("failed to calloc!");
  int ret;
  ret = _file_init(l, log_filename, max_file_size, max_file_bak);
  if (ret != 0) {
    free(l);
    return NULL;
  }

  ret = _iobuf_init(l, max_iobuf_size);
  if (ret != 0) {
    free(l->file_path);
    free(l);
    return NULL;
  }

  ret = pthread_mutex_init(&l->mutex, NULL);
  assert(ret == 0);

  trace_add_handle((void **)&l);

  return (void *)l;
}

static void _lock_uninit(handle_file_t *l) {
  pthread_mutex_destroy(&l->mutex);
}

static void _backup_file(handle_file_t *fh) {
  char new_filename[PATH_MAX] = {0};

  sprintf(new_filename, "%s.bak%lu", fh->file_path, fh->cur_bak_num % fh->max_bak_num);
  rename(fh->file_path, new_filename);
}

static void _update_file(handle_file_t *fh) {
  if (!fh->f_log)
    return;

  fclose(fh->f_log);
  fh->f_log = NULL;
  if (fh->max_bak_num == 0) { // without backup
    remove(fh->file_path);
  } else {
    _backup_file(fh);
  }
  fh->cur_bak_num++;
  fh->cur_file_size = 0;
}

void file_handle_flush(handle_file_t *fh) {
  if (fh == NULL)
    return;

  _log_lock(fh);

  fclose(fh->f_log);
  fh->f_log = NULL;

  _log_unlock(fh);
}

void file_handle_destory(handle_file_t *fh) {
  if (fh == NULL)
    return;

  _log_lock(fh);
  if (fh->f_log != NULL) {
    fclose(fh->f_log);

    fh->f_log = NULL;
  }
  if (fh->file_path != NULL) {
    free(fh->file_path);
    fh->file_path = NULL;
  }
  //_backup_file(fh);

  free(fh->io_buf);
  free(fh->wkey);

  trace_remove_handle((void **)&fh);

  _log_unlock(fh);

  _lock_uninit(fh);

  free(fh);

  fh = NULL;
}

static int _file_handle_request(handle_file_t *fh) {
  if (fh->f_log)
    return 0;

  fh->f_log = fopen(fh->file_path, "a+");

  if (!fh->f_log)
    return 1;

  if (setvbuf(fh->f_log, fh->io_buf, _IOFBF, fh->io_cap) != 0)
    return 1;

  return 0;
}

void write_file(handle_file_t *fh, char *msg, size_t len) {
  _log_lock(fh);

  if (0 != _file_handle_request(fh)) {
    perror("Failed to open the file\n");
    return;
  }

  size_t print_size = fwrite(msg, sizeof(char), len, fh->f_log);
  if (print_size != len) {
    error_display("Failed to write the file path(%s), len(%lu)\n", fh->file_path, print_size);
    return;
  }
  fh->cur_file_size += print_size;

  if (fh->cur_file_size > fh->max_file_size) {
    _update_file(fh);
  }

  _log_unlock(fh);
}

#define RESET "\x1B[0m"

#define NON_NULL(x) ((x) ? x : "")

void write_stream(handle_stream_t *sh, log_level_t level, char *msg) {
  fprintf((sh->streams)[level], "%s%s%s", (sh->style)[level], msg, RESET);
}

void *_stream_handle_create(uint8_t streams) {
  handle_stream_t *sh = calloc(1, S_LOG_STREAM_SIZE);
  assert(sh != NULL);
  int i = _ERROR_LEVEL + 1;
  while (--i >= _DEBUG_LEVEL) {
    (sh->streams)[i] = streams & (i + 1) ? stderr : stdout;
  }
  return (void *)sh;
}

void *set_stream_param(void *sh, log_level_t level, const char *color, const char *bgcolor, const char *style) {
  if (!sh || ((log_handle_t *)sh)->tag != S_MODE || !(((log_handle_t *)sh)->hld)) {
    error_display("INPUT Stream Handle ERROR!");
    return sh;
  }
  memset((((handle_stream_t *)(((log_handle_t *)sh)->hld))->style)[level], 0, STYLE_SIZE);
  sprintf((((handle_stream_t *)(((log_handle_t *)sh)->hld))->style)[level], "%s%s%s", NON_NULL(color), NON_NULL(bgcolor), NON_NULL(style));
  return sh;
}

void stream_handle_flush() {
  fflush(stdout);
}

void stream_handle_destory(handle_stream_t *sh) {
  stream_handle_flush();
  free(sh);
}
