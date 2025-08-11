#ifdef _WINDOWS
#include <Windows.h>
#include <stdio.h>
#include "nvpfm.h"
#include "commondef.h"
#include "utils.h"

void COMMONUSLEEP(int microseconds) {
  Sleep(microseconds / 1000);
}

#else
#include "nvpfm.h"
#include "utils.h"
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
void COMMONUSLEEP(int microseconds) {
  int seconds = microseconds / 1000000;
  if (seconds > 0) {
    sleep(seconds);
  }
  usleep(microseconds % 1000000);
}

#endif
#include <string.h>

char *mydirname(const char *path) {
  char tmppath[256] = {0};
  memset(tmppath, 0, 256);
  char *tmpbasename = mybasename(path);
  int filelen = strlen(tmpbasename);
  memcpy(tmppath, path, strlen(path) - filelen);
  free(tmpbasename);
  return strdup(tmppath);
}
char *mybasename(const char *path) {
  int i = strlen(path);
  for (; i >= 0; i--) {
    if (path[i] == '\\' || path[i] == '/') {
      break;
    }
  }
  return strdup(&path[i + 1]);
}
MUTEXHANDLE CREATEMUTEX() {
#ifdef _WINDOWS
  return CreateMutex(NULL, FALSE, NULL);
#else
  pthread_mutex_t *mutex = (pthread_mutex_t *)calloc(1, sizeof(pthread_mutex_t));
  pthread_mutex_init(mutex, NULL);
  return mutex;
#endif
}

void MUTEXLOCK(MUTEXHANDLE handle) {
#ifdef _WINDOWS
  // printf("mutexlock tid:0x%X\n", GetCurrentThreadId());
  DWORD dw = WaitForSingleObject(handle, INFINITE);
  if (WAIT_FAILED == dw || WAIT_TIMEOUT == dw) {
    CloseHandle(handle);
  }
#else
  pthread_mutex_lock(handle);
#endif
}
void MUTEXUNLOCK(MUTEXHANDLE handle) {
#ifdef _WINDOWS
  // printf("mutexunlock tid:0x%X\n", GetCurrentThreadId());
  ReleaseMutex(handle);
#else
  pthread_mutex_unlock(handle);
#endif
}

void CLOSEMUTEX(MUTEXHANDLE handle) {
#ifdef _WINDOWS
  CloseHandle(handle);
#else
  pthread_mutex_destroy(handle);
  free(handle);
#endif
}
/*
void SEM_INIT(SEM_T *sem, int pshared, unsigned int value)
{
#ifdef _WINDOWS
  *sem = CreateSemaphore(NULL, 0, 1, NULL);
#else
  sem_init(sem, pshared, value);
#endif
}

void SEM_DESTROY(SEM_T *sem)
{
#ifdef _WINDOWS
  CloseHandle(*sem);
#else
  sem_destroy(sem);
#endif
}
void SEM_POST(SEM_T *sem)
{
#ifdef _WINDOWS
  ReleaseSemaphore(*sem, 1, NULL);
#else
  sem_post(sem);
#endif
}
int SEM_TIMEDWAIT(SEM_T *sem, int milliseconds)
{
#ifdef _WINDOWS
  DWORD dwWaitResult = WaitForSingleObject(
    *sem,		   // handle to semaphore
    milliseconds); // zero-second time-out interval

  switch (dwWaitResult)
  {
  case WAIT_OBJECT_0:
    return 0;

    // The semaphore was nonsignaled, so a time-out occurred.
  case WAIT_TIMEOUT:
    // printf("Thread %d: wait timed out\n", GetCurrentThreadId());
    return -2;
  }
  return -1;
#else
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += milliseconds / 1000;
  ts.tv_nsec += (milliseconds % 1000) * 1000000;

  int ret = sem_timedwait(sem, &ts);
  if (ret == -1 && errno == ETIMEDOUT) {
    return -2;
  }
  if (ret == -1 && errno != ETIMEDOUT) {
    return -1;
  }
  return ret;
#endif
}*/

#ifdef _WINDOWS
// typedef usb_dev_handle *USBHANDLE;
typedef HANDLE THREADHANDLE;
typedef HANDLE SEM_T;
typedef struct timeval2 MYTIMEVAL;
#else
typedef int SOCKET;
typedef pthread_t THREADHANDLE;
typedef sem_t SEM_T;
typedef struct timeval MYTIMEVAL;
#endif

void SEM_INIT(SEM_T *sem, int pshared, unsigned int value) {
#ifdef _WINDOWS
  *sem = CreateSemaphore(NULL, 0, 1, NULL);
#else
  sem_init(sem, pshared, value);
#endif
}

void SEM_DESTROY(SEM_T *sem) {
#ifdef _WINDOWS
  CloseHandle(*sem);
#else
  sem_destroy(sem);
#endif
}
void SEM_POST(SEM_T *sem) {
#ifdef _WINDOWS
  ReleaseSemaphore(*sem, 1, NULL);
#else
  sem_post(sem);
#endif
}
int SEM_TIMEDWAIT(SEM_T *sem, int milliseconds) {
#ifdef _WINDOWS
  DWORD dwWaitResult = WaitForSingleObject(
      *sem,          // handle to semaphore
      milliseconds); // zero-second time-out interval

  switch (dwWaitResult) {
  case WAIT_OBJECT_0:
    return 0;

    // The semaphore was nonsignaled, so a time-out occurred.
  case WAIT_TIMEOUT:
    // printf("Thread %d: wait timed out\n", GetCurrentThreadId());
    return -1;
  }
  return -1;
#else
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += milliseconds / 1000;
  ts.tv_nsec += (milliseconds % 1000) * 1000000;

  return sem_timedwait(sem, &ts);
#endif
}
int SEM_TRYWAIT(SEM_T *sem) {
  return SEM_TIMEDWAIT(sem, 30);
}
int SEM_TRYWAIT_MILLISECONDS(SEM_T *sem, int ms) {
  return SEM_TIMEDWAIT(sem, ms);
}

void THREADJOIN(THREADHANDLE handle, void *pointer) {
#ifndef _WINDOWS
  pthread_join(handle, pointer);
#else
  DWORD retfirst = WaitForSingleObject(handle, INFINITE);
#endif
}