#ifndef __FEYNMAN_UTILS__
#define __FEYNMAN_UTILS__
#include "transferlayer/commondata.h"
#include "ring_queue.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct {
		unsigned char r;
		unsigned char g;
		unsigned char b;
	}MYRGB;

#ifdef _WINDOWS
#include <windows.h>
#else
#include <time.h>
#include <semaphore.h>
#endif

#ifdef _WINDOWS
#define log_printf(format, ...)                                                     \
    do                                                                              \
    {                                                                               \
        char tmpstr[512];                                                           \
        SYSTEMTIME sTime;                                                           \
        GetLocalTime(&sTime);                                                       \
        sprintf(tmpstr, "[%02d/%02d %02d:%02d:%02d.%03d", sTime.wMonth, sTime.wDay, \
                sTime.wHour, sTime.wMinute, sTime.wSecond, sTime.wMilliseconds);    \
        printf("%s %s %d] " format, tmpstr, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
    } while (0)
#else
#define log_printf(format, ...)                                                               \
    do                                                                                        \
    {                                                                                         \
        char tmpstr[512];                                                                     \
        struct timeval tv;                                                                    \
        gettimeofday(&tv, NULL);                                                              \
        struct tm *sTime = localtime(&tv.tv_sec);                                             \
        sprintf(tmpstr, "[%02d/%02d %02d:%02d:%02d.%03ld", sTime->tm_mon + 1, sTime->tm_mday, \
                sTime->tm_hour, sTime->tm_min, sTime->tm_sec, tv.tv_usec / 1000);             \
        printf("%s %s %d] " format, tmpstr, __FUNCTION__, __LINE__, ##__VA_ARGS__);           \
    } while (0)
#endif

	typedef struct {
		int row;
		int col;
	}RGBPOS;

#ifdef _WINDOWS
	struct timezone2
	{
		__int32 tz_minuteswest; /* minutes W of Greenwich */
		BOOL tz_dsttime;		/* type of dst correction */
	};

	struct timeval2
	{
		__int32 tv_sec;	 /* seconds */
		__int32 tv_usec; /* microseconds */
	};


	int gettimeofday(struct timeval2 *tv, struct timezone2 *tz);
    // typedef usb_dev_handle *USBHANDLE;
    typedef HANDLE THREADHANDLE;
    typedef HANDLE SEM_T;
    typedef HANDLE MUTEXHANDLE;
    typedef struct timeval2 MYTIMEVAL;
#else
typedef int SOCKET;
typedef pthread_t THREADHANDLE;
typedef sem_t SEM_T;
typedef struct timeval MYTIMEVAL;
typedef pthread_mutex_t *MUTEXHANDLE;
#endif

    int SEM_TRYWAIT(SEM_T *sem);
    void SEM_INIT(SEM_T *sem, int pshared, unsigned int value);

    int SEM_TIMEDWAIT(SEM_T *sem, int milliseconds);
    void SEM_DESTROY(SEM_T *sem);
    void SEM_POST(SEM_T *sem);
    void COMMONUSLEEP(int microseconds);

    MUTEXHANDLE CREATEMUTEX();
    void MUTEXLOCK(MUTEXHANDLE handle);
    void MUTEXUNLOCK(MUTEXHANDLE handle);
    void CLOSEMUTEX(MUTEXHANDLE handle);
    /*
    #ifdef _WINDOWS
        // typedef usb_dev_handle *USBHANDLE;
        typedef HANDLE THREADHANDLE;
        typedef HANDLE SEM_T;
        typedef struct timeval2 MYTIMEVAL;
        typedef HANDLE MUTEXHANDLE;
    #else
    typedef int SOCKET;
    typedef pthread_t THREADHANDLE;
    typedef sem_t SEM_T;
    typedef struct timeval MYTIMEVAL;
    typedef pthread_mutex_t *MUTEXHANDLE;
    #endif
    */

/*
	typedef struct
	{
		int timeout;
		NVPFM_COMMAND_SUB_TYPE responsesubtype;
		void *responsedata;
		int *responselen;
		MYTIMEVAL starttm;
#ifdef _WINDOWS
		HANDLE waitsem;
#else
		SEM_T *waitsem;
#endif
		BOOL *hastimeout;
	} sendcmd_t;
*/

/*
typedef struct _nvpfm_list_async_node
{
	time_t committime;
	NVPFM_PRIMARY_TYPE responsetype;
	NVPFM_COMMAND_SUB_TYPE responsesubtype;

	ASYNCCALLBACK callback;
	void* userdata;
	struct _nvpfm_list_async_node *next;
} NVPFM_LIST_ASYNC_NODE;
*/

    typedef struct
    {
        double _x;
        double _y;
    } CSample;
typedef struct _sampledeque
{
    struct _sampledeque *prev;
    struct _sampledeque *next;
    CSample sample;
} sampledeque;

typedef struct{
double _prev_a;
double _prev_b; // Linear regression coeffitions - previously used values.
double _dest_a;
double _dest_b; // Linear regression coeffitions - recently calculated.
double _prev_time; 
double _last_request_time;
sampledeque *_last_values;
CSample _base_sample;// = {0, 0};
double max_device_time;
}TIMESTAMPINFO;

char* mybasename(const char*);
char* mydirname(const char*);


void THREADJOIN(THREADHANDLE handle, void* pointer);
#ifdef __cplusplus
}
#endif
#endif
