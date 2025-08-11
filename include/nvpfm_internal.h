#ifndef __NVPFM_INTERNAL__
#define __NVPFM_INTERNAL__
#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  ASYNCCALLBACK callback;
  void *userdata;

  int timeout;
  NVPFM_PRIMARY_TYPE responsetype;
  NVPFM_COMMAND_SUB_TYPE responsesubtype;
  void *responsedata;
  int *responselen;
  MYTIMEVAL starttm;
#ifdef _WINDOWS
  HANDLE waitsem;
#else
  SEM_T *waitsem;
#endif
} sendcmd_t;

typedef struct _nvpfm_list_node {
  sendcmd_t data;
  struct _nvpfm_list_node *next;
} NVPFM_LIST_NODE;

typedef struct
{
  NVPTL_INSTANCE *transferlayerhandle;

  unsigned long index;
  int currentpacketlen;
  SOCKET device_net_socket;
  grouppkt_info_custom_t groupbuffer;

  MUTEXHANDLE timestampmutex;
  MUTEXHANDLE setgetmutex;

  int timestamp_is_ready;
  TIMESTAMPINFO *tminfo;

  int thread_running_flag;
  int global_time_enabled;
  int irwidth;
  int irheight;
  int rgbwidth;
  int rgbheight;
  int rgb_rectify_mode;
  // EVENTCALLBACK eventcallback;
  //	Ring_Queue *sendqueue;
  Ring_Queue *responsequeue;
  NVPFM_LIST_NODE *responselisthead;
  // NVPFM_LIST_ASYNC_NODE *asyncresponsehead;
  FRAMECALLBACK framecallback;
  Ring_Queue *otherpktqueue;
  Ring_Queue *depthpktqueue;
  Ring_Queue *leftirpktqueue;
  Ring_Queue *rightirpktqueue;
  Ring_Queue *rgbpktqueue;
  Ring_Queue *grouppktqueue;

  Ring_Queue *savequeue;
  int b_save;
  Ring_Queue *imupktqueue;
  Ring_Queue *goodfeaturepktqueue;
  Ring_Queue *savepktqueue;
  THREADHANDLE recvthread;
  THREADHANDLE timestampthread;
  THREADHANDLE depththread;
  THREADHANDLE rgbthread;
  THREADHANDLE groupthread;
  THREADHANDLE imuthread;
  THREADHANDLE goodfeaturethread;
  THREADHANDLE otherthread;
  THREADHANDLE leftirthread;
  THREADHANDLE rightirthread;
  THREADHANDLE savethread;
  //	THREADHANDLE sendthread;
  uint8_t *usb_buf;
  uint8_t *tmpbuf;
  int willclose;
  s_nvpfm_upgrade_result upgraderesult;

  SEM_T upgradesem;
  SEM_T depthsem;
  SEM_T rgbsem;
  SEM_T groupsem;
  SEM_T leftirsem;
  SEM_T rightirsem;
  SEM_T imusem;
  SEM_T goodfeaturesem;
  SEM_T othersem;
  SEM_T postconnectsem;
  FRAMECALLBACK depthcallback;
  FRAMECALLBACK rgbcallback;
  GROUPFRAMECALLBACK groupcallback;
  FRAMECALLBACK leftircallback;
  FRAMECALLBACK rightircallback;
  FRAMECALLBACK savecallback;
  FRAMECALLBACK othercallback;
  FRAMECALLBACK imucallback;
  FRAMECALLBACK goodfeaturecallback;

  struct libusb_transfer *transfer;
  struct libusb_transfer *sendtransfer;

  int timesynccycle;
  int b_timesync;
  time_t lasttimesync;
} NVPFM_INSTANCE;

#ifdef __cplusplus
}
#endif

#endif