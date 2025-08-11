#ifdef _WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <time.h>
#include <strsafe.h>
#include <stringapiset.h>
#pragma comment(lib, "ws2_32.lib")
#include <process.h>
#include <iptypes.h>
#include <iphlpapi.h>
#pragma comment(lib, "iphlpapi.lib")
#else
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/tcp.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <unistd.h>
#include <netdb.h>
#include <ifaddrs.h>
#include <signal.h>
#include <stdarg.h>
#endif
#include <stdlib.h>
#include <string.h>

#include "transferlayer/commondata.h"

#define BUFFER_SIZE 1024 * 1024
#define POST_SEND 1
#define POST_RECV 2
#define KEY_CLOSE 1
#define EPOLL_NUM 8
#define OFFSET_OF(TYPE, MEMBER) ((size_t) & ((TYPE *)0)->MEMBER)

#if defined(_WINDOWS)
typedef struct _PER_IO_CONTEXT {
  OVERLAPPED ov;
  WSABUF wsabuf;
  char buffer[BUFFER_SIZE];
  char *data;
  int datalen;
  int recvd;
  int type;
  struct _PER_IO_CONTEXT *next;
  struct _PER_IO_CONTEXT *prev;
} PER_IO_CONTEXT, *PPER_IO_CONTEXT;

static int print_dbg_msg(const char *func, const int line, char *fmt, ...) {
  TCHAR *buf = NULL;
  int code = (int)GetLastError();
  FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL, code, 0, (LPTSTR)&buf, 0, NULL);
  if (buf != NULL) {
    int size = WideCharToMultiByte(CP_ACP, 0, buf, -1, NULL, 0, NULL, NULL);
    char *mb = LocalAlloc(LPTR, size);
    WideCharToMultiByte(CP_ACP, 0, buf, -1, mb, size, NULL, NULL);
    LocalFree(buf);
    // char* ch = strchr(mb, '\r');
    // if (ch != NULL) *ch = 0x00;
    *(mb + strlen(mb) - 4) = 0x00;
    char format[1024];
    SYSTEMTIME st = {0};
    GetLocalTime(&st);
    snprintf(format, sizeof(format) - 1, "dbg %02d:%02d:%02d.%03d000 %s:%d wincode %d:%s %s\n",
             st.wHour, st.wMinute, st.wSecond, st.wMilliseconds, func, line, code, (char *)mb, fmt);
    LocalFree(mb);
    char output[2048];
    va_list arg_ptr;
    va_start(arg_ptr, fmt);
    vsnprintf(output, sizeof(output) - 1, format, arg_ptr);
    va_end(arg_ptr);
    OutputDebugStringA(output);
    printf(output);
  }
  return code;
}

#else
typedef struct _WSABUF {
  unsigned long len;
  char *buf;
} WSABUF, *LPWSABUF;

typedef struct _PER_IO_CONTEXT {
  SOCKET fd;
  WSABUF wsabuf;
  char buffer[BUFFER_SIZE];
  void *data;
  int datalen;
  int recvd;
  int type;
  struct _PER_IO_CONTEXT *next;
  struct _PER_IO_CONTEXT *prev;
} PER_IO_CONTEXT, *PPER_IO_CONTEXT;

static int print_dbg_msg(const char *func, const int line, char *fmt, ...) {
  int code = errno;
  char format[1024];
  struct timespec ts = {0, 0};
  clock_gettime(CLOCK_REALTIME, &ts);
  unsigned long long tm = (unsigned long long)(ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL);
  time_t sec = tm / 1000000LL;
  struct tm *ptm = localtime(&sec);
  snprintf(format, sizeof(format) - 1, "dbg %02d:%02d:%02d.%06d %s:%d gnucode %d:%s %s\n",
           ptm->tm_hour, ptm->tm_min, ptm->tm_sec, (int)(tm % 1000000LL), func, line, code, (char *)strerror(code), fmt);
  char output[2048];
  va_list arg_ptr;
  va_start(arg_ptr, fmt);
  vsnprintf(output, sizeof(output) - 1, format, arg_ptr);
  va_end(arg_ptr);
  printf("%s", (char *)output);
  return code;
}

static void handle_sigpipe(int sig) {
  printf("net wire plug out!\n");
  (void)sig;
}
#endif

#define PRINT_DBG_MSG(fmt, ...) print_dbg_msg(__FUNCTION__, __LINE__, fmt, ##__VA_ARGS__)

static int do_data(NVPTL_INSTANCE *inst, PPER_IO_CONTEXT ctx, int trans) {
  int ret = 0;
  int offset = 0;
  bool flag = ctx->datalen > BUFFER_SIZE ? true : false;
  char *data = flag ? ctx->data : ctx->buffer;
  int datalen = flag ? ctx->datalen : BUFFER_SIZE;
  ctx->recvd += trans;
__do_data:
  if (ctx->recvd >= sizeof(NVPTL_HeaderDataPacket)) {
    if (0 == strncmp("NEXT_VPU", data + offset, 8)) {
      NVPTL_HeaderDataPacket *header = (NVPTL_HeaderDataPacket *)(data + offset);
      ctx->datalen = header->len + sizeof(NVPTL_HeaderDataPacket);
      if (ctx->recvd >= ctx->datalen) {
        if (inst->recvframecallback != NULL) {
          inst->recvframecallback((NVPTL_DEVICE_HANDLE)inst, (void *)(data + offset), ctx->datalen, inst->userdata);
          // if (ctx->datalen > 60)
          {
            // PRINT_DBG_MSG("camera %s callback data len[%d]", inst->devinfo.net_camera_ip, ctx->datalen);
          }
        }
        ctx->recvd -= ctx->datalen;
        offset += ctx->datalen;
        ctx->datalen = 0;
        goto __do_data;
      } else {
        if (ctx->datalen < BUFFER_SIZE) {
          if (offset > 0) {
            memcpy(ctx->buffer, data + offset, ctx->recvd);
          }
          if (flag) {
            free(ctx->data);
            ctx->data = NULL;
          }
        } else {
          if (flag) {
            if (ctx->datalen < datalen) {
              if (offset > 0) {
                memcpy(ctx->data, data + offset, ctx->recvd);
              }
            } else {
              char *buf = malloc(ctx->datalen);
              if (buf != NULL) {
                memcpy(buf, data + offset, ctx->recvd);
                free(ctx->data);
                ctx->data = buf;
              } else {
                free(ctx->data);
                ctx->data = NULL;
                ret = -1;
                PRINT_DBG_MSG("malloc fail");
              }
            }
          } else {
            ctx->data = malloc(ctx->datalen);
            if (ctx->data != NULL) {
              memcpy(ctx->data, data + offset, ctx->recvd);
            } else {
              ret = -1;
              PRINT_DBG_MSG("malloc fail");
            }
          }
        }
      }
    } else {
      ctx->datalen = 0;
      ctx->recvd = 0;
    }
  } else {
    if (offset > 0 && ctx->recvd > 0) {
      memcpy(ctx->buffer, data + offset, ctx->recvd);
    }
    if (flag) {
      free(ctx->data);
      ctx->data = NULL;
    }
  }
  if (ctx->datalen > BUFFER_SIZE) {
    ctx->wsabuf.buf = ctx->data + ctx->recvd;
    ctx->wsabuf.len = ctx->datalen - ctx->recvd;
  } else {
    ctx->wsabuf.buf = ctx->buffer + ctx->recvd;
    ctx->wsabuf.len = BUFFER_SIZE - ctx->recvd;
  }
  return ret;
}
static void nvptl_recv_frame_callback(NVPTL_DEVICE_HANDLE handle, uint8_t *data, unsigned long len, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  if (inst->recvframecallback != NULL) {
    inst->recvframecallback(handle, data, len, userdata);
  }
}
/*#if defined(_WINDOWS)
static unsigned __stdcall get_data(void *param) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)param;
  ULONG_PTR key = 0;
  DWORD flags = 0;
  BOOL status = FALSE;
  while (true) {
    LPOVERLAPPED lpol = NULL;
    DWORD trans = 0;
    status = GetQueuedCompletionStatus(inst->device_net_handle, (LPDWORD)&trans, (PULONG_PTR)&key, (LPOVERLAPPED *)&lpol, INFINITE);
    if (status) {
      PPER_IO_CONTEXT ctx = CONTAINING_RECORD(lpol, PER_IO_CONTEXT, ov);
      if (NULL == ctx) {
        PRINT_DBG_MSG("camera %s", inst->devinfo.net_camera_ip);
        continue;
      }
      if (0 == trans) {
        if (key == KEY_CLOSE) {
          // free(ctx->data);
          free(ctx);
          InterlockedExchange(&inst->device_net_connect, STOPPED);
          PRINT_DBG_MSG("camera %s do close iocp stopped", inst->devinfo.net_camera_ip);
          break;
        }
      }
      if (ctx->type == POST_SEND) {
        PRINT_DBG_MSG("camera %s do send len[%d] data[%*.*s]", inst->devinfo.net_camera_ip, ctx->wsabuf.len,
                      sizeof(NVPTL_HeaderDataPacket), sizeof(NVPTL_HeaderDataPacket), (char *)ctx->wsabuf.buf);
        if (InterlockedExchangeAdd(&inst->device_net_sendcnt, 0) > 1) {
          PPER_IO_CONTEXT sendctx = (PPER_IO_CONTEXT)inst->device_net_sendlist;
          InterlockedExchangeAdd(&inst->device_net_sendcnt, -1);
          inst->device_net_sendlist = (void *)sendctx->next;
          if (sendctx->datalen > BUFFER_SIZE) {
            free(sendctx->data);
          }
          free(sendctx);
        }
      } else if (ctx->type == POST_RECV) {
        if (trans > 0 && do_data(inst, ctx, trans) != 0) {
          InterlockedExchange(&inst->device_net_connect, STOPPED);
          PRINT_DBG_MSG("camera %s do recv iocp stopped", inst->devinfo.net_camera_ip);
          break;
        }
        DWORD recvd = 0;
        if (NO_ERROR != WSARecv(inst->device_net_socket, &ctx->wsabuf, 1, &recvd, &flags, (LPWSAOVERLAPPED)&ctx->ov, NULL)) {
          if (WSA_IO_PENDING != GetLastError()) {
            InterlockedExchange(&inst->device_net_connect, STOPPED);
            PRINT_DBG_MSG("camera %s post recv iocp stopped", inst->devinfo.net_camera_ip);
            break;
          }
        }
      }
    } else {
      PRINT_DBG_MSG("camera[%s] iocp status[%d]", inst->devinfo.net_camera_ip, status);
    }
  }

  return 0;
}
#else*/
#if defined(_WINDOWS)
static unsigned __stdcall get_data(void *param) {
#else
static void *get_data(void *param) {
#endif
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)param;
  if (inst->usb_buf == NULL)
    inst->usb_buf = (uint8_t *)malloc(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);
  inst->index = 0;
  while (!inst->willclose) {
    int buffersize = ((USB_PACKET_MAX_SIZE - inst->index) > (64 * 1024) ? (64 * 1024) : (USB_PACKET_MAX_SIZE - inst->index));
    // printf("will recv buffer:%d bytes!\n", buffersize);
    ssize_t ret = recv(inst->device_net_socket, inst->usb_buf + inst->index, buffersize, 0);
    if (ret > 0) {
      //  printf("recv %d bytes\n", ret);
      inst->index += ret;
      if ((long)inst->index >= (long)sizeof(NVPTL_USBHeaderDataPacket)) {
        if (0 == strncmp("NEXT_VPU", (const char *)inst->usb_buf, 8)) {
          inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
          if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
            //	printf("1userdata:0x%X\n", inst);
            nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);
            inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
            if (inst->index > 0) {
              memcpy(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
            } else {
              //	printf("no more data left:index=%d!\n", inst->index);
            }
          } else {
            //	printf("index too small tobe a complete frame:want:%d,has:%d\n", (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
          }
        } else {
          //	printf("first is not NEXT_VPU!will search\n");
          int searchednextvpu = 0;
          for (unsigned long i = 0; i < (inst->index - 8); i++) {
            if (0 == memcmp(&inst->usb_buf[i], "NEXT_VPU", 8)) {
              //	printf("searched NEXT_VPU!\n");
              searchednextvpu = 1;
              memcpy(inst->usb_buf, &inst->usb_buf[i], inst->index - i);
              inst->index = inst->index - i;

              inst->currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
              if (inst->index >= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
                //	printf("2userdata:0x%X\n", inst);
                nvptl_recv_frame_callback(inst, inst->usb_buf, (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->userdata);
                inst->index -= (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket));
                if (inst->index > 0) {
                  memcpy(inst->usb_buf, inst->usb_buf + (inst->currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket)), inst->index);
                }
              }
              break;
            }
          }
          if (!searchednextvpu) {
            //	printf("has no nextvpu,so be it!!!\n");
            inst->index = 0;
          }
        }
      } else {
        //	printf("index too small tobe header:%d\n",inst->index);
      }
    } else {
      nvpfm_debug_printf("recv return:%d,err:%d\n", ret, errno);
      if (inst->eventcallback != NULL) {
        inst->eventcallback(PLUGOUT, inst->connectuserdata);
        break;
      }
    }
  }

  if (inst->usb_buf != NULL)
    free(inst->usb_buf);
  /* struct epoll_event evts[EPOLL_NUM];
   int ready = 0;
   while (true) {
     ready = epoll_wait(inst->device_net_handle, (struct epoll_event *)&evts, EPOLL_NUM, -1);
     if (ready == -1) {
       if (errno == EINTR) {
         // evts[0].events = EPOLLIN;
         // ready = 1;
         // goto __do_event;
         continue;
       }
       PRINT_DBG_MSG("camera %s epoll stopped", inst->devinfo.net_camera_ip);
       break;
     }
     //__do_event:
     for (int i = 0; i < ready; ++i) {
       if (evts[i].events & EPOLLOUT) {
         PPER_IO_CONTEXT ctx = (PPER_IO_CONTEXT)evts[i].data.ptr;
         if (ctx != NULL) {
           if (ctx->fd != inst->device_net_socket) {
             PRINT_DBG_MSG("camera %s do send len[%d] data[%*.*s]", inst->devinfo.net_camera_ip, ctx->datalen,
                           sizeof(NVPTL_HeaderDataPacket), sizeof(NVPTL_HeaderDataPacket), (char *)ctx->data);
             continue;
           }
           int offset = 0;
           while (true) {
             int sent = send(ctx->fd, (char *)ctx->data + offset, ctx->datalen - offset, 0);
             if (sent > 0) {
               offset += sent;
               if (offset == ctx->datalen) {
                 break;
               }
             } else if (sent == -1) {
               if (errno == EAGAIN) {
                 offset = ctx->datalen;
                 break;
               } else if (errno == ECONNRESET) {
                 epoll_ctl(inst->device_net_handle, EPOLL_CTL_DEL, ctx->fd, 0);
                 // shutdown(ctx->fd, SHUT_RDWR);
                 // close(ctx->fd);
                 ctx->fd = INVALID_SOCKET;
                 break;
               } else if (errno == EINTR) {
                 continue;
               } else {
                 epoll_ctl(inst->device_net_handle, EPOLL_CTL_DEL, ctx->fd, 0);
                 // shutdown(ctx->fd, SHUT_RDWR);
                 // close(ctx->fd);
                 ctx->fd = INVALID_SOCKET;
                 break;
               }
             } else if (sent == 0) {
               epoll_ctl(inst->device_net_handle, EPOLL_CTL_DEL, ctx->fd, 0);
               // shutdown(ctx->fd, SHUT_RDWR);
               // close(ctx->fd);
               ctx->fd = INVALID_SOCKET;
               break;
             }
           }
           if (__sync_fetch_and_add(&inst->device_net_sendcnt, 0) > 1) {
             PPER_IO_CONTEXT sendctx = (PPER_IO_CONTEXT)inst->device_net_sendlist;
             __sync_fetch_and_sub(&inst->device_net_sendcnt, 1);
             inst->device_net_sendlist = (void *)sendctx->next;
             if (sendctx->datalen > BUFFER_SIZE) {
               free(sendctx->data);
             }
             free(sendctx);
           }
           if (ctx->fd != INVALID_SOCKET) {
             struct epoll_event evt;
             evt.data.ptr = inst->device_net_recvlast;
             evt.events = EPOLLIN | EPOLLET;
             epoll_ctl(inst->device_net_handle, EPOLL_CTL_MOD, ctx->fd, &evt);
             // PRINT_DBG_MSG("camera %s do send len[%d] data[%*.*s]", inst->devinfo.net_camera_ip, ctx->datalen,
             //	sizeof(NVPTL_HeaderDataPacket), sizeof(NVPTL_HeaderDataPacket), (char*)ctx->data);
           } else {
             __sync_synchronize();
             __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
             PRINT_DBG_MSG("camera %s do send epoll stopped", inst->devinfo.net_camera_ip);
             return 0;
           }
         } else {
           epoll_ctl(inst->device_net_handle, EPOLL_CTL_DEL, ctx->fd, 0);
           __sync_synchronize();
           __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
           PRINT_DBG_MSG("camera %s do close epoll stopped", inst->devinfo.net_camera_ip);
           return 0;
         }
       } else if (evts[i].events & EPOLLIN) {
         PPER_IO_CONTEXT ctx = (PPER_IO_CONTEXT)inst->device_net_recvlast;
         if (ctx != NULL) {
           int trans = 0;
           while (true) {
             int recvd = recv(ctx->fd, ctx->wsabuf.buf + trans, ctx->wsabuf.len - trans, 0);
             if (recvd > 0) {
               // PRINT_DBG_MSG("camera %s do recv epoll ---------------- recvd[%d] ----------------", inst->devinfo.net_camera_ip, recvd);
               trans += recvd;
               if (trans == ctx->wsabuf.len) {
                 if (do_data(inst, ctx, trans) != 0) {
                   __sync_synchronize();
                   __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
                   PRINT_DBG_MSG("camera %s do recv epoll stopped", inst->devinfo.net_camera_ip);
                   return 0;
                 }
                 trans = 0;
               }
               continue;
             } else if (recvd == -1) {
               if (errno == EAGAIN || errno == EWOULDBLOCK) {
                 break;
               } else if (errno == EINTR) {
                 continue;
               } else {
                 epoll_ctl(inst->device_net_handle, EPOLL_CTL_DEL, ctx->fd, 0);
                 // shutdown(ctx->fd, SHUT_RDWR);
                 // close(ctx->fd);
                 ctx->fd = INVALID_SOCKET;
                 break;
               }
             } else if (recvd == 0) {
               epoll_ctl(inst->device_net_handle, EPOLL_CTL_DEL, ctx->fd, 0);
               // shutdown(ctx->fd, SHUT_RDWR);
               // close(ctx->fd);
               ctx->fd = INVALID_SOCKET;
               break;
             }
           }
           if (trans > 0 && do_data(inst, ctx, trans) != 0) {
             __sync_synchronize();
             __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
             PRINT_DBG_MSG("camera %s do recv epoll stopped", inst->devinfo.net_camera_ip);
             return 0;
           }
           if (ctx->fd != INVALID_SOCKET) {
             struct epoll_event evt;
             evt.data.ptr = (void *)ctx;
             evt.events = EPOLLIN | EPOLLET;
             epoll_ctl(inst->device_net_handle, EPOLL_CTL_MOD, ctx->fd, &evt);
           } else {
             __sync_synchronize();
             __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
             PRINT_DBG_MSG("camera %s do recv epoll stopped", inst->devinfo.net_camera_ip);
             return 0;
           }
         } else {
           PRINT_DBG_MSG("camera %s do recv epoll data null", inst->devinfo.net_camera_ip);
           continue;
         }
       } else if ((evts[i].events & EPOLLERR) || (evts[i].events & EPOLLHUP) || (evts[i].events & EPOLLRDHUP)) {
         // close(evts[i].data.fd);
         __sync_synchronize();
         __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
         PRINT_DBG_MSG("camera %s epoll stopped", inst->devinfo.net_camera_ip);
         return 0;
       } else {
         PRINT_DBG_MSG("camera %s epoll event %d", inst->devinfo.net_camera_ip, evts[i].events);
       }
     }
   }*/
  return 0;
}
// #endif

void nvptl_net_init() {
#if defined(_WINDOWS)
  WORD ver = MAKEWORD(2, 2);
  WSADATA wasdata;
  WSAStartup(ver, &wasdata);
#else
  signal(SIGPIPE, handle_sigpipe);
#endif
}

void nvptl_net_deinit() {
#if defined(_WINDOWS)
  WSACleanup();
#else
#endif
}

NVPTL_RESULT nvptl_net_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices) {
  int ret = NVPTL_OK;
  NVPTL_DEVICE_INFO *devices = NULL;
#if defined(_WINDOWS)
  PIP_ADAPTER_INFO adapter_infos = (PIP_ADAPTER_INFO)calloc(1, sizeof(IP_ADAPTER_INFO));
  unsigned long adapter_size = sizeof(IP_ADAPTER_INFO);
  if ((ret = GetAdaptersInfo(adapter_infos, &adapter_size)) == ERROR_BUFFER_OVERFLOW) {
    free(adapter_infos);
    adapter_infos = (PIP_ADAPTER_INFO)calloc(1, adapter_size);
  }
  if ((ret = GetAdaptersInfo(adapter_infos, &adapter_size)) == ERROR_SUCCESS) {
    PIP_ADAPTER_INFO adapter = adapter_infos;
    while (adapter) {
      IP_ADDR_STRING *ip_addr = &(adapter->IpAddressList);
      while (ip_addr) {
        if (ip_addr->IpAddress.String != NULL) {
          struct sockaddr_in server;
          int len = sizeof(server);
          server.sin_family = AF_INET;
          server.sin_port = htons(8079);
          server.sin_addr.s_addr = inet_addr("255.255.255.255");
          SOCKET sk = socket(AF_INET, SOCK_DGRAM, 0);
          bool opt = true;
          setsockopt(sk, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));
          struct sockaddr_in addr;
          addr.sin_family = AF_INET;
          addr.sin_port = htons(0);
          addr.sin_addr.s_addr = inet_addr(ip_addr->IpAddress.String);
          if (0 != (ret = bind(sk, (struct sockaddr *)(&addr), sizeof(addr)))) {
            break;
          }
          int rto = 30;
          setsockopt(sk, SOL_SOCKET, SO_RCVTIMEO, (char *)&rto, sizeof(rto));
          char buffer[1024] = "list";
          bool ok = false;
          for (int i = 0; i < 2 && !ok; ++i) {
            if (sendto(sk, buffer, strlen(buffer) + 1, 0, (struct sockaddr *)&server, len) != SOCKET_ERROR) {
              for (int j = 0; j < 2 && !ok; ++j) {
                int n = recvfrom(sk, buffer, sizeof(buffer), 0, (struct sockaddr *)&server, &len);
                if (n > 0) {
                  if (0 == strncmp(buffer, "this is Falcon Broadcast", 25)) {
                    char ip[32] = {0};
                    strcpy(ip, inet_ntoa(server.sin_addr));
                    NVPTL_DEVICE_INFO *dev = *ppdevices;
                    for (; dev != NULL; dev = dev->next) {
                      if (strcmp(dev->net_camera_ip, ip) == 0) {
                        break;
                      }
                    }
                    if (dev == NULL) {
                      NVPTL_DEVICE_INFO *devinfo = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
                      devinfo->type = NVPTL_NETWORK_INTERFACE;
                      strcpy(devinfo->net_camera_ip, ip);
                      if (devices != NULL) {
                        devices->next = devinfo;
                      } else {
                        *ppdevices = devinfo;
                      }
                      devices = devinfo;
                      *ptotal += 1;
                      ok = true;
                    }
                  }
                }
              }
            }
          }
          closesocket(sk);
        }
        ip_addr = ip_addr->Next;
      }
      adapter = adapter->Next;
    }
  }
  free(adapter_infos);
#else
  struct ifaddrs *ifa = NULL;
  struct ifaddrs *ifa_ = NULL;
  getifaddrs(&ifa);
  for (ifa_ = ifa; ifa_ != NULL; ifa_ = ifa_->ifa_next) {
    if (!ifa_->ifa_addr)
      continue;
    if (ifa_->ifa_addr->sa_family == AF_INET) {
      char ip[32] = {0};
      inet_ntop(AF_INET, &((struct sockaddr_in *)ifa_->ifa_addr)->sin_addr, ip, sizeof(ip));
      if (ip[0] != 0x00) {
        struct sockaddr_in server;
        socklen_t len = sizeof(server);
        server.sin_family = AF_INET;
        server.sin_port = htons(8079);
        server.sin_addr.s_addr = inet_addr("255.255.255.255");
        SOCKET sk = socket(AF_INET, SOCK_DGRAM, 0);
        int opt = 1;
        setsockopt(sk, SOL_SOCKET, SO_REUSEADDR | SO_BROADCAST, (char *)&opt, sizeof(opt));
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(0);
        addr.sin_addr.s_addr = inet_addr(ip);
        if (0 != (ret = bind(sk, (struct sockaddr *)(&addr), sizeof(addr)))) {
          break;
        }
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 1000;
        setsockopt(sk, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, (socklen_t)sizeof(tv));
        char buffer[1024] = "list";
        bool ok = false;
        for (int i = 0; i < 2 && !ok; ++i) {
          if (sendto(sk, buffer, strlen(buffer) + 1, 0, (struct sockaddr *)&server, len) != -1) {
            for (int j = 0; j < 3 && !ok; ++j) {
              int n = recvfrom(sk, (void *)buffer, sizeof(buffer), 0, (struct sockaddr *)&server, &len);
              if (n > 0) {
                if (0 == strncmp(buffer, "this is Falcon Broadcast", 25)) {
                  strcpy(ip, inet_ntoa(server.sin_addr));
                  NVPTL_DEVICE_INFO *dev = *ppdevices;
                  for (; dev != NULL; dev = dev->next) {
                    if (strcmp(dev->net_camera_ip, ip) == 0) {
                      break;
                    }
                  }
                  if (dev == NULL) {
                    NVPTL_DEVICE_INFO *devinfo = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
                    devinfo->type = NVPTL_NETWORK_INTERFACE;
                    strcpy(devinfo->net_camera_ip, ip);
                    if (devices != NULL) {
                      devices->next = devinfo;
                    } else {
                      *ppdevices = devinfo;
                    }
                    devices = devinfo;
                    *ptotal += 1;
                    ok = true;
                  }
                }
              }
            }
          }
        }
        close(sk);
      }
    }
  }
  if (ifa != NULL) {
    freeifaddrs(ifa);
  }
#endif
  return ret;
}

NVPTL_DEVICE_HANDLE nvptl_net_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata) {
  // open and create read thread
  int ret = 0;
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)calloc(1, sizeof(NVPTL_INSTANCE));
  strcpy(inst->devinfo.net_camera_ip, dev_info->net_camera_ip);
  inst->devinfo.type = dev_info->type;
  inst->userdata = userdata;
  struct sockaddr_in si;
  memset(&si, 0, sizeof(struct sockaddr_in)); // 清零
  si.sin_family = AF_INET;
  si.sin_port = htons(8000);
  /*#if defined(_WINDOWS)
    si.sin_addr.S_un.S_addr = inet_addr(dev_info->net_camera_ip);
    inst->device_net_socket = WSASocket(PF_INET, SOCK_STREAM, IPPROTO_IP, NULL, 0, WSA_FLAG_OVERLAPPED);
    if (inst->device_net_socket != INVALID_SOCKET) {
      int ttl = 1;
      setsockopt(inst->device_net_socket, IPPROTO_IP, IP_TTL, (char *)&ttl, sizeof(ttl));
      int keepalive = 3;
      setsockopt(inst->device_net_socket, SOL_SOCKET, SO_KEEPALIVE, (char *)&keepalive, sizeof(keepalive));
      int timeo = 3000;
      setsockopt(inst->device_net_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeo, sizeof(timeo));
      setsockopt(inst->device_net_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeo, sizeof(timeo));
      int bufsize = BUFFER_SIZE;
      setsockopt(inst->device_net_socket, SOL_SOCKET, SO_SNDBUF, (char *)&bufsize, sizeof(bufsize));
      setsockopt(inst->device_net_socket, SOL_SOCKET, SO_RCVBUF, (char *)&bufsize, sizeof(bufsize));
      ret = connect(inst->device_net_socket, (struct sockaddr *)&si, sizeof(si));
      if (0 == ret) {
        unsigned long flag = 1;
        ioctlsocket(inst->device_net_socket, FIONBIO, &flag);
        inst->device_net_handle = CreateIoCompletionPort(INVALID_HANDLE_VALUE, 0, 0, 0);
        if (NULL != CreateIoCompletionPort((HANDLE)inst->device_net_socket, inst->device_net_handle, (DWORD)0, 0)) {
          inst->recvframecallback = callback;
          InterlockedExchange(&inst->device_net_connect, STARTED);
          inst->device_net_thread = (HANDLE)_beginthreadex(NULL, 0, &get_data, (void *)inst, 0, NULL);
          inst->status = CONNECTED;
          PRINT_DBG_MSG("camera %s iocp started", inst->devinfo.net_camera_ip);
        } else {
          ret = -1;
        }
      }
      if (0 == ret) {
        PPER_IO_CONTEXT ctx = (PPER_IO_CONTEXT)calloc(1, sizeof(PER_IO_CONTEXT));
        if (ctx != NULL) {
          ctx->type = POST_RECV;
          ctx->wsabuf.buf = ctx->buffer;
          ctx->wsabuf.len = BUFFER_SIZE;
          DWORD recvd = 0;
          DWORD flags = 0;
          if (NO_ERROR == WSARecv(inst->device_net_socket, &ctx->wsabuf, 1, &recvd, &flags, (LPWSAOVERLAPPED)&ctx->ov, NULL) || WSA_IO_PENDING == WSAGetLastError()) {
            inst->device_net_recvlist = (void *)ctx;
            inst->device_net_recvlast = (void *)ctx;
            ret = 0;
          } else {
            free(ctx);
            ret = -1;
            InterlockedExchange(&inst->device_net_connect, STOPPED);
            inst->status = INITIAL;
            PRINT_DBG_MSG("camera %s post recv iocp stopped", inst->devinfo.net_camera_ip);
          }
        } else {
          ret = -1;
        }
      }
    }
  #else*/
  si.sin_addr.s_addr = inet_addr(dev_info->net_camera_ip);
  inst->device_net_socket = socket(AF_INET, SOCK_STREAM, 0);
  if (inst->device_net_socket != INVALID_SOCKET) {
    /* int ttl = 1;
     setsockopt(inst->device_net_socket, IPPROTO_IP, IP_TTL, (char *)&ttl, sizeof(ttl));
     int keepalive = 3;
     setsockopt(inst->device_net_socket, SOL_SOCKET, SO_KEEPALIVE, (char *)&keepalive, sizeof(keepalive));
     int timeo = 100;
     struct timeval tv = {timeo / 1000, timeo % 1000 * 1000};
     setsockopt(inst->device_net_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(tv));
     setsockopt(inst->device_net_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
     int bufsize = BUFFER_SIZE;
     setsockopt(inst->device_net_socket, SOL_SOCKET, SO_SNDBUF, (char *)&bufsize, sizeof(bufsize));
     setsockopt(inst->device_net_socket, SOL_SOCKET, SO_RCVBUF, (char *)&bufsize, sizeof(bufsize));
     struct linger sl;
     sl.l_onoff = 0;
     // sl.l_linger=20;
     setsockopt(inst->device_net_socket, SOL_SOCKET, SO_LINGER, &sl, sizeof(sl));
     */
#ifdef _WINDOWS
    int timeo = 2000;
    setsockopt(inst->device_net_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeo, sizeof(timeo));
    setsockopt(inst->device_net_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeo, sizeof(timeo));
#else
    struct timeval tv = {2, 0};
    setsockopt(inst->device_net_socket, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(tv));
    setsockopt(inst->device_net_socket, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(tv));
#endif
    nvpfm_debug_printf("will connect to %s\n", dev_info->net_camera_ip);
    if ((ret = connect(inst->device_net_socket, (struct sockaddr *)&si, sizeof(si))) == 0) { //|| (ret == EINPROGRESS && (ret = connect(inst->device_net_socket, (struct sockaddr *)&si, sizeof(si))) == 0)) {
                                                                                             /*  int flags = fcntl(inst->device_net_socket, F_GETFL);
                                                                                               flags |= O_NONBLOCK;
                                                                                               fcntl(inst->device_net_socket, F_SETFL, flags);
                                                                                               int flag = 1;
                                                                                               setsockopt(inst->device_net_socket, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
                                                                                               setsockopt(inst->device_net_socket, IPPROTO_TCP, TCP_QUICKACK, (char *)&flag, sizeof(int));
                                                                                               // setsockopt(inst->device_net_socket, IPPROTO_TCP, TCP_CORK, (char *)&flag, sizeof(int));
                                                                                               setsockopt(inst->device_net_socket, IPPROTO_TCP, TCP_USER_TIMEOUT, &tv, sizeof(struct timeval));
                                                                                               struct sockaddr_in addr;
                                                                                               socklen_t addrlen = sizeof(addr);
                                                                                               if ((ret = getsockname(inst->device_net_socket, (struct sockaddr *)&addr, &addrlen)) == 0) {
                                                                                                 inst->device_net_handle = epoll_create(EPOLL_NUM); // getdtablesize()
                                                                                                 struct epoll_event evt;
                                                                                                 evt.data.ptr = NULL;
                                                                                                 evt.events = EPOLLIN | EPOLLET;
                                                                                                 if ((ret = epoll_ctl(inst->device_net_handle, EPOLL_CTL_ADD, inst->device_net_socket, &evt)) == 0) {*/
      inst->recvframecallback = callback;
      inst->eventcallback = NULL;
      //    __sync_synchronize();
      //    __sync_lock_test_and_set(&inst->device_net_connect, STARTED);
      inst->status = CONNECTED;
      nvpfm_debug_printf("will create get_data thread!\n");
#ifdef _WINDOWS
      inst->device_net_thread = (HANDLE)_beginthreadex(NULL, 0, get_data, (void *)inst, 0, NULL);
#else
      pthread_create((pthread_t *)&inst->device_net_thread, 0, get_data, (void *)inst);
#endif
      //  PRINT_DBG_MSG("camera %s epoll started", inst->devinfo.net_camera_ip);
      // }
      //  }
    }
    /* if (0 == ret) {
       PPER_IO_CONTEXT ctx = (PPER_IO_CONTEXT)calloc(1, sizeof(PER_IO_CONTEXT));
       if (ctx != NULL) {
         ctx->wsabuf.buf = ctx->buffer;
         ctx->wsabuf.len = BUFFER_SIZE;
         ctx->fd = inst->device_net_socket;
         struct epoll_event evt;
         evt.data.ptr = (void *)ctx;
         evt.events = EPOLLIN | EPOLLET;
         if (0 == (ret = epoll_ctl(inst->device_net_handle, EPOLL_CTL_MOD, inst->device_net_socket, &evt))) {
           inst->device_net_recvlist = (void *)ctx;
           inst->device_net_recvlast = (void *)ctx;
         } else {
           free(ctx);
           __sync_synchronize();
           __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
           inst->status = INITIAL;
           PRINT_DBG_MSG("camera %s post recv epoll stopped", inst->devinfo.net_camera_ip);
         }
       } else {
         ret = -1;
       }
     }*/
  }
  // #endif
  if (0 != ret) {
    printf("fail to open net camera!\n");
    nvptl_net_close((NVPTL_DEVICE_HANDLE)inst);
  }
  return ret == 0 ? inst : NULL;
}

void nvptl_net_close(NVPTL_DEVICE_HANDLE handle) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  /*#if defined(_WINDOWS)
    if (inst->device_net_handle != NULL) {
      PPER_IO_CONTEXT ctx = calloc(1, sizeof(PER_IO_CONTEXT));
      PostQueuedCompletionStatus((HANDLE)inst->device_net_handle, 0, (DWORD)KEY_CLOSE, (LPOVERLAPPED)&ctx->ov);
    }
    if (inst->device_net_thread != NULL) {
      WaitForSingleObject(inst->device_net_thread, INFINITE);
      CloseHandle(inst->device_net_thread);
    }
    if (inst->device_net_socket != INVALID_SOCKET) {
      if (InterlockedExchangeAdd(&inst->device_net_connect, 0) == STARTED) {
        shutdown(inst->device_net_socket, SD_BOTH);
      }
      closesocket(inst->device_net_socket);
      inst->device_net_socket = INVALID_SOCKET;
    }
    InterlockedExchange(&inst->device_net_connect, INITIAL);
    InterlockedExchange(&inst->device_net_sendcnt, 0);
  #else*/
  /*  if (inst->device_net_handle > 0) {
      struct epoll_event evt;
      evt.data.ptr = NULL;
      evt.events = EPOLLET | EPOLLOUT;
      epoll_ctl(inst->device_net_handle, EPOLL_CTL_MOD, inst->device_net_socket, &evt);
    }*/
  inst->willclose = 1;
  if (inst->device_net_thread != NULL) {
#ifdef _WINDOWS
    WaitForSingleObject(inst->device_net_thread, INFINITE);
    CloseHandle(inst->device_net_thread);
#else
    pthread_join((pthread_t)inst->device_net_thread, NULL);
#endif
  }
  if (inst->device_net_socket != INVALID_SOCKET) {
#ifdef _WINDOWS
    shutdown(inst->device_net_socket, SD_BOTH);
    closesocket(inst->device_net_socket);
#else
    //   if (__sync_fetch_and_add(&inst->device_net_connect, 0) == STARTED) {
    shutdown(inst->device_net_socket, SHUT_RDWR);
    //  }
    close(inst->device_net_socket);
#endif
    inst->device_net_socket = INVALID_SOCKET;
  }
  // __sync_lock_release(&inst->device_net_connect, INITIAL);
  // __sync_lock_release(&inst->device_net_sendcnt, 0);

  // #endif
  /* if (inst->device_net_sendlist != NULL) {
     PPER_IO_CONTEXT ctx = (PPER_IO_CONTEXT)inst->device_net_sendlist;
     for (; ctx != NULL;) {
       PPER_IO_CONTEXT sendctx = ctx;
       ctx = ctx->next;
       if (sendctx->datalen > BUFFER_SIZE) {
         free(sendctx->data);
       }
       free(sendctx);
     }
     inst->device_net_sendlist = NULL;
     inst->device_net_sendlast = NULL;
   }
   if (inst->device_net_recvlist != NULL) {
     PPER_IO_CONTEXT ctx = (PPER_IO_CONTEXT)inst->device_net_recvlist;
     for (; ctx != NULL;) {
       PPER_IO_CONTEXT recvctx = ctx;
       ctx = ctx->next;
       if (recvctx->datalen > BUFFER_SIZE) {
         free(recvctx->data);
       }
       free(recvctx);
     }
     inst->device_net_recvlist = NULL;
     inst->device_net_recvlast = NULL;
   }
   PRINT_DBG_MSG("camera %s net closed", inst->devinfo.net_camera_ip);
   */
  free(inst);
  return;
}

int nvptl_net_send(NVPTL_DEVICE_HANDLE handle, unsigned char *buffer, size_t len) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  int ret = -1;
  /*#if defined(_WINDOWS)
    if (InterlockedExchangeAdd(&inst->device_net_connect, 0) != STARTED)
  #else
    // if (__sync_fetch_and_add(&inst->device_net_connect, 0) != STARTED)
  #endif
    {
      nvpfm_info_printf("fail net send!\n");
      return ret;
    }

  #if defined(_WINDOWS)
    PPER_IO_CONTEXT ctx = calloc(1, sizeof(PER_IO_CONTEXT));
    if (ctx != NULL) {
      if (len > BUFFER_SIZE) {
        ctx->data = malloc(len);
      } else {
        ctx->data = (void *)ctx->buffer;
      }
      if (ctx->data != NULL) {
        memcpy(ctx->data, buffer, len);
        ctx->type = POST_SEND;
        ctx->datalen = len;
        ctx->wsabuf.buf = ctx->data;
        ctx->wsabuf.len = len;
        if (inst->device_net_sendlast != NULL) {
          ((PPER_IO_CONTEXT)(inst->device_net_sendlast))->next = ctx;
          inst->device_net_sendlast = (void *)ctx;
        } else {
          inst->device_net_sendlist = (void *)ctx;
          inst->device_net_sendlast = inst->device_net_sendlist;
        }
        DWORD sent = 0;
        DWORD flags = 0;
        if (NO_ERROR == (ret = WSASend(inst->device_net_socket, &ctx->wsabuf, 1, &sent, flags, (LPWSAOVERLAPPED)&ctx->ov, NULL)) || WSA_IO_PENDING == WSAGetLastError()) {
          InterlockedExchangeAdd(&inst->device_net_sendcnt, 1);
          ret = 0;
        } else {
          if (InterlockedExchangeAdd(&inst->device_net_sendcnt, 0) == 0) {
            inst->device_net_sendlist = NULL;
            inst->device_net_sendlast = NULL;
          } else {
            inst->device_net_sendlast = ((PPER_IO_CONTEXT)(inst->device_net_sendlast))->prev;
            ((PPER_IO_CONTEXT)(inst->device_net_sendlast))->next = NULL;
          }
          if (len > BUFFER_SIZE) {
            free(ctx->data);
          }
          free(ctx);
          InterlockedExchange(&inst->device_net_connect, STOPPED);
          PRINT_DBG_MSG("camera %s post send iocp stopped", inst->devinfo.net_camera_ip);
        }
      } else {
        free(ctx);
      }
    }
  #else*/
  int index = 0;
  while (index < len) {
    int ret = send(inst->device_net_socket, buffer + index, len - index, 0);
    if (ret > 0) {
      index += ret;
    }
  }
  /*  PPER_IO_CONTEXT ctx = (PPER_IO_CONTEXT)calloc(1, sizeof(PER_IO_CONTEXT));
    if (ctx != NULL) {
      if (len > BUFFER_SIZE) {
        ctx->data = malloc(len);
      } else {
        ctx->data = (void *)ctx->buffer;
      }
      if (ctx->data != NULL) {
        memcpy(ctx->data, (void *)buffer, len);
        ctx->datalen = len;
        ctx->fd = inst->device_net_socket;
        if (inst->device_net_sendlast != NULL) {
          ((PPER_IO_CONTEXT)(inst->device_net_sendlast))->next = ctx;
          inst->device_net_sendlast = (void *)ctx;
        } else {
          inst->device_net_sendlist = (void *)ctx;
          inst->device_net_sendlast = inst->device_net_sendlist;
        }
        struct epoll_event evt;
        evt.data.ptr = (void *)ctx;
        evt.events = EPOLLOUT | EPOLLET;
        if (0 == (ret = epoll_ctl(inst->device_net_handle, EPOLL_CTL_MOD, inst->device_net_socket, &evt))) {
          __sync_fetch_and_add(&inst->device_net_sendcnt, 1);
          nvpfm_info_printf("camera %s post send len[%d] data[%*.*s]", inst->devinfo.net_camera_ip, ctx->datalen,
                            sizeof(NVPTL_HeaderDataPacket), sizeof(NVPTL_HeaderDataPacket), (char *)ctx->data);
          // PRINT_DBG_MSG("camera %s post send len[%d] data[%*.*s]", inst->devinfo.net_camera_ip, ctx->datalen,
          //	sizeof(NVPTL_HeaderDataPacket), sizeof(NVPTL_HeaderDataPacket), (char*)ctx->data);
        } else {
          if (__sync_fetch_and_add(&inst->device_net_sendcnt, 0) == 0) {
            inst->device_net_sendlist = NULL;
            inst->device_net_sendlast = NULL;
          } else {
            inst->device_net_sendlast = ((PPER_IO_CONTEXT)(inst->device_net_sendlast))->prev;
            ((PPER_IO_CONTEXT)(inst->device_net_sendlast))->next = NULL;
          }
          if (len > BUFFER_SIZE) {
            free(ctx->data);
          }
          free(ctx);
          __sync_synchronize();
          __sync_lock_test_and_set(&inst->device_net_connect, STOPPED);
          nvpfm_info_printf("camera %s post send epoll stopped", inst->devinfo.net_camera_ip);
          PRINT_DBG_MSG("camera %s post send epoll stopped", inst->devinfo.net_camera_ip);
        }
      } else {
        free(ctx);
      }
    }*/
  // #endif
  return 0;
  // ret;
}
