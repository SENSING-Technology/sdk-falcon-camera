#include <stdio.h>
#include <stdint.h>
// #include <string.h>
#ifdef _WINDOWS
#include <Windows.h>
#include <time.h>
#else
#include <sys/time.h>
#include <unistd.h>
#endif
#include <iostream>
#include <vector>

#include <algorithm>
/////////////////////////////////////////////
// #include <winusb.h>
#include <Windows.h>
#include <Sddl.h>
#include <string>
#include <regex>
#include <mutex>

#include <usbioctl.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <atlcomcli.h>
#include <strmif.h>
#include <Ks.h>
#include <ksproxy.h>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <comdef.h>
#include <Cfgmgr32.h>
#include <SetupAPI.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <functional>
#include <cassert>

#include <string>
#include <Windows.h>
#include <functional>
#include <comdef.h>
#include <sstream>

#define NVPTL_ENDPOINT_IN (LIBUSB_ENDPOINT_IN + 1)
#define NVPTL_ENDPOINT_OUT (LIBUSB_ENDPOINT_OUT + 1)
#include <map>
#include <vector>
#include "Shlwapi.h"
#include <Windows.h>
#include <limits>
#include "mfapi.h"
#include <vidcap.h>
#include <ksmedia.h> // Metadata Extension
#include <Mferror.h>

#include <functional> // For function
#include <thread>     // For this_thread::sleep_for
#include <vector>
#include <algorithm>
#include <set>
#include <iterator>
#include <map>
#include <cstring>
#include <string>
#include <sstream>
#include <fstream>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <atlcomcli.h>
#include <strmif.h>
#include <Ks.h>
#include <ksproxy.h>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <string.h>
#include <initguid.h>
#include <devpkey.h> // DEVPKEY_...

#include "transferlayer/commondata.h"

#include <memory> // For shared_ptr
#include <sstream>
#include <tuple>
#include <string>
enum usb_spec : uint16_t {
  usb_undefined = 0,
  usb1_type = 0x0100,
  usb1_1_type = 0x0110,
  usb2_type = 0x0200,
  usb2_01_type = 0x0201,
  usb2_1_type = 0x0210,
  usb3_type = 0x0300,
  usb3_1_type = 0x0310,
  usb3_2_type = 0x0320,
};

struct uvc_device_info {
  std::string id = ""; // to distinguish between different pins of the same device
  uint16_t vid = 0;
  uint16_t pid = 0;
  uint16_t mi = 0;
  std::string unique_id = "";
  std::string device_path = "";
  std::string serial = "";
  usb_spec conn_spec = usb_undefined;
  uint32_t uvc_capabilities = 0;
  bool has_metadata_node = false;
  std::string metadata_node_id = "";

  operator std::string() {
    std::stringstream s;
    s << "id- " << id << "\nvid- " << std::hex << vid << "\npid- " << std::hex << pid << "\nmi- " << mi << "\nunique_id- " << unique_id << "\npath- " << device_path << "\nsusb specification- " << std::hex << (uint16_t)conn_spec << std::dec << (has_metadata_node ? ("\nmetadata node-" + metadata_node_id) : "");

    return s.str();
  }

  bool operator<(const uvc_device_info &obj) const {
    return (std::make_tuple(id, vid, pid, mi, unique_id, device_path) < std::make_tuple(obj.id, obj.vid, obj.pid, obj.mi, obj.unique_id, obj.device_path));
  }
};
class wmf_uvc_device;
void mfuvc_init();
typedef void (*UVCDEVICECALLBACK)(struct uvc_device_info *pinfo, void *userdata);
int mfuvc_transfer(std::shared_ptr<wmf_uvc_device> dev, unsigned char *data, int len);
class wmf_uvc_device;
wmf_uvc_device *get_uvc_device_by_vidpid_path(uint16_t vid, uint16_t pid, const char *path);

typedef void (*STREAMCALLBACK)(const void *pixels, size_t framesize, void *userdata);
void mfuvc_start_stream(STREAMCALLBACK callback, wmf_uvc_device * /*std::shared_ptr<wmf_uvc_device>*/ wmfdevice, void *userdata);

#ifndef WITH_TRACKING
DEFINE_GUID(GUID_DEVINTERFACE_USB_DEVICE, 0xA5DCBF10L, 0x6530, 0x11D2, 0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED);
#endif
#ifndef KSCATEGORY_SENSOR_CAMERA
DEFINE_GUIDSTRUCT("24E552D7-6523-47F7-A647-D3465BF1F5CA", KSCATEGORY_SENSOR_CAMERA);
#define KSCATEGORY_SENSOR_CAMERA DEFINE_GUIDNAMED(KSCATEGORY_SENSOR_CAMERA)
#endif // !KSCATEGORY_SENSOR_CAMERA

#pragma comment(lib, "Shlwapi.lib")
#pragma comment(lib, "mf.lib")
#pragma comment(lib, "mfplat.lib")
#pragma comment(lib, "mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "cfgmgr32.lib")
#pragma comment(lib, "setupapi.lib")

inline bool operator==(const uvc_device_info &a,
                       const uvc_device_info &b) {
  return (a.vid == b.vid) &&
         (a.pid == b.pid) &&
         (a.mi == b.mi) &&
         (a.unique_id == b.unique_id) &&
         (a.id == b.id) &&
         (a.device_path == b.device_path) &&
         (a.conn_spec == b.conn_spec);
}

static const std::vector<std::vector<std::pair<GUID, GUID>>> attributes_params = {
    {{MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID},
     {MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_CATEGORY, KSCATEGORY_SENSOR_CAMERA}},
    {{MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID}},
};
UINT codepage = 936;
inline std::string win_to_default(const WCHAR *s, int wlen = -1) {
  auto len = WideCharToMultiByte(codepage, 0, s, wlen, nullptr, 0, nullptr, nullptr);
  if (len == 0) {
    std::ostringstream ss;
    ss << "WideCharToMultiByte(...) returned 0 and GetLastError() is " << GetLastError();
    throw std::runtime_error(ss.str());
  }

  std::string buffer;
  buffer.resize(len - 1); // len includes the \0
  len = WideCharToMultiByte(codepage, 0, s, wlen, &buffer[0], len, nullptr, nullptr);
  if (len == 0) {
    std::ostringstream ss;
    ss << "WideCharToMultiByte(...) returned 0 and GetLastError() is " << GetLastError();
    throw std::runtime_error(ss.str());
  }

  return buffer;
}
inline std::string win_to_utf(const WCHAR *s, int wlen = -1) {
  auto len = WideCharToMultiByte(CP_UTF8, 0, s, wlen, nullptr, 0, nullptr, nullptr);
  if (len == 0) {
    std::ostringstream ss;
    ss << "WideCharToMultiByte(...) returned 0 and GetLastError() is " << GetLastError();
    throw std::runtime_error(ss.str());
  }

  std::string buffer;
  buffer.resize(len - 1); // len includes the \0
  len = WideCharToMultiByte(CP_UTF8, 0, s, wlen, &buffer[0], len, nullptr, nullptr);
  if (len == 0) {
    std::ostringstream ss;
    ss << "WideCharToMultiByte(...) returned 0 and GetLastError() is " << GetLastError();
    throw std::runtime_error(ss.str());
  }

  return buffer;
}

inline std::string win_to_utf(std::wstring const &s) {
  return win_to_utf(s.c_str(), (int)s.length());
}

inline std::string hr_to_string(HRESULT hr) {
  _com_error err(hr);
  std::wstring errorMessage = (err.ErrorMessage()) ? err.ErrorMessage() : L"";
  std::ostringstream ss;
  ss << "HResult 0x" << std::hex << hr << ": \"" << win_to_utf(errorMessage.data()) << "\"";
  return ss.str();
}
#define CHECK_HR_STR(call, hr)                       \
  if (FAILED(hr)) {                                  \
    std::ostringstream ss;                           \
    ss << call << " returned: " << hr_to_string(hr); \
    std::string descr = ss.str();                    \
    throw std::runtime_error(descr);                 \
  }
std::vector<std::string> tokenize(std::string string, char separator) {
  std::vector<std::string> tokens;
  std::string::size_type i1 = 0;
  while (true) {
    auto i2 = string.find(separator, i1);
    if (i2 == std::string::npos) {
      tokens.push_back(string.substr(i1));
      return tokens;
    }
    tokens.push_back(string.substr(i1, i2 - i1));
    i1 = i2 + 1;
  }
}
bool parse_usb_path_multiple_interface(uint16_t &vid, uint16_t &pid, uint16_t &mi, std::string &unique_id, const std::string &path, std::string &device_guid) {
  auto name = path;
  std::transform(begin(name), end(name), begin(name), ::tolower);
  auto tokens = tokenize(name, '#');
  if (tokens.size() < 1 || (tokens[0] != R"(\\?\usb)" && tokens[0] != R"(\\?\hid)"))
    return false; // Not a USB device
  if (tokens.size() < 3) {
    std::cout << "malformed usb device path: " << name << std::endl;
    return false;
  }

  auto ids = tokenize(tokens[1], '&');
  if (ids[0].size() != 8 || ids[0].substr(0, 4) != "vid_" || !(std::istringstream(ids[0].substr(4, 4)) >> std::hex >> vid)) {
    std::cout << "malformed vid string: " << tokens[1] << std::endl;
    return false;
  }

  if (ids[1].size() != 8 || ids[1].substr(0, 4) != "pid_" || !(std::istringstream(ids[1].substr(4, 4)) >> std::hex >> pid)) {
    std::cout << "malformed pid string: " << tokens[1] << std::endl;
    return false;
  }

  if (ids.size() > 2 && (ids[2].size() != 5 || ids[2].substr(0, 3) != "mi_" || !(std::istringstream(ids[2].substr(3, 2)) >> mi))) {
    std::cout << "malformed mi string: " << tokens[1] << std::endl;
    return false;
  }

  ids = tokenize(tokens[2], '&');
  if (ids.size() == 0) {
    std::cout << "malformed id string: " << tokens[2] << std::endl;
    return false;
  }

  if (ids.size() > 2)
    unique_id = ids[1];
  else
    unique_id = "";

  if (tokens.size() >= 3)
    device_guid = tokens[3];

  return true;
}
template <class T>
static void safe_release(T &ppT) {
  if (ppT) {
    ppT.Release();
    ppT = NULL;
  }
}
#define CHECK_HR(x) CHECK_HR_STR(#x, x)
typedef std::function<void(const uvc_device_info &, IMFActivate *)> enumeration_callback;
void foreach_uvc_device(enumeration_callback action) {
  for (auto attributes_params_set : attributes_params) {
    CComPtr<IMFAttributes> pAttributes = nullptr;
    CHECK_HR(MFCreateAttributes(&pAttributes, 1));
    for (auto attribute_params : attributes_params_set) {
      CHECK_HR(pAttributes->SetGUID(attribute_params.first, attribute_params.second));
    }

    IMFActivate **ppDevices;
    UINT32 numDevices;
    CHECK_HR(MFEnumDeviceSources(pAttributes, &ppDevices, &numDevices));

    for (UINT32 i = 0; i < numDevices; ++i) {
      CComPtr<IMFActivate> pDevice;
      *&pDevice = ppDevices[i];

      WCHAR *wchar_name = nullptr;
      UINT32 length;
      CHECK_HR(pDevice->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &wchar_name, &length));
      auto name = win_to_utf(wchar_name);
      CoTaskMemFree(wchar_name);

      uint16_t vid, pid, mi;
      std::string unique_id, guid;
      if (!parse_usb_path_multiple_interface(vid, pid, mi, unique_id, name, guid))
        continue;

      uvc_device_info info;
      info.vid = vid;
      info.pid = pid;
      info.unique_id = unique_id;
      info.mi = mi;
      info.device_path = name;
      try {
        action(info, ppDevices[i]);
      } catch (...) {
        // TODO
      }
    }
    safe_release(pAttributes);
    CoTaskMemFree(ppDevices);
  }
}
DEFINE_GUID(GUID_DEVINTERFACE_IMAGE_WIN10, 0x6bdd1fc6L, 0x810f, 0x11d0, 0xbe, 0xc7, 0x08, 0x00, 0x2b, 0xe2, 0x09, 0x2f);
DEFINE_GUID(GUID_DEVINTERFACE_CAMERA_WIN10, 0xca3e7ab9, 0xb4c3, 0x4ae6, 0x82, 0x51, 0x57, 0x9e, 0xf9, 0x33, 0x89, 0x0f);

// Intel(R) RealSense(TM) 415 Depth - MI 0: [Interface 0 video control] [Interface 1 video stream] [Interface 2 video stream]
DEFINE_GUID(GUID_DEVINTERFACE_IMAGE_WIN7, 0xe659c3ec, 0xbf3c, 0x48a5, 0x81, 0x92, 0x30, 0x73, 0xe8, 0x22, 0xd7, 0xcd);

// Intel(R) RealSense(TM) 415 RGB - MI 3: [Interface 3 video control] [Interface 4 video stream]
DEFINE_GUID(GUID_DEVINTERFACE_CAMERA_WIN7, 0x50537bc3, 0x2919, 0x452d, 0x88, 0xa9, 0xb1, 0x3b, 0xbf, 0x7d, 0x24, 0x59);

bool get_id(DEVINST devinst, std::string *p_out_str) {
  ULONG cch_required = 0;
  if (CM_Get_Device_ID_Size(&cch_required, devinst, 0) != CR_SUCCESS)
    return false;

  if (p_out_str) {
    std::vector<WCHAR> buf(cch_required + 1);
    if (CM_Get_Device_ID(devinst, buf.data(), cch_required, 0) != CR_SUCCESS)
      return false;
    *p_out_str = win_to_utf(buf.data());
  }

  return true;
}
bool parse_usb_path_from_device_id(uint16_t &vid, uint16_t &pid, uint16_t &mi, std::string &unique_id, const std::string &device_id) {
  auto name = device_id;
  std::transform(begin(name), end(name), begin(name), ::tolower);
  auto tokens = tokenize(name, '\\');
  if (tokens.size() < 1)
    return false;
  if (tokens[0] != "usb" && tokens[0] != "hid")
    return false;

  // Expecting VID, PID, and MI
  auto ids = tokenize(tokens[1], '&');
  if (ids.size() < 3) {
    // MI may be missing, especially when we look at composite devices
    if (ids.size() < 2)
      std::cout << "incomplete device id: " << device_id << std::endl;
    return false;
  }

  if (ids[0].size() != 8 || ids[0].substr(0, 4) != "vid_" || !(std::istringstream(ids[0].substr(4, 4)) >> std::hex >> vid)) {
    std::cout << "malformed vid string: " << tokens[1] << std::endl;
    return false;
  }

  if (ids[1].size() != 8 || ids[1].substr(0, 4) != "pid_" || !(std::istringstream(ids[1].substr(4, 4)) >> std::hex >> pid)) {
    std::cout << "malformed pid string: " << tokens[1] << std::endl;
    return false;
  }

  if (ids[2].size() != 5 || ids[2].substr(0, 3) != "mi_" || !(std::istringstream(ids[2].substr(3, 2)) >> mi)) {
    std::cout << "malformed mi string: " << tokens[1] << std::endl;
    return false;
  }

  ids = tokenize(tokens[2], '&');
  if (ids.size() < 2) {
    std::cout << "malformed id string: " << tokens[2] << std::endl;
    return false;
  }
  unique_id = ids[1];
  return true;
}
template <typename T>
size_t vector_bytes_size(const typename std::vector<T> &vec) {
  static_assert((std::is_arithmetic<T>::value), "vector_bytes_size requires numeric type for input data");
  return sizeof(T) * vec.size();
}
// Parse the following USB path format = \?usb#vid_vvvv&pid_pppp#ssss#{gggggggg-gggg-gggg-gggg-gggggggggggg}
// vvvv = USB vendor ID represented in 4 hexadecimal characters.
// pppp = USB product ID represented in 4 hexadecimal characters.
// ssss = USB serial string represented in n characters.
// gggggggg-gggg-gggg-gggg-gggggggggggg = device interface GUID assigned in the driver or driver INF file and is used to link applications to device with specific drivers loaded.
bool parse_usb_path_single_interface(uint16_t &vid, uint16_t &pid, std::string &serial, const std::string &path) {
  auto name = path;
  std::transform(begin(name), end(name), begin(name), ::tolower);
  auto tokens = tokenize(name, '#');
  if (tokens.size() < 1 || (tokens[0] != R"(\\?\usb)" && tokens[0] != R"(\\?\hid)"))
    return false; // Not a USB device
  if (tokens.size() < 3) {
    std::cout << "malformed usb device path: " << name << std::endl;
    return false;
  }

  auto ids = tokenize(tokens[1], '&');
  if (ids[0].size() != 8 || ids[0].substr(0, 4) != "vid_" || !(std::istringstream(ids[0].substr(4, 4)) >> std::hex >> vid)) {
    std::cout << "malformed vid string: " << tokens[1] << std::endl;
    return false;
  }

  if (ids[1].size() != 8 || ids[1].substr(0, 4) != "pid_" || !(std::istringstream(ids[1].substr(4, 4)) >> std::hex >> pid)) {
    std::cout << "malformed pid string: " << tokens[1] << std::endl;
    return false;
  }

  serial = tokens[2];

  return true;
}
std::wstring get_path(HANDLE h, ULONG index) {
  // get name length
  USB_NODE_CONNECTION_NAME name;
  name.ConnectionIndex = index;
  if (!DeviceIoControl(h, IOCTL_USB_GET_NODE_CONNECTION_NAME, &name, sizeof(name), &name, sizeof(name), nullptr, nullptr)) {
    return std::wstring(L"");
  }

  // alloc space
  if (name.ActualLength < sizeof(name))
    return std::wstring(L"");
  auto alloc = std::malloc(name.ActualLength);
  auto pName = std::shared_ptr<USB_NODE_CONNECTION_NAME>(reinterpret_cast<USB_NODE_CONNECTION_NAME *>(alloc), std::free);

  // get name
  pName->ConnectionIndex = index;
  if (DeviceIoControl(h, IOCTL_USB_GET_NODE_CONNECTION_NAME, pName.get(), name.ActualLength, pName.get(), name.ActualLength, nullptr, nullptr)) {
    return std::wstring(pName->NodeName);
  }

  return std::wstring(L"");
}
bool handle_node(const std::wstring &targetKey, HANDLE h, ULONG index) {
  USB_NODE_CONNECTION_DRIVERKEY_NAME key;
  key.ConnectionIndex = index;

  if (!DeviceIoControl(h, IOCTL_USB_GET_NODE_CONNECTION_DRIVERKEY_NAME, &key, sizeof(key), &key, sizeof(key), nullptr, nullptr)) {
    return false;
  }

  if (key.ActualLength < sizeof(key))
    return false;

  auto alloc = std::malloc(key.ActualLength);
  if (!alloc)
    throw std::bad_alloc();

  auto pKey = std::shared_ptr<USB_NODE_CONNECTION_DRIVERKEY_NAME>(reinterpret_cast<USB_NODE_CONNECTION_DRIVERKEY_NAME *>(alloc), std::free);

  pKey->ConnectionIndex = index;
  if (DeviceIoControl(h, IOCTL_USB_GET_NODE_CONNECTION_DRIVERKEY_NAME, pKey.get(), key.ActualLength, pKey.get(), key.ActualLength, nullptr, nullptr)) {
    // std::wcout << pKey->DriverKeyName << std::endl;
    if (targetKey == pKey->DriverKeyName) {
      return true;
    } else
      return false;
  }

  return false;
}
std::tuple<std::string, usb_spec> handle_usb_hub(const std::wstring &targetKey, const std::wstring &path) {
  auto res = std::make_tuple(std::string(), usb_spec::usb_undefined);

  if (path.empty())
    return res;
  std::wstring fullPath = L"\\\\.\\" + path;

  HANDLE h = CreateFile(fullPath.c_str(), GENERIC_WRITE, FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE)
    return res;
  auto h_gc = std::shared_ptr<void>(h, CloseHandle);

  USB_NODE_INFORMATION info{};
  if (!DeviceIoControl(h, IOCTL_USB_GET_NODE_INFORMATION, &info, sizeof(info), &info, sizeof(info), nullptr, nullptr))
    return res;

  // for each port on the hub
  for (ULONG i = 1; i <= info.u.HubInformation.HubDescriptor.bNumberOfPorts; ++i) {
    // allocate something or other
    char buf[sizeof(USB_NODE_CONNECTION_INFORMATION_EX)] = {0};
    PUSB_NODE_CONNECTION_INFORMATION_EX pConInfo = reinterpret_cast<PUSB_NODE_CONNECTION_INFORMATION_EX>(buf);

    // get info about port i
    pConInfo->ConnectionIndex = i;
    if (!DeviceIoControl(h, IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX, pConInfo, sizeof(buf), pConInfo, sizeof(buf), nullptr, nullptr)) {
      continue;
    }

    // check if device is connected
    if (pConInfo->ConnectionStatus != DeviceConnected) {
      continue; // almost assuredly silently. I think this flag gets set for any port without a device
    }

    // if connected, handle correctly, setting the location info if the device is found
    if (pConInfo->DeviceIsHub)
      res = handle_usb_hub(targetKey, get_path(h, i)); // Invoke recursion to traverse USB hubs chain
    else {
      if (handle_node(targetKey, h, i)) // exit condition
      {
        return std::make_tuple(win_to_utf(fullPath.c_str()) + " " + std::to_string(i),
                               static_cast<usb_spec>(pConInfo->DeviceDescriptor.bcdUSB));
      }
    }

    if (!std::get<0>(res).empty())
      return res;
  }

  return res;
}
// Provides Port Id and the USB Specification (USB type)
bool get_usb_device_descriptors(DEVINST devinst, uint16_t device_vid, uint16_t device_pid, const std::string &device_uid, std::string &location, usb_spec &spec, std::string &serial, std::string &parent_uid) {
  unsigned long buf_size = 0;

  // Check if this is our device
  std::string device_id;
  if (!get_id(devinst, &device_id)) {
    std::cout << "CM_Get_Device_ID failed" << std::endl;
    return false;
  }
  // LOG_DEBUG( "???  dev ID " << device_id );
  uint16_t usb_vid, usb_pid, usb_mi;
  std::string usb_unique_id;
  if (!parse_usb_path_from_device_id(usb_vid, usb_pid, usb_mi, usb_unique_id, device_id))
    return false;
  // LOG_DEBUG( "     uid " << usb_unique_id );
  if (usb_vid != device_vid || usb_pid != device_pid || /* usb_mi != device->mi || */ usb_unique_id != device_uid)
    return false;

  // Get parent (composite device) instance
  DEVINST parent;
  if (CM_Get_Parent(&parent, devinst, 0) != CR_SUCCESS) {
    std::cout << "CM_Get_Parent failed" << std::endl;
    return false;
  }

  // Get the buffer size required to hold the parent (composite) device instance ID
  if (CM_Get_Device_ID_Size(&buf_size, parent, 0) != CR_SUCCESS) {
    std::cout << "CM_Get_Device_ID_Size failed" << std::endl;
    return false;
  }

  std::vector<WCHAR> pInstID2(buf_size + 1);

  if (CM_Get_Device_ID(parent, pInstID2.data(), ULONG(vector_bytes_size(pInstID2)), 0) != CR_SUCCESS) {
    std::cout << "CM_Get_Device_ID failed" << std::endl;
    return false;
  }
  std::string parent_id = win_to_utf(pInstID2.data());
  // LOG_DEBUG( "...  parent device id " << parent_id );
  uint16_t parent_vid, parent_pid, parent_mi;
  parse_usb_path_from_device_id(parent_vid, parent_pid, parent_mi, parent_uid, parent_id); // may fail -- but we try to get the parent_uid

  // Upgrade to DEVINFO_DATA for SetupDiGetDeviceRegistryProperty
  HDEVINFO device_info = SetupDiGetClassDevs(nullptr, pInstID2.data(), nullptr, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE | DIGCF_ALLCLASSES);
  if (device_info == INVALID_HANDLE_VALUE) {
    // HID devices should always have a valid parent...
    if (0 == parent_id.compare(0, 4, "HID\\"))
      std::cout << "SetupDiGetClassDevs failed" << std::endl;
    return false;
  }

  // Add automatic destructor to the device info
  auto di = std::shared_ptr<void>(device_info, SetupDiDestroyDeviceInfoList);

  SP_DEVICE_INTERFACE_DATA interfaceData = {sizeof(SP_DEVICE_INTERFACE_DATA)};
  if (SetupDiEnumDeviceInterfaces(device_info, nullptr, &GUID_DEVINTERFACE_USB_DEVICE, 0, &interfaceData) == FALSE) {
    std::cout << "SetupDiEnumDeviceInterfaces failed" << std::endl;
    return false;
  }

  // get the SP_DEVICE_INTERFACE_DETAIL_DATA object, and also grab the SP_DEVINFO_DATA object for the device
  buf_size = 0;
  SetupDiGetDeviceInterfaceDetail(device_info, &interfaceData, nullptr, 0, &buf_size, nullptr);
  if (GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
    std::cout << "SetupDiGetDeviceInterfaceDetail failed" << std::endl;
    return false;
  }

  std::vector<BYTE> detail_data_buff(buf_size);
  SP_DEVICE_INTERFACE_DETAIL_DATA *detail_data = reinterpret_cast<SP_DEVICE_INTERFACE_DETAIL_DATA *>(detail_data_buff.data());

  detail_data->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);
  SP_DEVINFO_DATA parent_data = {sizeof(SP_DEVINFO_DATA)};
  if (!SetupDiGetDeviceInterfaceDetail(device_info, &interfaceData, detail_data, ULONG(vector_bytes_size(detail_data_buff)), nullptr, &parent_data)) {
    std::cout << "SetupDiGetDeviceInterfaceDetail failed" << std::endl;
    return false;
  }

  uint16_t vid = 0;
  uint16_t pid = 0;
  uint16_t mi = 0;
  std::string guid;
  std::wstring ws(detail_data->DevicePath);
  std::string path(win_to_utf(detail_data->DevicePath));

  /* Parse the following USB path format = \?usb#vid_vvvv&pid_pppp&mi_ii#aaaaaaaaaaaaaaaa#{gggggggg-gggg-gggg-gggg-gggggggggggg} */
  parse_usb_path_multiple_interface(vid, pid, mi, parent_uid, path, guid);
  if (parent_uid.empty()) {
    /* Parse the following USB path format = \?usb#vid_vvvv&pid_pppp#ssss#{gggggggg - gggg - gggg - gggg - gggggggggggg} */
    parse_usb_path_single_interface(vid, pid, serial, path);
  }

  // get driver key for composite device
  buf_size = 0;
  SetupDiGetDeviceRegistryProperty(device_info, &parent_data, SPDRP_DRIVER, nullptr, nullptr, 0, &buf_size);
  if (GetLastError() != ERROR_INSUFFICIENT_BUFFER) {
    std::cout << "SetupDiGetDeviceRegistryProperty failed in an unexpected manner" << std::endl;
    return false;
  }

  std::vector<BYTE> driver_key(buf_size);

  if (!SetupDiGetDeviceRegistryProperty(device_info, &parent_data, SPDRP_DRIVER, nullptr, driver_key.data(), (ULONG)vector_bytes_size(driver_key), nullptr)) {
    std::cout << "SetupDiGetDeviceRegistryProperty failed" << std::endl;
    return false;
  }

  // contains composite device key
  std::wstring targetKey(reinterpret_cast<const wchar_t *>(driver_key.data()));

  // recursively check all hubs, searching for composite device
  for (int i = 0;; i++) {
    std::wstringstream buf;
    buf << "\\\\.\\HCD" << i;
    std::wstring hcd = buf.str();

    // grab handle
    HANDLE h = CreateFile(hcd.c_str(), GENERIC_WRITE | GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
    auto h_gc = std::shared_ptr<void>(h, CloseHandle);
    if (h == INVALID_HANDLE_VALUE) {
      std::cout << "CreateFile failed" << std::endl;
      break;
    } else {
      USB_ROOT_HUB_NAME name;

      // get required space
      if (!DeviceIoControl(h, IOCTL_USB_GET_ROOT_HUB_NAME, nullptr, 0, &name, sizeof(name), nullptr, nullptr)) {
        std::cout << "DeviceIoControl failed" << std::endl;
        return false; // alt: fail silently and hope its on a different root hub
      }

      std::vector<char> name_buff(name.ActualLength);
      USB_ROOT_HUB_NAME *pName = reinterpret_cast<USB_ROOT_HUB_NAME *>(name_buff.data());

      // get name
      if (!DeviceIoControl(h, IOCTL_USB_GET_ROOT_HUB_NAME, nullptr, 0, pName, (ULONG)vector_bytes_size(name_buff), nullptr, nullptr)) {
        std::cout << "DeviceIoControl failed" << std::endl;
        return false; // alt: fail silently and hope its on a different root hub
      }

      // return location if device is connected under this root hub, also provide the port USB spec/speed
      auto usb_res = handle_usb_hub(targetKey, std::wstring(pName->RootHubName));
      if (!std::get<0>(usb_res).empty()) {
        location = std::get<0>(usb_res);
        spec = std::get<1>(usb_res);
        return true;
      }
    }
  }
  return false;
}

// Provides Port Id and the USB Specification (USB type)
bool get_usb_descriptors(uint16_t device_vid, uint16_t device_pid, const std::string &device_uid, std::string &location, usb_spec &spec, std::string &serial) {
  SP_DEVINFO_DATA devInfo = {sizeof(SP_DEVINFO_DATA)};
  std::vector<GUID> guids = {
      GUID_DEVINTERFACE_IMAGE_WIN7,
      GUID_DEVINTERFACE_CAMERA_WIN7,
      GUID_DEVINTERFACE_IMAGE_WIN10,
      GUID_DEVINTERFACE_CAMERA_WIN10};

  for (auto guid : guids) {
    // Build a device info represent all imaging devices
    HDEVINFO device_info = SetupDiGetClassDevsEx(static_cast<const GUID *>(&guid), nullptr, nullptr, DIGCF_PRESENT, nullptr, nullptr, nullptr);

    // Add automatic destructor to the device info
    auto di = std::shared_ptr<void>(device_info, SetupDiDestroyDeviceInfoList);

    if (device_info == INVALID_HANDLE_VALUE) {
      return false;
    }

    // Enumerate all imaging devices
    for (int member_index = 0;; ++member_index) {
      // Get device information element from the device information set
      if (SetupDiEnumDeviceInfo(device_info, member_index, &devInfo) == FALSE) {
        if (GetLastError() == ERROR_NO_MORE_ITEMS)
          break;  // stop when none left
        continue; // silently ignore other errors
      }

      std::string parent_uid;
      if (get_usb_device_descriptors(devInfo.DevInst, device_vid, device_pid, device_uid, location, spec, serial, parent_uid))
        return true;
    }
  }
  std::cout << "Could not find camera (vid " << std::hex << device_vid << " pid " << std::hex << device_pid << " uid " << device_uid << ") in windows device tree" << std::endl;
  return false;
}
std::string get_device_serial(uint16_t device_vid, uint16_t device_pid, const std::string &device_uid) {
  std::string device_serial = "";
  std::string location = "";
  usb_spec spec = usb_undefined;

  get_usb_descriptors(device_vid, device_pid, device_uid, location, spec, device_serial);

  return device_serial;
}

static void query_uvc_devices(UVCDEVICECALLBACK callback, uint16_t vid, uint16_t pid, void *userdata)
// std::vector<uvc_device_info> query_uvc_devices()
{
  std::vector<uvc_device_info> devices;

  auto action = [&devices](const uvc_device_info &info, IMFActivate *) {
    uvc_device_info device_info = info;
    device_info.serial = get_device_serial(info.vid, info.pid, info.unique_id);
    devices.push_back(device_info);
  };

  foreach_uvc_device(action);

  for (int i = 0; i < devices.size(); i++) {
    if (devices[i].vid == vid && devices[i].pid == pid) {
      // printf("got:\nvid:0x%04X\npid:0x%04X\n", devices[i].vid, devices[i].pid);
      //	auto dev = std::make_shared<wmf_uvc_device>(devices[i]);
      callback(&devices[i], userdata);
    }
  }
}

typedef std::tuple<uint32_t, uint32_t, uint32_t, uint32_t> stream_profile_tuple;
// class wmf_backend;
struct stream_profile {
  uint32_t width;
  uint32_t height;
  uint32_t fps;
  uint32_t format;

  operator stream_profile_tuple() const {
    return std::make_tuple(width, height, fps, format);
  }
};
inline bool operator==(const stream_profile &a,
                       const stream_profile &b) {
  return (a.width == b.width) &&
         (a.height == b.height) &&
         (a.fps == b.fps) &&
         (a.format == b.format);
}
typedef double rs2_time_t; /**< Timestamp format. units are milliseconds */
struct frame_object {
  size_t frame_size;
  uint8_t metadata_size;
  const void *pixels;
  const void *metadata;
  rs2_time_t backend_time;
};
typedef std::function<void(stream_profile, frame_object, std::function<void()>)> frame_callback;
struct profile_and_callback {
  stream_profile profile;
  frame_callback callback = nullptr;
};

typedef std::function<void(const uvc_device_info &, IMFActivate *)>
    enumeration_callback;

typedef struct frame_rate {
  unsigned int denominator;
  unsigned int numerator;
} frame_rate;

typedef struct mf_profile {
  uint32_t index;
  frame_rate min_rate;
  frame_rate max_rate;
  stream_profile profile;
} mf_profile;
/** \brief Severity of the librealsense logger. */
typedef enum rs2_log_severity {
  RS2_LOG_SEVERITY_DEBUG,                       /**< Detailed information about ordinary operations */
  RS2_LOG_SEVERITY_INFO,                        /**< Terse information about ordinary operations */
  RS2_LOG_SEVERITY_WARN,                        /**< Indication of possible failure */
  RS2_LOG_SEVERITY_ERROR,                       /**< Indication of definite failure */
  RS2_LOG_SEVERITY_FATAL,                       /**< Indication of unrecoverable failure */
  RS2_LOG_SEVERITY_NONE,                        /**< No logging will occur */
  RS2_LOG_SEVERITY_COUNT,                       /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
  RS2_LOG_SEVERITY_ALL = RS2_LOG_SEVERITY_DEBUG /**< Include any/all log messages */
} rs2_log_severity;
/** \brief Category of the librealsense notification. */
typedef enum rs2_notification_category {
  RS2_NOTIFICATION_CATEGORY_FRAMES_TIMEOUT,              /**< Frames didn't arrived within 5 seconds */
  RS2_NOTIFICATION_CATEGORY_FRAME_CORRUPTED,             /**< Received partial/incomplete frame */
  RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR,              /**< Error reported from the device */
  RS2_NOTIFICATION_CATEGORY_HARDWARE_EVENT,              /**< General Hardeware notification that is not an error */
  RS2_NOTIFICATION_CATEGORY_UNKNOWN_ERROR,               /**< Received unknown error from the device */
  RS2_NOTIFICATION_CATEGORY_FIRMWARE_UPDATE_RECOMMENDED, /**< Current firmware version installed is not the latest available */
  RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION,         /**< A relocalization event has updated the pose provided by a pose sensor */
  RS2_NOTIFICATION_CATEGORY_COUNT                        /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
} rs2_notification_category;
#define LOG_INFO(aa) std::cout << "log:" << aa << std::endl
struct notification {
  notification(rs2_notification_category category, int type, rs2_log_severity severity, std::string description)
      : category(category), type(type), severity(severity), description(description) {
    timestamp = std::chrono::duration<double, std::milli>(std::chrono::system_clock::now().time_since_epoch()).count();
    LOG_INFO(description);
  }

  rs2_notification_category category;
  int type;
  rs2_log_severity severity;
  std::string description;
  double timestamp;
  std::string serialized_data;
};
enum power_state {
  D0,
  D3
};
struct control_range {
  control_range() {}

  control_range(int32_t in_min, int32_t in_max, int32_t in_step, int32_t in_def) {
    populate_raw_data(min, in_min);
    populate_raw_data(max, in_max);
    populate_raw_data(step, in_step);
    populate_raw_data(def, in_def);
  }
  control_range(std::vector<uint8_t> in_min, std::vector<uint8_t> in_max, std::vector<uint8_t> in_step, std::vector<uint8_t> in_def) {
    min = in_min;
    max = in_max;
    step = in_step;
    def = in_def;
  }
  std::vector<uint8_t> min;
  std::vector<uint8_t> max;
  std::vector<uint8_t> step;
  std::vector<uint8_t> def;

private:
  void populate_raw_data(std::vector<uint8_t> &vec, int32_t value);
};
void control_range::populate_raw_data(std::vector<uint8_t> &vec, int32_t value) {
  vec.resize(sizeof(value));
  auto data = reinterpret_cast<const uint8_t *>(&value);
  std::copy(data, data + sizeof(value), vec.data());
}
struct guid {
  uint32_t data1;
  uint16_t data2, data3;
  uint8_t data4[8];
};
struct extension_unit {
  int subdevice;
  uint8_t unit;
  int node;
  guid id;
};
const uint8_t DEFAULT_V4L2_FRAME_BUFFERS = 4;
/** \brief Defines general configuration controls.
These can generally be mapped to camera UVC controls, and can be set / queried at any time unless stated otherwise.
*/
typedef enum rs2_option {
  RS2_OPTION_BACKLIGHT_COMPENSATION,                  /**< Enable / disable color backlight compensation*/
  RS2_OPTION_BRIGHTNESS,                              /**< Color image brightness*/
  RS2_OPTION_CONTRAST,                                /**< Color image contrast*/
  RS2_OPTION_EXPOSURE,                                /**< Controls exposure time of color camera. Setting any value will disable auto exposure*/
  RS2_OPTION_GAIN,                                    /**< Color image gain*/
  RS2_OPTION_GAMMA,                                   /**< Color image gamma setting*/
  RS2_OPTION_HUE,                                     /**< Color image hue*/
  RS2_OPTION_SATURATION,                              /**< Color image saturation setting*/
  RS2_OPTION_SHARPNESS,                               /**< Color image sharpness setting*/
  RS2_OPTION_WHITE_BALANCE,                           /**< Controls white balance of color image. Setting any value will disable auto white balance*/
  RS2_OPTION_ENABLE_AUTO_EXPOSURE,                    /**< Enable / disable color image auto-exposure*/
  RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,               /**< Enable / disable color image auto-white-balance*/
  RS2_OPTION_VISUAL_PRESET,                           /**< Provide access to several recommend sets of option presets for the depth camera */
  RS2_OPTION_LASER_POWER,                             /**< Power of the laser emitter, with 0 meaning projector off*/
  RS2_OPTION_ACCURACY,                                /**< Set the number of patterns projected per frame. The higher the accuracy value the more patterns projected. Increasing the number of patterns help to achieve better accuracy. Note that this control is affecting the Depth FPS */
  RS2_OPTION_MOTION_RANGE,                            /**< Motion vs. Range trade-off, with lower values allowing for better motion sensitivity and higher values allowing for better depth range*/
  RS2_OPTION_FILTER_OPTION,                           /**< Set the filter to apply to each depth frame. Each one of the filter is optimized per the application requirements*/
  RS2_OPTION_CONFIDENCE_THRESHOLD,                    /**< The confidence level threshold used by the Depth algorithm pipe to set whether a pixel will get a valid range or will be marked with invalid range*/
  RS2_OPTION_EMITTER_ENABLED,                         /**< Emitter select: 0 – disable all emitters. 1 – enable laser. 2 – enable auto laser. 3 – enable LED.*/
  RS2_OPTION_FRAMES_QUEUE_SIZE,                       /**< Number of frames the user is allowed to keep per stream. Trying to hold-on to more frames will cause frame-drops.*/
  RS2_OPTION_TOTAL_FRAME_DROPS,                       /**< Total number of detected frame drops from all streams */
  RS2_OPTION_AUTO_EXPOSURE_MODE,                      /**< Auto-Exposure modes: Static, Anti-Flicker and Hybrid */
  RS2_OPTION_POWER_LINE_FREQUENCY,                    /**< Power Line Frequency control for anti-flickering Off/50Hz/60Hz/Auto */
  RS2_OPTION_ASIC_TEMPERATURE,                        /**< Current Asic Temperature */
  RS2_OPTION_ERROR_POLLING_ENABLED,                   /**< disable error handling */
  RS2_OPTION_PROJECTOR_TEMPERATURE,                   /**< Current Projector Temperature */
  RS2_OPTION_OUTPUT_TRIGGER_ENABLED,                  /**< Enable / disable trigger to be outputed from the camera to any external device on every depth frame */
  RS2_OPTION_MOTION_MODULE_TEMPERATURE,               /**< Current Motion-Module Temperature */
  RS2_OPTION_DEPTH_UNITS,                             /**< Number of meters represented by a single depth unit */
  RS2_OPTION_ENABLE_MOTION_CORRECTION,                /**< Enable/Disable automatic correction of the motion data */
  RS2_OPTION_AUTO_EXPOSURE_PRIORITY,                  /**< Allows sensor to dynamically ajust the frame rate depending on lighting conditions */
  RS2_OPTION_COLOR_SCHEME,                            /**< Color scheme for data visualization */
  RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED,          /**< Perform histogram equalization post-processing on the depth data */
  RS2_OPTION_MIN_DISTANCE,                            /**< Minimal distance to the target */
  RS2_OPTION_MAX_DISTANCE,                            /**< Maximum distance to the target */
  RS2_OPTION_TEXTURE_SOURCE,                          /**< Texture mapping stream unique ID */
  RS2_OPTION_FILTER_MAGNITUDE,                        /**< The 2D-filter effect. The specific interpretation is given within the context of the filter */
  RS2_OPTION_FILTER_SMOOTH_ALPHA,                     /**< 2D-filter parameter controls the weight/radius for smoothing.*/
  RS2_OPTION_FILTER_SMOOTH_DELTA,                     /**< 2D-filter range/validity threshold*/
  RS2_OPTION_HOLES_FILL,                              /**< Enhance depth data post-processing with holes filling where appropriate*/
  RS2_OPTION_STEREO_BASELINE,                         /**< The distance in mm between the first and the second imagers in stereo-based depth cameras*/
  RS2_OPTION_AUTO_EXPOSURE_CONVERGE_STEP,             /**< Allows dynamically ajust the converge step value of the target exposure in Auto-Exposure algorithm*/
  RS2_OPTION_INTER_CAM_SYNC_MODE,                     /**< Impose Inter-camera HW synchronization mode. Applicable for D400/L500/Rolling Shutter SKUs */
  RS2_OPTION_STREAM_FILTER,                           /**< Select a stream to process */
  RS2_OPTION_STREAM_FORMAT_FILTER,                    /**< Select a stream format to process */
  RS2_OPTION_STREAM_INDEX_FILTER,                     /**< Select a stream index to process */
  RS2_OPTION_EMITTER_ON_OFF,                          /**< When supported, this option make the camera to switch the emitter state every frame. 0 for disabled, 1 for enabled */
  RS2_OPTION_ZERO_ORDER_POINT_X,                      /**< Deprecated!!! - Zero order point x*/
  RS2_OPTION_ZERO_ORDER_POINT_Y,                      /**< Deprecated!!! - Zero order point y*/
  RS2_OPTION_LLD_TEMPERATURE,                         /**< LDD temperature*/
  RS2_OPTION_MC_TEMPERATURE,                          /**< MC temperature*/
  RS2_OPTION_MA_TEMPERATURE,                          /**< MA temperature*/
  RS2_OPTION_HARDWARE_PRESET,                         /**< Hardware stream configuration */
  RS2_OPTION_GLOBAL_TIME_ENABLED,                     /**< disable global time  */
  RS2_OPTION_APD_TEMPERATURE,                         /**< APD temperature*/
  RS2_OPTION_ENABLE_MAPPING,                          /**< Enable an internal map */
  RS2_OPTION_ENABLE_RELOCALIZATION,                   /**< Enable appearance based relocalization */
  RS2_OPTION_ENABLE_POSE_JUMPING,                     /**< Enable position jumping */
  RS2_OPTION_ENABLE_DYNAMIC_CALIBRATION,              /**< Enable dynamic calibration */
  RS2_OPTION_DEPTH_OFFSET,                            /**< Offset from sensor to depth origin in millimetrers*/
  RS2_OPTION_LED_POWER,                               /**< Power of the LED (light emitting diode), with 0 meaning LED off*/
  RS2_OPTION_ZERO_ORDER_ENABLED,                      /**< DEPRECATED! - Toggle Zero-Order mode */
  RS2_OPTION_ENABLE_MAP_PRESERVATION,                 /**< Preserve previous map when starting */
  RS2_OPTION_FREEFALL_DETECTION_ENABLED,              /**< Enable/disable sensor shutdown when a free-fall is detected (on by default) */
  RS2_OPTION_AVALANCHE_PHOTO_DIODE,                   /**< Changes the exposure time of Avalanche Photo Diode in the receiver */
  RS2_OPTION_POST_PROCESSING_SHARPENING,              /**< Changes the amount of sharpening in the post-processed image */
  RS2_OPTION_PRE_PROCESSING_SHARPENING,               /**< Changes the amount of sharpening in the pre-processed image */
  RS2_OPTION_NOISE_FILTERING,                         /**< Control edges and background noise */
  RS2_OPTION_INVALIDATION_BYPASS,                     /**< Enable\disable pixel invalidation */
  RS2_OPTION_AMBIENT_LIGHT,                           /**< DEPRECATED! - Use RS2_OPTION_DIGITAL_GAIN instead. */
  RS2_OPTION_DIGITAL_GAIN = RS2_OPTION_AMBIENT_LIGHT, /**< Change the depth digital gain see rs2_digital_gain for values */
  RS2_OPTION_SENSOR_MODE,                             /**< The resolution mode: see rs2_sensor_mode for values */
  RS2_OPTION_EMITTER_ALWAYS_ON,                       /**< Enable Laser On constantly (GS SKU Only) */
  RS2_OPTION_THERMAL_COMPENSATION,                    /**< Depth Thermal Compensation for selected D400 SKUs */
  RS2_OPTION_TRIGGER_CAMERA_ACCURACY_HEALTH,          /**< DEPRECATED as of 2.46! */
  RS2_OPTION_RESET_CAMERA_ACCURACY_HEALTH,            /**< DEPRECATED as of 2.46! */
  RS2_OPTION_HOST_PERFORMANCE,                        /**< Set host performance mode to optimize device settings so host can keep up with workload, for example, USB transaction granularity, setting option to low performance host leads to larger USB transaction size and reduced number of transactions which improves performance and stability if host is relatively weak as compared to workload */
  RS2_OPTION_HDR_ENABLED,                             /**< Enable / disable HDR */
  RS2_OPTION_SEQUENCE_NAME,                           /**< HDR Sequence name */
  RS2_OPTION_SEQUENCE_SIZE,                           /**< HDR Sequence size */
  RS2_OPTION_SEQUENCE_ID,                             /**< HDR Sequence ID - 0 is not HDR; sequence ID for HDR configuration starts from 1 */
  RS2_OPTION_HUMIDITY_TEMPERATURE,                    /**< Humidity temperature [Deg Celsius]*/
  RS2_OPTION_ENABLE_MAX_USABLE_RANGE,                 /**< Turn on/off the maximum usable depth sensor range given the amount of ambient light in the scene */
  RS2_OPTION_ALTERNATE_IR,                            /**< Turn on/off the alternate IR, When enabling alternate IR, the IR image is holding the amplitude of the depth correlation. */
  RS2_OPTION_NOISE_ESTIMATION,                        /**< Noise estimation - indicates the noise on the IR image */
  RS2_OPTION_ENABLE_IR_REFLECTIVITY,                  /**< Enables data collection for calculating IR pixel reflectivity  */
  RS2_OPTION_AUTO_EXPOSURE_LIMIT,                     /**< Set and get auto exposure limit in microseconds. If the requested exposure limit is greater than frame time, it will be set to frame time at runtime. Setting will not take effect until next streaming session. */
  RS2_OPTION_AUTO_GAIN_LIMIT,                         /**< Set and get auto gain limits ranging from 16 to 248. If the requested gain limit is less than 16, it will be set to 16. If the requested gain limit is greater than 248, it will be set to 248. Setting will not take effect until next streaming session. */
  RS2_OPTION_AUTO_RX_SENSITIVITY,                     /**< Enable receiver sensitivity according to ambient light, bounded by the Receiver Gain control. */
  RS2_OPTION_TRANSMITTER_FREQUENCY,                   /**<changes the transmitter frequencies increasing effective range over sharpness. */
  RS2_OPTION_VERTICAL_BINNING,                        /**< Enables vertical binning which increases the maximal sensed distance. */
  RS2_OPTION_RECEIVER_SENSITIVITY,                    /**< Control receiver sensitivity to incoming light, both projected and ambient (same as APD on L515). */
  RS2_OPTION_AUTO_EXPOSURE_LIMIT_TOGGLE,              /**< Enable / disable color image auto-exposure*/
  RS2_OPTION_AUTO_GAIN_LIMIT_TOGGLE,                  /**< Enable / disable color image auto-gain*/
  RS2_OPTION_COUNT                                    /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
} rs2_option;
class uvc_device {
public:
  virtual void probe_and_commit(stream_profile profile, frame_callback callback, int buffers = DEFAULT_V4L2_FRAME_BUFFERS) = 0;
  virtual void stream_on(std::function<void(const notification &n)> error_handler = [](const notification &n) {}) = 0;
  virtual void start_callbacks() = 0;
  virtual void stop_callbacks() = 0;
  virtual void close(stream_profile profile) = 0;

  virtual void set_power_state(power_state state) = 0;
  virtual power_state get_power_state() const = 0;

  virtual void init_xu(const extension_unit &xu) = 0;
  virtual bool set_xu(const extension_unit &xu, uint8_t ctrl, const uint8_t *data, int len) = 0;
  virtual bool get_xu(const extension_unit &xu, uint8_t ctrl, uint8_t *data, int len) const = 0;
  virtual control_range get_xu_range(const extension_unit &xu, uint8_t ctrl, int len) const = 0;

  virtual bool get_pu(rs2_option opt, int32_t &value) const = 0;
  virtual bool set_pu(rs2_option opt, int32_t value) = 0;
  virtual control_range get_pu_range(rs2_option opt) const = 0;

  virtual std::vector<stream_profile> get_profiles() const = 0;

  virtual void lock() const = 0;
  virtual void unlock() const = 0;

  virtual std::string get_device_location() const = 0;
  virtual usb_spec get_usb_specification() const = 0;

  virtual ~uvc_device() = default;

protected:
  std::function<void(const notification &n)> _error_handler;
};
class event_base {
public:
  virtual ~event_base();
  virtual bool set();
  virtual bool wait(DWORD timeout) const;

  static event_base *wait(const std::vector<event_base *> &events, bool waitAll, int timeout);
  static event_base *wait_any(const std::vector<event_base *> &events, int timeout);
  static event_base *wait_all(const std::vector<event_base *> &events, int timeout);

  HANDLE get_handle() const { return _handle; }

protected:
  explicit event_base(HANDLE handle);

  HANDLE _handle;

private:
  event_base() = delete;

  // Disallow copy:
  event_base(const event_base &) = delete;
  event_base &operator=(const event_base &) = delete;
};

#define MAX_HANDLES 64

event_base::event_base(HANDLE handle)
    : _handle(handle) {}

event_base::~event_base() {
  if (_handle != nullptr) {
    CloseHandle(_handle);
    _handle = nullptr;
  }
}

bool event_base::set() {
  if (_handle == nullptr)
    return false;
  SetEvent(_handle);
  return true;
}

bool event_base::wait(DWORD timeout) const {
  if (_handle == nullptr)
    return false;

  return WaitForSingleObject(_handle, timeout) == WAIT_OBJECT_0; // Return true only if object was signaled
}

event_base *event_base::wait(const std::vector<event_base *> &events, bool waitAll, int timeout) {
  if (events.size() > MAX_HANDLES)
    return nullptr; // WaitForMultipleObjects doesn't support waiting on more then 64 handles

  HANDLE handles[MAX_HANDLES];
  auto i = 0;
  for (auto &evnt : events) {
    handles[i] = evnt->get_handle();
    ++i;
  }
  auto res = WaitForMultipleObjects(static_cast<DWORD>(events.size()), handles, waitAll, timeout);
  if (res < (WAIT_OBJECT_0 + events.size())) {
    return events[res - WAIT_OBJECT_0];
  } else {
    return nullptr;
  }
}

event_base *event_base::wait_all(const std::vector<event_base *> &events, int timeout) {
  return wait(events, true, timeout);
}

event_base *event_base::wait_any(const std::vector<event_base *> &events, int timeout) {
  return wait(events, false, timeout);
}
class auto_reset_event : public event_base {
public:
  auto_reset_event();
};
auto_reset_event::auto_reset_event()
    : event_base(CreateEvent(nullptr, FALSE, FALSE, nullptr)) {}
class manual_reset_event : public event_base {
public:
  manual_reset_event();

  bool reset() const;
};
bool manual_reset_event::reset() const {
  if (_handle == nullptr)
    return false;
  return ResetEvent(_handle) != 0;
}

manual_reset_event::manual_reset_event()
    : event_base(CreateEvent(nullptr, TRUE, FALSE, nullptr)) {}
enum create_and_open_status {
  Mutex_Succeed,
  Mutex_TotalFailure,
  Mutex_AlreadyExist
};
class named_mutex {
public:
  named_mutex(const char *id, unsigned timeout);
  ~named_mutex();
  bool try_lock() const;
  void lock() const { acquire(); }
  void unlock() const { release(); }

private:
  create_and_open_status create_named_mutex(const char *camID);
  create_and_open_status open_named_mutex(const char *camID);
  void update_id(const char *id);
  void acquire() const;
  void release() const;
  void close();

  unsigned _timeout;
  HANDLE _winusb_mutex;
};
named_mutex::named_mutex(const char *id, unsigned timeout)
    : _timeout(timeout),
      _winusb_mutex(nullptr) {
  update_id(id);
}
PSECURITY_DESCRIPTOR make_allow_all_security_descriptor(void) {
  WCHAR *pszStringSecurityDescriptor;
  pszStringSecurityDescriptor = L"D:(A;;GA;;;WD)(A;;GA;;;AN)S:(ML;;NW;;;ME)";
  PSECURITY_DESCRIPTOR pSecDesc;
  if (!ConvertStringSecurityDescriptorToSecurityDescriptor(
          pszStringSecurityDescriptor, SDDL_REVISION_1, &pSecDesc, nullptr))
    return nullptr;

  return pSecDesc;
}
create_and_open_status named_mutex::create_named_mutex(const char *camID) {
  // IVCAM_DLL string is left in librealsense to allow safe
  // interoperability with existing tools like DCM
  std::string lstr("Global\\IVCAM_DLL_WINUSB_MUTEX");
  lstr += camID;
  auto pSecDesc = make_allow_all_security_descriptor();
  if (pSecDesc) {
    SECURITY_ATTRIBUTES SecAttr;
    SecAttr.nLength = sizeof(SECURITY_ATTRIBUTES);
    SecAttr.lpSecurityDescriptor = pSecDesc;
    SecAttr.bInheritHandle = FALSE;

    _winusb_mutex = CreateMutexA(
        &SecAttr,
        FALSE,
        lstr.c_str());
    LocalFree(pSecDesc);
  }
  // CreateMutex failed
  if (_winusb_mutex == nullptr) {
    return Mutex_TotalFailure;
  } else if (GetLastError() == ERROR_ALREADY_EXISTS) {
    return Mutex_AlreadyExist;
  }
  return Mutex_Succeed;
}

create_and_open_status named_mutex::open_named_mutex(const char *camID) {
  // IVCAM_DLL string is left in librealsense to allow safe
  // interoperability with existing tools like DCM
  std::string lstr("Global\\IVCAM_DLL_WINUSB_MUTEX");
  lstr += camID;

  _winusb_mutex = OpenMutexA(MUTEX_ALL_ACCESS, // request full access
                             FALSE,            // handle not inheritable
                             lstr.c_str());    // object name

  if (_winusb_mutex == nullptr) {
    return Mutex_TotalFailure;
  }

  return Mutex_Succeed;
}

#define CREATE_MUTEX_RETRY_NUM (5)
void named_mutex::update_id(const char *camID) {
  auto stsCreateMutex = Mutex_Succeed;
  auto stsOpenMutex = Mutex_Succeed;

  if (_winusb_mutex == nullptr) {

    for (int i = 0; i < CREATE_MUTEX_RETRY_NUM; i++) {
      stsCreateMutex = create_named_mutex(camID);

      switch (stsCreateMutex) {
      case Mutex_Succeed:
        return;
      case Mutex_TotalFailure:
        throw std::runtime_error("CreateNamedMutex returned Mutex_TotalFailure");
      case Mutex_AlreadyExist: {
        stsOpenMutex = open_named_mutex(camID);

        // if OpenMutex failed retry to create the mutex
        // it can caused by termination of the process that created the mutex
        if (stsOpenMutex == Mutex_TotalFailure) {
          continue;
        } else if (stsOpenMutex == Mutex_Succeed) {
          return;
        } else {
          throw std::runtime_error("OpenNamedMutex returned error " + stsOpenMutex);
        }
      }
      default:
        break;
      };
    }
    throw std::runtime_error("Open mutex failed!");
  }
  // Mutex is already exist this mean that
  // the mutex already opened by this process and the method called again after connect event.
  else {
    for (auto i = 0; i < CREATE_MUTEX_RETRY_NUM; i++) {
      auto tempMutex = _winusb_mutex;
      stsCreateMutex = create_named_mutex(camID);

      switch (stsCreateMutex) {
        // if creation succeed this mean that new camera connected
        // and we need to close the old mutex
      case Mutex_Succeed: {
        auto res = CloseHandle(tempMutex);
        if (!res) {
          throw std::runtime_error("CloseHandle failed");
        }
        return;
      }
      case Mutex_TotalFailure: {
        throw std::runtime_error("CreateNamedMutex returned Mutex_TotalFailure");
      }
      // Mutex already created by:
      //  1. This process - which mean the same camera connected.
      //  2. Other process created the mutex.
      case Mutex_AlreadyExist: {
        stsOpenMutex = open_named_mutex(camID);

        if (stsOpenMutex == Mutex_TotalFailure) {
          continue;
        } else if (stsOpenMutex == Mutex_Succeed) {
          return;
        } else {
          throw std::runtime_error("OpenNamedMutex failed with error " + stsOpenMutex);
        }
      }
      default:
        break;
      }
    }

    throw std::runtime_error("Open mutex failed!");
  }
}

bool named_mutex::try_lock() const {
  return (WaitForSingleObject(_winusb_mutex, _timeout) == WAIT_TIMEOUT) ? false : true;
}

void named_mutex::acquire() const {
  if (!try_lock()) {
    throw std::runtime_error("Acquire failed!");
  }
}

void named_mutex::release() const {
  auto sts = ReleaseMutex(_winusb_mutex);
  if (!sts) {
    throw std::runtime_error("Failed to release winUsb named Mutex! LastError: " + GetLastError());
  }
}

named_mutex::~named_mutex() {
  close();
}

void named_mutex::close() {
  if (_winusb_mutex != nullptr) {
    CloseHandle(_winusb_mutex);
    _winusb_mutex = nullptr;
  }
}
class wmf_uvc_device : // public std::enable_shared_from_this<wmf_uvc_device>,
                       public uvc_device {
public:
  wmf_uvc_device(const uvc_device_info &info); //, std::shared_ptr<const wmf_backend> backend);
  ~wmf_uvc_device();

  EVENTCALLBACK eventcallback;
  void *connectuserdata;

  void probe_and_commit(stream_profile profile, frame_callback callback, int buffers) override;
  void stream_on(std::function<void(const notification &n)> error_handler = [](const notification &n) {}) override;
  void start_callbacks() override;
  void stop_callbacks() override;
  void close(stream_profile profile) override;
  void set_power_state(power_state state) override;
  power_state get_power_state() const override { return _power_state; }
  std::vector<stream_profile> get_profiles() const override;

  static bool is_connected(const uvc_device_info &info);
  static void foreach_uvc_device(enumeration_callback action);

  void init_xu(const extension_unit &xu) override;
  bool set_xu(const extension_unit &xu, uint8_t ctrl, const uint8_t *data, int len) override;
  bool get_xu(const extension_unit &xu, uint8_t ctrl, uint8_t *data, int len) const override;
  control_range get_xu_range(const extension_unit &xu, uint8_t ctrl, int len) const override;

  bool get_pu(rs2_option opt, int32_t &value) const override;
  bool set_pu(rs2_option opt, int value) override;
  control_range get_pu_range(rs2_option opt) const override;

  void lock() const override { _systemwide_lock.lock(); }
  void unlock() const override { _systemwide_lock.unlock(); }

  std::string get_device_location() const override { return _location; }
  usb_spec get_usb_specification() const override { return _device_usb_spec; }
  IAMVideoProcAmp *get_video_proc() const;
  IAMCameraControl *get_camera_control() const;

private:
  friend class source_reader_callback;

  void play_profile(stream_profile profile, frame_callback callback);
  void stop_stream_cleanup(const stream_profile &profile, std::vector<profile_and_callback>::iterator &elem);
  void flush(int sIndex);
  void check_connection() const;
  void close_all();
  IKsControl *get_ks_control(const extension_unit &xu) const;
  CComPtr<IMFAttributes> create_device_attrs();
  CComPtr<IMFAttributes> create_reader_attrs();
  void foreach_profile(std::function<void(const mf_profile &profile, CComPtr<IMFMediaType> media_type, bool &quit)> action) const;

  void set_d0();
  void set_d3();

  // Don't move the position of wmf_backend member. This object must be destroyed only after COM objects.
  // std::shared_ptr<const wmf_backend>      _backend;

  const uvc_device_info _info;
  power_state _power_state = D3;

  CComPtr<IMFSourceReader> _reader = nullptr;
  CComPtr<IMFMediaSource> _source = nullptr;
  CComPtr<IMFAttributes> _device_attrs = nullptr;
  CComPtr<IMFAttributes> _reader_attrs = nullptr;

  CComPtr<IAMCameraControl> _camera_control = nullptr;
  CComPtr<IAMVideoProcAmp> _video_proc = nullptr;
  std::unordered_map<int, CComPtr<IKsControl>> _ks_controls;

  auto_reset_event _is_flushed;
  manual_reset_event _has_started;
  HRESULT _readsample_result = S_OK;

  uint16_t _streamIndex;
  std::vector<profile_and_callback> _streams;
  std::mutex _streams_mutex;

  named_mutex _systemwide_lock;
  std::string _location;
  usb_spec _device_usb_spec;
  std::string _device_serial;
  std::vector<stream_profile> _profiles;
  std::vector<frame_callback> _frame_callbacks;
  bool _streaming = false;
  std::atomic<bool> _is_started = false;
  std::wstring _device_id;
};

class source_reader_callback : public IMFSourceReaderCallback {
public:
  explicit source_reader_callback(wmf_uvc_device * /*std::weak_ptr<wmf_uvc_device>*/ owner) : _owner(owner){};
  virtual ~source_reader_callback(){};
  STDMETHODIMP QueryInterface(REFIID iid, void **ppv) override;
  STDMETHODIMP_(ULONG)
  AddRef() override;
  STDMETHODIMP_(ULONG)
  Release() override;
  STDMETHODIMP OnReadSample(HRESULT /*hrStatus*/,
                            DWORD dwStreamIndex,
                            DWORD /*dwStreamFlags*/,
                            LONGLONG /*llTimestamp*/,
                            IMFSample *sample) override;
  STDMETHODIMP OnEvent(DWORD /*sidx*/, IMFMediaEvent * /*event*/) override;
  STDMETHODIMP OnFlush(DWORD) override;

private:
  wmf_uvc_device * /*std::weak_ptr<wmf_uvc_device>*/ _owner;
  long _refCount = 0;
};

STDMETHODIMP source_reader_callback::QueryInterface(REFIID iid, void **ppv) {
#pragma warning(push)
#pragma warning(disable : 4838)
  static const QITAB qit[] =
      {
          QITABENT(source_reader_callback, IMFSourceReaderCallback),
          {nullptr},
      };
  return QISearch(this, qit, iid, ppv);
#pragma warning(pop)
};

STDMETHODIMP_(ULONG)
source_reader_callback::AddRef() { return InterlockedIncrement(&_refCount); }

STDMETHODIMP_(ULONG)
source_reader_callback::Release() {
  ULONG count = InterlockedDecrement(&_refCount);
  if (count <= 0) {
    delete this;
  }
  return count;
}
double monotonic_to_realtime(double monotonic) {
  auto realtime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  auto time_since_epoch = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  return monotonic + (realtime - time_since_epoch);
}
STDMETHODIMP source_reader_callback::OnReadSample(HRESULT hrStatus,
                                                  DWORD dwStreamIndex,
                                                  DWORD dwStreamFlags,
                                                  LONGLONG llTimestamp,
                                                  IMFSample *sample) {
  //_owner->lock();
  auto owner = _owner; // ->lock();
  if (owner && _owner->_reader) {
    if (FAILED(hrStatus)) {
      owner->_readsample_result = hrStatus;
      if (dwStreamFlags == MF_SOURCE_READERF_ERROR) {
        owner->close_all();
        if (owner->eventcallback != NULL) {
          owner->eventcallback(PLUGOUT, owner->connectuserdata);
        }
        return S_OK;
      }
    }
    owner->_has_started.set();

    /*LOG_HR*/ (owner->_reader->ReadSample(dwStreamIndex, 0, nullptr, nullptr, nullptr, nullptr));

    if (!owner->_is_started)
      return S_OK;

    if (sample) {
      CComPtr<IMFMediaBuffer> buffer = nullptr;
      if (SUCCEEDED(sample->GetBufferByIndex(0, &buffer))) {
        byte *byte_buffer = nullptr;
        DWORD max_length{}, current_length{};
        if (SUCCEEDED(buffer->Lock(&byte_buffer, &max_length, &current_length))) {
          byte *metadata = nullptr;
          uint8_t metadata_size = 0;
#ifdef METADATA_SUPPORT
          try_read_metadata(sample, metadata_size, &metadata);
#endif
          try {
            auto &stream = owner->_streams[dwStreamIndex];
            std::lock_guard<std::mutex> lock(owner->_streams_mutex);
            auto profile = stream.profile;
            frame_object f{current_length, metadata_size, byte_buffer, metadata, monotonic_to_realtime(llTimestamp / 10000.f)};

            auto continuation = [buffer, this]() {
              buffer->Unlock();
            };

            stream.callback(profile, f, continuation);
          } catch (...) {
            // TODO: log
          }
        }
      }
    }
  }

  return S_OK;
};
STDMETHODIMP source_reader_callback::OnEvent(DWORD /*sidx*/, IMFMediaEvent * /*event*/) { return S_OK; }
STDMETHODIMP source_reader_callback::OnFlush(DWORD) {
  // _owner->lock();
  auto owner = _owner;
  if (owner) {
    owner->_is_flushed.set();
  }
  return S_OK;
}

bool wmf_uvc_device::is_connected(const uvc_device_info &info) {
  auto result = false;
  foreach_uvc_device([&result, &info](const uvc_device_info &i, IMFActivate *) {
    if (i == info)
      result = true;
  });
  return result;
}

IKsControl *wmf_uvc_device::get_ks_control(const extension_unit &xu) const {
  auto it = _ks_controls.find(xu.node);
  if (it != std::end(_ks_controls))
    return it->second;
  throw std::runtime_error("Extension control must be initialized before use!");
}

void wmf_uvc_device::init_xu(const extension_unit &xu) {
  if (!_source)
    throw std::runtime_error("Could not initialize extensions controls!");

  // Attempt to retrieve IKsControl
  CComPtr<IKsTopologyInfo> ks_topology_info = nullptr;
  CHECK_HR(_source->QueryInterface(__uuidof(IKsTopologyInfo),
                                   reinterpret_cast<void **>(&ks_topology_info)));

  DWORD nNodes = 0;
  /*LOG_HR_STR*/ ("get_NumNodes", ks_topology_info->get_NumNodes(&nNodes));

  /*for (unsigned int i = 0; i < nNodes; i++)
  {
  WCHAR buffer[512];
  GUID guidNodeType;
  DWORD len;
  ks_topology_info->get_NodeName(i, buffer, 512*sizeof(WCHAR), &len);
  std::string tmpstr = win_to_default(buffer);
  printf("nodename:%s\n", tmpstr.c_str());
  ks_topology_info->get_NodeType(i, &guidNodeType);
  printf("got guid node type:%x,%x,%x,%x,%x,%x,%x,%x,%x,%x,%x\n",
  guidNodeType.Data1,
  guidNodeType.Data2,
  guidNodeType.Data3,
  guidNodeType.Data4[0],
  guidNodeType.Data4[1],
  guidNodeType.Data4[2],
  guidNodeType.Data4[3],
  guidNodeType.Data4[4],
  guidNodeType.Data4[5],
  guidNodeType.Data4[6],
  guidNodeType.Data4[7]);
  }*/
  CComPtr<IUnknown> unknown = nullptr;
  CHECK_HR(ks_topology_info->CreateNodeInstance(xu.node, IID_IUnknown,
                                                reinterpret_cast<LPVOID *>(&unknown)));

  CComPtr<IKsControl> ks_control = nullptr;
  CHECK_HR(unknown->QueryInterface(__uuidof(IKsControl),
                                   reinterpret_cast<void **>(&ks_control)));
  _ks_controls[xu.node] = ks_control;
}

#define DEVICE_NOT_READY_ERROR _HRESULT_TYPEDEF_(0x80070015L)
// struct extension_unit { int subdevice; uint8_t unit; int node; guid id; };
bool wmf_uvc_device::set_xu(const extension_unit &xu, uint8_t ctrl, const uint8_t *data, int len) { // 1.xu中的unit对应xu_query中的unit
                                                                                                    //  2.ctrl对应xu_query中的selector=1
                                                                                                    //  3.KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY对应xu_query中的query=UVC_SET_CUR
                                                                                                    //  3.KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY对应xu_query中的query=UVC_GET_CUR
                                                                                                    //  4.windows上还需要提供guid和node,node是枚举出来的guid的index
  auto ks_control = get_ks_control(xu);

  KSP_NODE node;
  memset(&node, 0, sizeof(KSP_NODE));
  node.Property.Set = reinterpret_cast<const GUID &>(xu.id);
  node.Property.Id = ctrl;
  node.Property.Flags = KSPROPERTY_TYPE_SET | KSPROPERTY_TYPE_TOPOLOGY;
  node.NodeId = xu.node;

  ULONG bytes_received = 0;
  auto hr = ks_control->KsProperty(reinterpret_cast<PKSPROPERTY>(&node),
                                   sizeof(KSP_NODE), (void *)data, len, &bytes_received);

  if (hr == DEVICE_NOT_READY_ERROR)
    return false;

  CHECK_HR(hr);
  return true;
}
struct to_string {
  std::ostringstream ss;
  template <class T>
  to_string &operator<<(const T &val) {
    ss << val;
    return *this;
  }
  operator std::string() const { return ss.str(); }
};
bool wmf_uvc_device::get_xu(const extension_unit &xu, uint8_t ctrl, uint8_t *data, int len) const {
  auto ks_control = get_ks_control(xu);

  KSP_NODE node;
  memset(&node, 0, sizeof(KSP_NODE));
  node.Property.Set = reinterpret_cast<const GUID &>(xu.id);
  node.Property.Id = ctrl;
  node.Property.Flags = KSPROPERTY_TYPE_GET | KSPROPERTY_TYPE_TOPOLOGY;
  node.NodeId = xu.node;

  ULONG bytes_received = 0;
  auto hr = ks_control->KsProperty(reinterpret_cast<PKSPROPERTY>(&node),
                                   sizeof(node), data, len, &bytes_received);

  if (hr == DEVICE_NOT_READY_ERROR)
    return false;
  CHECK_HR(hr);

  if (bytes_received != len)
    throw std::runtime_error(to_string() << "Get XU n:" << (int)ctrl << " received " << bytes_received << "/" << len << " bytes");

  return true;
}
void copy(void *dst, void const *src, size_t size) {
  auto from = reinterpret_cast<uint8_t const *>(src);
  std::copy(from, from + size, reinterpret_cast<uint8_t *>(dst));
}
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif
void ReadFromBuffer(control_range &cfg, BYTE *buffer, int length) {
  BYTE *next_struct = buffer;

  PKSPROPERTY_DESCRIPTION pDesc = reinterpret_cast<PKSPROPERTY_DESCRIPTION>(next_struct);
  next_struct += sizeof(KSPROPERTY_DESCRIPTION);

  if (pDesc->MembersListCount < 1)
    throw std::exception("no data ksprop");

  PKSPROPERTY_MEMBERSHEADER pHeader = reinterpret_cast<PKSPROPERTY_MEMBERSHEADER>(next_struct);
  next_struct += sizeof(KSPROPERTY_MEMBERSHEADER);

  if (pHeader->MembersCount < 1)
    throw std::exception("no data ksprop");

  // The data fields are up to four bytes
  auto field_width = std::min(sizeof(uint32_t), (size_t)length);
  auto option_range_size = std::max(sizeof(uint32_t), (size_t)length);
  switch (pHeader->MembersFlags) {
    /* member flag is not set correctly in current IvCam Implementation */
  case KSPROPERTY_MEMBER_RANGES:
  case KSPROPERTY_MEMBER_STEPPEDRANGES: {
    if (pDesc->DescriptionSize < sizeof(KSPROPERTY_DESCRIPTION) + sizeof(KSPROPERTY_MEMBERSHEADER) + 3 * sizeof(UCHAR)) {
      throw std::exception("no data ksprop");
    }

    auto pStruct = next_struct;
    cfg.step.resize(option_range_size);
    copy(cfg.step.data(), pStruct, field_width);
    pStruct += length;
    cfg.min.resize(option_range_size);
    copy(cfg.min.data(), pStruct, field_width);
    pStruct += length;
    cfg.max.resize(option_range_size);
    copy(cfg.max.data(), pStruct, field_width);
    return;
  }
  case KSPROPERTY_MEMBER_VALUES: {
    /*
     *   we don't yet support reading a list of values, only min-max.
     *   so we only support reading default value from a list
     */

    if (pHeader->Flags == KSPROPERTY_MEMBER_FLAG_DEFAULT && pHeader->MembersCount == 1) {
      if (pDesc->DescriptionSize < sizeof(KSPROPERTY_DESCRIPTION) + sizeof(KSPROPERTY_MEMBERSHEADER) + sizeof(UCHAR)) {
        throw std::exception("no data ksprop");
      }

      cfg.def.resize(option_range_size);
      copy(cfg.def.data(), next_struct, field_width);
    }
    return;
  }
  default:
    throw std::exception("unsupported");
  }
}

control_range wmf_uvc_device::get_xu_range(const extension_unit &xu, uint8_t ctrl, int len) const {
  auto ks_control = get_ks_control(xu);

  /* get step, min and max values*/
  KSP_NODE node;
  memset(&node, 0, sizeof(KSP_NODE));
  node.Property.Set = reinterpret_cast<const GUID &>(xu.id);
  node.Property.Id = ctrl;
  node.Property.Flags = KSPROPERTY_TYPE_BASICSUPPORT | KSPROPERTY_TYPE_TOPOLOGY;
  node.NodeId = xu.node;

  KSPROPERTY_DESCRIPTION description;
  unsigned long bytes_received = 0;
  CHECK_HR(ks_control->KsProperty(
      reinterpret_cast<PKSPROPERTY>(&node),
      sizeof(node),
      &description,
      sizeof(KSPROPERTY_DESCRIPTION),
      &bytes_received));

  auto size = description.DescriptionSize;
  std::vector<BYTE> buffer(static_cast<long>(size));

  CHECK_HR(ks_control->KsProperty(
      reinterpret_cast<PKSPROPERTY>(&node),
      sizeof(node),
      buffer.data(),
      size,
      &bytes_received));

  if (bytes_received != size) {
    throw std::runtime_error("wrong data");
  }

  control_range result{};
  ReadFromBuffer(result, buffer.data(), len);

  /* get def value*/
  memset(&node, 0, sizeof(KSP_NODE));
  node.Property.Set = reinterpret_cast<const GUID &>(xu.id);
  node.Property.Id = ctrl;
  node.Property.Flags = KSPROPERTY_TYPE_DEFAULTVALUES | KSPROPERTY_TYPE_TOPOLOGY;
  node.NodeId = xu.node;

  bytes_received = 0;
  CHECK_HR(ks_control->KsProperty(
      reinterpret_cast<PKSPROPERTY>(&node),
      sizeof(node),
      &description,
      sizeof(KSPROPERTY_DESCRIPTION),
      &bytes_received));

  size = description.DescriptionSize;
  buffer.clear();
  buffer.resize(size);

  CHECK_HR(ks_control->KsProperty(
      reinterpret_cast<PKSPROPERTY>(&node),
      sizeof(node),
      buffer.data(),
      size,
      &bytes_received));

  if (bytes_received != size) {
    throw std::runtime_error("wrong data");
  }

  ReadFromBuffer(result, buffer.data(), len);

  return result;
}

struct pu_control {
  rs2_option option;
  long property;
  bool enable_auto;
};
static const pu_control pu_controls[] = {
    {RS2_OPTION_BRIGHTNESS, KSPROPERTY_VIDEOPROCAMP_BRIGHTNESS},
    {RS2_OPTION_CONTRAST, KSPROPERTY_VIDEOPROCAMP_CONTRAST},
    {RS2_OPTION_HUE, KSPROPERTY_VIDEOPROCAMP_HUE},
    {RS2_OPTION_SATURATION, KSPROPERTY_VIDEOPROCAMP_SATURATION},
    {RS2_OPTION_SHARPNESS, KSPROPERTY_VIDEOPROCAMP_SHARPNESS},
    {RS2_OPTION_GAMMA, KSPROPERTY_VIDEOPROCAMP_GAMMA},
    {RS2_OPTION_WHITE_BALANCE, KSPROPERTY_VIDEOPROCAMP_WHITEBALANCE},
    {RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, KSPROPERTY_VIDEOPROCAMP_WHITEBALANCE, true},
    {RS2_OPTION_BACKLIGHT_COMPENSATION, KSPROPERTY_VIDEOPROCAMP_BACKLIGHT_COMPENSATION},
    {RS2_OPTION_GAIN, KSPROPERTY_VIDEOPROCAMP_GAIN},
    {RS2_OPTION_POWER_LINE_FREQUENCY, KSPROPERTY_VIDEOPROCAMP_POWERLINE_FREQUENCY}};

// Camera Terminal controls will be handled with  PU option transport and handling mechanism
static const pu_control ct_controls[] = {
    {RS2_OPTION_AUTO_EXPOSURE_PRIORITY, KSPROPERTY_CAMERACONTROL_AUTO_EXPOSURE_PRIORITY},
};

long to_100micros(long v) {
  double res = pow(2.0, v);
  return static_cast<long>(res * 10000);
}

long from_100micros(long val) {
  double d = val * 0.0001;
  double l = (d != 0) ? std::log2(d) : 1;
  long v = static_cast<long>(std::roundl(l));
  // Exposure values use logarithmic scale and can reach -13 with D400
  assert(v <= 0 && v >= -15);
  return v;
}

bool wmf_uvc_device::get_pu(rs2_option opt, int32_t &value) const {
  long val = 0, flags = 0;
  if ((opt == RS2_OPTION_EXPOSURE) || (opt == RS2_OPTION_ENABLE_AUTO_EXPOSURE)) {
    auto hr = get_camera_control()->Get(CameraControl_Exposure, &val, &flags);
    if (hr == DEVICE_NOT_READY_ERROR)
      return false;

    value = (opt == RS2_OPTION_EXPOSURE) ? to_100micros(val) : (flags == CameraControl_Flags_Auto);
    CHECK_HR(hr);
    return true;
  }

  for (auto &pu : pu_controls) {
    if (opt == pu.option) {
      auto hr = get_video_proc()->Get(pu.property, &val, &flags);
      if (hr == DEVICE_NOT_READY_ERROR)
        return false;

      value = (pu.enable_auto) ? (flags == VideoProcAmp_Flags_Auto) : val;

      CHECK_HR(hr);
      return true;
    }
  }

  for (auto &ct : ct_controls) {
    if (opt == ct.option) {
      auto hr = get_camera_control()->Get(ct.property, &val, &flags);
      if (hr == DEVICE_NOT_READY_ERROR)
        return false;

      value = val;

      CHECK_HR(hr);
      return true;
    }
  }

  throw std::runtime_error(to_string() << "Unsupported control - " << opt);
}

#define SEMAPHORE_TIMEOUT_ERROR _HRESULT_TYPEDEF_(0x80070079L)
bool wmf_uvc_device::set_pu(rs2_option opt, int value) {
  if (opt == RS2_OPTION_EXPOSURE) {
    auto hr = get_camera_control()->Set(CameraControl_Exposure, from_100micros(value), CameraControl_Flags_Manual);
    if (hr == DEVICE_NOT_READY_ERROR)
      return false;

    CHECK_HR(hr);
    return true;
  }
  if (opt == RS2_OPTION_ENABLE_AUTO_EXPOSURE) {
    if (value) {
      auto hr = get_camera_control()->Set(CameraControl_Exposure, 0, CameraControl_Flags_Auto);
      if (hr == DEVICE_NOT_READY_ERROR)
        return false;

      CHECK_HR(hr);
    } else {
      long min, max, step, def, caps;
      auto hr = get_camera_control()->GetRange(CameraControl_Exposure, &min, &max, &step, &def, &caps);
      if (hr == DEVICE_NOT_READY_ERROR)
        return false;

      CHECK_HR(hr);

      hr = get_camera_control()->Set(CameraControl_Exposure, def, CameraControl_Flags_Manual);
      if (hr == DEVICE_NOT_READY_ERROR)
        return false;

      CHECK_HR(hr);
    }
    return true;
  }

  for (auto &pu : pu_controls) {
    if (opt == pu.option) {
      if (pu.enable_auto) {
        if (value) {
          auto hr = get_video_proc()->Set(pu.property, 0, VideoProcAmp_Flags_Auto);
          if (hr == DEVICE_NOT_READY_ERROR)
            return false;

          CHECK_HR(hr);
        } else {
          long min, max, step, def, caps;
          auto hr = get_video_proc()->GetRange(pu.property, &min, &max, &step, &def, &caps);
          if (hr == DEVICE_NOT_READY_ERROR)
            return false;

          CHECK_HR(hr);

          hr = get_video_proc()->Set(pu.property, def, VideoProcAmp_Flags_Manual);
          if (hr == DEVICE_NOT_READY_ERROR)
            return false;

          CHECK_HR(hr);
        }
      } else {
        auto hr = get_video_proc()->Set(pu.property, value, VideoProcAmp_Flags_Manual);

        // We found 2 cases when we want to return false and let the backend retry mechanism call another set command.
        // DEVICE_NOT_READY_ERROR: Can be return if the device is busy, not a real error.
        // SEMAPHORE_TIMEOUT_ERROR: We get this error at a very low statistics when setting multiple PU commands (i.e. gain command)
        // It is not expected but we decided to raise a log_debug and allow a retry on that case [DSO-17181].
        if (hr == DEVICE_NOT_READY_ERROR || hr == SEMAPHORE_TIMEOUT_ERROR) {
          if (hr == SEMAPHORE_TIMEOUT_ERROR)
            std::cout << "set_pu returned error code: "
                      << hr_to_string(hr) << std::endl;
          return false;
        }

        CHECK_HR(hr);
      }
      return true;
    }
  }
  for (auto &ct : ct_controls) {
    if (opt == ct.option) {
      if (ct.enable_auto) {
        if (value) {
          auto hr = get_camera_control()->Set(ct.property, 0, CameraControl_Flags_Auto);
          if (hr == DEVICE_NOT_READY_ERROR)
            return false;

          CHECK_HR(hr);
        } else {
          long min, max, step, def, caps;
          auto hr = get_camera_control()->GetRange(ct.property, &min, &max, &step, &def, &caps);
          if (hr == DEVICE_NOT_READY_ERROR)
            return false;

          CHECK_HR(hr);

          hr = get_camera_control()->Set(ct.property, def, CameraControl_Flags_Manual);
          if (hr == DEVICE_NOT_READY_ERROR)
            return false;

          CHECK_HR(hr);
        }
      } else {
        auto hr = get_camera_control()->Set(ct.property, value, CameraControl_Flags_Manual);
        if (hr == DEVICE_NOT_READY_ERROR)
          return false;

        CHECK_HR(hr);
      }
      return true;
    }
  }
  throw std::runtime_error(to_string() << "Unsupported control - " << opt);
}

control_range wmf_uvc_device::get_pu_range(rs2_option opt) const {
  if (opt == RS2_OPTION_ENABLE_AUTO_EXPOSURE ||
      opt == RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE) {
    static const int32_t min = 0, max = 1, step = 1, def = 1;
    control_range result(min, max, step, def);
    return result;
  }

  long minVal = 0, maxVal = 0, steppingDelta = 0, defVal = 0, capsFlag = 0;
  if (opt == RS2_OPTION_EXPOSURE) {
    CHECK_HR(get_camera_control()->GetRange(CameraControl_Exposure, &minVal, &maxVal, &steppingDelta, &defVal, &capsFlag));
    long min = to_100micros(minVal), max = to_100micros(maxVal), def = to_100micros(defVal);
    control_range result(min, max, min, def);
    return result;
  }
  for (auto &pu : pu_controls) {
    if (opt == pu.option) {
      CHECK_HR(get_video_proc()->GetRange(pu.property, &minVal, &maxVal, &steppingDelta, &defVal, &capsFlag));
      control_range result(minVal, maxVal, steppingDelta, defVal);
      return result;
    }
  }
  for (auto &ct : ct_controls) {
    if (opt == ct.option) {
      CHECK_HR(get_camera_control()->GetRange(ct.property, &minVal, &maxVal, &steppingDelta, &defVal, &capsFlag));
      control_range result(minVal, maxVal, steppingDelta, defVal);
      return result;
    }
  }
  throw std::runtime_error("unsupported control");
}

void wmf_uvc_device::foreach_uvc_device(enumeration_callback action) {
  for (auto attributes_params_set : attributes_params) {
    CComPtr<IMFAttributes> pAttributes = nullptr;
    CHECK_HR(MFCreateAttributes(&pAttributes, 1));
    for (auto attribute_params : attributes_params_set) {
      CHECK_HR(pAttributes->SetGUID(attribute_params.first, attribute_params.second));
    }

    IMFActivate **ppDevices;
    UINT32 numDevices;
    CHECK_HR(MFEnumDeviceSources(pAttributes, &ppDevices, &numDevices));

    for (UINT32 i = 0; i < numDevices; ++i) {
      CComPtr<IMFActivate> pDevice;
      *&pDevice = ppDevices[i];

      WCHAR *wchar_name = nullptr;
      UINT32 length;
      CHECK_HR(pDevice->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &wchar_name, &length));
      auto name = win_to_utf(wchar_name);
      CoTaskMemFree(wchar_name);

      uint16_t vid, pid, mi;
      std::string unique_id, guid;
      if (!parse_usb_path_multiple_interface(vid, pid, mi, unique_id, name, guid))
        continue;

      uvc_device_info info;
      info.vid = vid;
      info.pid = pid;
      info.unique_id = unique_id;
      info.mi = mi;
      info.device_path = name;
      try {
        action(info, ppDevices[i]);
      } catch (...) {
        // TODO
      }
    }
    safe_release(pAttributes);
    CoTaskMemFree(ppDevices);
  }
}

void wmf_uvc_device::set_power_state(power_state state) {
  if (state == _power_state)
    return;

  switch (state) {
  case D0:
    set_d0();
    break;
  case D3:
    set_d3();
    break;
  default:
    throw std::runtime_error("illegal power state request");
  }
}

#define did_guid MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK
#define DEVICE_ID_MAX_SIZE 256
#define WAIT_FOR_MUTEX_TIME_OUT (5000)
#define MAX_PINS 5
wmf_uvc_device::wmf_uvc_device(const uvc_device_info &info /*,
                               std::shared_ptr<const wmf_backend> backend*/
                               )
    : _streamIndex(MAX_PINS), _info(info), _is_flushed(), _has_started(), // _backend(std::move(backend)),
      _systemwide_lock(info.unique_id.c_str(), WAIT_FOR_MUTEX_TIME_OUT),
      _location(""), _device_usb_spec(usb3_type) {
  if (!is_connected(info)) {
    throw std::runtime_error("Camera not connected!");
  }
  try {
    if (!get_usb_descriptors(info.vid, info.pid, info.unique_id, _location, _device_usb_spec, _device_serial)) {
      std::cout << "Could not retrieve USB descriptor for device " << std::hex << info.vid << ":"
                << info.pid << " , id:" << info.unique_id << std::dec << std::endl;
    }
  } catch (...) {
    std::cout << "Accessing USB info failed for " << std::hex << info.vid << ":"
              << info.pid << " , id:" << info.unique_id << std::dec << std::endl;
  }
  foreach_uvc_device([this](const uvc_device_info &i, IMFActivate *device) {
    if (i == _info && device) {
      _device_id.resize(DEVICE_ID_MAX_SIZE);
      /*CHECK_HR*/ (device->GetString(did_guid, const_cast<LPWSTR>(_device_id.c_str()), UINT32(_device_id.size()), nullptr));
    }
  });
}

wmf_uvc_device::~wmf_uvc_device() {
  try {
    if (_streaming) {
      flush(MF_SOURCE_READER_ALL_STREAMS);
    }

    set_power_state(D3);

    safe_release(_device_attrs);
    safe_release(_reader_attrs);
    for (auto &&c : _ks_controls)
      safe_release(c.second);
    _ks_controls.clear();
  } catch (...) {
    std::cout << "Exception thrown while flushing MF source" << std::endl;
  }
}

#define type_guid MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
CComPtr<IMFAttributes> wmf_uvc_device::create_device_attrs() {
  CComPtr<IMFAttributes> device_attrs = nullptr;

  /*CHECK_HR*/ (MFCreateAttributes(&device_attrs, 2));
  /*CHECK_HR*/ (device_attrs->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, type_guid));
  /*CHECK_HR*/ (device_attrs->SetString(did_guid, _device_id.c_str()));
  return device_attrs;
}

CComPtr<IMFAttributes> wmf_uvc_device::create_reader_attrs() {
  CComPtr<IMFAttributes> reader_attrs = nullptr;

  /*CHECK_HR*/ (MFCreateAttributes(&reader_attrs, 3));
  /*CHECK_HR*/ (reader_attrs->SetUINT32(MF_SOURCE_READER_DISCONNECT_MEDIASOURCE_ON_SHUTDOWN, FALSE));
  /*CHECK_HR*/ (reader_attrs->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE));
  /*CHECK_HR*/ (reader_attrs->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK,
                                         static_cast<IUnknown *>(new source_reader_callback(this /*shared_from_this()*/))));
  return reader_attrs;
}

void wmf_uvc_device::set_d0() {
  if (!_device_attrs)
    _device_attrs = create_device_attrs();

  if (!_reader_attrs)
    _reader_attrs = create_reader_attrs();
  _streams.resize(_streamIndex);

  // enable source
  // CHECK_HR(MFCreateDeviceSource(_device_attrs, &_source));
  HRESULT res = MFCreateDeviceSource(_device_attrs, &_source);
  /*LOG_HR*/ (_source->QueryInterface(__uuidof(IAMCameraControl), reinterpret_cast<void **>(&_camera_control)));
  /*LOG_HR*/ (_source->QueryInterface(__uuidof(IAMVideoProcAmp), reinterpret_cast<void **>(&_video_proc)));

  // enable reader
  CHECK_HR(MFCreateSourceReaderFromMediaSource(_source, _reader_attrs, &_reader));
  CHECK_HR(_reader->SetStreamSelection(static_cast<DWORD>(MF_SOURCE_READER_ALL_STREAMS), TRUE));
  _power_state = D0;
}

void wmf_uvc_device::set_d3() {
  safe_release(_camera_control);
  safe_release(_video_proc);
  safe_release(_reader);
  _source->Shutdown(); // Failure to call Shutdown can result in memory leak
  safe_release(_source);
  for (auto &elem : _streams)
    elem.callback = nullptr;
  _power_state = D3;
}

#pragma pack(push, 1)
template <class T>
class big_endian {
  T be_value;

public:
  operator T() const {
    T le_value = 0;
    for (unsigned int i = 0; i < sizeof(T); ++i)
      *(reinterpret_cast<char *>(&le_value) + i) = *(reinterpret_cast<const char *>(&be_value) + sizeof(T) - i - 1);
    return le_value;
  }
}; // convert to standard fourcc codes
const std::unordered_map<uint32_t, uint32_t> fourcc_map = {
    {0x59382020, 0x47524559}, /* 'GREY' from 'Y8  ' */
    {0x52573130, 0x70524141}, /* 'pRAA' from 'RW10'.*/
    {0x32000000, 0x47524559}, /* 'GREY' from 'L8  ' */
    {0x50000000, 0x5a313620}, /* 'Z16'  from 'D16 ' */
    {0x52415738, 0x47524559}, /* 'GREY' from 'RAW8' */
    {0x52573136, 0x42595232}  /* 'RW16' from 'BYR2' */
};
#pragma pack(pop)
void wmf_uvc_device::foreach_profile(std::function<void(const mf_profile &profile, CComPtr<IMFMediaType> media_type, bool &quit)> action) const {
  bool quit = false;
  CComPtr<IMFMediaType> pMediaType = nullptr;
  for (unsigned int sIndex = 0; sIndex < _streams.size(); ++sIndex) {
    for (auto k = 0;; k++) {
      auto hr = _reader->GetNativeMediaType(sIndex, k, &pMediaType.p);
      if (FAILED(hr) || pMediaType == nullptr) {
        safe_release(pMediaType);
        if (hr != MF_E_NO_MORE_TYPES) // An object ran out of media types to suggest therefore the requested chain of streaming objects cannot be completed
          std::cout << "_reader->GetNativeMediaType(sIndex, k, &pMediaType.p)" << hr << std::endl;

        break;
      }

      GUID subtype;
      CHECK_HR(pMediaType->GetGUID(MF_MT_SUBTYPE, &subtype));

      unsigned width = 0;
      unsigned height = 0;

      CHECK_HR(MFGetAttributeSize(pMediaType, MF_MT_FRAME_SIZE, &width, &height));

      frame_rate frameRateMin;
      frame_rate frameRateMax;

      CHECK_HR(MFGetAttributeRatio(pMediaType, MF_MT_FRAME_RATE_RANGE_MIN, &frameRateMin.numerator, &frameRateMin.denominator));
      CHECK_HR(MFGetAttributeRatio(pMediaType, MF_MT_FRAME_RATE_RANGE_MAX, &frameRateMax.numerator, &frameRateMax.denominator));

      if (static_cast<float>(frameRateMax.numerator) / frameRateMax.denominator <
          static_cast<float>(frameRateMin.numerator) / frameRateMin.denominator) {
        std::swap(frameRateMax, frameRateMin);
      }
      int currFps = frameRateMax.numerator / frameRateMax.denominator;

      uint32_t device_fourcc = reinterpret_cast<const big_endian<uint32_t> &>(subtype.Data1);
      if (fourcc_map.count(device_fourcc))
        device_fourcc = fourcc_map.at(device_fourcc);

      stream_profile sp;
      sp.width = width;
      sp.height = height;
      sp.fps = currFps;
      sp.format = device_fourcc;

      mf_profile mfp;
      mfp.index = sIndex;
      mfp.min_rate = frameRateMin;
      mfp.max_rate = frameRateMax;
      mfp.profile = sp;

      action(mfp, pMediaType, quit);

      safe_release(pMediaType);

      if (quit)
        return;
    }
  }
}

std::vector<stream_profile> wmf_uvc_device::get_profiles() const {
  check_connection();

  if (get_power_state() != D0)
    throw std::runtime_error("Device must be powered to query supported profiles!");

  std::vector<stream_profile> results;
  foreach_profile([&results](const mf_profile &mfp, CComPtr<IMFMediaType> media_type, bool &quit) {
    results.push_back(mfp.profile);
  });

  return results;
}

#define RS2_DEFAULT_TIMEOUT 15000
void wmf_uvc_device::play_profile(stream_profile profile, frame_callback callback) {
  bool profile_found = false;
  foreach_profile([this, profile, callback, &profile_found](const mf_profile &mfp, CComPtr<IMFMediaType> media_type, bool &quit) {
    if (mfp.profile.format != profile.format &&
        (fourcc_map.count(mfp.profile.format) == 0 ||
         profile.format != fourcc_map.at(mfp.profile.format)))
      return;

    if ((mfp.profile.width == profile.width) && (mfp.profile.height == profile.height)) {
      if (mfp.max_rate.denominator && mfp.min_rate.denominator) {
        if (mfp.profile.fps == int(profile.fps)) {
          auto hr = _reader->SetCurrentMediaType(mfp.index, nullptr, media_type);
          if (SUCCEEDED(hr) && media_type) {
            for (unsigned int i = 0; i < _streams.size(); ++i) {
              if (mfp.index == i || (_streams[i].callback))
                continue;

              _reader->SetStreamSelection(i, FALSE);
            }

            CHECK_HR(_reader->SetStreamSelection(mfp.index, TRUE));

            {
              std::lock_guard<std::mutex> lock(_streams_mutex);
              if (_streams[mfp.index].callback)
                throw std::runtime_error("Camera already streaming via this stream index!");

              _streams[mfp.index].profile = profile;
              _streams[mfp.index].callback = callback;
            }

            _readsample_result = S_OK;
            CHECK_HR(_reader->ReadSample(mfp.index, 0, nullptr, nullptr, nullptr, nullptr));

            const auto timeout_ms = RS2_DEFAULT_TIMEOUT;
            if (_has_started.wait(timeout_ms)) {
              std::cout << "_reader->ReadSample(...)" << _readsample_result << std::endl;
            } else {
              std::cout << "First frame took more then " << timeout_ms << "ms to arrive!" << std::endl;
            }
            profile_found = true;
            quit = true;
            return;
          } else {
            throw std::runtime_error("Could not set Media Type. Device may be locked");
          }
        }
      }
    }
  });
  if (!profile_found)
    throw std::runtime_error("Stream profile not found!");
}

void wmf_uvc_device::probe_and_commit(stream_profile profile, frame_callback callback, int /*buffers*/) {
  if (_streaming)
    throw std::runtime_error("Device is already streaming!");

  _profiles.push_back(profile);
  _frame_callbacks.push_back(callback);
}

IAMVideoProcAmp *wmf_uvc_device::get_video_proc() const {
  if (get_power_state() != D0)
    throw std::runtime_error("Device must be powered to query video_proc!");
  if (!_video_proc.p)
    throw std::runtime_error("The device does not support adjusting the qualities of an incoming video signal, such as brightness, contrast, hue, saturation, gamma, and sharpness.");
  return _video_proc.p;
}

IAMCameraControl *wmf_uvc_device::get_camera_control() const {
  if (get_power_state() != D0)
    throw std::runtime_error("Device must be powered to query camera_control!");
  if (!_camera_control.p)
    throw std::runtime_error("The device does not support camera settings such as zoom, pan, aperture adjustment, or shutter speed.");
  return _camera_control.p;
}

void wmf_uvc_device::stream_on(std::function<void(const notification &n)> error_handler) {
  if (_profiles.empty())
    throw std::runtime_error("Stream not configured");

  if (_streaming)
    throw std::runtime_error("Device is already streaming!");

  check_connection();

  try {
    for (uint32_t i = 0; i < _profiles.size(); ++i) {
      play_profile(_profiles[i], _frame_callbacks[i]);
    }

    _streaming = true;
  } catch (...) {
    close_all();

    throw;
  }
}

void wmf_uvc_device::start_callbacks() {
  _is_started = true;
}

void wmf_uvc_device::stop_callbacks() {
  _is_started = false;
}

void wmf_uvc_device::stop_stream_cleanup(const stream_profile &profile, std::vector<profile_and_callback>::iterator &elem) {
  if (elem != _streams.end()) {
    elem->callback = nullptr;
    elem->profile.format = 0;
    elem->profile.fps = 0;
    elem->profile.width = 0;
    elem->profile.height = 0;
  }

  auto pos = std::find(_profiles.begin(), _profiles.end(), profile) - _profiles.begin();
  if (pos != _profiles.size()) {
    _profiles.erase(_profiles.begin() + pos);
    _frame_callbacks.erase(_frame_callbacks.begin() + pos);
  }

  if (_profiles.empty())
    _streaming = false;

  _has_started.reset();
}

void wmf_uvc_device::close(stream_profile profile) {
  _is_started = false;

  check_connection();

  auto &elem = std::find_if(_streams.begin(), _streams.end(),
                            [&](const profile_and_callback &pac) {
                              return (pac.profile == profile && (pac.callback));
                            });

  if (elem == _streams.end() && _frame_callbacks.empty())
    throw std::runtime_error("Camera is not streaming!");

  if (elem != _streams.end()) {
    try {
      flush(int(elem - _streams.begin()));
    } catch (...) {
      stop_stream_cleanup(profile, elem); // TODO: move to RAII
      throw;
    }
  }
  stop_stream_cleanup(profile, elem);
}

// ReSharper disable once CppMemberFunctionMayBeConst
void wmf_uvc_device::flush(int sIndex) {
  if (is_connected(_info)) {
    if (_reader != nullptr) {
      auto sts = _reader->Flush(sIndex);
      if (sts != S_OK) {
        if (sts == MF_E_HW_MFT_FAILED_START_STREAMING)
          throw std::runtime_error("Camera already streaming");

        throw std::runtime_error(to_string() << "Flush failed" << sts);
      }

      _is_flushed.wait(INFINITE);
    }
  }
}

void wmf_uvc_device::check_connection() const {
  if (!is_connected(_info))
    throw std::runtime_error("Camera is no longer connected!");
}

void wmf_uvc_device::close_all() {
  for (auto &elem : _streams)
    if (elem.callback) {
      try {
        close(elem.profile);
      } catch (...) {
      }
    }

  _profiles.clear();
  _frame_callbacks.clear();
}
/** \brief A stream's format identifies how binary data is encoded within a frame. */
typedef enum rs2_format {
  RS2_FORMAT_ANY,           /**< When passed to enable stream, librealsense will try to provide best suited format */
  RS2_FORMAT_Z16,           /**< 16-bit linear depth values. The depth is meters is equal to depth scale * pixel value. */
  RS2_FORMAT_DISPARITY16,   /**< 16-bit float-point disparity values. Depth->Disparity conversion : Disparity = Baseline*FocalLength/Depth. */
  RS2_FORMAT_XYZ32F,        /**< 32-bit floating point 3D coordinates. */
  RS2_FORMAT_YUYV,          /**< 32-bit y0, u, y1, v data for every two pixels. Similar to YUV422 but packed in a different order - https://en.wikipedia.org/wiki/YUV */
  RS2_FORMAT_RGB8,          /**< 8-bit red, green and blue channels */
  RS2_FORMAT_BGR8,          /**< 8-bit blue, green, and red channels -- suitable for OpenCV */
  RS2_FORMAT_RGBA8,         /**< 8-bit red, green and blue channels + constant alpha channel equal to FF */
  RS2_FORMAT_BGRA8,         /**< 8-bit blue, green, and red channels + constant alpha channel equal to FF */
  RS2_FORMAT_Y8,            /**< 8-bit per-pixel grayscale image */
  RS2_FORMAT_Y16,           /**< 16-bit per-pixel grayscale image */
  RS2_FORMAT_RAW10,         /**< Four 10 bits per pixel luminance values packed into a 5-byte macropixel */
  RS2_FORMAT_RAW16,         /**< 16-bit raw image */
  RS2_FORMAT_RAW8,          /**< 8-bit raw image */
  RS2_FORMAT_UYVY,          /**< Similar to the standard YUYV pixel format, but packed in a different order */
  RS2_FORMAT_NV12,          /**< i add this */
  RS2_FORMAT_MOTION_RAW,    /**< Raw data from the motion sensor */
  RS2_FORMAT_MOTION_XYZ32F, /**< Motion data packed as 3 32-bit float values, for X, Y, and Z axis */
  RS2_FORMAT_GPIO_RAW,      /**< Raw data from the external sensors hooked to one of the GPIO's */
  RS2_FORMAT_6DOF,          /**< Pose data packed as floats array, containing translation vector, rotation quaternion and prediction velocities and accelerations vectors */
  RS2_FORMAT_DISPARITY32,   /**< 32-bit float-point disparity values. Depth->Disparity conversion : Disparity = Baseline*FocalLength/Depth */
  RS2_FORMAT_Y10BPACK,      /**< 16-bit per-pixel grayscale image unpacked from 10 bits per pixel packed ([8:8:8:8:2222]) grey-scale image. The data is unpacked to LSB and padded with 6 zero bits */
  RS2_FORMAT_DISTANCE,      /**< 32-bit float-point depth distance value.  */
  RS2_FORMAT_MJPEG,         /**< Bitstream encoding for video in which an image of each frame is encoded as JPEG-DIB   */
  RS2_FORMAT_Y8I,           /**< 8-bit per pixel interleaved. 8-bit left, 8-bit right.  */
  RS2_FORMAT_Y12I,          /**< 12-bit per pixel interleaved. 12-bit left, 12-bit right. Each pixel is stored in a 24-bit word in little-endian order. */
  RS2_FORMAT_INZI,          /**< multi-planar Depth 16bit + IR 10bit.  */
  RS2_FORMAT_INVI,          /**< 8-bit IR stream.  */
  RS2_FORMAT_W10,           /**< Grey-scale image as a bit-packed array. 4 pixel data stream taking 5 bytes */
  RS2_FORMAT_Z16H,          /**< Variable-length Huffman-compressed 16-bit depth values. */
  RS2_FORMAT_FG,            /**< 16-bit per-pixel frame grabber format. */
  RS2_FORMAT_Y411,          /**< 12-bit per-pixel. */
  RS2_FORMAT_COUNT          /**< Number of enumeration values. Not a valid input: intended to be used in for-loops. */
} rs2_format;
template <typename T>
uint32_t rs_fourcc(const T a, const T b, const T c, const T d) {
  static_assert((std::is_integral<T>::value), "rs_fourcc supports integral built-in types only");
  return ((static_cast<uint32_t>(a) << 24) |
          (static_cast<uint32_t>(b) << 16) |
          (static_cast<uint32_t>(c) << 8) |
          (static_cast<uint32_t>(d) << 0));
}
const std::map<uint32_t, rs2_format> platform_color_fourcc_to_rs2_format = {
    {rs_fourcc('Y', 'U', 'Y', '2'), RS2_FORMAT_YUYV},
    {rs_fourcc('Y', 'U', 'Y', 'V'), RS2_FORMAT_YUYV},
    {rs_fourcc('N', 'V', '1', '2'), RS2_FORMAT_NV12},
    {rs_fourcc('M', 'J', 'P', 'G'), RS2_FORMAT_MJPEG},
};
#define UNKNOWN_VALUE "UNKNOWN"
const char *get_string(rs2_format value) {
#define CASE(X)        \
  case RS2_FORMAT_##X: \
    return #X;
  switch (value) {
    CASE(ANY)
    CASE(Z16)
    CASE(DISPARITY16)
    CASE(DISPARITY32)
    CASE(XYZ32F)
    CASE(YUYV)
    CASE(RGB8)
    CASE(BGR8)
    CASE(RGBA8)
    CASE(BGRA8)
    CASE(Y8)
    CASE(Y16)
    CASE(RAW10)
    CASE(RAW16)
    CASE(RAW8)
    CASE(UYVY)
    CASE(MOTION_RAW)
    CASE(MOTION_XYZ32F)
    CASE(GPIO_RAW)
    CASE(6DOF)
    CASE(Y10BPACK)
    CASE(DISTANCE)
    CASE(MJPEG)
    CASE(Y8I)
    CASE(Y12I)
    CASE(INZI)
    CASE(INVI)
    CASE(W10)
    CASE(Z16H)
    CASE(FG)
    CASE(Y411)
    CASE(NV12)
  default:
    // assert(!is_valid(value));
    return UNKNOWN_VALUE;
  }
#undef CASE
}

static std::vector<extension_unit> _xus;

/*
VideoControl Interface Descriptor:
bLength                26
bDescriptorType        36
bDescriptorSubtype      6 (EXTENSION_UNIT)

bUnitID                 6
guidExtensionCode         {41769ea2-04de-e347-8b2b-f4341aff003b}
bNumControl             3
bNrPins                 1

baSourceID( 0)          2
bControlSize            1
bmControls( 0)       0x07
iExtension              0
*/
// subdevice[h] unit[fw], node[h] guid[fw]
const extension_unit my_xu = {0, 6, 3, {0x41769ea2, 0x04de, 0xe347, {0x8b, 0x2b, 0xf4, 0x34, 0x1a, 0xff, 0x00, 0x3b}}};
/*
HRESULT FindExtensionNode(IKsTopologyInfo *pKsTopologyInfo, GUID guid, DWORD *node)
{
HRESULT hr = E_FAIL;
DWORD dwNumNodes = 0;
GUID guidNodeType;
IKsControl *pKsControl = NULL;
ULONG ulBytesReturned = 0;
KSP_NODE ExtensionProp;
if (!pKsTopologyInfo || !node)
return E_POINTER;
// Retrieve the number of nodes in the filter
hr = pKsTopologyInfo->get_NumNodes(&dwNumNodes);
if (!SUCCEEDED(hr))
return hr;
if (dwNumNodes == 0)
return E_FAIL;
// Find the extension unit node that corresponds to the given GUID
for (unsigned int i = 0; i < dwNumNodes; i++)
{
hr = E_FAIL;
pKsTopologyInfo->get_NodeType(i, &guidNodeType);
if (IsEqualGUID(guidNodeType, KSNODETYPE_DEV_SPECIFIC))
{
printf("found one xu node\n");
IExtensionUnit*   pExtensionUnit = NULL;
hr = pKsTopologyInfo->CreateNodeInstance(i, __uuidof(IExtensionUnit), (void **)&pExtensionUnit);
if (SUCCEEDED(hr))
{
ExtensionProp.Property.Set = guid;
ExtensionProp.Property.Id = 0;
ExtensionProp.Property.Flags = KSPROPERTY_TYPE_SETSUPPORT | KSPROPERTY_TYPE_TOPOLOGY;
ExtensionProp.NodeId = i;
ExtensionProp.Reserved = 0;
*node = i;
return hr;

//hr = pKsControl->KsProperty((PKSPROPERTY)&ExtensionProp, sizeof(ExtensionProp), NULL, 0, &ulBytesReturned);
//if(SUCCEEDED(hr))
//{
//*node = i;
//break;
//}

}
else
{
printf("CreateNodeInstance failed - 0x%x\n", hr);
}
}
}
return hr;
}*/
// 嵌入到现有的sdk框架中去
// 1.枚举接口是否要修改？
// 2.读写接口需要搞一下
/*
void mfuvc_init() {
#ifdef COM_MULTITHREADED
  CoInitializeEx(nullptr, COINIT_MULTITHREADED); // when using COINIT_APARTMENTTHREADED, calling _pISensor->SetEventSink(NULL) to stop sensor can take several seconds
#else
  CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED); // Apartment model
#endif

  MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET);
  //printf("hello,world!\n");

  _xus.push_back(std::move(my_xu));
}
*/
int mfuvc_transfer(wmf_uvc_device *dev, unsigned char *data, int len) {
  // 每次发送60字节
  wmf_uvc_device *tmpdev = dev;
  int offset = 0;
  int tmplen = 0;
  int index = 0;
  unsigned char *tmpbuf = (unsigned char *)calloc(1, 64);
  while (offset < len) {
    if ((len - offset) >= 58) {
      tmplen = 58;
    } else {
      tmplen = len - offset;
    }
    memset(tmpbuf, 0, 60);
    *tmpbuf = tmplen + 2;
    *tmpbuf |= ((((len - offset) <= 58) ? 1 : 0) << 6);
    *(tmpbuf + 1) = index;
    memcpy(tmpbuf + 2, data + offset, tmplen);
    /*
    printf("will send index:%d, pkg size:%d bytes",index, tmplen + 2);
    printf("\n--------------------\n");
    for(int i = 0; i < tmplen + 2; i++)
    {
    printf("%lu ", tmpbuf[i]);
    }
    printf("\n--------------------\n");
    */
    if (tmpdev->set_xu(my_xu, 1, tmpbuf, 60)) {
      offset += tmplen;
      index++;
      // printf("ok to send %d bytes\n",tmplen);
    } else {
      // printf("fail to send %d bytes\n", tmplen);
      break;
    }
  }
  if (offset < len) {
    return -1;
  } else
    return 0;
}
void mfuvc_start_stream(STREAMCALLBACK framecallback, wmf_uvc_device * /*std::shared_ptr<wmf_uvc_device>*/ wmfdevice, void *userdata) {
  wmf_uvc_device * /*std::shared_ptr<wmf_uvc_device>*/ dev = wmfdevice;
  dev->set_power_state(D0);
  // for (auto && xu : _xus)
  dev->init_xu(my_xu);

  // dev->set_power_state(D3);

  auto profiles = dev->get_profiles();
  for (int i = 0; i < profiles.size(); i++) {
    printf("support:%dx%d,%d,%s\n", profiles[i].width, profiles[i].height, profiles[i].fps, get_string(platform_color_fourcc_to_rs2_format.at(profiles[i].format)));
    if (profiles[i].width == 1920 &&
        profiles[i].height == 1080 &&
        profiles[i].fps == 60 && 0 == strcmp("MJPEG", get_string(platform_color_fourcc_to_rs2_format.at(profiles[i].format)))) {
      dev->probe_and_commit(
          profiles[i], [framecallback, userdata](stream_profile p, frame_object f, std::function<void()> continuation) mutable {
            static unsigned int frameindex = 0;
            //	printf("got frame:\nsize:%llu\nmetadata_size:%u\npixels:0x%X\n,metadata:0x%X\n,backend_time:%f,%u!!!!\n",
            //		f.frame_size,f.metadata_size,f.pixels,f.metadata,f.backend_time,frameindex++);
            framecallback(f.pixels, f.frame_size, userdata);
          },
          DEFAULT_V4L2_FRAME_BUFFERS);
      dev->start_callbacks();
      dev->stream_on([&](const notification &n) {
        printf("stream_on notification!\n");
      });
      break;
    }
  }
}

wmf_uvc_device *get_uvc_device_by_vidpid_path(uint16_t vid, uint16_t pid, const char *path)
// std::vector<uvc_device_info> query_uvc_devices()
{
  std::vector<uvc_device_info> devices;

  auto action = [&devices](const uvc_device_info &info, IMFActivate *) {
    uvc_device_info device_info = info;
    device_info.serial = get_device_serial(info.vid, info.pid, info.unique_id);
    devices.push_back(device_info);
  };

  foreach_uvc_device(action);

  for (int i = 0; i < devices.size(); i++) {
    if (devices[i].vid == vid && devices[i].pid == pid && 0 == strcmp(path, devices[i].device_path.c_str())) {
      return new wmf_uvc_device(devices[i]);
    }
  }
  return NULL;
}
void nvptl_mfuvc_init() {
#ifdef COM_MULTITHREADED
  CoInitializeEx(nullptr, COINIT_MULTITHREADED); // when using COINIT_APARTMENTTHREADED, calling _pISensor->SetEventSink(NULL) to stop sensor can take several seconds
#else
  CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED); // Apartment model
#endif

  MFStartup(MF_VERSION, MFSTARTUP_NOSOCKET);
  // printf("hello,world!\n");

  //_xus.push_back(std::move(my_xu));
}

void nvptl_mfuvc_deinit() {
  CoUninitialize();
}
static void devicecallback(uvc_device_info *pinfo, void *userdata) {
  NVPTL_DEVICE_INFO **pptmpdevice = (NVPTL_DEVICE_INFO **)userdata;
  // printf("got falcon device!\n");
  NVPTL_DEVICE_INFO *thedevice = (NVPTL_DEVICE_INFO *)calloc(1, sizeof(NVPTL_DEVICE_INFO));
  thedevice->next = *pptmpdevice;
  sprintf((char *)thedevice->usb_camera_name, "%s", pinfo->device_path.c_str());
  thedevice->type = NVPTL_UVC_INTERFACE;
  *pptmpdevice = thedevice;
}

NVPTL_RESULT nvptl_mfuvc_enum(int *ptotal, NVPTL_DEVICE_INFO **ppdevices) {
  int countdevices = 0;
  /*static*/ NVPTL_DEVICE_INFO *tmpdevice = NULL;
  /*if (NULL != tmpdevice)
  {
    nvptl_freedevices(tmpdevice);
    tmpdevice = NULL;
  }*/

  query_uvc_devices(devicecallback, NVPTL_VID, NVPTL_PID, (void *)&tmpdevice);

  if (tmpdevice != NULL) {
    NVPTL_DEVICE_INFO *justdevice = tmpdevice;
    while (justdevice != NULL) {
      countdevices++;
      justdevice = justdevice->next;
    }
    *ptotal = countdevices;
    *ppdevices = tmpdevice;
    return NVPTL_OK;
  } else
    return NVPTL_FAILED;
}

static void nvptl_recv_frame_callback(NVPTL_DEVICE_HANDLE handle, uint8_t *data, unsigned long len, void *userdata) {
  //	printf("recv whole frame from mfuvc!\n");
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  if (inst->recvframecallback != NULL) {
    inst->recvframecallback(handle, data, len, userdata);
  }
}
void nvptl_mfuvc_set_eventcallback(NVPTL_DEVICE_HANDLE handle, EVENTCALLBACK eventcallback, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  wmf_uvc_device *mfuvcdev = (wmf_uvc_device *)inst->device_usb_handle;
  mfuvcdev->eventcallback = eventcallback;
  mfuvcdev->connectuserdata = userdata;
}
static NVPTL_RESULT usb_hal_open(NVPTL_INSTANCE *inst, int bus, char *path) {
  (void)bus;
  int countdevices = 0;
  (void)countdevices;
  static NVPTL_DEVICE_INFO *tmpdevice = NULL;
  if (NULL != tmpdevice) {
    nvptl_freedevices(tmpdevice);
    tmpdevice = NULL;
  }

  auto devinfo = get_uvc_device_by_vidpid_path(NVPTL_VID, NVPTL_PID, path);

  if (devinfo != NULL) {
    inst->device_usb_handle = (void *)devinfo;

    return NVPTL_OK;
  } else
    return NVPTL_FAILED;
}

static void loop_recv_callback(const void *pixels, size_t framesize, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)userdata;
  // FRAMECALLBACK callback = (FRAMECALLBACK)inst->framecallback;
  // depthtool* toolobj = (depthtool*)pParam;
  if (inst->usb_buf == NULL)
    inst->usb_buf = (uint8_t *)malloc(sizeof(uint8_t) * USB_PACKET_MAX_SIZE);
  // printf("after malloc usb_buf!\n");
  //	static unsigned char *tmpbuf = NULL;
  if (NULL == inst->tmpbuf)
    inst->tmpbuf = (unsigned char *)malloc(USB_PACKET_MAX_SIZE);
  // printf("after malloc tmpbuf!\n");

  if (inst->devinfo.type == NVPTL_UVC_INTERFACE) {
    // on_usb_data_receive(inst, (uint8_t*)pixels, framesize);
    size_t bytesTransffered = framesize;
    static unsigned long index = 0;
    static unsigned long currentpacketlen = 0;
    if (((long)bytesTransffered >= (long)sizeof(NVPTL_USBHeaderDataPacket)) && (index == 0) && (0 == strncmp("NEXT_VPU", (const char *)pixels, 8))) { //
      memcpy(inst->usb_buf, pixels, bytesTransffered);
      index += bytesTransffered;
      currentpacketlen = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->len;
      int type = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->type;
      int sub_type = ((NVPTL_USBHeaderDataPacket *)inst->usb_buf)->sub_type;
      if (type == 11)
        printf("-----type:%d sub_type:%d-----\n", type, sub_type);
      if (index == (currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
        nvptl_recv_frame_callback(inst, inst->usb_buf, index, inst->userdata);
        index = 0;
        currentpacketlen = 0;
      }
    } else if (index >= sizeof(NVPTL_USBHeaderDataPacket)) {
      if ((index + bytesTransffered) <= USB_PACKET_MAX_SIZE) {
        memcpy(inst->usb_buf + index, pixels, bytesTransffered);
        index += bytesTransffered;
        if (index == (currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
          nvptl_recv_frame_callback(inst, inst->usb_buf, index, inst->userdata);
          index = 0;
          currentpacketlen = 0;
        } else if (index > (currentpacketlen + sizeof(NVPTL_USBHeaderDataPacket))) {
          printf("fail data!!!\n");
          index = 0;
          currentpacketlen = 0;
        }
      } else {
        printf("false data!\n");
        index = 0;
        currentpacketlen = 0;
      }
    } else {
      printf("false data,give up....\n");
      index = 0;
      currentpacketlen = 0;
    }
    return;
  }
}
NVPTL_DEVICE_HANDLE nvptl_mfuvc_open(NVPTL_DEVICE_INFO *dev_info, NVPTL_RECV_FRAME_CALLBACK callback, void *userdata) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)calloc(1, sizeof(NVPTL_INSTANCE));
  inst->devinfo = *dev_info;
  if (usb_hal_open(inst, 0, dev_info->usb_camera_name) == NVPTL_OK) {
    inst->eventcallback = NULL;
    inst->connectuserdata = NULL;

    inst->recvframecallback = callback;
    inst->status = STARTED;

    inst->userdata = userdata;
    mfuvc_start_stream(loop_recv_callback, (wmf_uvc_device *)inst->device_usb_handle, inst);
    nvptl_debug_printf("connect ok!!!!\n");
    return inst;
  }
  free(inst);
  return NULL;
}

void nvptl_mfuvc_close(NVPTL_DEVICE_HANDLE handle) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;

  inst->status = STOPPED;

  wmf_uvc_device *tmpdev = (wmf_uvc_device *)inst->device_usb_handle;
  delete tmpdev;

  if (inst->tmpbuf != NULL) {
    free(inst->tmpbuf);
    inst->tmpbuf = NULL;
  }
  if (inst->usb_buf != NULL) {
    free(inst->usb_buf);
    inst->usb_buf = NULL;
  }

  free(inst);
}
int nvptl_mfuvc_send(NVPTL_DEVICE_HANDLE handle, unsigned char *sendBuffer, size_t len) {
  NVPTL_INSTANCE *inst = (NVPTL_INSTANCE *)handle;
  mfuvc_transfer((wmf_uvc_device *)inst->device_usb_handle, (unsigned char *)sendBuffer, len);
  return 0;
}