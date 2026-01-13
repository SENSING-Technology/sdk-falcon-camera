from sympy import fps, true
import falcon_sdk
import os
import time
import cv2
import numpy as np
import multiprocessing
import cProfile
import json

import time

dct = {
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_1280_1080: "IMAGE_1280_1080",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_1280_480: "IMAGE_1280_720",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_1280_720: "IMAGE_1280_720",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_1280_800: "IMAGE_1280_800",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_1280_960: "IMAGE_1280_960",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_1600_1200: "IMAGE_1600_1200",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_1920_1080: "IMAGE_1920_1080",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_320_200: "IMAGE_320_200",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_320_240: "IMAGE_320_240",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_480_300: "IMAGE_480_300",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_480_640: "IMAGE_480_640",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_640_360: "IMAGE_640_360",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_640_400: "IMAGE_640_400",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_640_480: "IMAGE_640_480",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_768_480: "IMAGE_768_480",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_800_600: "IMAGE_800_600",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_848_480: "IMAGE_848_480",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_960_1280: "IMAGE_960_1280",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_960_600: "IMAGE_960_600",
    falcon_sdk.py_nvpfm.NVPFM_IMAGE_SIZE.IMAGE_UNKNOWN: "IMAGE_UNKNOWN",
}


def ImageSzTrans(isz):
    # return dct[isz]
    return str(isz)


def RESCOMBO2dict(res: falcon_sdk.RESCOMBO):
    return {
        "valid": res.valid,
        "sensorres": ImageSzTrans(res.sensorres),
        "ispres": ImageSzTrans(res.ispres),
        "fps": res.fps,
    }


def dev2dict(dev_info: falcon_sdk.s_nvpfm_dev_info):
    return {
        "sn": dev_info.sn,
        "product": dev_info.product,
        "cpu_net_type": dev_info.cpu_net_type,
        "comm_type": dev_info.comm_type,
        "projector_type": dev_info.projector_type,
        "imu_type0": dev_info.imu_type0,
        "imu_type1": dev_info.imu_type1,
        "software_version": dev_info.software_version,
        "cpu_temperature": dev_info.cpu_temperature,
        "projector_temperature": dev_info.projector_temperature,
        "sensor": dev_info.sensor,
        "depth": dev_info.depth,
        "depth0": dev_info.depth0,
        "depth1": dev_info.depth1,
        "rgb": dev_info.rgb,
        "rgb0": dev_info.rgb0,
        "rgb1": dev_info.rgb1,
        "irsupported": RESCOMBO2dict(dev_info.irsupported),
        "colorsupported": RESCOMBO2dict(dev_info.colorsupported),
    }


def falcon_loop(data_q):
    # data_q = multiprocessing.Queue()
    nvpfm = falcon_sdk.py_nvpfm()
    # dev thread and enum thread will run
    nvpfm.init_and_start("./FC1.log", 50 * 1024 * 1024)
    print("Go go go!")
    camera_type = nvpfm.get_dev_info()
    print("camera type:", camera_type)
    while True:
        data_list = nvpfm.read_data()
        if len(data_list) > 0:
            for data in data_list:
                data_q.put(data)


def main_loop(data_q):
    # data_q = multiprocessing.Queue()
    while True:
        data = data_q.get(True)
        height = int(data.frame_height * 3 / 2)
        img = np.array(data.frame_data, dtype=np.uint8)
        frame = img
        frame.shape = (height, data.frame_width, 1)
        frame = cv2.cvtColor(img, cv2.COLOR_YUV2RGBA_NV12)
        cv2.imshow(data.dev_name + " rgb frame", frame)
        cv2.waitKey(33)


fps_time_count = 0

def show_align_image(data):
    global fps_time_count
    width = data.frame_width
    height = data.frame_height
    
    if fps_time_count % 120 == 0:
        print("frame shape:", data.frame_data_array.shape, "width:{}, height:{}".format(data.frame_width, data.frame_height))
    print("data type:",data.frame_data_array)
    img = np.frombuffer(data.frame_data_array.tobytes(), dtype='<H')
    img = img.reshape(height,width)

    cv2.imshow(data.dev_name + " align frame", img)
    if fps_time_count % 60 == 0:
        print("align fps:", data.fps)


def show_rgb_image(data):
    global fps_time_count
    width = data.frame_width
    height = data.frame_height
    tmpheight = int(height * 3 / 2)
    # 1 ms
    # a = np.array(3, dtype=np.uint8)
    if fps_time_count % 120 == 0:
        print("frame shape:", data.frame_data_array.shape, "width:{}, height:{}".format(data.frame_width, data.frame_height))
    img = data.frame_data_array
    img.shape = (tmpheight, width, 1)
    img = cv2.cvtColor(img, cv2.COLOR_YUV2BGRA_NV12)
    cv2.imshow(data.dev_name + " rgb frame", img)
    if fps_time_count % 60 == 0:
        print("rgb fps:", data.fps)


def show_ir_image(data, frame_str: str):
    img = data.frame_data_array
    img.shape = (data.frame_height, data.frame_width, 1)
    # img = cv2.cvtColor(img, cv2.COLOR)
    cv2.imshow(data.dev_name + (" %s frame" % frame_str), img)
    if fps_time_count % 60 == 0:
        print("%s fps:" % (frame_str), data.fps)


def show_pseudo_image(data):
    width = data.frame_width
    height = data.frame_height
    img = data.frame_data_array
    img.shape = (height, width, 3)
    cv2.imshow(data.dev_name + " pseudo frame", img)
    if fps_time_count % 60 == 0:
        print("presudo fps:", data.fps)

def show_alignpseudo_image(data):
    width = data.frame_width
    height = data.frame_height
    img = data.frame_data_array
    img.shape = (height, width, 3)
    cv2.imshow(data.dev_name + " depthalignrgb pseudo frame", img)
    if fps_time_count % 60 == 0:
        print("align presudo fps:", data.fps)

def toggle_event(nvpfm: falcon_sdk.py_nvpfm, mode):
    mode_en = not (nvpfm.get_event_enabled(mode))
    nvpfm.set_event_enabled(mode, mode_en)


def add_imu_unit(imu_data):
    dct = dict()
    dct["fx"] = imu_data.fx
    dct["fy"] = imu_data.fy
    dct["fz"] = imu_data.fz
    return dct


def print_imu(data):
    if fps_time_count % 60:
        return
    d_imu = dict()
    # data = falcon_sdk.event_data()
    d_imu["channel"] = data.imu_channel
    d_imu["data_number"] = data.imu_data_number
    d_imu["data_type"] = data.imu_data_type
    d_imu["is_factory"] = data.imu_is_factory

    if data.imu_is_factory:
        d_imu["factory_data"] = dict()
        d_imu["factory_data"]["accel_cali_data"] = add_imu_unit(
            data.imu_factory_data.accel_cali_data
        )
        d_imu["factory_data"]["gyro_cali_data"] = add_imu_unit(
            data.imu_factory_data.gyro_cali_data
        )
        d_imu["factory_data"]["mag_cali_data"] = add_imu_unit(
            data.imu_factory_data.mag_cali_data
        )
        d_imu["factory_data"]["tem_cali_data"] = data.imu_factory_data.tem_cali_data

        d_imu["factory_data"]["timestamp"] = data.imu_factory_data.timestamp
    else:
        d_imu["app_data"] = dict()
        d_imu["app_data"]["accel_cali_data"] = add_imu_unit(
            data.imu_application_data.accel_cali_data
        )
        d_imu["app_data"]["gyro_cali_data"] = add_imu_unit(
            data.imu_application_data.gyro_cali_data
        )
        d_imu["app_data"]["mag_cali_data"] = add_imu_unit(
            data.imu_application_data.mag_cali_data
        )
        d_imu["app_data"]["tem_cali_data"] = data.imu_application_data.tem_cali_data

        d_imu["app_data"]["timestamp"] = data.imu_application_data.timestamp
    imu_str_content = json.dumps(d_imu, indent=4)
    print("imu============================>\n")
    print(imu_str_content)
    print("imu============================>\n")


def py_get_dev_info(nvpfm: falcon_sdk.py_nvpfm):
    infos = nvpfm.get_dev_info()
    if len(infos) == 0:
        print("Failed to get device info.")
        return
    # return
    for info in infos:
        print("device info ==============================>\n")
        js = json.dumps(dev2dict(info), indent=4)
        print(js)
        print("device info ==============================>\n")


if __name__ == "__main__":
    """
    queue = multiprocessing.Queue()
    pw = multiprocessing.Process(target=falcon_loop, args=(queue,))
    pw.start()
    main_loop(queue)
    pw.join()
    """

    nvpfm = falcon_sdk.py_nvpfm()
    # dev thread and enum thread will run
    nvpfm.init_and_start("./FC1.log", 50 * 1024 * 1024)
    print("Go go go!")
    camera_info= nvpfm.get_dev_info()
    if len(camera_info) > 0:
        print("camera info:",camera_info)
    else:
        print("fail to get dev info!\n")
    data_list = list()
    cam_param_list = list()
    last_show_time = -1
    # enable all events.
    nvpfm.set_event_enabled(nvpfm.ev_type.RGB, True)
    nvpfm.set_event_enabled(nvpfm.ev_type.LeftIR, True)
    nvpfm.set_event_enabled(nvpfm.ev_type.RightIR, True)
    nvpfm.set_event_enabled(nvpfm.ev_type.DepthPseudo, True)
    nvpfm.set_event_enabled(nvpfm.ev_type.DepthAlignRGB, True)
    nvpfm.set_event_enabled(nvpfm.ev_type.DepthAlignRGBPseudo, True)
    nvpfm.set_event_enabled(nvpfm.ev_type.Depth, True)
    nvpfm.set_event_enabled(nvpfm.ev_type.IMU, True)
    while True:
        # camera_type=nvpfm.get_camera_type()
        # print("camera type:",camera_type)
        # 3ms
        # cProfile.runctx("data_list = nvpfm.read_data()", globals(), locals())
        # py_get_dev_info(nvpfm)
        if fps_time_count % 120 == 0:
            print("before read_cam_param!")
            cam_param_list = nvpfm.read_cam_param()
            print("after read_cam_param!")
            cam_param_len = len(cam_param_list)
            print("cam_param len:",cam_param_len)
            if cam_param_len > 0 and cam_param_list[0].result == 0:
                print("left fx,fy,px,py:",cam_param_list[0].left_ir_focus[0],cam_param_list[0].left_ir_focus[1],cam_param_list[0].left_ir_photocenter[0],cam_param_list[0].left_ir_photocenter[1])
                print("right fx,fy,px,py:",cam_param_list[0].right_ir_focus[0],cam_param_list[0].right_ir_focus[1],cam_param_list[0].right_ir_photocenter[0],cam_param_list[0].right_ir_photocenter[1])
                print("color fx,fy,px,py:",cam_param_list[0].color_focus[0],cam_param_list[0].color_focus[1],cam_param_list[0].color_photocenter[0],cam_param_list[0].color_photocenter[1])
                print("left2right:")
                print(cam_param_list[0].left2right_matrix[0],cam_param_list[0].left2right_matrix[1],cam_param_list[0].left2right_matrix[2],cam_param_list[0].left2right_matrix[9])
                print(cam_param_list[0].left2right_matrix[3],cam_param_list[0].left2right_matrix[4],cam_param_list[0].left2right_matrix[5],cam_param_list[0].left2right_matrix[10])
                print(cam_param_list[0].left2right_matrix[7],cam_param_list[0].left2right_matrix[7],cam_param_list[0].left2right_matrix[8],cam_param_list[0].left2right_matrix[11])
                print(0,0,0,1)
                print("left2color:")
                print(cam_param_list[0].left2color_matrix[0],cam_param_list[0].left2color_matrix[1],cam_param_list[0].left2color_matrix[2],cam_param_list[0].left2color_matrix[9])
                print(cam_param_list[0].left2color_matrix[3],cam_param_list[0].left2color_matrix[4],cam_param_list[0].left2color_matrix[5],cam_param_list[0].left2color_matrix[10])
                print(cam_param_list[0].left2color_matrix[7],cam_param_list[0].left2color_matrix[7],cam_param_list[0].left2color_matrix[8],cam_param_list[0].left2color_matrix[11])
                print(0,0,0,1)
                

        data_list = nvpfm.read_data()
        d_len = len(data_list)
        # tbd 移出来，动态的调整各模式
        # cv2.waitKey(33)
        # 帧率控制? --- 两张图不是一起画的
        code = cv2.waitKey(4)
        if code == 27:
            nvpfm.stop()
            print("Program quit!")
            exit(0)
        elif code == ord("1"):
            toggle_event(nvpfm, nvpfm.ev_type.RGB)
        elif code == ord("2"):
            toggle_event(nvpfm, nvpfm.ev_type.LeftIR)
        elif code == ord("3"):
            toggle_event(nvpfm, nvpfm.ev_type.RightIR)
        elif code == ord("4"):
            toggle_event(nvpfm, nvpfm.ev_type.DepthPseudo)
        elif code == ord("5"):
            toggle_event(nvpfm, nvpfm.ev_type.Depth)
        elif code == ord("6"):
            toggle_event(nvpfm, nvpfm.ev_type.IMU)
        fps_time_count += 1
        for data in data_list:
            if data.event_type == nvpfm.ev_type.RGB:
                show_rgb_image(data)
            elif data.event_type == nvpfm.ev_type.LeftIR:
                # 左右目帧率很低
                show_ir_image(data, "left ir")
            elif data.event_type == nvpfm.ev_type.RightIR:
                show_ir_image(data, "right ir")
            elif data.event_type == nvpfm.ev_type.DepthPseudo:
                show_pseudo_image(data)
            elif data.event_type == nvpfm.ev_type.DepthAlignRGB:
                show_align_image(data)
            elif data.event_type == nvpfm.ev_type.DepthAlignRGBPseudo:
                show_alignpseudo_image(data)
            elif data.event_type == nvpfm.ev_type.IMU:
                print_imu(data)
            else:
                continue
        if fps_time_count % 120 == 0:
            py_get_dev_info(nvpfm)
