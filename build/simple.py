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
    nvpfm = falcon_sdk.py_nvpfm()
    # dev thread and enum thread will run
    nvpfm.init_and_start("./FC1.log", 50 * 1024 * 1024)
    print("Go go go!")
    while True:
        py_get_dev_info(nvpfm)
        time.sleep(1)
