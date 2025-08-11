# -*- coding: utf-8 -*-
"""
本模块提供了feyman深度相机的python使用接口.用户需实例化本模块内的py_nvpfm对象,并将其初始化,再循环调用read\\_data接口获取数据。

典型用法是
::

    1. 导入SDK import falcon_sdk
    2. 构造SDK Instance nvpfm = falcon_sdk.py_nvpfm()
    3. 使能各数据事件 nvpfm.set_event_enabled(the_event, True) 
    4. 初始化 nvpfm.init_and_start()
    5. 循环分析数据 data_list = nvpfm.read_data()
    6. 终止sdk nvpfm.stop()

使用注意事项
::

    1. 一个设备上只能同时构造一个nvpfm 实例
    2. 在构造新的nvpfm之前, 必须要将旧的实例stop, 最好解除旧的实例引用, 使其销毁

"""
from encodings import utf_8
from typing import Any, ClassVar

import numpy


class py_nvpfm:
    class NVPFM_IMAGE_SIZE:
        pass

    class ev_type:
        pass

    pass


class IMU_APPLICATION_DATA:
    """
    IMU 传感器应用数据定义
    """

    def __init__(self) -> None:
        ...

    @property
    def fx(self) -> float:
        """
        X轴数据
        """
        ...

    @property
    def fy(self) -> float:
        """
        Y轴数据
        """
        ...

    @property
    def fz(self) -> float:
        """
        Z轴数据
        """
        ...


class IMU_APPLICATION_DATA_STRUC:
    """
    FeymanDevice IMU 的应用模式数据定义
    """

    def __init__(self) -> None:
        ...

    @property
    def accel_cali_data(self) -> IMU_APPLICATION_DATA:
        """
        加速度计标定数据
        """
        ...

    @property
    def gyro_cali_data(self) -> IMU_APPLICATION_DATA:
        """
        角速度计标定数据
        """
        ...

    @property
    def mag_cali_data(self) -> IMU_APPLICATION_DATA:
        """
        地磁计标定数据
        """
        ...

    @property
    def tem_cali_data(self) -> float:
        """
        温度计标定数据
        """
        ...

    @property
    def timestamp(self) -> int:
        """
        本帧时间戳
        """
        ...


class IMU_FACTORY_DATA:
    def __init__(self) -> None:
        """
        IMU 传感器的工厂数据定义
        """
        ...

    @property
    def fx(self) -> int:
        """
        X轴数据
        """
        ...

    @property
    def fy(self) -> int:
        """
        Y轴数据
        """
        ...

    @property
    def fz(self) -> int:
        """
        Z轴数据
        """
        ...


class IMU_FACTORY_DATA_STRUC:
    """
    FeymanDevice IMU 的工厂模式数据定义
    """

    def __init__(self) -> None:
        ...

    @property
    def accel_cali_data(self) -> IMU_FACTORY_DATA:
        """
        加速度计标定数据
        """
        ...

    @property
    def gyro_cali_data(self) -> IMU_FACTORY_DATA:
        """
        角速度计标定数据
        """
        ...

    @property
    def mag_cali_data(self) -> IMU_FACTORY_DATA:
        """
        地磁计标定数据
        """
        ...

    @property
    def tem_cali_data(self) -> float:
        """
        温度计标定数据
        """
        ...

    @property
    def timestamp(self) -> int:
        """
        本帧时间戳
        """
        ...


class RESCOMBO:
    """
    Sensor 能力描述
    """

    def __init__(self) -> None:
        ...

    @property
    def fps(self) -> int:
        """
        帧率
        """
        ...

    @property
    def ispres(self) -> py_nvpfm.NVPFM_IMAGE_SIZE:
        """
        isp 输出分辨率
        """
        ...

    @property
    def sensorres(self) -> py_nvpfm.NVPFM_IMAGE_SIZE:
        """
        sensor 原始分辨率
        """
        ...

    @property
    def valid(self) -> int:
        """
        内部使用
        """
        ...


class event_data:
    """
    Python和SDK数据交互使用的数据结构

    event_data可以分为两类数据: IMU类数据和frame类数据. 对于IMU数据而言,所有以"imu"为前缀的数据成员是有效的；
    对于frame数据而言,frame_data_array 数据是有效的.这两类数据不会在一个event_data实例中传递.

    注意：
    ::

        IMU类型数据是指事件类型为ev_type.IMU的数据
        frame类型数据是指除了类型为IMU类数据之外的数据


    """

    def __init__(self) -> None:
        ...

    @property
    def dev_name(self) -> str:
        """
        设备名称
        """
        ...

    @property
    def event_type(self) -> Any:
        """
        事件类型
        """
        ...

    @property
    def fps(self) -> float:
        """
        帧率,所有事件类型有效
        """
        ...

    @property
    def frame_data_array(self) -> numpy.ndarray:
        """
        图像数据,仅frame类型数据有效
        """
        ...

    @property
    def frame_height(self) -> int:
        """
        帧高度
        """
        ...

    @property
    def frame_width(self) -> int:
        """
        帧宽度
        """
        ...

    @property
    def imu_application_data(self) -> IMU_APPLICATION_DATA_STRUC:
        """
        imu应用类型数据,仅当设备处于应用模式才会传递此数据.

        若SDK未传递此数据,则该成员为None
        """
        ...

    @property
    def imu_channel(self) -> int:
        """
        imu通道
        """
        ...

    @property
    def imu_data_number(self) -> int:
        """
        本帧数据内包含多少个imu数据包

        说明:
        ::

            因为imu sensor 输出频率过高,因此device侧会将若干个imu数据包组装成一帧发给SDK,
            data_number描述的即为有多少个数据包

        """
        ...

    @property
    def imu_data_type(self) -> int:
        """
        imu数据类型,使用bitfield方式表征各传感器数据是否有效(1 有效,0 无效)

        位说明:
        ::

            bit0:gyro
            bit1:accel
            bit2:mang
            bit3:temp


        """
        ...

    @property
    def imu_factory_data(self) -> IMU_FACTORY_DATA_STRUC:
        """
        imu厂测标定数据,仅当设备处于工厂模式才会传递此数据.

        若SDK未传递此数据,则该成员为None
        """
        ...

    @property
    def imu_is_factory(self) -> bool:
        """
        imu是否处于工厂模式
        """
        ...


class s_nvpfm_dev_info:
    def __init__(self) -> None:
        ...

    @property
    def colorsupported(self) -> RESCOMBO:
        """
        设备分辨率等能力描述
        """
        ...

    @property
    def comm_type(self) -> str:
        """
        内部使用
        """
        ...

    @property
    def cpu_net_type(self) -> str:
        """
        内部使用
        """
        ...

    @property
    def cpu_temperature(self) -> float:
        """
        CPU 温度
        """
        ...

    @property
    def depth(self) -> list[bool]:
        """
        多少路深度数据可用

        说明:
        ::

            如果某路数据可用,则depth[index] is True
        """
        ...

    @property
    def depth0(self) -> list[bool]:
        """
        depth0 通道所使用的sensor index

        说明:
        ::

            对应的self.depth[index] is True


        """
        ...

    @property
    def depth1(self) -> list[bool]:
        """
        depth1 通道所使用的sensor index

        说明:
        ::

            对应的self.depth[index] is True


        """
        ...

    @property
    def imu_type0(self) -> str:
        """
        内部使用
        """
        ...

    @property
    def imu_type1(self) -> str:
        """
        内部使用
        """
        ...

    @property
    def irsupported(self) -> RESCOMBO:
        """
        红外能力信息
        """
        ...

    @property
    def product(self) -> str:
        """
        产品信息
        """
        ...

    @property
    def projector_temperature(self) -> float:
        """
        投射器温度
        """
        ...

    @property
    def projector_type(self) -> str:
        """
        投射器类型
        """
        ...

    @property
    def rgb(self) -> list[bool]:
        """
        多少路rgb sensor可用, 至多2路

        说明:
        ::

            如果某路sensor可用,则depth[index] is True


        """
        ...

    @property
    def rgb0(self) -> list[bool]:
        """
        rgb0 所使用的sensor index
        """
        ...

    @property
    def rgb1(self) -> list[bool]:
        """
        rgb1 所使用的sensor index
        """
        ...

    @property
    def sensor(self) -> list[bool]:
        """
        包含IR、RGB sensor在内的,总共可用sensor列表
        """
        ...

    @property
    def sn(self) -> str:
        """
        产品序列号
        """
        ...

    @property
    def software_version(self) -> str:
        """
        软件版本
        """
        ...


class py_nvpfm:
    class NVPFM_IMAGE_SIZE:
        """图像分辨率枚举定义

        falcon device所能支持的所有分辨率
        """

        __members__: ClassVar[dict] = ...  # read-only
        IMAGE_1280_1080: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_1280_480: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_1280_720: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_1280_800: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_1280_960: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_1600_1200: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_1920_1080: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_320_200: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_320_240: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_480_300: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_480_640: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_640_360: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_640_400: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_640_480: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_768_480: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_800_600: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_848_480: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_960_1280: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_960_600: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        IMAGE_UNKNOWN: ClassVar[py_nvpfm.NVPFM_IMAGE_SIZE] = ...
        __entries: ClassVar[dict] = ...

        def __init__(self, value: int) -> None:
            ...

        def __eq__(self, other: object) -> bool:
            ...

        def __getstate__(self) -> int:
            ...

        def __hash__(self) -> int:
            ...

        def __index__(self) -> int:
            ...

        def __int__(self) -> int:
            ...

        def __ne__(self, other: object) -> bool:
            ...

        def __setstate__(self, state: int) -> None:
            ...

        @property
        def name(self) -> str:
            ...

        @property
        def value(self) -> int:
            ...

    class ev_type:
        """
        事件类型定义
        """

        __members__: ClassVar[dict] = ...  # read-only
        Depth: ClassVar[py_nvpfm.ev_type] = ...
        """
        深度类型
        """
        DepthPseudo: ClassVar[py_nvpfm.ev_type] = ...
        """
        深度伪彩图类型
        """
        Group: ClassVar[py_nvpfm.ev_type] = ...
        """
        组类型,SDK暂不支持类型数据
        """
        IMU: ClassVar[py_nvpfm.ev_type] = ...
        """
        IMU类型
        """
        LeftIR: ClassVar[py_nvpfm.ev_type] = ...
        """
        左目红外类型
        """
        RGB: ClassVar[py_nvpfm.ev_type] = ...
        """
        RGB Sensor类型
        """
        RightIR: ClassVar[py_nvpfm.ev_type] = ...
        """
        右目红外类型
        """
        __entries: ClassVar[dict] = ...

        def __init__(self, value: int) -> None:
            ...

        def __eq__(self, other: object) -> bool:
            ...

        def __getstate__(self) -> int:
            ...

        def __hash__(self) -> int:
            ...

        def __index__(self) -> int:
            ...

        def __int__(self) -> int:
            ...

        def __ne__(self, other: object) -> bool:
            ...

        def __setstate__(self, state: int) -> None:
            ...

        @property
        def name(self) -> str:
            ...

        @property
        def value(self) -> int:
            ...

    Depth: ClassVar[py_nvpfm.ev_type] = ...
    DepthPseudo: ClassVar[py_nvpfm.ev_type] = ...
    Group: ClassVar[py_nvpfm.ev_type] = ...
    IMU: ClassVar[py_nvpfm.ev_type] = ...
    LeftIR: ClassVar[py_nvpfm.ev_type] = ...
    RGB: ClassVar[py_nvpfm.ev_type] = ...
    RightIR: ClassVar[py_nvpfm.ev_type] = ...

    def __init__(self) -> None:
        ...

    def get_dev_info(self) -> list[s_nvpfm_dev_info]:
        """获取设备信息

        获取当前枚举到的所有设备信息

        :return: 失败返回None,否则返回s_nvpfm_dev_info对象
        """
        ...

    def get_event_enabled(self, event) -> bool:
        """检查指定事件是否使能

        :param event: 事件
        :return: 使能返回True,否则返回False
        """
        ...

    def init_and_start(self, log_file_path: str, log_file_size: int) -> int:
        """初始化SDK,并启动枚举线程

        :param log_file_path: SDK的log保存路径
        :param log_file_size: Log文件大小限制
        """
        ...

    def read_data(self) -> list[event_data]:
        """获取事件列表

        此函数会检查SDK的事件数据队列中有无数据,如果有,则会一次性读出,并以列表形式返回,用户则需要在业务代码中遍历该列表,并依次分析各数据

        SDK中的事件数据队列有大小限制,如果业务代码没有及时调用此接口取走数据,在队列满之后,所有后续到达的数据都会被丢弃

        如果用户业务层面取数据频率较低, 可能会导致上报频率较低的数据(如depth)一直得不到上报,因为队列清空时, 上报频率高的数据会快速填充队列,depth
        之类的数据能得到填充的机率较低

        :return: 队列中的所有事件数据
        """
        ...

    def set_event_enabled(self, event, enable: bool) -> None:
        """使能或者失能某事件
        使能/失能会在下一次回调事件失效,不会影响当前的数据状态.比如,失能某事件,不会清空当前事件数据队列中的该事件数据,用户可能在下次或者下几次read_data的结果中依然看到该事件的数据

        :param event: 事件类型
        :param enable: 设置为True使能该事件,False失能该事件

        """
        ...

    def stop(self) -> None:
        """停止SDK
        此函数会停止所有SDK线程、封装层代码的线程,并且阻塞调用者线程直到所线程退出.此函数返回后,意味着SDK Instance已经处于失效状态不可再次使用.
        """
        ...
