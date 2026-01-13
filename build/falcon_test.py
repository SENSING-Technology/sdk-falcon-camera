# coding:utf-8
import traceback
import pytest
import falcon_sdk
import os
import asyncio
import time

data_q = asyncio.Queue(maxsize=3)
nvpfm = None

configs = {
    "dev_info_time_wait": 1000,
    "dev_info_retry_count": 10,
    "log_path": "./FC1.log",
    "log_size": 50 * 1024 * 1024,
}


# 支持同步测试和异步测试
@pytest.mark.run(order=1)
def test_create_nvpfm():
    """
    测试能否正常创建nvpfm实例
    """
    global nvpfm
    nvpfm = falcon_sdk.py_nvpfm()
    assert nvpfm is not None


@pytest.mark.run(order=2)
def test_init_and_start():
    """
    测试能否正常启动和初始化实例
    """
    global nvpfm
    #     nvpfm = falcon_sdk.py_nvpfm()
    r = nvpfm.init_and_start("./FC1.log", 50 * 1024 * 1024)
    assert r == 0
    assert os.path.exists("./FC1.log")


@pytest.mark.run(order=3)
def test_get_device_info():
    """
    测试在给定时间和轮询次数内,能否获取到设备信息,以及设备信息是否有效
    目前仅检查设备的sn和软件版本是否有值
    如果这个case没有获取到设备信息,说明存在严重故障, 会导致pytest退出
    """
    cnt = 0
    got_info = False
    infos = None
    while cnt < configs["dev_info_time_wait"]:
        infos = nvpfm.get_dev_info()
        if len(infos):
            got_info = True
            break
        cnt += 1
        time.sleep(configs["dev_info_time_wait"] / 1000)
    assert got_info
    if not got_info:
        pytest.exit("Please insert a falcon camera!", -1)

    assert infos[0].sn != ""
    assert infos[0].software_version != ""
    assert infos[0].product != ""
    # 其他信息, 比如根据配置文件来的分辨率 or what.


def toggle_nvpfm_state(nvpfm: falcon_sdk.py_nvpfm, event):
    en = nvpfm.get_event_enabled(event)
    nvpfm.set_event_enabled(event, not en)


def test_toggle_state():
    """
    测试PySDK的动态修改事件标志能否生效以及能否正常读回
    """
    global nvpfm
    events = [
        falcon_sdk.py_nvpfm.Depth,
        falcon_sdk.py_nvpfm.DepthPseudo,
        falcon_sdk.py_nvpfm.Group,
        falcon_sdk.py_nvpfm.IMU,
        falcon_sdk.py_nvpfm.LeftIR,
        falcon_sdk.py_nvpfm.RightIR,
    ]
    for ev in events:
        old_state = nvpfm.get_event_enabled(ev)
        toggle_nvpfm_state(nvpfm, ev)
        assert nvpfm.get_event_enabled(ev) != old_state
        toggle_nvpfm_state(nvpfm, ev)
        assert nvpfm.get_event_enabled(ev) == old_state


# 限定10s内执行完毕
# @pytest.mark.timeout(10)
def test_stop_nvpfm():
    """
    测试停止nvpfm实例, 如果该过程发生异常, 会导致此case失败
    assert True 用于pytest tag, 后同
    """
    global nvpfm
    print("On Stop!")
    nvpfm.stop()
    nvpfm = None
    print("Stopped!")
    assert True


#
# 循环创建3次
#
def test_recreate_and_stop():
    """
    测试快速启动和停止,反复尝试3次, 此过程如果发生异常会导致
    此case失败
    """
    global nvpfm
    for i in range(3):
        nvpfm = falcon_sdk.py_nvpfm()
        nvpfm.init_and_start("./FC1.log", 50 * 1024 * 1024)
        nvpfm.set_event_enabled(nvpfm.ev_type.RGB, True)
        nvpfm.set_event_enabled(nvpfm.ev_type.LeftIR, True)
        nvpfm.set_event_enabled(nvpfm.ev_type.RightIR, True)
        nvpfm.set_event_enabled(nvpfm.ev_type.DepthPseudo, True)
        nvpfm.set_event_enabled(nvpfm.ev_type.Depth, True)
        nvpfm.set_event_enabled(nvpfm.ev_type.IMU, True)
        time.sleep(0.03)
        nvpfm.stop()
        nvpfm = None
    assert True


class TestNvpfmData:
    nvpfm: falcon_sdk.py_nvpfm

    def setup_class(self):
        """
        准备函数,提前初始化设备
        """
        self.nvpfm = falcon_sdk.py_nvpfm()
        r = self.nvpfm.init_and_start("./FC1.log", 50 * 1024 * 1024)
        print("++++++++++++++++")
        print("nvpfm started!")
        print("++++++++++++++++")
        if r:
            pytest.exit("Failed to start nvpfm!", -1)

        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.RGB, True)
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.LeftIR, True)
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.RightIR, True)
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.DepthPseudo, True)
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.Depth, True)
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.IMU, True)
        # time.sleep(5)

    def teardown_class(self):
        """
        测试用例执行完毕, 停止sdk实例
        """
        print("Tear down!")
        self.nvpfm.stop()
        print("Tear down!!!!")

    async def data_loop(self, ev_type):
        """
        获取给定事件的数据, 此处的休眠用于等待sdk填充数据
        """
        while True:
            await asyncio.sleep(0.01)
            data_list = self.nvpfm.read_data()
            for data in data_list:
                if data.event_type == ev_type:
                    # print(">>>>>>>>>>>>>>>>>>>>> Got data, type:", int(data.event_type))
                    return data

    @pytest.mark.asyncio
    async def test_read_imu(self):
        """
        测试读取imu数据,认为在给定时间内拿到数据即有效
        """
        # data = await self.data_queue_imu.get()
        # 1s 内必须等到数据
        print("test imu!")
        task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.IMU))
        data = await asyncio.wait_for(task, 10)
        print("test imu done!")
        assert data is not None
        # some 数据分析, 比如帧率数据有效, imu数据包解析成多个独立包
        assert True

    @pytest.mark.asyncio
    async def test_read_left_ir(self):
        """
        测试读取left ir 数据,认为在给定时间内拿到数据即有效
        """
        print("test left IR!")
        task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.LeftIR))
        data = await asyncio.wait_for(task, 10)
        print("test left IR done!")
        assert data is not None
        # some 数据分析，比如左红外数据的有效性检查
        assert True

    @pytest.mark.asyncio
    async def test_read_rgb(self):
        """
        测试读取rgb 数据,认为在给定时间内拿到数据即有效
        """
        print("test RGB!")
        task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.RGB))
        data = await asyncio.wait_for(task, 10)
        print("test RGB done!")
        assert data is not None
        # some 数据分析，比如RGB数据的有效性检查
        assert True

    @pytest.mark.asyncio
    async def test_read_right_ir(self):
        """
        测试读取right ir数据,认为在给定时间内拿到数据即有效
        """
        print("test right IR!")
        task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.RightIR))
        data = await asyncio.wait_for(task, 10)
        print("test right IR done!")
        assert data is not None
        # some 数据分析，比如右红外数据的有效性检查
        assert True

    @pytest.mark.asyncio
    async def test_read_depth(self):
        """
        测试读取depth 数据,认为在给定时间内拿到数据即有效
        """
        print("test depth!")
        task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.Depth))
        data = await asyncio.wait_for(task, 10)
        print("test depth done!")
        assert data is not None
        # some 数据分析，比如深度数据的有效性检查
        assert True

    @pytest.mark.asyncio
    async def test_read_depth_pseudo(self):
        """
        测试读取深度伪彩图 数据,认为在给定时间内拿到数据即有效
        """
        print("test depth pseudo!")
        task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.DepthPseudo))
        data = await asyncio.wait_for(task, 10)
        print("test depth pseudo done!")
        assert data is not None
        # some 数据分析，比如伪深度数据的有效性检查
        assert True

    @pytest.mark.asyncio
    async def test_disable_imu(self):
        """
        测试禁用imu数据, 禁用数据且flush后, 读取imu数据超时即认为通过
        """
        # 此 set enabled 可能需要一轮或者多轮才会被callback线程看到
        self.nvpfm.set_event_enabled(falcon_sdk.py_nvpfm.IMU, False)
        data = None
        while True:
            try:
                task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.IMU))
                data = None
                data = await asyncio.wait_for(task, 3)
            except asyncio.TimeoutError as e:
                assert data is None
                break
            except:
                print("Got unexpected exception!")
                traceback.print_exc()
                assert False
                break
        self.nvpfm.set_event_enabled(falcon_sdk.py_nvpfm.IMU, True)
        print("test disable imu done!")

    @pytest.mark.asyncio
    async def test_disable_depth(self):
        """
        测试禁用深度数据, 禁用数据且flush后, 读取深度数据超时即认为通过
        """
        print("test disable depth!")
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.Depth, False)
        data = None
        while True:
            try:
                task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.Depth))
                data = None
                data = await asyncio.wait_for(task, 3)
            except asyncio.TimeoutError as e:
                assert data is None
                break
            except:
                print("Got unexpected exception!")
                traceback.print_exc()
                assert False
                break
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.Depth, True)
        print("test disable depth done!")

    @pytest.mark.asyncio
    async def test_disable_depth_pseudo(self):
        """
        测试禁用伪彩图数据, 禁用数据且flush后, 读取伪彩图数据超时即认为通过
        """
        print("test disable depth pseudo!")
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.DepthPseudo, False)
        data = None
        while True:
            try:
                task = asyncio.create_task(
                    self.data_loop(self.nvpfm.ev_type.DepthPseudo)
                )
                data = None
                data = await asyncio.wait_for(task, 3)
            except asyncio.TimeoutError as e:
                assert data is None
                break
            except:
                print("Got unexpected exception!")
                traceback.print_exc()
                assert False
                break
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.DepthPseudo, True)
        print("test disable depth pseudo done!")

    @pytest.mark.asyncio
    async def test_disable_left_ir(self):
        """
        测试禁用left ir数据, 禁用数据且flush后, 读取left ir数据超时即认为通过
        """
        print("test disable LeftIR!")
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.LeftIR, False)
        data = None
        while True:
            try:
                task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.LeftIR))
                data = None
                data = await asyncio.wait_for(task, 3)
            except asyncio.TimeoutError as e:
                assert data is None
                break
            except:
                print("Got unexpected exception!")
                traceback.print_exc()
                assert False
                break
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.LeftIR, True)
        print("test disable LeftIR done!")

    @pytest.mark.asyncio
    async def test_disable_right_ir(self):
        """
        测试禁用right ir数据, 禁用数据且flush后, 读取right ir数据超时即认为通过
        """
        print("test disable RightIR!")
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.RightIR, False)
        data = None
        while True:
            try:
                task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.RightIR))
                data = None
                data = await asyncio.wait_for(task, 3)
            except asyncio.TimeoutError as e:
                assert data is None
                break
            except:
                print("Got unexpected exception!")
                traceback.print_exc()
                assert False
                break
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.RightIR, True)
        print("test disable RightIR done!")

    @pytest.mark.asyncio
    async def test_disable_rgb(self):
        """
        测试禁用rgb数据, 禁用数据且flush后, 读取rgb数据超时即认为通过
        """
        print("test disable RightIR!")
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.RGB, False)
        data = None
        while True:
            try:
                task = asyncio.create_task(self.data_loop(self.nvpfm.ev_type.RGB))
                data = None
                data = await asyncio.wait_for(task, 3)
            except asyncio.TimeoutError as e:
                assert data is None
                break
            except:
                print("Got unexpected exception!")
                traceback.print_exc()
                assert False
                break
        self.nvpfm.set_event_enabled(self.nvpfm.ev_type.RGB, True)
        print("test disable RGB done!")


if __name__ == "__main__":
    pytest.main()
