from typing import Tuple, Optional

import math
import random
import time

import cv2
import numpy as np
from typing_extensions import TypeAlias

from utils.image_utils import preprocess_image, resize_image
from inference_LW_IRSTNet_onnx import InferDetail


TPositionInfo_: TypeAlias = Tuple[int, int, int, int, int, int]
TPositionInfo_.__doc__ = """目标的详细位置信息

Returns:
    Tuple[int, int, int, int, int, int]: 经度， 纬度， 方位角， 俯仰角， 目标距离， 海拔高度
"""


def calc_situation(long1, lati1, deg, beta, dis, h) -> Tuple[float, float, float]:
    """结算目标经纬度和海拔高度

    Args:
        long1 (_type_): _description_
        lati1 (_type_): _description_
        deg (_type_): _description_
        beta (_type_): _description_
        dis (_type_): _description_
        h (_type_): _description_

    Returns:
        Tuple[float, float, float]: 经度，纬度，海拔高度
    """
    deg = deg * math.pi / 180
    beta = beta * math.pi / 180
    arc = 6371.393 * 1000
    long2 = long1 + dis * math.cos(beta) * math.sin(deg) / (
        arc * math.cos(lati1) * 2 * math.pi / 360
    )
    lati2 = lati1 + dis * math.cos(beta) * math.cos(deg) / (arc * 2 * math.pi / 360)
    H = h + dis * math.sin(beta)

    return long2, lati2, H


def get_target_position_info(
    centroids_x: float, centroids_y: float, aspectratio: float, pixel: int
) -> TPositionInfo_:
    """获取目标对应的位置信息

    Args:
        centroids_x (float): 质心横坐标
        centroids_y (float): 质心纵坐标
        aspectratio (float): 目标长宽比
        pixel (int): 目标总像素数量

    Returns:
        TPositionInfo_: 计算的目标具体位置的详细信息
    """
    return (
        random.randint(113, 116),  # 红外探测系统经度 中国经度（73°33′E至135°05′E）
        random.randint(37, 39),  # 红外探测系统纬度 中国纬度（3°86′N至53°55′N）
        random.randint(0, 90),  # 红外探测系统相对目标的方位角 以正北为0，顺时针为正。
        random.randint(0, 60),  # 红外探测系统相对目标的俯仰角
        random.randint(1000, 1500),  # 红外探测系统相对目标的距离
        random.randint(2250, 2500),  # 红外探测系统的海拔高度
    )


def draw_frame(
    original_image: cv2.Mat,
    result_image: cv2.Mat,
    infer_result: InferDetail,
    selected_target: Optional[int],
) -> cv2.Mat:
    """绘制单帧图像

    Args:
        original_image (cv2.Mat): 视频帧原图
        result_image (cv2.Mat): 推断结果图
        infer_result (InferDetail): 推断详细结果数据
        selected_target (Optional[int]): 选择的目标序号（从零开始）

    Returns:
        cv2.Mat: 合成以后的帧数据
    """
    num_labels1, __labels1, stats1, centroids1 = infer_result

    # 显示实时时间
    time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

    # 用cv2.imshow同时显示红外图像和分割结果，把plt.ion()和plt.ioff()，以及最开始的cv2.imshow('capture', frame)删掉
    # 原图，原图+热力图，预测结果
    heatmapshow = cv2.normalize(
        original_image,
        None,
        alpha=0,
        beta=255,
        norm_type=cv2.NORM_MINMAX,  # type: ignore
        dtype=cv2.CV_8U,
    )
    heatmapshow = cv2.applyColorMap(heatmapshow, cv2.COLORMAP_JET)
    result_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB)
    original_image = cv2.resize(
        original_image, (256, 256), interpolation=cv2.INTER_LINEAR
    )
    add_img = cv2.addWeighted(original_image, 0.7, heatmapshow, 0.3, 0)
    imgs = np.hstack([original_image, add_img, result_image])

    # 红外探测系统经纬度、方位俯仰角、海拔高度（需要在界面上显示，目前是随机假设的值，需要通过GPS和陀螺仪传感器实时获取数据，并加载到该程序中，且在界面显示出来）
    long1 = random.randint(113, 116)  # 红外探测系统经度 中国经度（73°33′E至135°05′E）
    lati1 = random.randint(37, 39)  # 红外探测系统纬度 中国纬度（3°86′N至53°55′N）
    deg = random.randint(0, 90)  # 红外探测系统相对目标的方位角 以正北为0，顺时针为正。
    dis = random.randint(1000, 1500)  # 红外探测系统相对目标的距离
    h = random.randint(2250, 2500)  # 红外探测系统的海拔高度
    beta = random.randint(0, 60)  # 红外探测系统相对目标的俯仰角

    # 图片,要添加的文字,文字添加到图片上的位置,字体的类型,字体大小,字体颜色,字体粗细
    cv2.line(imgs, (0, 127), (255, 127), (255, 0, 0), 1, cv2.LINE_8)  # X轴
    cv2.line(imgs, (107, 127), (107, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (87, 127), (87, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (67, 127), (67, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (47, 127), (47, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (27, 127), (27, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (7, 127), (7, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 127), (127, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (147, 127), (147, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (167, 127), (167, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (187, 127), (187, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (207, 127), (207, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (227, 127), (227, 117), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (247, 127), (247, 117), (255, 0, 0), 1, cv2.LINE_8)

    cv2.line(imgs, (127, 0), (127, 255), (255, 0, 0), 1, cv2.LINE_8)  # Y轴
    cv2.line(imgs, (127, 107), (137, 107), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 87), (137, 87), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 67), (137, 67), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 47), (137, 47), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 27), (137, 27), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 7), (137, 7), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 127), (137, 127), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 147), (137, 147), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 167), (137, 167), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 187), (137, 187), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 207), (137, 207), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 227), (137, 227), (255, 0, 0), 1, cv2.LINE_8)
    cv2.line(imgs, (127, 247), (137, 247), (255, 0, 0), 1, cv2.LINE_8)
    # 在屏幕上显示当前时间
    cv2.putText(imgs, time_str, (5, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
    cv2.putText(
        imgs,
        "Analysis of interpretability",
        (320, 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.3,
        (0, 0, 0),
        1,
    )
    # 在屏幕上显示识别出来几个目标
    cv2.putText(
        imgs,
        f"Number of targets: {str(num_labels1-1)}",
        (515, 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.3,
        (255, 255, 255),
        1,
    )
    if num_labels1 > 1:
        # 在屏幕上显示目标质心和长宽比
        # cv2.putText(
        #     imgs,
        #     f"Target aspect ratio: {np.round(stats1[1:, 2] / stats1[1:, 3], 2)}",
        #     (515, 30),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.3,
        #     (255, 255, 255),
        #     1,
        # )
        # 在屏幕上显示目标的锚框并标上序号
        for m in range(1, num_labels1):
            # 确定目标锚框
            cv2.rectangle(
                imgs,
                (int(stats1[m, 0]), int(stats1[m, 1])),
                (
                    int(stats1[m, 0] + stats1[m, 2]),
                    int(stats1[m, 1] + stats1[m, 3]),
                ),
                (0, 255, 0),
                1,
            )
            cv2.putText(
                imgs,
                f"{str(m)}",
                (int(stats1[m, 0]), int(stats1[m, 1]) - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (0, 255, 0),
                1,
            )

        # 如果目标为多个，则首先确定要跟踪X个目标，然后计算目标经度纬度，并利用波门计算距离视场中心的偏移量，从而进行跟踪
        # if num_labels1 >= 3:

        # 解算得到目标的经纬度及海拔高度
        positon_results = calc_situation(long1, lati1, deg, beta, dis, h)
        nsl = positon_results[0]  # 纬度
        ewl = positon_results[1]  # 经度

        # 将计算结果转换为度-分-秒
        angle1 = int(nsl)
        angle2 = int(ewl)
        min1 = int((nsl - angle1) * 60)
        min2 = int((ewl - angle2) * 60)
        second1 = round((((nsl - angle1) * 60) - min1) * 60)
        second2 = round((((ewl - angle2) * 60) - min2) * 60)

        # 在屏幕上显示目标的经纬度及海拔高度
        cv2.putText(
            imgs,
            f"E: {angle1}deg {min1}min {second1}sec",
            (515, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            imgs,
            f"N: {angle2}deg {min2}min {second2}sec",
            (515, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            imgs,
            f"Height: {np.round(positon_results[2], 2)}",
            (515, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.3,
            (255, 255, 255),
            1,
        )

        if selected_target and 0 < selected_target <= len(centroids1[1:]):
            cv2.putText(
                imgs,
                f"Trace target index: {selected_target} / {len(centroids1[1:])}",
                (515, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
            )
            # 在屏幕上显示第1或第2号目标质心到视场中心的偏移线和偏移
            centroids_x = int(centroids1[1:][selected_target - 1][0])
            centroids_y = int(centroids1[1:][selected_target - 1][1])
            cv2.line(
                imgs,
                (127, 127),
                (
                    centroids_x,
                    centroids_y,
                ),
                (0, 0, 255),
                1,
                cv2.LINE_8,
            )
            cv2.putText(
                imgs,
                f"Gate tracking offset: ({centroids_x - 127}",
                (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (0, 0, 255),
                1,
            )
            cv2.putText(
                imgs,
                f",{centroids_y - 127})",
                (140, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (0, 0, 255),
                1,
            )

    # 在分割结果图上显示中心十字标
    cv2.line(imgs, (630, 127), (650, 127), (255, 0, 255), 1, cv2.LINE_8)
    cv2.line(imgs, (640, 117), (640, 137), (255, 0, 255), 1, cv2.LINE_8)

    return imgs


class CamaroCap:
    # TODO(batu1579):Camera ??

    # 打开摄像头
    # 使用640×512摄像头，直接修改分辨率参数即可
    # 使用1280×1024摄像头，如果显示不正常，打开ASICCoreController软件，点击视频-数字视频-COMS内容-YUV422
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        # 在连接摄像头的时候，界面可能要设置一个分辨率选择按钮
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # 1280 640
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 512)  # 1024 512
        self.cap.get(cv2.CAP_PROP_FPS)
        if not self.cap.isOpened():
            print("Error: Cannot open camera")

    # 图片信息打印
    def get_image_info(self, image):
        print(type(image))
        print(image.shape)
        print(image.size)
        print(image.dtype)
        pixel_data = np.array(image)
        print(pixel_data)

    # 逐帧读取数据
    def Camaro_image(self):
        global output
        i = 0
        start0 = time.perf_counter()
        while True:
            """
            ret：True或者False，代表有没有读取到图片
            frame：表示截取到一帧的图片
            """
            start1 = time.perf_counter()
            ret, frame = self.cap.read()

            # load image
            base_size = 256
            img = resize_image(frame, base_size)
            preprocessed_image = preprocess_image(img)  # (1,3,256,256)

            # inference in cpu
            print("...inference in progress")

            # 用cv2.dnn推理onnx框架
            net.setInput(preprocessed_image)
            output0 = net.forward()  # (1,1,256,256)

            # 实时显示推理结果
            output0 = output0.reshape(base_size, base_size)  # (256,256)
            # 这个阈值默认0，希望能在界面上进行调节
            output = output0 > 0  # (256,256)
            output = output.astype(np.uint8) * 255  # (256,256)

            # 连通域分析，增加了阈值设置。
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
                output, connectivity=8
            )
            output1 = np.zeros((output.shape[0], output.shape[1]), np.uint8)
            # 目标阈值调整（需要在界面上进行调节控制）
            for k in range(1, num_labels):
                mask = labels == k
                # 目标像素大小的下限/上限可调，表示当目标像素大于100或小于5的时候，该目标自动舍弃（希望在界面设可以控制）
                if stats[:, 4][k] >= 100 or stats[:, 4][k] <= 5:
                    output1[:, :][mask] = 0
                # 目标长宽比的下限/上限可调，表示当目标长宽比大于5或小于0.2的时候，该目标自动舍弃（希望在界面设可以控制）
                elif (stats[:, 2][k] / stats[:, 3][k]) >= 5 or (
                    stats[:, 2][k] / stats[:, 3][k]
                ) <= 0.2:
                    output1[:, :][mask] = 0
                else:
                    output1[:, :][mask] = 255

            # 统计阈值范围内连通域数据，
            num_labels1, labels1, stats1, centroids1 = cv2.connectedComponentsWithStats(
                output1, connectivity=8
            )
            # 连通域数量
            print("num_labels = ", num_labels1 - 1)
            # 显示目标质心
            print("centroids = ", centroids1[1:])
            # 连通域的信息：对应各个轮廓的x、y、width、height和面积
            print("stats = ", stats1[1:])
            # print(str(stats1[1,0]))
            # 目标长宽比
            print("stats = ", stats1[1:, 2] / stats1[1:, 3])

            # 显示实时时间
            time_str = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())

            # 用cv2.imshow同时显示红外图像和分割结果，把plt.ion()和plt.ioff()，以及最开始的cv2.imshow('capture', frame)删掉
            # 原图，原图+热力图，预测结果
            heatmapshow = None
            heatmapshow = cv2.normalize(
                output0,
                heatmapshow,
                alpha=0,
                beta=255,
                norm_type=cv2.NORM_MINMAX,  # type: ignore
                dtype=cv2.CV_8U,
            )
            heatmapshow = cv2.applyColorMap(heatmapshow, cv2.COLORMAP_JET)
            output1 = cv2.cvtColor(output1, cv2.COLOR_BGR2RGB)
            frame = cv2.resize(frame, (256, 256), interpolation=cv2.INTER_LINEAR)
            add_img = cv2.addWeighted(frame, 0.7, heatmapshow, 0.3, 0)
            imgs = np.hstack([frame, add_img, output1])

            # 红外探测系统经纬度、方位俯仰角、海拔高度（需要在界面上显示，目前是随机假设的值，需要通过GPS和陀螺仪传感器实时获取数据，并加载到该程序中，且在界面显示出来）
            long1 = random.randint(113, 116)  # 红外探测系统经度 中国经度（73°33′E至135°05′E）
            lati1 = random.randint(37, 39)  # 红外探测系统纬度 中国纬度（3°86′N至53°55′N）
            deg = random.randint(0, 90)  # 红外探测系统相对目标的方位角 以正北为0，顺时针为正。
            dis = random.randint(1000, 1500)  # 红外探测系统相对目标的距离
            h = random.randint(2250, 2500)  # 红外探测系统的海拔高度
            beta = random.randint(0, 60)  # 红外探测系统相对目标的俯仰角

            end1 = time.perf_counter()
            FPS1 = int(1 / (end1 - start1))
            print("界面显示的FPS = ", FPS1)

            # 图片,要添加的文字,文字添加到图片上的位置,字体的类型,字体大小,字体颜色,字体粗细
            cv2.line(imgs, (0, 127), (255, 127), (255, 0, 0), 1, cv2.LINE_8)  # X轴
            cv2.line(imgs, (107, 127), (107, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (87, 127), (87, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (67, 127), (67, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (47, 127), (47, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (27, 127), (27, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (7, 127), (7, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 127), (127, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (147, 127), (147, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (167, 127), (167, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (187, 127), (187, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (207, 127), (207, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (227, 127), (227, 117), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (247, 127), (247, 117), (255, 0, 0), 1, cv2.LINE_8)

            cv2.line(imgs, (127, 0), (127, 255), (255, 0, 0), 1, cv2.LINE_8)  # Y轴
            cv2.line(imgs, (127, 107), (137, 107), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 87), (137, 87), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 67), (137, 67), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 47), (137, 47), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 27), (137, 27), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 7), (137, 7), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 127), (137, 127), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 147), (137, 147), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 167), (137, 167), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 187), (137, 187), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 207), (137, 207), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 227), (137, 227), (255, 0, 0), 1, cv2.LINE_8)
            cv2.line(imgs, (127, 247), (137, 247), (255, 0, 0), 1, cv2.LINE_8)
            # 在屏幕上显示当前时间
            cv2.putText(
                imgs, time_str, (5, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1
            )
            # 在屏幕上显示当前视频的第i帧
            cv2.putText(
                imgs,
                f"Frame:{i}",
                (180, 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (0, 0, 0),
                1,
            )
            cv2.putText(
                imgs,
                "Analysis of interpretability",
                (320, 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (0, 0, 0),
                1,
            )
            # 在屏幕上显示识别出来几个目标
            cv2.putText(
                imgs,
                f"Number of targets: {str(num_labels1-1)}",
                (515, 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
            )
            # 在屏幕上显示的计算速度FPS
            cv2.putText(
                imgs,
                f"FPS:{str(FPS1)}",
                (700, 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.3,
                (255, 255, 255),
                1,
            )
            if num_labels1 - 1 >= 1:
                # 在屏幕上显示目标质心和长宽比
                cv2.putText(
                    imgs,
                    f"Centroids: {np.round(centroids1[1:], 0)}",
                    (515, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (255, 255, 255),
                    1,
                )
                cv2.putText(
                    imgs,
                    f"Target aspect ratio: {np.round(stats1[1:, 2] / stats1[1:, 3], 2)}",
                    (515, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.3,
                    (255, 255, 255),
                    1,
                )
                # 在屏幕上显示目标的锚框并标上序号
                for m in range(1, num_labels1):
                    # 确定目标锚框
                    cv2.rectangle(
                        imgs,
                        (int(stats1[m, 0]), int(stats1[m, 1])),
                        (
                            int(stats1[m, 0] + stats1[m, 2]),
                            int(stats1[m, 1] + stats1[m, 3]),
                        ),
                        (0, 255, 0),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"{str(m)}",
                        (int(stats1[m, 0]), int(stats1[m, 1]) - 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 255, 0),
                        1,
                    )

                # 如果目标为多个，则首先确定要跟踪X个目标，然后计算目标经度纬度，并利用波门计算距离视场中心的偏移量，从而进行跟踪
                if num_labels1 - 1 == 2:
                    # 解算得到目标的经纬度及海拔高度
                    Results = calc_situation(long1, lati1, deg, beta, dis, h)
                    nsl = Results[0]  # 纬度
                    ewl = Results[1]  # 经度
                    # 将计算结果转换为度-分-秒
                    """度"""
                    angle1 = int(nsl)
                    angle2 = int(ewl)
                    """分"""
                    min1 = int((nsl - angle1) * 60)
                    min2 = int((ewl - angle2) * 60)
                    """秒"""
                    second1 = round((((nsl - angle1) * 60) - min1) * 60)
                    second2 = round((((ewl - angle2) * 60) - min2) * 60)
                    print("东经：{}度{}分{}秒".format(angle1, min1, second1))
                    print("北纬：{}度{}分{}秒".format(angle2, min2, second2))
                    # 在屏幕上显示目标的经纬度及海拔高度
                    cv2.putText(
                        imgs,
                        f"E: {angle1}deg {min1}min {second1}sec",
                        (515, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"N: {angle2}deg {min2}min {second2}sec",
                        (515, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"Height: {np.round(Results[2],2)}",
                        (515, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    # 在屏幕上显示第1或第2号目标质心到视场中心的偏移线和偏移量
                    # 目前设置的是第2个目标(int(centroids1[1:][1][0]), int(centroids1[1:][1][1]))
                    cv2.line(
                        imgs,
                        (127, 127),
                        (int(centroids1[1:][1][0]), int(centroids1[1:][1][1])),
                        (0, 0, 255),
                        1,
                        cv2.LINE_8,
                    )
                    cv2.putText(
                        imgs,
                        f"Gate tracking offset: ({int(centroids1[1:][1][0])-127}",
                        (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f",{int(centroids1[1:][1][1]) - 127})",
                        (140, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )

                elif num_labels1 - 1 == 3:
                    # 解算得到目标的经纬度及海拔高度
                    Results = calc_situation(long1, lati1, deg, beta, dis, h)
                    nsl = Results[0]  # 纬度
                    ewl = Results[1]  # 经度
                    # 将计算结果转换为度-分-秒
                    """度"""
                    angle1 = int(nsl)
                    angle2 = int(ewl)
                    """分"""
                    min1 = int((nsl - angle1) * 60)
                    min2 = int((ewl - angle2) * 60)
                    """秒"""
                    second1 = round((((nsl - angle1) * 60) - min1) * 60)
                    second2 = round((((ewl - angle2) * 60) - min2) * 60)
                    print("东经：{}度{}分{}秒".format(angle1, min1, second1))
                    print("北纬：{}度{}分{}秒".format(angle2, min2, second2))
                    # 在屏幕上显示目标的经纬度及海拔高度
                    cv2.putText(
                        imgs,
                        f"E: {angle1}deg {min1}min {second1}sec",
                        (515, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"N: {angle2}deg {min2}min {second2}sec",
                        (515, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"Height: {np.round(Results[2],2)}",
                        (515, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    # 在屏幕上显示第1或第2号或第3号目标质心到视场中心的偏移线和偏移量
                    # 目前设置的是第3个目标(int(centroids1[1:][2][0]), int(centroids1[1:][2][1]))
                    cv2.line(
                        imgs,
                        (127, 127),
                        (int(centroids1[1:][2][0]), int(centroids1[1:][2][1])),
                        (0, 0, 255),
                        1,
                        cv2.LINE_8,
                    )
                    cv2.putText(
                        imgs,
                        f"Gate tracking offset: ({int(centroids1[1:][1][0])-127}",
                        (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f",{int(centroids1[1:][1][1]) - 127})",
                        (140, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )

                elif num_labels1 - 1 == 4:
                    # 解算得到目标的经纬度及海拔高度
                    Results = calc_situation(long1, lati1, deg, beta, dis, h)
                    nsl = Results[0]  # 纬度
                    ewl = Results[1]  # 经度
                    # 将计算结果转换为度-分-秒
                    """度"""
                    angle1 = int(nsl)
                    angle2 = int(ewl)
                    """分"""
                    min1 = int((nsl - angle1) * 60)
                    min2 = int((ewl - angle2) * 60)
                    """秒"""
                    second1 = round((((nsl - angle1) * 60) - min1) * 60)
                    second2 = round((((ewl - angle2) * 60) - min2) * 60)
                    print("东经：{}度{}分{}秒".format(angle1, min1, second1))
                    print("北纬：{}度{}分{}秒".format(angle2, min2, second2))
                    # 在屏幕上显示目标的经纬度及海拔高度
                    cv2.putText(
                        imgs,
                        f"E: {angle1}deg {min1}min {second1}sec",
                        (515, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"N: {angle2}deg {min2}min {second2}sec",
                        (515, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"Height: {np.round(Results[2],2)}",
                        (515, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    # 在屏幕上显示第1或第2号或第3号目标或第4号目标质心到视场中心的偏移线和偏移量
                    # 目前设置的是第3个目标(int(centroids1[1:][3][0]), int(centroids1[1:][3][1]))
                    cv2.line(
                        imgs,
                        (127, 127),
                        (int(centroids1[1:][3][0]), int(centroids1[1:][3][1])),
                        (0, 0, 255),
                        1,
                        cv2.LINE_8,
                    )
                    cv2.putText(
                        imgs,
                        f"Gate tracking offset: ({int(centroids1[1:][1][0])-127}",
                        (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f",{int(centroids1[1:][1][1]) - 127})",
                        (140, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )

                else:
                    # 解算得到目标的经纬度及海拔高度
                    Results = calc_situation(long1, lati1, deg, beta, dis, h)
                    nsl = Results[0]  # 纬度
                    ewl = Results[1]  # 经度
                    # 将计算结果转换为度-分-秒
                    """度"""
                    angle1 = int(nsl)
                    angle2 = int(ewl)
                    """分"""
                    min1 = int((nsl - angle1) * 60)
                    min2 = int((ewl - angle2) * 60)
                    """秒"""
                    second1 = round((((nsl - angle1) * 60) - min1) * 60)
                    second2 = round((((ewl - angle2) * 60) - min2) * 60)
                    print("东经：{}度{}分{}秒".format(angle1, min1, second1))
                    print("北纬：{}度{}分{}秒".format(angle2, min2, second2))
                    # 在屏幕上显示目标的经纬度及海拔高度
                    cv2.putText(
                        imgs,
                        f"E: {angle1}deg {min1}min {second1}sec",
                        (515, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"N: {angle2}deg {min2}min {second2}sec",
                        (515, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f"Height: {np.round(Results[2], 2)}",
                        (515, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (255, 255, 255),
                        1,
                    )
                    # 在屏幕上显示目标质心到视场中心的偏移线和偏移量
                    cv2.line(
                        imgs,
                        (127, 127),
                        (int(centroids1[1:][0][0]), int(centroids1[1:][0][1])),
                        (0, 0, 255),
                        1,
                        cv2.LINE_8,
                    )
                    cv2.putText(
                        imgs,
                        f"Gate tracking offset: ({int(centroids1[1:][0][0]) - 127}",
                        (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )
                    cv2.putText(
                        imgs,
                        f",{int(centroids1[1:][0][1]) - 127})",
                        (140, 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.3,
                        (0, 0, 255),
                        1,
                    )

            # 在分割结果图上显示中心十字标
            cv2.line(imgs, (630, 127), (650, 127), (255, 0, 255), 1, cv2.LINE_8)
            cv2.line(imgs, (640, 117), (640, 137), (255, 0, 255), 1, cv2.LINE_8)
            cv2.imshow(
                "Original infrared image Vs Heatmap Vs Segmentation results", imgs
            )

            end2 = time.perf_counter()
            FPS2 = int(1 / (end2 - start1))
            print("完整推理并显示完毕1帧的FPS = ", FPS2)
            i = i + 1

            # 监测键盘输入是否为q，为q则退出程序
            # cv.waitKey(1)：这个表示每一秒（1的参数的作用）监测一下键盘上的输入, 并转化成二进制
            # 0xFF：转化为二进制为0b111111111
            if cv2.waitKey(1) & 0xFF == ord("q"):  # 按q退出
                break
        end3 = time.perf_counter()
        print("i = ", i)
        FPS3 = int(i / (end3 - start0))
        print("平均FPS = ", FPS3)


if __name__ == "__main__":
    # 通过cv2.dnn方式加载onnx框架
    net = cv2.dnn.readNetFromONNX("./LW_IRST.onnx")

    outmasages = CamaroCap()
    # 调用摄像头
    outmasages.Camaro_image()
    # 释放对象和销毁窗口
    outmasages.cap.release()
    cv2.destroyAllWindows()
