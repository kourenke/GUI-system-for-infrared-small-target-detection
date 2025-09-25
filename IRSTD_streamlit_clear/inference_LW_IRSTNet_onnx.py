from typing import Tuple, Any, Union
from typing_extensions import TypeAlias

from time import perf_counter

import cv2
import numpy as np
import matplotlib.pyplot as plt
import onnxruntime

from streamlit import cache_resource

from utils.image_utils import (
    resize_image,
    preprocess_image,
    preprocess_image_with_cache,
)


InferDetail: TypeAlias = Tuple[int, Any, np.ndarray, np.ndarray]
InferDetail.__doc__ = """检测结果类型

Returns:
    Tuple[int, Any, np.ndarray, np.ndarray]: 标签数量， _， 像素大小， 质心坐标
"""


@cache_resource(show_spinner=False, ttl=24 * 60 * 60)
def get_inference_result(
    __net, image: cv2.Mat, base_size: int
) -> Tuple[float, InferDetail, np.ndarray, np.ndarray]:
    """获取推断结果

    Args:
        __net (_type_): _description_
        uploaded_image (cv2.Mat): 上传的图片数组。
        base_size (int): _description_

    Returns:
        Tuple[float, InferDetail, np.ndarray, np.ndarray]: 推断时长, 推断结果, 原图, 结果图
    """
    origin_image = resize_image(image, base_size)
    after_process_image = preprocess_image_with_cache(origin_image)

    __net.setInput(after_process_image)

    start = perf_counter()
    output = __net.forward()
    output1 = output.reshape(base_size, base_size)
    output2 = output1 > 0  # args
    output3 = output2.astype(np.uint8) * 255
    end = perf_counter()

    result = cv2.connectedComponentsWithStats(output3, connectivity=8)

    return end - start, result, origin_image, output2  # type: ignore


def get_inference_result_with_args(
    __net,
    image: cv2.Mat,
    base_size: int,
    output_threshold: float,
    min_target_size: int,
    max_target_size: int,
    min_target_aspect_ratio: float,
    max_target_aspect_ratio: float,
) -> Tuple[float, InferDetail, np.ndarray, np.ndarray]:
    """获取推断结果

    Args:
        __net (_type_): _description_
        uploaded_image (Union[cv2.Mat, np.float32]): 上传的图片数组。
        base_size (int): _description_

    Returns:
        Tuple[float, InferDetail, np.ndarray, np.ndarray]: 推断时长, 推断结果, 原图, 结果图
    """
    origin_image = resize_image(image, base_size)
    after_process_image = preprocess_image(origin_image)

    __net.setInput(after_process_image)

    start = perf_counter()

    output0 = __net.forward()  # (1,1,256,256)
    output0 = output0.reshape(base_size, base_size)  # (256,256)
    output = output0 > output_threshold  # (256,256)
    output = output.astype(np.uint8) * 255  # (256,256)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        output, connectivity=8
    )

    output1 = np.zeros((output.shape[0], output.shape[1]), np.uint8)

    for k in range(1, num_labels):
        mask = labels == k
        if not (min_target_size <= stats[:, 4][k] <= max_target_size):
            output1[:, :][mask] = 0

        elif not (
            min_target_aspect_ratio
            <= stats[:, 2][k] / stats[:, 3][k]
            <= max_target_aspect_ratio
        ):
            output1[:, :][mask] = 0
        else:
            output1[:, :][mask] = 255

    end = perf_counter()

    # 统计阈值范围内连通域数据，
    result = cv2.connectedComponentsWithStats(output1, connectivity=8)

    return end - start, result, origin_image, output1  # type: ignore


if __name__ == "__main__":
    # load network
    net = cv2.dnn.readNetFromONNX("./LW_IRST.onnx") #./LW_IRST.onnx

    # load image
    base_size = 256
    img = cv2.imread("./single_image/1.png", 1)
    print(type(img))
    img = np.float32(cv2.resize(img, (base_size, base_size))) / 255  # type: ignore
    input_ = preprocess_image(img)
    net.setInput(input_)

    # inference in cpu
    print("...inference in progress")
    start = perf_counter()
    output = net.forward()
    output1 = output.reshape(base_size, base_size)
    print(output1)
    output2 = output1 > 0
    output3 = output2.astype(np.uint8) * 255
    end = perf_counter()
    running_FPS = 1 / (end - start)
    print("running_FPS:", running_FPS)

    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        output3, connectivity=8
    )
    # 连通域数量
    print("目标数量")
    print("num_labels = ", num_labels - 1)
    print("目标质心坐标")
    print("centroids = ", centroids[1:])
    print("目标长宽比")
    print("stats = ", stats[1:, 2] / stats[1:, 3])
    print("目标像素大小")
    print("stats = ", stats[1:, 4])

    plt.subplot(121)
    plt.imshow(img, cmap="gray")
    plt.title("Input image")

    plt.subplot(122)
    plt.imshow(output2, cmap="gray")
    plt.title("Inference result")

    plt.show()
