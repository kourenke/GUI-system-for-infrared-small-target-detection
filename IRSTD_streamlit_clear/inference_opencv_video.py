from typing import Tuple

import os
import time

import cv2
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm  # python 进度条库

from utils.image_utils import preprocess_image


def apart(video_path, video_name, image_path):
    """
    功能：将视频拆分成图片
    参数：
        video_path： 要拆分的视频路径
        video_name： 要拆分的视频名字（ 不带后缀 ）
        image_path： 拆分后图片的存放路径
    """

    # 在这里把后缀接上
    video = os.path.join(video_path, video_name + ".avi")
    if not os.path.exists(image_path):
        # 如果文件目录不存在则创建目录
        os.makedirs(image_path)
    # 获取视频
    use_video = cv2.VideoCapture(video)
    fps = use_video.get(cv2.CAP_PROP_FPS)
    width = int(use_video.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(use_video.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(fps, width, height)
    # 初始化计数器
    count = 0
    # 开始循环抽取图片
    print("Start extracting images!")
    while True:
        res, image = use_video.read()
        count += 1
        # 如果提取完图片，则退出循环
        if not res:
            print("not res , not image")
            break
        # 将图片写入文件夹中
        cv2.imwrite(image_path + "0" + format(str(count), "0>4s") + ".png", image)
        print(image_path + str(count) + ".png")
    print("End of image extraction!")
    use_video.release()


def get_video_info(__video_obj) -> Tuple[int, int, int, float]:
    """获取视频相关信息

    Args:
        __video_obj (__type__): 上传的视频对象

    Returns:
        Tuple[int, int, int, float]: 帧宽度, 帧高度, 总帧数, 帧率
    """

    width = int(__video_obj.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(__video_obj.get(cv2.CAP_PROP_FRAME_HEIGHT))
    duration = int(__video_obj.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = __video_obj.get(cv2.CAP_PROP_FPS)

    return width, height, duration, fps


if __name__ == "__main__":
    video_path = r"./original_video/"
    video_name = r"IR5"
    image_path1 = r"./original_image/"
    apart(video_path, video_name, image_path1)

    # load network
    net = cv2.dnn.readNetFromONNX("./LW_IRST.onnx")

    img_list = [
        os.path.join(nm)
        for nm in os.listdir(image_path1)
        if nm[-3:] in ["bmp", "png", "gif"]
    ]
    start1 = time.perf_counter()
    i = 0
    for i in img_list:
        path = os.path.join(image_path1, i)
        img = cv2.imread(path, 1)
        # load image
        base_size = 256
        img = np.float32(cv2.resize(img, (base_size, base_size))) / 255  # type: ignore
        input = preprocess_image(img)
        net.setInput(input)
        # inference in cpu
        print("...inference in progress")
        output = net.forward()
        output = output.reshape(base_size, base_size)
        figure = plt.figure()
        output2 = output > 0
        output3 = output2.astype(np.uint8) * 255
        # 连通域分析
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

        plt.imshow(output3, cmap="gray"), plt.axis("off"), plt.text(
            10,
            10,
            f"Target centroids:{np.round(centroids[1:],0)}",
            size=7,
            color="w",
            alpha=1,
        ), plt.text(
            200, 10, f"Numbers:{num_labels - 1}", size=7, color="w", alpha=1
        ), plt.text(
            200, 20, f"pixel size:{stats[1:, 4] - 1}", size=7, color="w", alpha=1
        ), plt.text(
            120, 130, "+", size=20, color="w", alpha=0.2
        )
        plt.savefig("./segmentation_image/{}".format(str(i)))
        plt.close()

    # 转视频
    image_path2 = r"./segmentation_image/"
    # img = cv2.imread('./data/inference/00001.png')
    # sp = img.shape
    fps = 20  # fps: frame per seconde 每秒帧数，数值可根据需要进行调整
    size = (640, 480)  # (sp[1], sp[0])
    fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
    inference_video_path = "./segmentation_video/"
    inference_video_name = inference_video_path + video_name + ".mp4"
    video = cv2.VideoWriter(inference_video_name, fourcc, fps, size, isColor=True)
    # video = cv2.VideoWriter(r"./segmentation_video/inference_video.mp4", fourcc, fps, size, isColor=True)
    image_list = sorted(
        [name for name in os.listdir(image_path2) if name.endswith(".png")]
    )  # 获取文件夹下所有格式为 png 图像的图像名，并按时间戳进行排序
    for image_name in tqdm(image_list):  # 遍历 image_list 中所有图像并添加进度条
        image_full_path = os.path.join(image_path2, image_name)  # 获取图像的全路经
        image = cv2.imread(image_full_path)  # 读取图像
        video.write(image)  # 将图像写入视频
    video.release()
    cv2.destroyAllWindows()
    end2 = time.perf_counter()
    running_time = end2 - start1
    print("running_time:", running_time)

    my_path1 = "C:/Users/RenkeKou/Desktop/Project/original_image/"
    my_path2 = "C:/Users/RenkeKou/Desktop/Project/segmentation_image/"

    for file_name in os.listdir(my_path1):
        if file_name.endswith(".png"):
            os.remove(my_path1 + file_name)

    for file_name in os.listdir(my_path2):
        if file_name.endswith(".png"):
            os.remove(my_path2 + file_name)
