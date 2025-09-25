from typing import Tuple, Any
import time
from argparse import ArgumentParser
import cv2
import torch
import numpy as np
import torch.utils.data as Data
import pandas as pd
from tqdm import tqdm
from thop import profile
from utils import data as utils_data
from utils.net import get_net_by_device, get_device
from utils.image_utils import preprocess_image, resize_image
from models.LW_IRST_Net.LW_IRST_ablation import LW_IRST_ablation
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from sosmetrics.metrics import PixelPrecisionRecallF1IoU, PixelROCPrecisionRecall, PixelNormalizedIoU, HybridNormalizedIoU, TargetPrecisionRecallF1, TargetPdPixelFaROC, TargetPdPixelFa, TargetAveragePrecision
from sosmetrics.metrics.mNoCoAP import mNoCoAP



def parse_args():
    # Setting parameters
    parser = ArgumentParser(description="Evaluation of networks")

    # Network parameters
    parser.add_argument(
        "--pkl-path",
        type=str,
        default=r"./checkpoint/01_LW_IRST_ablation_Iter-18400_mIoU-0.6638_fmeasure-0.7979.pkl",
        help="checkpoint path",
    )

    # Dataset parameters
    parser.add_argument(
        "--base-size", type=int, default=256, help="base size of images"
    )
    parser.add_argument(
        "--dataset", type=str, default="sirst_427", help="choose datasets"
    )
    '''
    default='sirstaug/sirst_427/NUDT_SirstDataset_1327/mdfa/merged/Dense_infrared_small_targets
    Visible_light_drone/Visible_light_aircraft/Visible_light_ships/SAR_ships/
    Multimodal
    '''

    # Evaluation parameters
    parser.add_argument(
        "--batch-size", type=int, default=64, help="batch size for training"
    )
    parser.add_argument("--ngpu", type=int, default=0, help="GPU number")
    parser.add_argument("--conf-thresh", type=int, default=0.5, help="Confidence threshold")


    args = parser.parse_args()
    return args


# @st.cache_resource(show_spinner=False)
def get_dataloader(
    dataset_name: utils_data.DatasetType, base_size: int, batch_size: int
) -> Data.DataLoader:
    """加载数据集

    Args:
        dataset_name (str): 数据集名称
        base_size (int): _description_
        batch_size (int): _description_

    Returns:
        Data.DataLoader: 加载后的数据集
    """
    if dataset_name is utils_data.DatasetType.SirstAugDataset:
        __dataset = utils_data.SirstAugDataset(mode="test")

    elif dataset_name is utils_data.DatasetType.SirstDataset_427:
        __dataset = utils_data.SirstDataset_427(mode="test")

    elif dataset_name is utils_data.DatasetType.NUDT_SirstDataset_1327:
        __dataset = utils_data.NUDT_SirstDataset_1327(mode="test")

    elif dataset_name is utils_data.DatasetType.MDFADataset:
        __dataset = utils_data.MDFADataset(mode="test",base_size=base_size)

    elif dataset_name is utils_data.DatasetType.MergedDataset:
        __dataset = utils_data.MergedDataset(mode="test", base_size=base_size)

    elif dataset_name is utils_data.DatasetType.Dense_infrared_small_targets:
        __dataset = utils_data.Dense_infrared_small_targets(mode="test")

    elif dataset_name is utils_data.DatasetType.Visible_light_aircraft:
        __dataset = utils_data.Visible_light_aircraft(mode="test")

    elif dataset_name is utils_data.DatasetType.Visible_light_drone:
        __dataset = utils_data.Visible_light_drone(mode="test")

    elif dataset_name is utils_data.DatasetType.Visible_light_ships:
        __dataset = utils_data.Visible_light_ships(mode="test")

    elif dataset_name is utils_data.DatasetType.SAR_ships:
        __dataset = utils_data.SAR_ships(mode="test")

    elif dataset_name is utils_data.DatasetType.Multimodal:
        __dataset = utils_data.Multimodal(mode="test")


    else:
        raise NotImplementedError(f"Dataset <{dataset_name}> is not implemented")

    return Data.DataLoader(__dataset, batch_size=batch_size, shuffle=False)


class FPS():
    def __init__(self):
        self.fps = []

    def update(self, num_image, time):
        self.fps.append(num_image/time)

    def get(self):
        return np.array(self.fps).mean()

    def reset(self):
        self.fps = []

# @st.cache_resource(show_spinner=False)
# def get_metrics() -> Tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any]:
def get_metrics(
    Metrics_hyperparameter
    ) -> Tuple[Any, Any, Any, Any, Any, Any, Any, Any, Any, Any]:

    """创建矩阵

    Returns:
        Tuple[Any, Any, Any, Any, Any]: metrics, metric_roc, metric_pr, metric_center, metric_pd_fd
    """
    # print(Metrics_hyperparameter.get_var("selected_dis_match_type").name)

    return (
        PixelPrecisionRecallF1IoU(conf_thr = Metrics_hyperparameter.get_var("conf_thr"), debug=False),
        PixelROCPrecisionRecall(conf_thrs = Metrics_hyperparameter.get_var("conf_thrs"), debug=False),
        PixelNormalizedIoU(conf_thr = Metrics_hyperparameter.get_var("conf_thr"), debug=False),
        HybridNormalizedIoU(conf_thr = Metrics_hyperparameter.get_var("conf_thr"), dilate_kernel= Metrics_hyperparameter.get_var("dilate_kernel"), match_alg = Metrics_hyperparameter.get_var("selected_dis_match_type").name,second_match = Metrics_hyperparameter.get_var("selected_second_match_type").name, debug=False),
        TargetPrecisionRecallF1(conf_thr = Metrics_hyperparameter.get_var("conf_thr"), dilate_kernel= Metrics_hyperparameter.get_var("dilate_kernel"), match_alg = Metrics_hyperparameter.get_var("selected_dis_match_type").name, second_match = Metrics_hyperparameter.get_var("selected_second_match_type").name, debug=False),
        TargetAveragePrecision(conf_thrs = Metrics_hyperparameter.get_var("conf_thrs"), dilate_kernel= Metrics_hyperparameter.get_var("dilate_kernel"), match_alg = Metrics_hyperparameter.get_var("selected_dis_match_type").name, second_match = Metrics_hyperparameter.get_var("selected_second_match_type").name, debug=False),
        TargetPdPixelFa(conf_thr = Metrics_hyperparameter.get_var("conf_thr"), dilate_kernel= Metrics_hyperparameter.get_var("dilate_kernel"), match_alg = Metrics_hyperparameter.get_var("selected_dis_match_type").name, second_match = Metrics_hyperparameter.get_var("selected_second_match_type").name, debug=False),
        TargetPdPixelFaROC(conf_thrs = Metrics_hyperparameter.get_var("conf_thrs"), dilate_kernel= Metrics_hyperparameter.get_var("dilate_kernel"), match_alg = Metrics_hyperparameter.get_var("selected_dis_match_type").name, second_match = Metrics_hyperparameter.get_var("selected_second_match_type").name, debug=False),
        mNoCoAP(conf_thr = Metrics_hyperparameter.get_var("conf_thr")),
        FPS()
    )

# @st.cache_resource(show_spinner=False)
def evaluate(
    _data,
    _labels,
    _metrics_PixelPrecisionRecallF1IoU,
    _metric_PixelROCPrecisionRecall,
    _metric_PixelNormalizedIoU,
    _metric_HybridNormalizedIoU,
    _metric_TargetPrecisionRecallF1,
    _metric_TargetAveragePrecision,
    _metric_TargetPdPixelFa,
    _metric_TargetPdPixelFaROC,
    _metric_mNoCoAP,
    _metric_FPS,
    model_path,
):
    """评估模型"""
    device = get_device()
    net = get_net_by_device(model_path, device)
    with torch.no_grad():
        _data = _data.to(device)
        _labels = _labels.to(device)
        num_img = _labels.shape[0]
        start = time.perf_counter()
        output = net(_data)  # (16,1,256,256)
        end = time.perf_counter()
        _metric_FPS.update(num_img, end-start)
    _metrics_PixelPrecisionRecallF1IoU.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_PixelROCPrecisionRecall.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_PixelNormalizedIoU.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_HybridNormalizedIoU.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_TargetPrecisionRecallF1.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_TargetAveragePrecision.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_TargetPdPixelFa.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_TargetPdPixelFaROC.update(labels=_labels, preds=torch.sigmoid(output))
    _metric_mNoCoAP.update(labels=_labels, preds=output, batch=_data)

# @st.cache_resource(show_spinner=False)
def get_model_complexity() -> Tuple[str, str]:
    """计算模型复杂度

    Returns:
        Tuple[Any, Any]: FLOPs, params
    """
    inputs = torch.randn((1, 3, 256, 256))
    models = LW_IRST_ablation(
        channel=(8, 32, 64),
        dilations=(2, 4, 8, 16),
        kernel_size=(7, 7, 7, 7),
        padding=(3, 3, 3, 3),
    )
    flops, params, *_ = profile(models, inputs=(inputs,))
    rounded_flops = round(flops / 1000000, 2)
    rounded_params = round(params / 1000000, 4)
    return (
        f"{rounded_flops}",
        f"{rounded_params}",
    )


# @st.cache_resource(show_spinner=False)
def get_model_fps(__net, base_size: int) -> float:
    """获取模型速度

    Args:
        __net (_type_): network
        base_size (int): _description_

    Returns:
        float: 运行速度（ 1 / 运行时长）
    """
    img = cv2.imread("./test.png", 1)
    img = resize_image(img, base_size)  # type: ignore

    start = time.perf_counter()

    __net = cv2.dnn.readNetFromONNX("./LW_IRST.onnx")
    __net.setInput(img)

    end = time.perf_counter()

    return 1 / (end - start)




# @st.cache_resource(show_spinner=False)
def get_evaluate_info(
    # __net: Any,
    __metrics_PixelPrecisionRecallF1IoU: Any,
    __metric_PixelROCPrecisionRecall: Any,
    __metric_PixelNormalizedIoU: Any,
    __metric_HybridNormalizedIoU: Any,
    __metric_TargetPrecisionRecallF1: Any,
    __metric_TargetAveragePrecision: Any,
    __metric_TargetPdPixelFa: Any,
    __metric_TargetPdPixelFaROC: Any,
    __metric_mNoCoAP: Any,
    __metric_FPS,
    *,
    base_size: int,
) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """获取评估数据

    Args:
        __metrics (Any): _description_
        __metric_roc (Any): _description_
        base_size (int): _description_

    Returns:
        Tuple[pd.DataFrame, pd.DataFrame]: 评估数据的元组，包含两组数据：

            1. 包含两列分别是 key 和对应的 value
            2. 用于绘图的数据
    """

    # 计算像素级检测精度、召回率、F1指标和IoU
    Pixel_precision, Pixel_recall, Pixel_f1, Pixel_iou = __metrics_PixelPrecisionRecallF1IoU.get()


    # 计算像素级ROC曲线_AUC值、PR曲线_AP值
    auc_roc, auc_pr, Pixel_fa_list, Pixel_pd_list, Pixel_precision_list, Pixel_recall_list, _ = __metric_PixelROCPrecisionRecall.get()
    print("——————————————————————Pixel_fa_list——————————————————————")
    print(Pixel_fa_list)
    print("——————————————————————Pixel_pd_list——————————————————————")
    print(Pixel_pd_list)
    print("——————————————————————Pixel_precision_list——————————————————————")
    print(Pixel_precision_list)
    print("——————————————————————Pixel_recall_list——————————————————————")
    print(Pixel_recall_list)

    # 计算nIou
    Pixel_nIoU = __metric_PixelNormalizedIoU.get()

    # 计算改进后nIou
    Target_nIoU = __metric_HybridNormalizedIoU.get()

    # 计算目标级检测精度、召回率、F1指标
    Target_precision, Target_recall, Target_f1 = __metric_TargetPrecisionRecallF1.get()

    # 计算目标级PR曲线, AP值
    Target_Precision_list, Target_Recall_list, Target_F1_list, Target_AP = __metric_TargetAveragePrecision.get()

    # 计算某置信度阈值下的目标级Pd，像素级Fa
    Target_Pd, Pixel_Fa = __metric_TargetPdPixelFa.get()

    # 计算目标级Pd，像素级Fa的ROC曲线级AUC值
    Target_Pd_list, Pixel_FA_list, auc_list = __metric_TargetPdPixelFaROC.get()
    # print('------Target_Pd_list-----')
    # print(Target_Pd_list)
    # print('------Pixel_FA_list-----')
    # print(Pixel_FA_list)

    # 计算mNoCoAP
    mNoCoAP  = __metric_mNoCoAP.get()


    flops, params = get_model_complexity()
    # flops = -1
    # params = -1

    # fps = get_model_fps(__net, base_size)
    fps = __metric_FPS.get()

    evaluate_info = pd.DataFrame(
        {
            "key": [
                "Pixel_Precision",
                "Target_precision(dis_Th=3)",
                "Pixel_Recall",
                "Target_recall(dis_Th=3)",
                "Pixel_F1",
                "Target_F1(dis_Th=3)",
                "Target_Pd(dis_Th=3)",
                "Pixel_Fa(dis_Th=3)",
                "Pixel_IoU",
                "Pixel_nIoU",
                "mNoCoAP",
                "Pixel_AUC",
                "Pixel_AP",
                "FLOPs(M)",
                "Params(M)",
                "FPS",
            ],
            "value": [
                '{:.4f}'.format(Pixel_precision[0]),
                Target_precision[2],
                '{:.4f}'.format(Pixel_recall[0]),
                Target_recall[2],
                '{:.4f}'.format(Pixel_f1[0]),
                Target_f1[2],
                Target_Pd[2],
                Pixel_Fa[2],
                '{:.4f}'.format(Pixel_iou[0]),
                Target_nIoU[2],
                mNoCoAP,
                auc_roc,
                auc_pr,
                flops,
                params,
                fps,
            ],
        }
    )

    roc_data = pd.DataFrame(Pixel_pd_list, Pixel_fa_list)
    pr_data = pd.DataFrame(Pixel_precision_list, Pixel_recall_list)

    return evaluate_info, roc_data, pr_data


if __name__ == "__main__":
    args = parse_args()

    # set device
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    # load checkpoint
    print("...load checkpoint: %s" % args.pkl_path)
    net = torch.load(args.pkl_path, map_location=device)
    net.eval()

    # define dataset
    if args.dataset == "sirstaug":
        dataset = utils_data.SirstAugDataset(mode="test")
    elif args.dataset == "sirst_427":
        dataset = utils_data.SirstDataset_427(mode="test")
    elif args.dataset == "NUDT_SirstDataset_1327":
        dataset = utils_data.NUDT_SirstDataset_1327(mode="test")
    elif args.dataset == "mdfa":
        dataset = utils_data.MDFADataset(mode="test",base_size=args.base_size)
    elif args.dataset == "merged":
        dataset = utils_data.MergedDataset(mode="test", base_size=args.base_size)
    elif args.dataset == "Dense_infrared_small_targets":
        dataset = utils_data.Dense_infrared_small_targets(mode="test")
    elif args.dataset == "Visible_light_drone":
        dataset = utils_data.Visible_light_drone(mode="test")
    elif args.dataset == "Visible_light_aircraft":
        dataset = utils_data.Visible_light_aircraft(mode="test")
    elif args.dataset == "Visible_light_ships":
        dataset = utils_data.Visible_light_ships(mode="test")
    elif args.dataset == "SAR_ships":
        dataset = utils_data.SAR_ships(mode="test")
    elif args.dataset == "Multimodal":
        dataset = utils_data.Multimodal(mode="test")
    else:
        raise NotImplementedError
    data_loader = Data.DataLoader(dataset, batch_size=args.batch_size, shuffle=False)



    # metrics
    metric_PixelPrecisionRecallF1IoU = PixelPrecisionRecallF1IoU(conf_thr = args.conf_thresh, debug=False)
    metric_PixelROCPrecisionRecall = PixelROCPrecisionRecall(conf_thrs = 10, debug=False)
    metric_PixelNormalizedIoU = PixelNormalizedIoU(conf_thr = args.conf_thresh, debug=False)
    metric_HybridNormalizedIoU = HybridNormalizedIoU(conf_thr = args.conf_thresh, dilate_kernel= 0, second_match='plus_mask', debug=False)
    metric_TargetPrecisionRecallF1 = TargetPrecisionRecallF1(conf_thr = args.conf_thresh, dilate_kernel= 0, second_match='plus_mask', debug=False)
    metric_TargetAveragePrecision = TargetAveragePrecision(conf_thrs = 10, dilate_kernel= 0, second_match='plus_mask', debug=False)
    metric_TargetPdPixelFa = TargetPdPixelFa(conf_thr=args.conf_thresh, dilate_kernel= 0,  second_match='plus_mask', debug=False)
    metric_TargetPdPixelFaROC = TargetPdPixelFaROC(conf_thrs = 10, dilate_kernel= 0,  second_match='plus_mask', debug=False)
    metric_mNoCoAP = mNoCoAP(conf_thr=args.conf_thresh)

    # evaluation
    tbar = tqdm(data_loader)
    for i, (data, labels) in enumerate(tbar):
        with torch.no_grad():
            data = data.to(device)
            labels = labels.to(device)
            output = net(data)  # (16,1,256,256)

        metric_PixelPrecisionRecallF1IoU.update(labels=labels, preds=torch.sigmoid(output))
        metric_PixelROCPrecisionRecall.update(labels=labels, preds=torch.sigmoid(output))
        metric_PixelNormalizedIoU.update(labels=labels, preds=torch.sigmoid(output))
        metric_HybridNormalizedIoU.update(labels=labels, preds=torch.sigmoid(output))
        metric_TargetPrecisionRecallF1.update(labels=labels, preds=torch.sigmoid(output))
        metric_TargetAveragePrecision.update(labels=labels, preds=torch.sigmoid(output))
        metric_TargetPdPixelFa.update(labels=labels, preds=torch.sigmoid(output))
        metric_TargetPdPixelFaROC.update(labels=labels, preds=torch.sigmoid(output))
        metric_mNoCoAP.update(labels=labels, preds=output, batch=data)

    print("——————————————————————计算像素级检测精度、召回率、F1指标和IoU——————————————————————")
    Pixel_precision, Pixel_recall, Pixel_f1, iou = metric_PixelPrecisionRecallF1IoU.get()

    print("——————————————————————计算像素级ROC曲线_AUC值、PR曲线_AP值——————————————————————")
    _,  _, Pixel_fpr_list, Pixel_tpr_list, Pixel_precision_list, Pixel_recall_list, _ = metric_PixelROCPrecisionRecall.get()
    # 计算 像素级ROC
    plt.plot(Pixel_fpr_list, Pixel_tpr_list)
    # print(fpr_list)
    # print(tpr_list)
    plt.axis([0, 1, 0, 1])
    plt.xlabel('Fa')
    plt.ylabel('Pd')
    plt.show()

    # 计算 像素级PR
    plt.plot(Pixel_recall_list, Pixel_precision_list)
    # print(precision_list)
    # print(recall_list)
    plt.axis([0, 1, 0, 1])
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.show()

    print("——————————————————————计算nIou——————————————————————")
    nIoU = metric_PixelNormalizedIoU.get()

    print("——————————————————————计算改进后nIou——————————————————————")
    target_niou = metric_HybridNormalizedIoU.get()

    print("——————————————————————计算目标级检测精度、召回率、F1指标——————————————————————")
    Target_precision, Target_recall, Target_f1 = metric_TargetPrecisionRecallF1.get()

    print("——————————————————————计算目标级PR曲线, AP值——————————————————————")
    Target_Precision_list, Target_Recall_list, Target_F1_list, Target_AP = metric_TargetAveragePrecision.get()
    # print("Target_Precision_list=", Target_Precision_list)
    # print("Target_Recall_list=", Target_Recall_list)

    print("——————————————————————计算某置信度阈值下的目标级Pd，像素级Fa——————————————————————")
    Target_Pd, Pixel_Fa = metric_TargetPdPixelFa.get()

    print("——————————————————————计算目标级Pd，像素级Fa的ROC曲线级AUC值——————————————————————")
    Target_Pd_list, Pixel_FA_list, auc_list = metric_TargetPdPixelFaROC.get()

    print("——————————————————————计算mNoCoAP——————————————————————")
    mnocoap, mean_nocoaps = metric_mNoCoAP.get()
    print(mean_nocoaps)

    print("——————————————————————计算模型复杂度——————————————————————")
    inputs = torch.randn((1, 3, 256, 256))
    models = LW_IRST_ablation(
        channel=(8, 32, 64),
        dilations=(2, 4, 8, 16),
        kernel_size=(7, 7, 7, 7),
        padding=(3, 3, 3, 3),
    )
    out = models(inputs)
    FLOPs, params, *_ = profile(models, inputs=(inputs,))
    print("FLOPs=", str(FLOPs / 1000000.0) + "{}".format("M"))
    print("params=", str(params / 1000000.0) + "{}".format("M"))

    # 计算onnx模型的FPS
    print("——————————————————————计算模型FPS——————————————————————")
    # load image
    base_size = 256
    img = cv2.imread("./test.png", 1)
    img = resize_image(img, base_size)  # type: ignore
    input_ = preprocess_image(img)
    start = time.perf_counter()
    net = cv2.dnn.readNetFromONNX("./LW_IRST.onnx")
    # load network
    net.setInput(img)
    end = time.perf_counter()
    running_FPS = 1 / (end - start)
    print("running_FPS:", running_FPS)



