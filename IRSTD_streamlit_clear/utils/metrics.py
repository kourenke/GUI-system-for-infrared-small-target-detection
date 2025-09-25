import threading
import numpy as np
import torch
import cv2
from scipy.spatial.distance import cdist
from typing import Union, List
from prettytable import PrettyTable
from skimage import measure
import pandas as pd
from sklearn.metrics import average_precision_score, precision_recall_curve, roc_auc_score, roc_curve, auc, f1_score
from prettytable import PrettyTable
import numpy as np
import pandas as pd

__all__ = ['SegmentationMetric', 'SegmentationMetricTPFNFP', 'ROCMetric', 'PRMetric', 'BinaryCenterMetric_old', 'PD_FA', 'ROC_PR_Metric']


def accuracy(output, target, topk=(1,)):
    """Computes the accuracy over the k top predictions for the specified values of k"""
    with torch.no_grad():
        maxk = max(topk)
        batch_size = target.size(0)

        _, pred = output.topk(maxk, 1, True, True)
        pred = pred.t()
        correct = pred.eq(target.view(1, -1).expand_as(pred))

        res = []
        for k in topk:
            correct_k = correct[:k].view(-1).float().sum(0, keepdim=True)
            res.append(correct_k.mul_(100.0 / batch_size))
        return res


def get_pixacc_miou(total_correct, total_label, total_inter, total_union):
    pixAcc = 1.0 * total_correct / (np.spacing(1) + total_label)
    IoU = 1.0 * total_inter / (np.spacing(1) + total_union)
    mIoU = IoU.mean()
    return pixAcc, mIoU


def get_miou_prec_recall_fscore(total_tp, total_fp, total_fn):
    miou = 1.0 * total_tp / (np.spacing(1) + total_tp + total_fp + total_fn)
    prec = 1.0 * total_tp / (np.spacing(1) + total_tp + total_fp)
    recall = 1.0 * total_tp / (np.spacing(1) + total_tp + total_fn)
    fscore = 2.0 * prec * recall / (np.spacing(1) + prec + recall)
    return miou, prec, recall, fscore


def batch_pix_accuracy(output, target, conf_thresh=0.5):
    """Batch Pixel Accuracy
    Args:
        predict: input 4D tensor
        target: label 3D tensor
    """
    output = output.detach().cpu().numpy()
    target = target.detach().cpu().numpy()

    predict = (output > conf_thresh).astype('int64')  # P
    pixel_labeled = np.sum(target > 0)  # T
    pixel_correct = np.sum((predict == target) * (target > 0))  # TP
    assert pixel_correct <= pixel_labeled
    return pixel_correct, pixel_labeled


def batch_intersection_union(output, target, nclass, conf_thresh=0.5):
    """Batch Intersection of Union
    Args:
        predict: input 4D tensor
        target: label 3D tensor
        nclass: number of categories (int)
    """
    mini = 1
    maxi = nclass
    nbins = nclass

    predict = (output.detach().cpu().numpy() > conf_thresh).astype('int64')  # P
    target = target.cpu().numpy().astype('int64')  # T
    intersection = predict * (predict == target)  # TP

    # areas of intersection and union
    area_inter, _ = np.histogram(intersection, bins=nbins, range=(mini, maxi))
    area_pred, _ = np.histogram(predict, bins=nbins, range=(mini, maxi))
    area_lab, _ = np.histogram(target, bins=nbins, range=(mini, maxi))
    area_union = area_pred + area_lab - area_inter
    assert (area_inter <= area_union).all()
    return area_inter, area_union


def batch_tp_fp_fn(output, target, conf_thresh=0.5):
    """Batch Intersection of Union
    Args:
        predict: input 4D tensor
        target: label 3D tensor
        nclass: number of categories (int)
    """

    mini = 1
    nclass = 1
    maxi = nclass
    nbins = nclass


    predict = (output.detach().cpu().numpy() > conf_thresh).astype('int64')  # P  #(16,1,256,256)
    # predict = (output.detach().numpy() > 0).astype('int64')  # P


    target = target.cpu().numpy().astype('int64')  # T
    # target = target.numpy().astype('int64')  # T
    intersection = predict * (predict == target)  # TP

    # areas of intersection and union
    area_inter, _ = np.histogram(intersection, bins=nbins, range=(mini, maxi))
    area_pred, _ = np.histogram(predict, bins=nbins, range=(mini, maxi))
    area_lab, _ = np.histogram(target, bins=nbins, range=(mini, maxi))

    # areas of TN FP FN
    area_tp = area_inter[0]
    area_fp = area_pred[0] - area_inter[0]
    area_fn = area_lab[0] - area_inter[0]

    # area_union = area_pred + area_lab - area_inter
    assert area_tp <= (area_tp + area_fn + area_fp)
    return area_tp, area_fp, area_fn


def pixel_accuracy(im_pred, im_lab):
    im_pred = np.asarray(im_pred)
    im_lab = np.asarray(im_lab)

    # Remove classes from unlabeled pixels in gt image.
    # We should not penalize detections in unlabeled portions of the image.
    pixel_labeled = np.sum(im_lab > 0)
    pixel_correct = np.sum((im_pred == im_lab) * (im_lab > 0))
    # pixel_accuracy = 1.0 * pixel_correct / pixel_labeled
    return pixel_correct, pixel_labeled


def intersection_and_union(im_pred, im_lab, num_class):
    im_pred = np.asarray(im_pred)
    im_lab = np.asarray(im_lab)
    # Remove classes from unlabeled pixels in gt image.
    im_pred = im_pred * (im_lab > 0)
    # Compute area intersection:
    intersection = im_pred * (im_pred == im_lab)
    area_inter, _ = np.histogram(intersection, bins=num_class - 1,
                                 range=(1, num_class - 1))
    # Compute area union:
    area_pred, _ = np.histogram(im_pred, bins=num_class - 1,
                                range=(1, num_class - 1))
    area_lab, _ = np.histogram(im_lab, bins=num_class - 1,
                               range=(1, num_class - 1))
    area_union = area_pred + area_lab - area_inter
    return area_inter, area_union


def cal_tp_pos_fp_neg(output, target, nclass, conf_thresh):
    mini = 1
    maxi = 1 # nclass
    nbins = 1 # nclass

    predict = (output.detach().cpu().numpy() > conf_thresh).astype('float32') # P
    target = target.detach().cpu().numpy().astype('int64')  # T
    # target = target.detach().numpy().astype('int64')  # T
    intersection = predict * (predict == target) # TP
    tp = intersection.sum()
    fp = (predict * (predict != target)).sum()  # FP
    tn = ((1 - predict) * (predict == target)).sum()  # TN
    fn = ((predict != target) * (1 - predict)).sum()   # FN
    pos = tp + fn  #实际正样本的总数
    neg = fp + tn  #实际为负例的样本总数
    return tp, pos, fp, neg, fn, tn


class ROCMetric():
    def __init__(self, nclass, bins):
        self.nclass = nclass
        self.bins = bins
        self.reset()

    def update(self, preds, labels):
        for iBin in range(self.bins+1):
            conf_thresh = (iBin + 0.0) / self.bins
            i_tp, i_pos, i_fp, i_neg, i_fn, i_tn = cal_tp_pos_fp_neg(preds, labels, self.nclass, conf_thresh)

            self.tp_arr[iBin] += i_tp
            self.pos_arr[iBin] += i_pos #实际正样本的总数
            self.fp_arr[iBin] += i_fp
            self.neg_arr[iBin] += i_neg #实际为负例的样本数
            self.fn_arr[iBin] += i_fn
            self.tn_arr[iBin] += i_tn #实际为负例的样本数


    def get(self):
        tp_rates = self.tp_arr / (self.pos_arr + 0.001)   # 真正例率（TPR）也被称为召回率（Recall）或灵敏度（Sensitivity）
        fp_rates = self.fp_arr / (self.neg_arr + 0.001)   # 假正例率（FPR）也被称为误报率(Fall-out),FPR =FP/(FP+TN),它计算的是在所有实际为负例的样本中，被模型错误预测为正例的比例

        return tp_rates, fp_rates

    def reset(self):
        self.tp_arr = np.zeros(self.bins + 1)
        self.pos_arr = np.zeros(self.bins + 1)
        self.fp_arr = np.zeros(self.bins + 1)
        self.neg_arr = np.zeros(self.bins + 1)
        self.tn_arr = np.zeros(self.bins + 1)
        self.fn_arr = np.zeros(self.bins + 1)


class PRMetric():
    def __init__(self, nclass, bins):
        self.nclass = nclass
        self.bins = bins
        self.reset()

    def update(self, preds, labels):
        for iBin in range(self.bins+1):
            conf_thresh = (iBin + 0.0) / self.bins
            i_tp, i_pos, i_fp, i_neg, i_fn, i_tn  = cal_tp_pos_fp_neg(preds, labels, self.nclass, conf_thresh)

            self.tp_arr[iBin] += i_tp
            self.fp_arr[iBin] += i_fp
            self.fn_arr[iBin] += i_fn

    def get(self):
        precision_rates = self.tp_arr / (self.tp_arr + self.fp_arr + 0.001)
        recall_rates = self.tp_arr / (self.tp_arr + self.fn_arr + 0.001)

        return precision_rates, recall_rates

    def reset(self):
        self.tp_arr = np.zeros(self.bins + 1)
        self.fp_arr = np.zeros(self.bins + 1)
        self.fn_arr = np.zeros(self.bins + 1)


class SegmentationMetricTPFNFP(object):
    """Computes pixAcc and mIoU metric scroes
    """

    def __init__(self, nclass, conf_thresh):
        self.lock = threading.Lock()
        self.conf_thresh = conf_thresh
        self.nclass = 1
        self.reset()

    def update(self, labels, preds):
        def evaluate_worker(self, label, pred):
            tp, fp, fn = batch_tp_fp_fn(pred, label, self.conf_thresh)
            with self.lock:
                self.total_tp += tp
                self.total_fp += fp
                self.total_fn += fn
            return

        if isinstance(preds, torch.Tensor):
            evaluate_worker(self, labels, preds)
        elif isinstance(preds, (list, tuple)):
            threads = [threading.Thread(target=evaluate_worker,
                                        args=(self, label, pred),
                                        )
                       for (label, pred) in zip(labels, preds)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
        else:
            raise NotImplemented

    def get_all(self):
        return self.total_tp, self.total_fp, self.total_fn

    def get(self):
        return get_miou_prec_recall_fscore(self.total_tp, self.total_fp, self.total_fn)

    def reset(self):
        self.total_tp = 0
        self.total_fp = 0
        self.total_fn = 0
        return


class SegmentationMetric(object):
    """Computes pixAcc and mIoU metric scroes,
       PixAcc is equivalent to Recall, that is, pixAcc=TP/T=TP/TP+FN
       Accuracy = (TP + TN) / (TP + FP + FN + TN)
       Accuracy has little significance in tasks with extremely imbalanced positive and negative samples
    """

    def __init__(self, nclass):
        self.nclass = nclass
        self.lock = threading.Lock()
        self.reset()

    def update(self, labels, preds):
        def evaluate_worker(self, label, pred):
            correct, labeled = batch_pix_accuracy(
                pred, label)
            inter, union = batch_intersection_union(
                pred, label, self.nclass)
            with self.lock:
                self.total_correct += correct
                self.total_label += labeled
                self.total_inter += inter
                self.total_union += union
            return

        if isinstance(preds, torch.Tensor):
            evaluate_worker(self, labels, preds)
        elif isinstance(preds, (list, tuple)):
            threads = [threading.Thread(target=evaluate_worker,
                                        args=(self, label, pred),
                                        )
                       for (label, pred) in zip(labels, preds)]
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
        else:
            raise NotImplemented

    def get_all(self):
        return self.total_correct, self.total_label, self.total_inter, self.total_union

    def get(self):
        return get_pixacc_miou(self.total_correct, self.total_label, self.total_inter, self.total_union)

    def reset(self):
        self.total_inter = 0
        self.total_union = 0
        self.total_correct = 0
        self.total_label = 0
        return


class PD_FA():
    def __init__(self,
                 thresholds: List[int] = [1,10],
                 conf_thresh=0.5
                 ):
        super(PD_FA, self).__init__()
        self.thresholds = np.arange(thresholds[0], thresholds[1] + 1)
        self.conf_thresh = conf_thresh
        self.lock = threading.Lock()
        self.reset()


    def update(self, labels: Union[torch.tensor, np.array], preds: Union[torch.tensor, np.array]):
        # bchw
        def evaluate_worker(self, label, pred):

            size = label.shape
            # image_area_total = []

            for idx, threshold in enumerate(self.thresholds):
                # print(f"{idx} , {threshold}")
                # print(idx)
                image = measure.label(pred, connectivity=2)
                coord_image = measure.regionprops(image)
                label = measure.label(label, connectivity=2)
                coord_label = measure.regionprops(label)
                distance_match = []
                true_img = np.zeros(pred.shape)
                for i in range(len(coord_label)):
                    centroid_label = np.array(list(coord_label[i].centroid))
                    for m in range(len(coord_image)):
                        centroid_image = np.array(list(coord_image[m].centroid))
                        distance = np.linalg.norm(centroid_image - centroid_label)
                        # area_image = np.array(coord_image[m].area)

                        if distance < threshold:
                            distance_match.append(distance)
                            true_img[coord_image[m].coords[:, 0], coord_image[m].coords[:, 1]] = 1
                            del coord_image[m]
                            break
                with self.lock:
                    self.target[idx] += len(coord_label)
                    self.dismatch_pixel[idx] += (pred - true_img).sum()
                    self.all_pixel[idx] += int(size[0] * size[1])
                    self.PD[idx] += len(distance_match)


        if isinstance(preds, torch.Tensor):
            preds = preds.cpu().numpy()
            labels = labels.cpu().numpy()

        labels = labels > 0   # sometimes is 0-1, force.
        labels = labels.astype('int64')
        preds = preds > self.conf_thresh   # sometimes is 0-1, force.
        preds = preds.astype('int64')
        # preds = preds.astype('int64')
        # labels = labels.astype('int64')

        # for i in range(len(labels)):
        #     evaluate_worker(labels[i].squeeze(0), preds[i].squeeze(0))
        threads = [threading.Thread(target=evaluate_worker,
                                    args=(self, labels[i].squeeze(0), preds[i].squeeze(0)),
                                    )
                   for i in range(len(labels))]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

    def get(self):
        Final_FA = self.dismatch_pixel / self.all_pixel
        Final_PD = self.PD / self.target
        head = ["Threshold"]
        head.extend(self.thresholds.tolist())
        _round = 5
        table = PrettyTable()
        table.add_column("Threshold", self.thresholds)
        table.add_column("TD", ['{:.5f}'.format(num) for num in np.around(self.PD, _round)])
        table.add_column("AT", ['{:.5f}'.format(num) for num in np.around(self.target, _round)])
        table.add_column("FD", ['{:.5f}'.format(num) for num in np.around(self.dismatch_pixel, _round)])
        table.add_column("NP", ['{:.5f}'.format(num) for num in np.around(self.all_pixel, _round)])
        table.add_column("target_Pd", ['{:.5f}'.format(num) for num in np.around(Final_PD, _round)])
        table.add_column("pixel_Fa", ['{:.5f}'.format(num) for num in np.around(Final_FA, _round)])
        print(table)

        TD = self.PD
        AT = self.target
        FD = self.dismatch_pixel
        NP = self.all_pixel
        all_metric = np.stack([self.thresholds, TD, AT, FD, NP, Final_PD, Final_FA], axis=1)
        df_pd_fa = pd.DataFrame(all_metric)
        df_pd_fa.columns = ['Thresholds','TD', 'AT', 'FD', 'NP', 'Pd', 'Fa']
        self.metrices_table= df_pd_fa


        return Final_PD, Final_FA

    def reset(self):

        self.FA = np.zeros_like(self.thresholds)
        self.PD = np.zeros_like(self.thresholds)
        self.dismatch_pixel = np.zeros_like(self.thresholds)
        self.all_pixel = np.zeros_like(self.thresholds)
        self.PD = np.zeros_like(self.thresholds)
        self.target = np.zeros_like(self.thresholds)


class BinaryCenterMetric_old():
    def __init__(self,
                 thresholds: List[int] = [1, 10],
                 dilate_kernel_size: List[int] = [7, 7],
                 conf_thresh = 0.5,
                 debug: bool = False
                 ):
        self.thresholds = np.arange(thresholds[0], thresholds[1] + 1)
        self.conf_thresh = conf_thresh
        self.dilate_kernel_size = dilate_kernel_size
        self.debug = debug
        self.lock = threading.Lock()
        self.reset()

    def update(self, labels: Union[np.array, torch.tensor], preds: Union[np.array, torch.tensor]):
        """_summary_

        Args:
            labels (Union[np.array, torch.tensor]): 0-255 or 0-1, bchw, bhwc, hwc, chw
            preds (Union[np.array, torch.tensor]): _description_
        """

        def evaluate_worker(self, label, pred):
            label = label > 0   # sometimes is 0-1, force.
            label = label.astype(np.uint8) * 255
            pred = pred > self.conf_thresh   # sometimes is 0-1, force.
            pred = pred.astype(np.uint8) * 255
            gt_centroids = np.array(self._find_centroids(label, mode='gt'))
            pred_centroids = np.array(self._find_centroids(pred, mode='pred'))
            for idx, threshold in enumerate(self.thresholds):
                TP, FN, FP = self._calculate_tp_fn_fp(gt_centroids, pred_centroids, threshold)
                with self.lock:
                    self.TP[idx] += TP
                    self.FP[idx] += FP
                    self.FN[idx] += FN
            return

        if isinstance(labels, torch.Tensor):
            labels = labels.detach().cpu().numpy()
            preds = preds.detach().cpu().numpy()

        if labels.ndim == 3:
            # hwc/chw -> bhwc/bchw
            labels = labels[np.newaxis, ...]
            preds = preds[np.newaxis, ...]

        if labels.shape[-1] != 1 and labels.shape[-1] != 3:
            # bchw -> bhwc
            labels = labels.transpose((0, 2, 3, 1))
            preds = preds.transpose((0, 2, 3, 1))

        if labels.shape[:-1] != preds.shape[:-1]:
            print(f"gt_img and pred_img must have same height and width, but got {labels.shape}, {preds.shape}")
            return None

        # for i in range(len(labels)):
        #     evaluate_worker(labels[i], preds[i])
        threads = [threading.Thread(target=evaluate_worker,
                                    args=(self, labels[i], preds[i]),
                                    )
                   for i in range(len(labels))]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

        if self.debug:
            print(f"---------------- 阈值为:{self.thresholds}")
            print(f"GT为正例，预测为正例(TP): {self.TP}")
            print(f"GT为正例，预测为负例(FN): {self.FN}")
            print(f"GT为负例，预测为正例(FP): {self.FP}")

    def get(self):
        """Compute metric

        Returns:
            _type_: Precision, Recall, micro-F1, shape == [1, num_threshold].
        """
        self.Precision = self.TP / (self.TP + self.FP)
        self.Recall = self.TP / (self.TP + self.FN)
        # micro F1 socre.
        self.F1 = 2 * self.Precision * self.Recall / (self.Precision + self.Recall)

        head = ["Threshold"]
        head.extend(self.thresholds.tolist())
        _round = 5
        table = PrettyTable()
        table.add_column("Threshold", self.thresholds)
        table.add_column("TP", ['{:.5f}'.format(num) for num in np.around(self.TP, _round)])
        table.add_column("FP", ['{:.5f}'.format(num) for num in np.around(self.FP, _round)])
        table.add_column("FN", ['{:.5f}'.format(num) for num in np.around(self.FN, _round)])
        table.add_column("target_Precision", ['{:.5f}'.format(num) for num in np.around(self.Precision, _round)])
        table.add_column("target_Recall", ['{:.5f}'.format(num) for num in np.around(self.Recall, _round)])
        table.add_column("target_F1score", ['{:.5f}'.format(num) for num in np.around(self.F1, _round)])
        print(table)

        TP = self.TP
        FP = self.FP
        FN = self.FN
        Re = self.Recall
        Pre = self.Precision
        F1 = self.F1
        all_metric = np.stack([self.thresholds, TP, FP, FN, Pre, Re, F1], axis=1)
        df = pd.DataFrame(all_metric)
        df.columns = ['Thresholds','TP', 'FP', 'FN', 'Precision', 'Recall', 'F1']
        self.metrices_table= df

        return self.Precision.tolist(), self.Recall.tolist(), self.F1.tolist()

    def reset(self):
        self.TP = np.zeros_like(self.thresholds)
        self.FP = np.zeros_like(self.thresholds)
        self.FN = np.zeros_like(self.thresholds)
        self.Precision = np.zeros_like(self.thresholds)
        self.Recall = np.zeros_like(self.thresholds)
        self.F1 = np.zeros_like(self.thresholds)
        self.TP = np.zeros_like(self.thresholds)
        self.FP = np.zeros_like(self.thresholds)
        self.FN = np.zeros_like(self.thresholds)

    def _find_centroids(self, image: str, mode: str = 'gt'):

        # 检查通道数
        channels = image.shape[2]

        if channels == 3:
            # 图像是3通道的，转换为单通道灰度图像
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # 在这里可以使用gray_image作为单通道图像
            # print("Converted to grayscale image.")
        else:
            # 图像已经是单通道的，保持不变
            gray_image = image
            # print("Image is already grayscale or not a color image.")

        # 如果图片不是二值图像，你需要先将其转换为二值图像
        # 例如，使用阈值操作：

        # _, thresh = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        new_image = gray_image
        if mode == 'pred':
            kernel = np.ones(self.dilate_kernel_size, np.uint8)
            # 对图像进行膨胀操作
            dilated_image = cv2.dilate(gray_image, kernel, iterations=1)
            new_image = dilated_image.copy()

        # 找到图像中的连通区域（轮廓）
        contours, _ = cv2.findContours(new_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centroids = []

        for contour in contours:
            # 计算每个轮廓的矩
            M = cv2.moments(contour)

            # 如果矩存在（即轮廓非空），则计算质心
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centroids.append((cX, cY))

        return centroids

    def _calculate_tp_fn_fp(self, gt_centroids: np.array, pred_centroids: np.array, threshold: int):
        """_summary_

        Args:
            gt_centroids (np.array): shape = (N,2)
            pred_centroids (np.array): shape = (M,2)
            threshold (int): threshold, Euclidean Distance.

        Returns:
            _type_: _description_
        """
        if gt_centroids.shape[0] == 0 and pred_centroids.shape[0] == 0:
            TP = 0
            FN = 0
            FP = 0
        elif gt_centroids.shape[0] == 0 and pred_centroids.shape[0] != 0:
            TP = 0
            FN = 0
            FP = pred_centroids.shape[0]
        elif gt_centroids.shape[0] != 0 and pred_centroids.shape[0] == 0:
            TP = 0
            FN = gt_centroids.shape[0]
            FP = 0
        else:
            # 使用scipy的cdist函数来计算所有真实标签点和预测点之间的欧式距离
            distances = cdist(gt_centroids, pred_centroids)

            # 找到每个预测点距离最近的真实标签点
            min_distances = np.min(distances, axis=0)

            # 计数预测正确的点的数量
            TP = np.sum(min_distances < threshold)
            FN = gt_centroids.shape[0] - TP
            FP = pred_centroids.shape[0] - TP

        return TP, FN, FP


class ROC_PR_Metric():
    def __init__(self):
        self.reset()

    def update(self, labels: np.array, preds: np.array):
        # labels, preds = convert2iterable(labels, preds) # -> [np.array, ...]

        # labels = np.concatenate([lbl.flatten() for lbl in labels])
        # preds = np.concatenate([pred.flatten() for pred in preds])
        labels = labels.flatten()
        preds = preds.flatten()
        tpr, fpr, ROC_AUC, Pre, Rec, PR_AUC = self._skl_roc_auc_pr(labels, preds)
        self.fpr = fpr
        self.tpr = tpr
        self.pre = Pre
        self.rec = Rec
        self.PR_AUC = PR_AUC
        self.ROC_AUC = ROC_AUC

    def _skl_roc_auc_pr(self, labels: np.array, preds: np.array):
        predict = preds.astype('float32')  # P
        target = labels.astype('int64')  # T
        fpr, tpr, _ = roc_curve(target, predict)
        ROC_AUC = auc(fpr, tpr)
        Pre, Rec, _ = precision_recall_curve(target, predict)
        PR_AUC = auc(Rec, Pre)
        return tpr, fpr, ROC_AUC, Pre, Rec, PR_AUC

    def get(self):
        head = ["ROC_AUC", "PR_AUC"]
        table = PrettyTable(head)
        # table.add_row(head)
        table.add_row([self.ROC_AUC, self.PR_AUC])
        print(table)
        return self.tpr, self.fpr, self.ROC_AUC, self.pre, self.rec, self.PR_AUC

    @property
    def table(self):
        all_metric = np.stack([self.ROC_AUC, self.PR_AUC])[:, np.newaxis].T
        df = pd.DataFrame(all_metric)
        df.columns = ['ROC_AUC', 'PR_AUC']
        return df

    def reset(self):
        pass

