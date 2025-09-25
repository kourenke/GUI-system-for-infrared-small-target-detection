
from enum import Enum

import torch
import torch.nn as nn
import torch.utils.data as Data
import torchvision.transforms as transforms

from PIL import Image, ImageChops, ImageOps, ImageFilter, ImageFile

ImageFile.LOAD_TRUNCATED_IMAGES = True
import os
import os.path as osp
import sys
import random

__all__ = [
    "SirstAugDataset",
    "SirstDataset_427",
    "MDFADataset",
    "NUDT_SirstDataset_1327",
    "MergedDataset",
    "Dense_infrared_small_targets",
    "Visible_light_drone",
    "Visible_light_aircraft",
    "Visible_light_ships",
    "SAR_ships",
    "Multimodal",
]


class DatasetType(Enum):
    MDFADataset = 0
    SirstAugDataset = 1
    SirstDataset_427 = 2
    NUDT_SirstDataset_1327 = 3
    MergedDataset = 4
    Dense_infrared_small_targets = 5
    Visible_light_drone = 6
    Visible_light_aircraft = 7
    Visible_light_ships = 8
    SAR_ships = 9
    Multimodal = 10

class MergedDataset(Data.Dataset):
    def __init__(
        self,
        mdfa_base_dir="./Datasets/MDFA",
        sirstaug_base_dir="./Datasets/sirst_aug",
        SirstDataset_427_base_dir="./Datasets/sirst_427",
        NUDT_SirstDataset_1327_base_dir="./Datasets/NUDT-SIRST_1327",
        mode="train",
        base_size=256,
    ):
        assert mode in ["train", "test"]

        self.sirstaug = SirstAugDataset(base_dir=sirstaug_base_dir, mode=mode)
        self.mdfa = MDFADataset(base_dir=mdfa_base_dir, mode=mode, base_size=base_size)
        self.SirstDataset_427 = SirstDataset_427(
            base_dir=SirstDataset_427_base_dir, mode=mode, base_size=base_size
        )
        self.NUDT_SirstDataset_1327 = NUDT_SirstDataset_1327(
            base_dir=NUDT_SirstDataset_1327_base_dir, mode=mode
        )

    def __getitem__(self, i):
        a = self.mdfa.__len__()
        b = self.mdfa.__len__() + self.sirstaug.__len__()
        c = (
            self.mdfa.__len__()
            + self.sirstaug.__len__()
            + self.SirstDataset_427.__len__()
        )
        d = (
            self.mdfa.__len__()
            + self.sirstaug.__len__()
            + self.SirstDataset_427.__len__()
            + self.NUDT_SirstDataset_1327.__len__()
        )
        if i <= a:
            return self.mdfa.__getitem__(i)
        elif a < i <= b:
            inx1 = b - i
            return self.sirstaug.__getitem__(inx1)
        elif b < i <= c:
            inx2 = c - i
            return self.SirstDataset_427.__getitem__(inx2)
        elif c < i <= d:
            inx3 = d - i
            return self.NUDT_SirstDataset_1327.__getitem__(inx3)

    def __len__(self):
        return (
            self.mdfa.__len__()
            + self.sirstaug.__len__()
            + self.SirstDataset_427.__len__()
            + self.NUDT_SirstDataset_1327.__len__()
        )


class MDFADataset(Data.Dataset):
    def __init__(self, base_dir="./Datasets/MDFA", mode="test", base_size=256):
        assert mode in ["train", "test"]

        self.mode = mode
        if mode == "train":
            self.img_dir = osp.join(base_dir, "training")
            self.mask_dir = osp.join(base_dir, "training")
        elif mode == "test":
            self.img_dir = osp.join(base_dir, "test_org")
            self.mask_dir = osp.join(base_dir, "test_gt")
        else:
            raise NotImplementedError

        self.img_transform = transforms.Compose(
            [
                transforms.Resize((base_size, base_size), interpolation=Image.BILINEAR),
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )
        self.mask_transform = transforms.Compose(
            [
                transforms.Resize((base_size, base_size), interpolation=Image.NEAREST),
                transforms.ToTensor(),
            ]
        )

    def __getitem__(self, i):
        if self.mode == "train":
            img_path = osp.join(self.img_dir, "%06d_1.png" % i)
            mask_path = osp.join(self.mask_dir, "%06d_2.png" % i)
        elif self.mode == "test":
            img_path = osp.join(self.img_dir, "%05d.png" % i)
            mask_path = osp.join(self.mask_dir, "%05d.png" % i)
        else:
            raise NotImplementedError

        img = Image.open(img_path).convert("RGB")
        img = ImageChops.invert(img)
        mask = Image.open(mask_path).convert("L")

        img, mask = self.img_transform(img), self.mask_transform(mask)
        return img, mask

    def __len__(self):
        if self.mode == "train":
            return 9977
        elif self.mode == "test":
            return 99
        else:
            raise NotImplementedError


class SirstAugDataset(Data.Dataset):
    def __init__(self, base_dir="./Datasets/sirst_aug", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)


class SirstDataset_427(Data.Dataset):
    def __init__(self, base_dir="./Datasets/sirst_427", mode="train", base_size=256):
        assert mode in ["train", "test"]
        if mode == "train":
            txtfile = "trainval.txt"
        elif mode == "test":
            txtfile = "test.txt"

        self.list_dir = osp.join(base_dir, "idx_427", txtfile)
        self.imgs_dir = osp.join(base_dir, "images")
        self.label_dir = osp.join(base_dir, "masks")

        self.names = []
        with open(self.list_dir, "r") as f:
            self.names += [line.strip() for line in f.readlines()]

        self.mode = mode
        self.crop_size = base_size  # 480
        self.base_size = base_size  # 512
        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                # transforms.Normalize([.485, .456, .406], [.229, .224, .225]),  # Default mean and std
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.imgs_dir, name + ".png")
        label_path = osp.join(self.label_dir, name + "_pixels0.png")

        img = Image.open(img_path).convert("RGB")
        mask = Image.open(label_path)

        if self.mode == "train":
            img, mask = self._sync_transform(img, mask)
        elif self.mode == "test":
            img, mask = self._testval_sync_transform(img, mask)
        else:
            raise ValueError("Unkown self.mode")

        # img = self.transform(img)
        # print(torch.max(img))
        # print(torch.min(img))

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)

    def _sync_transform(self, img, mask):
        # random mirror
        if random.random() < 0.5:
            img = img.transpose(Image.FLIP_LEFT_RIGHT)
            mask = mask.transpose(Image.FLIP_LEFT_RIGHT)
        crop_size = self.crop_size
        # random scale (short edge)
        long_size = random.randint(int(self.base_size * 0.5), int(self.base_size * 2.0))
        w, h = img.size
        if h > w:
            oh = long_size
            ow = int(1.0 * w * long_size / h + 0.5)
            short_size = ow
        else:
            ow = long_size
            oh = int(1.0 * h * long_size / w + 0.5)
            short_size = oh
        img = img.resize((ow, oh), Image.BILINEAR)
        mask = mask.resize((ow, oh), Image.NEAREST)
        # pad crop
        if short_size < crop_size:
            padh = crop_size - oh if oh < crop_size else 0
            padw = crop_size - ow if ow < crop_size else 0
            img = ImageOps.expand(img, border=(0, 0, padw, padh), fill=0)
            mask = ImageOps.expand(mask, border=(0, 0, padw, padh), fill=0)
        # random crop crop_size
        w, h = img.size
        x1 = random.randint(0, w - crop_size)
        y1 = random.randint(0, h - crop_size)
        img = img.crop((x1, y1, x1 + crop_size, y1 + crop_size))
        mask = mask.crop((x1, y1, x1 + crop_size, y1 + crop_size))
        # gaussian blur as in PSP
        if random.random() < 0.5:
            img = img.filter(ImageFilter.GaussianBlur(radius=random.random()))
        return img, mask

    def _val_sync_transform(self, img, mask):
        outsize = self.crop_size
        short_size = outsize
        w, h = img.size
        if w > h:
            oh = short_size
            ow = int(1.0 * w * oh / h)
        else:
            ow = short_size
            oh = int(1.0 * h * ow / w)
        img = img.resize((ow, oh), Image.BILINEAR)
        mask = mask.resize((ow, oh), Image.NEAREST)
        # center crop
        w, h = img.size
        x1 = int(round((w - outsize) / 2.0))
        y1 = int(round((h - outsize) / 2.0))
        img = img.crop((x1, y1, x1 + outsize, y1 + outsize))
        mask = mask.crop((x1, y1, x1 + outsize, y1 + outsize))

        return img, mask

    def _testval_sync_transform(self, img, mask):
        base_size = self.base_size
        img = img.resize((base_size, base_size), Image.BILINEAR)
        mask = mask.resize((base_size, base_size), Image.NEAREST)

        return img, mask


class NUDT_SirstDataset_1327(Data.Dataset):
    def __init__(self, base_dir="./Datasets/NUDT-SIRST_1327", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)


class Lightweight_9000(Data.Dataset):
    def __init__(self, base_dir="./Lightweight_9000", mode="train", base_size=512):
        assert mode in ["train", "test"]
        if mode == "train":
            txtfile = "train.txt"
        elif mode == "test":
            txtfile = "val.txt"

        self.list_dir = osp.join(base_dir, "idx_9000", txtfile)
        self.imgs_dir = osp.join(base_dir, "images")
        self.label_dir = osp.join(base_dir, "masks")

        self.names = []
        with open(self.list_dir, "r") as f:
            self.names += [line.strip() for line in f.readlines()]

        self.mode = mode
        self.crop_size = base_size  # 480
        self.base_size = base_size  # 512
        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                # transforms.Normalize([.485, .456, .406], [.229, .224, .225]),  # Default mean and std
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.imgs_dir, name + ".png")
        label_path = osp.join(self.label_dir, name + "_pixels0.png")

        img = Image.open(img_path).convert("RGB")
        mask = Image.open(label_path)

        if self.mode == "train":
            img, mask = self._sync_transform(img, mask)
        elif self.mode == "test":
            img, mask = self._testval_sync_transform(img, mask)
        else:
            raise ValueError("Unkown self.mode")

        # img = self.transform(img)
        # print(torch.max(img))
        # print(torch.min(img))
        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)

    def _sync_transform(self, img, mask):
        # random mirror
        if random.random() < 0.5:
            img = img.transpose(Image.FLIP_LEFT_RIGHT)
            mask = mask.transpose(Image.FLIP_LEFT_RIGHT)
        crop_size = self.crop_size
        # random scale (short edge)
        long_size = random.randint(int(self.base_size * 0.5), int(self.base_size * 2.0))
        w, h = img.size
        if h > w:
            oh = long_size
            ow = int(1.0 * w * long_size / h + 0.5)
            short_size = ow
        else:
            ow = long_size
            oh = int(1.0 * h * long_size / w + 0.5)
            short_size = oh
        img = img.resize((ow, oh), Image.BILINEAR)
        mask = mask.resize((ow, oh), Image.NEAREST)
        # pad crop
        if short_size < crop_size:
            padh = crop_size - oh if oh < crop_size else 0
            padw = crop_size - ow if ow < crop_size else 0
            img = ImageOps.expand(img, border=(0, 0, padw, padh), fill=0)
            mask = ImageOps.expand(mask, border=(0, 0, padw, padh), fill=0)
        # random crop crop_size
        w, h = img.size
        x1 = random.randint(0, w - crop_size)
        y1 = random.randint(0, h - crop_size)
        img = img.crop((x1, y1, x1 + crop_size, y1 + crop_size))
        mask = mask.crop((x1, y1, x1 + crop_size, y1 + crop_size))
        # gaussian blur as in PSP
        if random.random() < 0.5:
            img = img.filter(ImageFilter.GaussianBlur(radius=random.random()))
        return img, mask

    def _val_sync_transform(self, img, mask):
        outsize = self.crop_size
        short_size = outsize
        w, h = img.size
        if w > h:
            oh = short_size
            ow = int(1.0 * w * oh / h)
        else:
            ow = short_size
            oh = int(1.0 * h * ow / w)
        img = img.resize((ow, oh), Image.BILINEAR)
        mask = mask.resize((ow, oh), Image.NEAREST)
        # center crop
        w, h = img.size
        x1 = int(round((w - outsize) / 2.0))
        y1 = int(round((h - outsize) / 2.0))
        img = img.crop((x1, y1, x1 + outsize, y1 + outsize))
        mask = mask.crop((x1, y1, x1 + outsize, y1 + outsize))

        return img, mask

    def _testval_sync_transform(self, img, mask):
        base_size = self.base_size
        img = img.resize((base_size, base_size), Image.BILINEAR)
        mask = mask.resize((base_size, base_size), Image.NEAREST)

        return img, mask


class Visible_light_drone(Data.Dataset):
    def __init__(self, base_dir="./Datasets/Visible_light_drone", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)


class Visible_light_aircraft(Data.Dataset):
    def __init__(self, base_dir="./Datasets/Visible_light_aircraft", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)


class Visible_light_ships(Data.Dataset):
    def __init__(self, base_dir="./Datasets/Visible_light_ships", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)


class SAR_ships(Data.Dataset):
    def __init__(self, base_dir="./Datasets/SAR_ships", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images_add_haze")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images_add_haze", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)


class Multimodal(Data.Dataset):
    def __init__(self, base_dir="./Datasets/Multimodal", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)


class Dense_infrared_small_targets(Data.Dataset):
    def __init__(self, base_dir="./Datasets/Dense_infrared_small_targets", mode="train"):
        assert mode in ["train", "test"]

        if mode == "train":
            self.data_dir = osp.join(base_dir, "trainval")
        elif mode == "test":
            self.data_dir = osp.join(base_dir, "test")
        else:
            raise NotImplementedError

        self.names = []
        for filename in os.listdir(osp.join(self.data_dir, "images")):
            if filename.endswith("png"):
                self.names.append(filename)

        self.transform = transforms.Compose(
            [
                transforms.ToTensor(),
                transforms.Normalize(
                    [0.485, 0.456, 0.406], [0.229, 0.224, 0.225]
                ),  # Default mean and std
            ]
        )

    def __getitem__(self, i):
        name = self.names[i]
        img_path = osp.join(self.data_dir, "images", name)
        label_path = osp.join(self.data_dir, "masks", name)

        img = Image.open(img_path).convert("RGB")
        # img = Image.open(img_path).convert('L')
        # img = ImageChops.invert(img)
        mask = Image.open(label_path).convert("L")

        img, mask = self.transform(img), transforms.ToTensor()(mask)
        return img, mask

    def __len__(self):
        return len(self.names)