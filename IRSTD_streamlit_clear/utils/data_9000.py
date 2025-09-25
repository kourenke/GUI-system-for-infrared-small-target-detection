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

__all__ = ["Lightweight_9000",]

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