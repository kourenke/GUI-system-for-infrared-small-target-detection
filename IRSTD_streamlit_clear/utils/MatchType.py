from enum import Enum

__all__ = [
    "none",
    "mask",
    "bbox",
    "plus_mask",
    "plus_bbox",
]


class MatchType(Enum):
    none = 0
    mask = 1
    bbox = 2
    plus_mask = 3
    plus_bbox = 4

class none():
    none = 'none'
class mask():
    mask = 'mask'
class bbox():
    bbox = 'bbox'
class plus_mask():
    plus_mask = 'plus_mask'
class plus_bbox():
    plus_bbox = 'plus_bbox'