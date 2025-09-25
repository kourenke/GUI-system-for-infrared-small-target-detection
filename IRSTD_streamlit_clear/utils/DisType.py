from enum import Enum

__all__ = [
    "forloop",
    "hungarian",
]


class DisType(Enum):
    forloop = 0
    hungarian = 1

class forloop():
    forloop = 'forloop'
class hungarian():
    hungarian = 'hungarian'