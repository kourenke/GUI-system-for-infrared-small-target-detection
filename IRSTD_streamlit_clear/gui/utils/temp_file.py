from typing import Optional

from os import remove
from os.path import abspath
from uuid import uuid4
from pathlib import Path
from tempfile import _TemporaryFileWrapper, NamedTemporaryFile


TEMP_FILE_BASE_PATH = abspath("./TEMP/")


def generate_temp_file(suffix: Optional[str] = None) -> _TemporaryFileWrapper:
    """创建新的临时文件（使用 UUID 前缀避免冲突）

    Returns:
        _TemporaryFileWrapper: 临时文件对象
    """
    return NamedTemporaryFile(
        delete=False,
        suffix=f"-{uuid4()}{suffix or ''}",
        dir=TEMP_FILE_BASE_PATH,
    )


def delete_temp_file(temp_file: Optional[_TemporaryFileWrapper]):
    """释放当前使用的临时文件"""
    if temp_file is not None:
        temp_file_name = temp_file.name
        temp_file.close()

        remove(temp_file_name)


def get_temp_file_size(temp_file: _TemporaryFileWrapper) -> str:
    """获取临时文件大小

    Args:
        temp_file (_TemporaryFileWrapper): 临时文件对象

    Returns:
        str: 可读的文件大小字符串
    """
    file_size = Path(temp_file.name).stat().st_size
    return __convert_file_size(file_size)


def __convert_file_size(file_size: float) -> str:
    """将 Byte 自适应转换为其他易读的单位 (B, KB, MB, GB,...)

    Args:
        file_size (float): 文件大小，单位字节 (Byte)

    Returns:
        str: 转换后的字符串
    """
    units = ("B", "KB", "MB", "GB", "TB", "PB", "EB")

    for unit in units:
        if abs(file_size) < 1024.0:
            return f"{file_size:.2f} {unit}"
        file_size /= 1024.0

    raise ValueError(f"Can not convert {file_size:.2f} byte to any unit.")
