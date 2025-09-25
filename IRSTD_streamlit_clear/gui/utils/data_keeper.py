from typing import Union, overload, Iterable, Tuple
from threading import Lock

from typing_extensions import TypeVar


class DataKeeper:
    _T = TypeVar("_T")
    _KT = TypeVar("_KT")
    _VT = TypeVar("_VT")

    def __init__(self):
        self.__lock = Lock()
        self.__data_container = {}

    @overload
    def get(self, __key: _KT) -> Union[_VT, None]:
        ...

    @overload
    def get(self, __key: _KT, __default: Union[_VT, _T]) -> Union[_VT, _T]:
        ...

    def get(self, key: _KT, __default: Union[_VT, _T] = None) -> Union[_VT, _T]:
        """获取保存的数据的键对应的值

        Args:
            key (_KT): 键
            __default (Union[_VT, _T], optional): 如果键不存在则返回的默认值. Defaults to None.

        Returns:
            Union[_VT, _T]: 键对应的值，或默认值
        """
        if key not in self.__data_container:
            return __default
        with self.__lock:
            return self.__data_container.get(key, __default)

    def set(self, key: _KT, value: _VT) -> None:
        """将数据保存在键值对中

        Args:
            key (_KT): 键名
            value (_VT): 对应要存储的数据
        """
        with self.__lock:
            self.__data_container[key] = value

    @overload
    def update(self, __value: dict) -> None:
        ...

    @overload
    def update(self, __value: Iterable[Tuple[_KT, _VT]]) -> None:
        ...

    def update(self, __value: Union[dict, Iterable[Tuple[_KT, _VT]]]) -> None:
        """更新数据字典中的多个键值对

        Args:
            __value (Union[dict, Iterable[Tuple[_KT, _VT]]]): 键值对或字典
        """
        with self.__lock:
            self.__data_container.update(__value)

    def get_all_data(self) -> dict:
        """获取当前的全部数据

        Returns:
            dict: 数据字典
        """
        with self.__lock:
            return self.__data_container
