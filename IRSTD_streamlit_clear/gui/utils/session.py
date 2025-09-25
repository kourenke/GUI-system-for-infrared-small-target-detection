from typing import Any, Optional, Callable, NoReturn
from typing_extensions import TypeVar, Type, final, overload

from streamlit import session_state

_T = TypeVar("_T", bound="Any")
_SessionStateT = TypeVar("_SessionStateT", bound="SessionState")


@final
class SessionState:
    def __new__(cls: Type[_SessionStateT]) -> NoReturn:
        raise TypeError("This is a utility class that disallows instantiations.")

    @overload
    @staticmethod
    def get_var(var_key: str) -> Any:
        ...

    @overload
    @staticmethod
    def get_var(var_key: str, *, default: _T = None, set_default: bool = True) -> _T:
        ...

    @overload
    @staticmethod
    def get_var(
        var_key: str,
        *,
        default_factory: Optional[Callable[..., _T]] = None,
        factory_args: Optional[tuple] = None,
        factory_kwargs: Optional[dict] = None,
        set_default: bool = True,
    ) -> _T:
        ...

    @staticmethod
    def get_var(
        var_key: str,
        *,
        default: Any = None,
        default_factory: Optional[Callable[..., Any]] = None,
        factory_args: Optional[tuple] = None,
        factory_kwargs: Optional[dict] = None,
        set_default: bool = True,
    ) -> Any:
        """获取会话变量的值。

        Args:
            var_key (str): 会话变量名
            default (Any): 默认值. Defaults to None.
            default_factory (Optional[Callable[..., Any]], optional): 默认值工厂. Defaults to None.
            factory_args (Optional[tuple], optional): 默认值工厂位置参数. Defaults to None.
            factory_kwargs (Optional[dict], optional): 默认值工厂关键字参数. Defaults to None.
            set_default (bool, optional): 是否在会话变量不存在时保存默认值. Defaults to True.

        Returns:
            Any: 获取的会话变量值
        """
        session_var = session_state.get(var_key)

        if session_var is not None:
            return session_var

        if default_factory is not None:
            default = default_factory(
                *(factory_args or ()),
                **(factory_kwargs or {}),
            )

        if set_default:
            session_state[var_key] = default

        return default

    @staticmethod
    def set_var(**kwargs: Any) -> None:
        """设置会话变量的值

        Args:
            kwargs (Any): 新的值
        """
        session_state.update(**kwargs)

    @staticmethod
    def get_var_from_widget(widget_key: str) -> Any:
        """从控件中获取当前的值

        Args:
            widget_key (str): 控件的键名

        Returns:
            Any: 控件的值
        """
        widget_value = session_state.get(widget_key)

        return widget_value

    @staticmethod
    def set_var_from_widget(
        widget_key: str,
        var_key: str,
        *,
        callback: Optional[Callable[[Any], Any]] = None,
    ) -> None:
        """将控件的值保存到会话变量

        Args:
            widget_key (str): 控件名
            var_key (str): 会话变量名
            callback (Optional[Callable[[Any], Any]]): 回调函数。
                参数是从控件获取的值，需要返回处理后的值. Defaults to None.
        """
        widget_value = session_state.get(widget_key)

        if callback is not None:
            widget_value = callback(widget_value)

        session_state[var_key] = widget_value

    @staticmethod
    def set_var_from_widget_callback(
        widget_key: str,
        var_key: str,
        *,
        callback: Optional[Callable[[Any], Any]] = None,
    ) -> Callable[[], None]:
        """更新控件的值到会话变量

        Args:
            widget_key (str): 控件名
            value_key (str): 会话变量名
            callback (Optional[Callable[[Any], Any]]): 回调函数。
                参数是从控件获取的值，需要返回处理后的值. Defaults to None.

        Returns:
            Callable[[], None]: 用于传给 on_change 参数的回调函数。
        """

        def wrapper() -> None:
            widget_value = session_state.get(widget_key)

            if callback is not None:
                widget_value = callback(widget_value)

            session_state[var_key] = widget_value

        return wrapper

    @staticmethod
    def update_var(**kwargs: Any) -> None:
        """更新会话变量，如果原本不存在则不添加。

        Args:
            kwargs (Any): 新的值
        """
        for key, value in kwargs.items():
            if key in session_state:
                session_state[key] = value
