from functools import wraps
from typing import Any, Callable, Generator, List, Union, overload

from commands2 import Command, Subsystem
from commands2.button import Trigger as _Button

from ..coroutine import CoroutineCommand
from ..coroutine.coroutine_command import (
    ensure_generator_function,
    Unset,
    Coroutineable,
    _Unset,
)


class Button(_Button):
    def __call__(self, *args: Any, **kwds: Any) -> Any:
        return self.__bool__()


class CustomButton(Button):
    # new ones

    @overload
    def whenHeld(
        self,
        coroutine: Union[Coroutineable, _Unset] = Unset,
        /,
        *,
        interruptible: bool = True,
        requirements: List[Subsystem] = [],
    ) -> Union[Button, Callable[[Coroutineable], Button]]:
        ...

    @overload
    def whenPressed(
        self,
        coroutine: Union[Coroutineable, _Unset] = Unset,
        /,
        *,
        interruptible: bool = True,
        requirements: List[Subsystem] = [],
    ) -> Union[Button, Callable[[Coroutineable], Button]]:
        ...

    @overload
    def whenReleased(
        self,
        coroutine: Union[Coroutineable, _Unset] = Unset,
        /,
        *,
        interruptible: bool = True,
        requirements: List[Subsystem] = [],
    ) -> Union[Button, Callable[[Coroutineable], Button]]:
        ...

    # existing ones
    @overload
    def whenHeld(self, command: Command, /, interruptible: bool = True) -> Button:
        ...

    @overload
    def whenPressed(self, command: Command, /, interruptible: bool = True) -> Button:
        ...

    @overload
    def whenReleased(self, command: Command, /, interruptible: bool = True) -> Button:
        ...

    # new defs

    def whenHeld(self, *args, **kwargs) -> Any:
        coc = None
        if args:
            coc = args[0]
        if coc == None:

            def wrapper(coroutine: Coroutineable) -> Button:
                # __import__("code").interact(local={**locals(), **globals()})
                return self.whenHeld(coroutine, **kwargs)  # type: ignore

            return wrapper

        if not callable(coc):
            return super().whileTrue(*args, **kwargs)

        # __import__("code").interact(local={**locals(), **globals()})

        command = CoroutineCommand(
            ensure_generator_function(coc),
            requirements=kwargs.get("requirements", []),
            interruptible=kwargs.get("interruptible", True),
        )
        return super().whileTrue(command)

    def whenPressed(self, *args, **kwargs) -> Any:
        coc = None
        if args:
            coc = args[0]
        if coc == None:

            def wrapper(coroutine: Coroutineable) -> Button:
                # __import__("code").interact(local={**locals(), **globals()})
                return self.whenPressed(coroutine, **kwargs)  # type: ignore

            return wrapper

        if not callable(coc):
            return super().onTrue(*args, **kwargs)

        # __import__("code").interact(local={**locals(), **globals()})

        command = CoroutineCommand(
            ensure_generator_function(coc),
            requirements=kwargs.get("requirements", []),
            interruptible=kwargs.get("interruptible", True),
        )
        return super().onTrue(command)

    def whenReleased(self, *args, **kwargs) -> Any:
        coc = None
        if args:
            coc = args[0]
        if coc == None:

            def wrapper(coroutine: Coroutineable) -> Button:
                return self.whenReleased(coroutine, **kwargs)  # type: ignore

            return wrapper

        if not callable(coc):
            return super().onFalse(*args, **kwargs)

        # __import__("code").interact(local={**locals(), **globals()})

        command = CoroutineCommand(
            ensure_generator_function(coc),
            requirements=kwargs.get("requirements", []),
            interruptible=kwargs.get("interruptible", True),
        )
        return super().onFalse(command)
