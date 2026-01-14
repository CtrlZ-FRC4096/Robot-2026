import inspect
from functools import wraps
from typing import Any, Callable, Generator, Iterable, List, Optional, Union, overload

from commands2 import Command, CommandScheduler, Subsystem


class _Unset:
    pass


Unset = _Unset()

Coroutineable = Union[
    Callable[[], None],
    Callable[[], Generator[None, None, None]],
    Generator[None, None, None],
]

GenFuncable = Union[
    Callable[..., None],
    Callable[..., Generator[None, None, None]],
]


def ensure_generator_function(
    func: GenFuncable,
) -> Callable[..., Generator[None, None, None]]:
    if inspect.isgeneratorfunction(func):
        return func  # type: ignore

    @wraps(func)
    def wrapper(*args, **kwargs):
        func(*args, **kwargs)
        yield

    return wrapper


class CoroutineCommand(Command):
    coroutine: Union[
        Callable[..., None],
        Callable[..., Generator[None, None, None]],
        Generator[None, None, None],
    ]
    is_finished: bool

    def __init__(
        self,
        coroutine: Union[
            Callable[..., None],
            Callable[..., Generator[None, None, None]],
            Generator[None, None, None],
        ],
        requirements: Optional[List[Subsystem]] = None,
        interruptible: bool = True,
    ) -> None:
        super().__init__()
        self.coroutineable = coroutine
        self.coroutine = None  # type: ignore
        self.is_finished = False
        self.interruptible = (
            Command.InterruptionBehavior.kCancelSelf
            if interruptible
            else Command.InterruptionBehavior.kCancelIncoming
        )

        if requirements is not None:
            self.addRequirements(*requirements)

    def getInterruptionBehavior(self) -> Command.InterruptionBehavior:
        return self.interruptible

    def initialize(self) -> None:
        self.coroutine = ensure_generator_function(self.coroutineable)()  # type: ignore
        self.is_finished = False

    def execute(self):
        # __import__("code").interact(local={**locals(), **globals()})
        # print(self.coroutine)
        try:
            if not self.is_finished:
                if not inspect.isgenerator(self.coroutine):
                    # __import__("code").interact(local={**locals(), **globals()})
                    raise TypeError("This command was not properly initialized")
                next(self.coroutine)
        except StopIteration:
            self.is_finished = True

    def isFinished(self):
        return self.is_finished

    def end(self, interrupted: bool):
        if not self.is_finished:
            self.coroutine.close()

    def __call__(self, *args, **kwargs) -> "CoroutineCommand":
        if inspect.isgenerator(self.coroutine):
            return self

        return CoroutineCommand(ensure_generator_function(self.coroutine)(*args, **kwargs))  # type: ignore


def autoroutine2command(func) -> Callable[..., Command]:
    """
    Decorator that turns an autoroutine into a command that requires every single subsystem
    """

    @wraps(func)
    def wrapper(*args, **kwargs):
        gen = func(*args, **kwargs)

        class C(Command):
            def __init__(self) -> None:
                super().__init__()
                self.is_finished = False
                r: "Robot" = args[0]  # type: ignore
                self.addRequirements(*r.subsystems)
                self.addRequirements(*CommandScheduler.getInstance()._subsystems.keys())

            def execute(self) -> None:
                try:
                    next(gen)
                except StopIteration:
                    self.is_finished = True

            def isFinished(self) -> bool:
                return self.is_finished

            def __iter__(self):
                return gen

        return C()

    return wrapper


@overload
def commandify(
    *,
    interruptible: bool = True,
    requirements: List[Subsystem] = [],
) -> Callable[[GenFuncable], CoroutineCommand]:
    ...


@overload
def commandify(
    coroutine: GenFuncable,
    /,
) -> CoroutineCommand:
    ...


def commandify(*args, **kwargs) -> Any:
    try:
        from pathplannerlib.auto import NamedCommands

        pplncrc = NamedCommands.registerCommand
    except:
        pplncrc = lambda *args, **kwargs: None

    if args:
        coroutine = args[0]
        command = CoroutineCommand(
            ensure_generator_function(coroutine),
        )
        pplncrc(coroutine.__name__, command)
        print(coroutine.__name__, "registered")
        return command

    def wrapper(coroutine: GenFuncable) -> CoroutineCommand:
        command = CoroutineCommand(
            ensure_generator_function(coroutine),
            **kwargs,
        )
        pplncrc(coroutine.__name__, command)
        print(coroutine.__name__, "registered")
        return command

    return wrapper
