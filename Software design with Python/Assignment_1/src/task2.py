import inspect
import textwrap
from task1 import decorator_1, OUTPUT


def decorator_2(fun):
    def wrapper(*args, **kargs):
        res = decorator_1(fun)(*args, **kargs)

        indent = 20
        print(f"{'Name:': <20}{fun.__name__}")
        print(f"{'Type:': <20}{type(fun)}")
        print(f"{'Sign:': <20}{inspect.signature(fun)}")
        print(f"{'Args spec:': <20}{inspect.getargspec(fun)}")
        print(f"{'Args:': <20}positional {args}")
        print(f"{'': <20}keyworded {kargs}")

        print()
        doc = textwrap.indent(inspect.getdoc(fun), indent * ' ')[indent:]
        print(f"{'Doc:': <20}{doc}")

        print()
        source = textwrap.indent(inspect.getsource(fun), indent * ' ')[indent:]
        print(f"{'Source:': <20}{source}")

        print()
        output = textwrap.indent(OUTPUT[fun.__name__], indent * ' ')[indent:]
        print(f"{'Output:': <20}{output}")
        return res
    return wrapper

    
