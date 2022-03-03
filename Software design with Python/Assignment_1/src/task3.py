import io
import time
from contextlib import redirect_stdout
import inspect
import textwrap


class Decorator_1_class:
    def __init__(self, func):
        self.calls = 0
        self.func = func
        self.output = None

    def __call__(self, *args, **argv):
        self.calls += 1
        start = time.time()
        with redirect_stdout(io.StringIO()) as f:
            res = self.func(*args, **argv)
            self.output = f.getvalue()
        
        with open(".txt", 'a') as file:
            file.write(f'{self.func.__name__} call {self.calls} executed in {time.time() - start} sec\n')

        return res


class Decorator_2_class(Decorator_1_class):
    def __init__(self, func):
        super().__init__(func)

    def __call__(self, *args, **argv):
        res = super().__call__(*args, **argv)

        indent = 20
        with open(".txt", 'a') as file:
            file.write(f"{'Name:': <20}{self.func.__name__}\n")
            file.write(f"{'Type:': <20}{type(self.func)}\n")
            file.write(f"{'Sign:': <20}{inspect.signature(self.func)}\n")
            file.write(f"{'Args spec:': <20}{inspect.getargspec(self.func)}\n")
            file.write(f"{'Args:': <20}positional {args}\n")
            file.write(f"{'': <20}keyworded {argv}\n")

            file.write('\n')
            doc = textwrap.indent(inspect.getdoc(self.func), indent * ' ')[indent:]
            file.write(f"{'Doc:': <20}{doc}\n")

            file.write('\n')
            source = textwrap.indent(inspect.getsource(self.func), indent * ' ')[indent:]
            file.write(f"{'Source:': <20}{source}\n")

            file.write('\n')
            output = textwrap.indent(self.output, indent * ' ')[indent:]
            file.write(f"{'Output:': <20}{output}\n")
        return res


def rank_functions(*args):
    times = {}
    for fun in args:
        start = time.time()
        fun()
        times[fun.__name__] = time.time() - start
    times = sorted(times.items(), key=lambda item: item[1])

    row_format ="{:<15}|{:<15}|{:<15}" 
    print(row_format.format(*["PROGRAM", "RANK", "TIME ELAPSED"]))
    row_format ="{:<15}|{:<15}|{:<15.10f}" 
    
    for i, tuple_ in enumerate(times):
        print(row_format.format(*[tuple_[0], i+1, tuple_[1]]))


