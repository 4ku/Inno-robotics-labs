import io
import time
from contextlib import redirect_stdout


COUNTER = {}
OUTPUT = {}
def decorator_1(fun):
    global COUNTER
    global OUTPUT
    def wrapper(*args, **kargs):
        if fun.__name__ in COUNTER:
            COUNTER[fun.__name__] += 1
        else:
            COUNTER[fun.__name__] = 1
        
        with redirect_stdout(io.StringIO()) as f:
            start = time.time()
            result = fun(*args, **kargs)
            elapsed_time = time.time() - start
            OUTPUT[fun.__name__] = f.getvalue()
        print(f'{fun.__name__} call {COUNTER[fun.__name__]} executed in {elapsed_time} sec')
        return result
    return wrapper
