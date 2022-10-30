import math

def f():
    total = 0
    m = math.sin
    for x in range(1000000):
        total += m(x)
    return total

print(f())
