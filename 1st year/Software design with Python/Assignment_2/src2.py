import math

def f():
    total = 0
    for x in range(1000000):
        total += math.sin(x)
    return total

print(f())
