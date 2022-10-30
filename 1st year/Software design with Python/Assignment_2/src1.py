import math

def f():
    total = 0
    i = 0
    while i <= 1000000:
        total += math.sin(i)
        i += 1
    return total

print(f())
