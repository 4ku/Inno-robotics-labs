import sys
import os
import timeit
import subprocess

if len(sys.argv) <= 1:
    print("usage: task1.py [files]")
    print("This program ...")
    sys.exit()

result = {}

for i in sys.argv[1:]:
    if os.path.exists(i):
        timer = timeit.timeit(lambda: subprocess.run(
            ['python', i], stdout=subprocess.PIPE), number=1)
        result[i] = timer

result = sorted(result.items(), key=lambda item: item[1])
row_format ="{:<15}|{:<15}|{:<15}" 
print(row_format.format(*["PROGRAM", "RANK", "TIME ELAPSED"]))
row_format ="{:<15}|{:<15}|{:<15.10f}" 

for i, tuple_ in enumerate(result):
    print(row_format.format(*[tuple_[0], i+1, tuple_[1]]))
