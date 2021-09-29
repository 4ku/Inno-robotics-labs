"""
usage: task4.py action [-flag value]*
This program ...
compile
    -py file.py compile file into bytecode and store it as file.pyc
    -s "src" compile src into bytecode and store it as out.pyc
print
    -py src.py produce human-readable bytecode from python file
    -pyc src.pyc produce human-readable bytecode from compiled .pyc file
    -s "src" produce human-readable bytecode from normal string
"""


import sys
import py_compile
from tempfile import NamedTemporaryFile
from task2_3 import print_bytecode


def compile():
    for i in sys.argv[3:]:
        if sys.argv[2] == "-s":
            with NamedTemporaryFile("w", delete=False) as tmp:
                tmp.write(i)
                tmp.seek(0)
                py_compile.compile(tmp.name, cfile="out.pyc")
        elif sys.argv[2] == "-py":
            try:
                py_compile.compile(i, cfile=i+"c")
            except Exception as e:
                print(f"We are skipping...{i}")


if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print(__doc__)
        sys.exit()
    else:
        if sys.argv[1] == "print":
            print_bytecode(2)
        elif sys.argv[1] == "compile":
            compile()
        elif sys.argv[1] == "compare":
            pass
        else:
            print("Error")
