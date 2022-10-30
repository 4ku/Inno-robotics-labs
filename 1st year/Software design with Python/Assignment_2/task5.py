"""
usage: task5.py action [-flag value]*
This program ...
compile
    -py file.py compile file into bytecode and store it as file.pyc
    -s "src" compile src into bytecode and store it as out.pyc
print
    -py src.py
    -pyc src.pyc produce human-readable bytecode from python file produce human-readable bytecode from compiled .pyc file
    -s "src" produce human-readable bytecode from normal string
compare -format src [-format src]+
    produce bytecode comparison for giving sources 
    (supported formats -py, -pyc, -s)
"""

import sys
from task2_3 import get_bytecode, print_bytecode
from task4 import compile


def compare():
    filenames = []
    bytecodes = []
    for i in range(2, len(sys.argv), 2):
        filenames.append(sys.argv[i + 1])
        bytecodes.append(get_bytecode(i))

    dicts = []
    for bytecode in bytecodes:
        d = {}
        for instruction in bytecode:
            if instruction.opname not in d:
                d[instruction.opname] = 1
            else:
                d[instruction.opname] += 1
        dicts.append(d)

    max_dic = {}
    for d in dicts:
        for k, v in d.items():
            max_dic[k] = max(max_dic.get(k, 0), v)
    opcodes = dict(
        sorted(max_dic.items(), key=lambda item: item[1], reverse=True)).keys()

    with open("result.txt", "w") as output_file:
        row_format = "{:<15.15}|" * (len(filenames) + 1)
        columns = ["INSTRUCTION"] + filenames
        print(row_format.format(*columns))
        output_file.write(row_format.format(*columns) + '\n')
        row_format = "{:<15.15}|" + "{:<15}|" * len(filenames)
        for opcode in opcodes:
            row_values = [opcode]
            for d in dicts:
                v = d.get(opcode, 0)
                row_values.append(v)
            print(row_format.format(*row_values))
            output_file.write(row_format.format(*row_values) + '\n')


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
            compare()
        else:
            print("Error")
