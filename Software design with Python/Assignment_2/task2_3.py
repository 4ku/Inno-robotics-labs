"""
usage: task2_3.py -py src.py
This program ...
"""

import sys
import dis
import marshal


def expand_bytecode(bytecode):
    result = []
    for instruction in bytecode:
        if str(type(instruction.argval)) == "<class 'code'>":
            result += expand_bytecode(dis.Bytecode(instruction.argval))
        else:
            result.append(instruction)

    return result


def get_bytecode(flag_index):
    file_name = sys.argv[flag_index + 1]
    source = None
    if sys.argv[flag_index] == "-py":
        with open(file_name, 'r') as f:
            source = f.read()
    elif sys.argv[flag_index] == "-pyc":
        header_size = 12
        if sys.version_info >= (3, 7):
            header_size = 16
        with open(file_name, 'rb') as f:
            f.seek(header_size)
            source = marshal.load(f)
    elif sys.argv[flag_index] == "-s":
        source = file_name
    else:
        print("Error")
        return
    bc = dis.Bytecode(source)
    return expand_bytecode(bc)


def print_bytecode(flag_index):
    bytecode = get_bytecode(flag_index)
    for instruction in bytecode:
        print(f'{instruction.opname}\t {instruction.argrepr}')


if __name__ == '__main__':
    if len(sys.argv) <= 1:
        print(__doc__)
        sys.exit()

    print_bytecode(1)
