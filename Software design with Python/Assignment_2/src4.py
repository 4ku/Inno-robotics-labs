def printPascal(n):
    for i in range(1, n + 1):
        C = 1
        for j in range(1, i + 1):
            print(C, end = " ")
            C = int(C * (i - j) / j)
        print()

def sum_numbers(a, b):
    lambda_function = lambda a, b: a+b
    return lambda_function(a,b)
    
def print_first_n_numbers(n):
    lambda_function = lambda x: [i for i in range(x)]
    for i in lambda_function(n):
        print(i+1)


if __name__ == '__main__':
    printPascal(100)