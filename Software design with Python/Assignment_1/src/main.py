def quadratic_equation_solver(a, b, c):
    D = b**2 - 4 * a* c
    if D == 0:
        return -b/(2 * a)
    elif D > 0:
        x1 = (-b + D**0.5)/(2 * a)
        x2 = (-b - D**0.5)/(2 * a)
        return x1, x2
    else:
        print("There is no solution for quadratic equation")
        return None

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

print("Quadratic equation solver (a=1,b=2,c=1): ", quadratic_equation_solver(1,2,1))
print("Pascal triangle 5 rows")
printPascal(5)
print("3+2=", sum_numbers(3,2))
print("First 10 numbers:")
print_first_n_numbers(10)

import random
from task1 import decorator_1
from task2 import decorator_2
from task3 import Decorator_1_class, Decorator_2_class, rank_functions
from task4 import Decorator_1_class_error, Decorator_2_class_error

@decorator_1
def func():
    print("I am ready to Start")
    result = 0
    n =  random.randint(10,751)
    for i in range(n):
        result += (i**2)
        
@decorator_1
def funx(n=2, m=5):
    print("I am ready to do serious stuff")
    max_val = float('-inf')
    n =  random.randint(10,751)
    res = [pow(i,2) for i in range(n)]
    for i in res:
        if i > max_val: 
            max_val = i

@decorator_1
def fun_count(n):
    for i in range(n):
        a = 2 + 2
    
@decorator_2
def funh(bar1, bar2=""):
    """
    This function does something useful 
    :param bar1: description
    :param bar2: description
    """ 
    print("some\nmultiline\noutput")

    
    
#Task 1
print("-------------Task 1 -------------")
func()
funx()
func()
funx()
func()
fun_count(50000000)




#Task 2
print()
print("-------------Task 2 -------------")
funh(None, bar2="")



#Task 3
print()
print("-------------Task 3 -------------")   

def fun_count():
    for i in range(100000):
        a = 2 + 2

def funb():
    result = 0
    n =  random.randint(10,751)
    for i in range(n):
        result += (i**2)

def func():
    result = 0
    n =  random.randint(10,751)
    for i in range(n):
        result += (i**2)

rank_functions(fun_count, funb, func)




#Task 4
@Decorator_2_class_error
def error_function():
    return 1/0

error_function()