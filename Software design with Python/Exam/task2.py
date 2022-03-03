from datetime import datetime


def fibonacci(n):
    if n in [0, 1]:
        return n
    return fibonacci(n - 1) + fibonacci(n - 2)


class ID:
    CREATED_INSTANCES = 0

    def __init__(self):
        self.__serial_number = hex(fibonacci(ID.CREATED_INSTANCES + 7))
        self.__creation_date = datetime.now()
        ID.CREATED_INSTANCES += 1

    def get_serial_number(self):
        return self.__serial_number

    def print_serial_number(self):
        print(f"I am object number {self.__serial_number}")

    def __repr__(self):
        string_datetime = self.__creation_date.strftime("%d/%m/%Y, %H:%M:%S")
        return f"I am object number {self.__serial_number} with creation date {string_datetime}"


if __name__ == "__main__":
    ob1 = ID()
    ob2 = ID()
    ob3 = ID()

    ob1.print_serial_number()
    ob2.print_serial_number()
    ob3.print_serial_number()
