from datetime import datetime


class Decorator_class:
    def __init__(self, func):
        self.calls = 0
        self.func = func
        self.output = None

    def __call__(self, *args, **argv):
        self.calls += 1
        time = datetime.now()
        self.func(*args, **argv)
        with open("log.txt", 'a') as file:
            str_time = time.strftime("%d/%m/%Y, %H:%M:%S")
            file.write(
                f"{self.func.__name__} Time: {str_time} Invocation number: {self.calls}")


class TollBoth:
    def __init__(self):
        self.money = 0
        self.cars = 0

    @Decorator_class
    def payingCar(self):
        self.money += 0.5
        self.cars += 1

    @Decorator_class
    def nopayCar(self):
        self.cars += 1

    @Decorator_class
    def display(self):
        print("Collected money:", self.money)
        print("Cars:", self.cars)


if __name__ == "__main__":
    pass
