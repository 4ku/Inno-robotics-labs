from datetime import datetime 
import traceback
from task3 import Decorator_1_class, Decorator_2_class

class Decorator_1_class_error(Decorator_1_class):
    def __init__(self, func):
        super().__init__(func)
    
    def __call__(self, *args, **argv):
        try:
            res = super().__call__(*args, **argv)
            return res
        except Exception as e:
            with open("errors.log", 'a') as file:
                file.write("Timestamp: " + str(datetime.now()) + "\n")
                file.write(traceback.format_exc())
                file.write("\n")
            return None


class Decorator_2_class_error(Decorator_2_class):
    def __init__(self, func):
        super().__init__(func)
    
    def __call__(self, *args, **argv):
        try:
            res = super().__call__(*args, **argv)
            return res
        except Exception as e:
            with open("errors.log", 'a') as file:
                file.write("Timestamp: " + str(datetime.now()) + "\n")
                file.write(traceback.format_exc())
                file.write("\n")
            return None

