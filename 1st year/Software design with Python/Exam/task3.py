import traceback
from task1 import Angle
from task2 import ID


class Ship:
    __instance_number = 0

    def __new__(self, *args, **kwargs):
        instance = None
        if self.__instance_number < 3:
            instance = super().__new__(self)
            self.__instance_number += 1
        else:
            raise Exception(
                'There are only 3 instance creation of class allowed!')
        return instance

    def __init__(self, name, capacity):
        self.name = name
        self.capacity = capacity
        self.id = None
        self.position = None

    def set_position(self, position):
        self.position = position

    def set_id_and_position(self, id, position):
        self.id = id
        self.position = position

    def __repr__(self):
        return f"Name: {self.name}\nSerial number: {self.id}\nLocation: {self.position}\nCapacity: {self.capacity}"


if __name__ == "__main__":
    try:
        for i in range(4):
            print(f"Let's create {i + 1} ship")
            ship = Ship("Ship" + str(i + 1), i + 100)
            angle = Angle.interactive_creation()
            ship.set_id_and_position(ID(), angle)
            print(ship)
            print()

    except Exception as e:
        print(str(e))
