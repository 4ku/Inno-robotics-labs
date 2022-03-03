class Angle:
    def __init__(self, degrees: int, minutes: float, direction: str):
        self.degrees = degrees
        self.minutes = minutes
        self.direction = direction

    def __repr__(self):
        degree_sign = u'\N{DEGREE SIGN}'
        return str(self.degrees) + degree_sign + '{0:.1f}'.format(self.minutes) + "'" + self.direction

    def interactive_creation():
        print("Enter a degree")
        degrees = int(input())
        print("Enter a minutes")
        minutes = float(input())
        print("Enter a direction")
        direction = input()
        return Angle(degrees, minutes, direction)


if __name__ == "__main__":
    angle = Angle(179, 59.9, 'E')
    print("Angle:", angle)
    while True:
        angle = Angle.interactive_creation()
        print("Your angle:", angle)
        print()
