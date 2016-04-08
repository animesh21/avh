import math

from ps2 import RectangularRoom, WheelChair


angular_speed = 30 / 180.0 * math.pi
room = RectangularRoom(width=10, height=10)
wheel_chair = WheelChair(room=room, speed=1.0, angular_speed=angular_speed)

if __name__ == '__main__':
    wheel_chair.move()

