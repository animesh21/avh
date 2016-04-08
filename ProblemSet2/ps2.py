# 6.00.2x Problem Set 2: Simulating robots

import math
import random
import time
import datetime
import random

# import pylab
import serial

# For Python 2.7:
from ps2_verify_movement27 import testRobotMovement, test_wheelchair_movement

# If you get a "Bad magic number" ImportError, you are not using 
# Python 2.7 and using most likely Python 2.6:


# === Provided class Position
class Position(object):
    """
    A Position represents a location in a two-dimensional room.
    """
    def __init__(self, x, y):
        """
        Initializes a position with coordinates (x, y).
        """
        self.x = x
        self.y = y
        
    def getX(self):
        return self.x
    
    def getY(self):
        return self.y
    
    def getNewPosition(self, angle, speed):
        """
        Computes and returns the new Position after a single clock-tick has
        passed, with this object as the current position, and with the
        specified angle and speed.

        Does NOT test whether the returned position fits inside the room.

        angle: number representing angle in degrees, 0 <= angle < 360
        speed: positive float representing speed

        Returns: a Position object representing the new position.
        """
        old_x, old_y = self.getX(), self.getY()
        angle = float(angle)
        # Compute the change in position
        delta_y = speed * math.cos(math.radians(angle))
        delta_x = speed * math.sin(math.radians(angle))
        # Add that to the existing position
        new_x = old_x + delta_x
        new_y = old_y + delta_y
        return Position(new_x, new_y)

    def __str__(self):  
        return "(%f, %f)" % (self.x, self.y)


# === Problem 1
# Enter your code for RectangularRoom in this box
class RectangularRoom(object):
    """
    A RectangularRoom represents a rectangular region containing clean or dirty
    tiles.

    A room has a width and a height and contains (width * height) tiles. At any
    particular time, each of these tiles is either clean or dirty.
    """
    def __init__(self, width, height):
        """
        Initializes a rectangular room with the specified width and height.

        Initially, no tiles in the room have been cleaned.

        width: an integer > 0
        height: an integer > 0
        """
        self.width = width
        self.height = height
        self.tiles = width * height
        self.cleanTiles = []
        #raise NotImplementedError

    def cleanTileAtPosition(self, pos):
        """
        Mark the tile under the position POS as cleaned.

        Assumes that POS represents a valid position inside this room.

        pos: a Position
        """
        cleanPos = Position(int(pos.x), int(pos.y))
        if not((cleanPos.x, cleanPos.y) in [(p.x, p.y) for p in self.cleanTiles]):
            self.cleanTiles.append(cleanPos)
        #raise NotImplementedError

    def isTileCleaned(self, m, n):
        """
        Return True if the tile (m, n) has been cleaned.

        Assumes that (m, n) represents a valid tile inside the room.

        m: an integer
        n: an integer
        returns: True if (m, n) is cleaned, False otherwise
        """
        #raise NotImplementedError
        for pos in self.cleanTiles:
            if (pos.x, pos.y) == (m, n):
                return True
        return False
    
    def getNumTiles(self):
        """
        Return the total number of tiles in the room.

        returns: an integer
        """
        #raise NotImplementedError
        return self.tiles

    def getNumCleanedTiles(self):
        """
        Return the total number of clean tiles in the room.

        returns: an integer
        """
        #raise NotImplementedError
        return len(self.cleanTiles)

    def getRandomPosition(self):
        """
        Return a random position inside the room.

        returns: a Position object.
        """
        #raise NotImplementedError
        return Position(self.width*random.random(), self.height*random.random())

    def isPositionInRoom(self, pos):
        """
        Return True if pos is inside the room.

        pos: a Position object.
        returns: True if pos is in the room, False otherwise.
        """
        #raise NotImplementedError
        return pos.x >= 0 and pos.x < self.width and pos.y >= 0 and pos.y < self.height

    def get_next_position(self):
        """
        get the next position of the bot at current speed
        :return: the next position of the robot
        """


class Robot(object):
    """
    Represents a robot cleaning a particular room.

    At all times the robot has a particular position and direction in the room.
    The robot also has a fixed speed.

    Subclasses of Robot should provide movement strategies by implementing
    updatePositionAndClean(), which simulates a single time-step.
    """
    def __init__(self, room, speed):
        """
        Initializes a Robot with the given speed in the specified room. The
        robot initially has a random direction and a random position in the
        room. The robot cleans the tile it is on.

        room:  a RectangularRoom object.
        speed: a float (speed > 0)
        """
        #raise NotImplementedError
        self.room = room
        # self.position = room.getRandomPosition()
        self.position = Position(4, 4)
        self.direction = math.pi/2
        self.speed = speed
        self.room.cleanTileAtPosition(self.position)

    def getRobotPosition(self):
        """
        Return the position of the robot.

        returns: a Position object giving the robot's position.
        """
        #raise NotImplementedError
        return self.position
    
    def getRobotDirection(self):
        """
        Return the direction of the robot.

        returns: an integer d giving the direction of the robot as an angle in
        degrees, 0 <= d < 360.
        """
        #raise NotImplementedError
        return self.direction

    def setRobotPosition(self, position):
        """
        Set the position of the robot to POSITION.

        position: a Position object.
        """
        #raise NotImplementedError
        self.position = position

    def setRobotDirection(self, direction):
        """
        Set the direction of the robot to DIRECTION.

        direction: integer representing an angle in degrees
        """
        #raise NotImplementedError
        self.direction = direction

    def updatePositionAndClean(self):
        """
        Simulate the raise passage of a single time-step.

        Move the robot to a new position and mark the tile it is on as having
        been cleaned.
        """
        raise NotImplementedError # don't change this!


# === Problem 2
class StandardRobot(Robot):
    """
    A StandardRobot is a Robot with the standard movement strategy.

    At each time-step, a StandardRobot attempts to move in its current
    direction; when it would hit a wall, it *instead* chooses a new direction
    randomly.
    """
    def updatePositionAndClean(self):
        """
        Simulate the raise passage of a single time-step.

        Move the robot to a new position and mark the tile it is on as having
        been cleaned.
        """
        #raise NotImplementedError
        newPosition = self.position.getNewPosition(self.getRobotDirection(), self.speed)
        while not(self.room.isPositionInRoom(newPosition)):
            self.direction = random.random() * 360
            newPosition = self.position.getNewPosition(self.direction, self.speed)
        self.position = newPosition
        self.room.cleanTileAtPosition(self.position)
        

# Uncomment this line to see your implementation of StandardRobot in action!
# testRobotMovement(StandardRobot, RectangularRoom)


# === Problem 3
def runSimulation(num_robots, speed, width, height, min_coverage, num_trials,
                  robot_type):
    """
    Runs NUM_TRIALS trials of the simulation and returns the mean number of
    time-steps needed to clean the fraction MIN_COVERAGE of the room.

    The simulation is run with NUM_ROBOTS robots of type ROBOT_TYPE, each with
    speed SPEED, in a room of dimensions WIDTH x HEIGHT.

    num_robots: an int (num_robots > 0)
    speed: a float (speed > 0)
    width: an int (width > 0)
    height: an int (height > 0)
    min_coverage: a float (0 <= min_coverage <= 1.0)
    num_trials: an int (num_trials > 0)
    robot_type: class of robot to be instantiated (e.g. StandardRobot or
                RandomWalkRobot)
    """
    raise NotImplementedError

# Uncomment this line to see how much your simulation takes on average
##print  runSimulation(1, 1.0, 10, 10, 0.75, 30, StandardRobot)


# === Problem 4
class RandomWalkRobot(Robot):
    """
    A RandomWalkRobot is a robot with the "random walk" movement strategy: it
    chooses a new direction at random at the end of each time-step.
    """
    def updatePositionAndClean(self):
        """
        Simulate the passage of a single time-step.

        Move the robot to a new position and mark the tile it is on as having
        been cleaned.
        """
        raise NotImplementedError


def showPlot1(title, x_label, y_label):
    """
    What information does the plot produced by this function tell you?
    """
    num_robot_range = range(1, 11)
    times1 = []
    times2 = []
    for num_robots in num_robot_range:
        print "Plotting", num_robots, "robots..."
        times1.append(runSimulation(num_robots, 1.0, 20, 20, 0.8, 20, StandardRobot))
        times2.append(runSimulation(num_robots, 1.0, 20, 20, 0.8, 20, RandomWalkRobot))
    pylab.plot(num_robot_range, times1)
    pylab.plot(num_robot_range, times2)
    pylab.title(title)
    pylab.legend(('StandardRobot', 'RandomWalkRobot'))
    pylab.xlabel(x_label)
    pylab.ylabel(y_label)
    pylab.show()


def showPlot2(title, x_label, y_label):
    """
    What information does the plot produced by this function tell you?
    """
    aspect_ratios = []
    times1 = []
    times2 = []
    for width in [10, 20, 25, 50]:
        height = 300/width
        print "Plotting cleaning time for a room of width:", width, "by height:", height
        aspect_ratios.append(float(width) / height)
        times1.append(runSimulation(2, 1.0, width, height, 0.8, 200, StandardRobot))
        times2.append(runSimulation(2, 1.0, width, height, 0.8, 200, RandomWalkRobot))
    pylab.plot(aspect_ratios, times1)
    pylab.plot(aspect_ratios, times2)
    pylab.title(title)
    pylab.legend(('StandardRobot', 'RandomWalkRobot'))
    pylab.xlabel(x_label)
    pylab.ylabel(y_label)
    pylab.show()


# === Problem 5
#
# 1) Write a function call to showPlot1 that generates an appropriately-labeled
#     plot.
#
#       (... your call here ...)
#

#
# 2) Write a function call to showPlot2 that generates an appropriately-labeled
#     plot.
#
#       (... your call here ...)
#

class WheelChair(Robot):
    """
    to track the path traversed by wheelchair
    """
    def __init__(self, room, speed, angular_speed):
        super(WheelChair, self).__init__(room, speed)
        self.position = Position(4.0, 4.0)
        self.direction = math.pi / 2.0
        self.angular_speed = angular_speed
        self.path_tracked = []
        self.status = 0
        self.initial_time = None
        self.final_time = None
        self.ser = serial.Serial('/dev/ttyACM0', 9600)

    def isTileCleaned(self, m, n):
        path = self.path_tracked
        for point in path:
            if (int(point[0]), int(point[1])) == (m, n):
                return True
        return False

    def track_and_move(self, backward=False):
        """
        should be called every one second
        :param backward: if to move in backward direction
        :return: None
        """
        self.path_tracked.append((self.position.x, self.position.y))
        td = self.final_time - self.initial_time
        self.initial_time = None
        self.final_time = None

        delta_x = self.speed * td.seconds * math.cos(self.direction)
        delta_y = self.speed * td.seconds * math.sin(self.direction)

        if backward:
            self.position.x -= delta_x
            self.position.y -= delta_y
        else:
            print("X increment: %f" % delta_x)
            print("Y increment: %f" % delta_y)
            self.position.x += delta_x
            self.position.y += delta_y
            print 'after incrementing position: ', self.position
        self.room.cleanTileAtPosition(self.position)

    def change_direction(self, clockwise=False, theta=None):
        """
        changes the direction of Wheelchair according to duration of rotation
        :param clockwise: if direction is clockwise
        :param theta: angle through which turn should be there
        :return: final direction of the Wheelchair
        """
        if theta:
            self.direction += theta
        else:
            td = self.final_time - self.initial_time
            angle = td.seconds * self.angular_speed
            self.initial_time = None
            self.final_time = None
            print 'changing direction by angle: ', math.degrees(angle), ' degrees'
            if clockwise:
                self.direction -= angle
            else:
                self.direction += angle
        while self.direction > 2*math.pi:
            self.direction -= 2*math.pi
        while self.direction < 0:
            self.direction += 2*math.pi
        return self.direction

    def move(self, anim, room):
        while True:
            try:
                read_status = self.ser.readline()
                read_status = int(read_status[0])
                print 'reading: ', read_status
            except ValueError:
                continue
            if read_status == 1 or read_status == 4 or read_status == 3:
                self.status = read_status
                if not self.initial_time:
                    self.initial_time = datetime.datetime.now()
                    anim.update(room, [self])
                else:
                    self.final_time = datetime.datetime.now()
                    if read_status == 1:
                        self.track_and_move()
                    elif read_status == 2:
                        self.track_and_move(backward=True)
                    elif read_status == 4:
                        self.change_direction(clockwise=True)
                    else:
                        self.change_direction()
                    anim.update(room, [self])
            elif read_status == 0:
                if self.initial_time:
                    self.final_time = datetime.datetime.now()
                    if self.status == 1:
                        self.track_and_move()
                    elif self.status == 4:
                        self.change_direction(clockwise=True)
                    elif self.status == 3:
                        self.change_direction()
                anim.update(room, [self])
            elif read_status == 2:
                print("path tracked by wheelchair:")
                for position in self.path_tracked:
                    print(position)
                print(self.direction/math.pi*180)
                break
        self.retrace(anim)

    def retrace(self, anim):
        """
        retraces the path stored in path_tracked attribute
        :return: True if retraced successfully
        """
        self.u_turn()
        path = self.path_tracked

        while path:
            pos = path.pop()
            distance, theta = WheelChair.get_distance_and_angle(
                (self.position.x, self.position.y),
                pos)
            t = distance / self.speed
            delta_theta = theta - self.direction
            if delta_theta > math.pi:
                delta_theta -= 2 * math.pi
            elif delta_theta < -math.pi:
                delta_theta += 2 * math.pi
            if delta_theta < 0:
                clockwise = True
            else:
                clockwise = False
            print "angle difference: ", math.degrees(delta_theta)
            t_angular = abs(delta_theta) / self.angular_speed
            self.direction = theta
            anim.update(self.room, [self])
            self.turn(t_angular, clockwise)
            self.position.x += math.cos(self.direction)
            self.position.y += math.sin(self.direction)
            anim.update(self.room, [self])
            self.forward(t)
        print("path traced successfully")
        self.ser.write('9')
        return True

    def forward(self, t):
        self.ser.write('1')
        time.sleep(t)
        self.ser.write('0')

    def turn(self, t, clockwise):
        if clockwise:
            self.ser.write('4')
        else:
            self.ser.write('3')
        time.sleep(t)
        self.ser.write('0')

    def u_turn(self):
        """
        makes the chair turn by 180 degree
        :return: True
        """
        t = math.pi / self.angular_speed
        self.turn(t, clockwise=False)
        self.direction += math.pi
        if self.direction > 2 * math.pi:
            self.direction -= 2 * math.pi

    @staticmethod
    def get_random_status():
        choices = [1, 2, 4, 3]
        return random.choice(choices)

    @staticmethod
    def get_distance_and_angle(pos1, pos2):
        """
        calculates distance between two positions and angle between them
        :param pos1: position 1
        :param pos2: position 2
        :return: float, distance between 2 points
        """
        delta_x = pos2[0] - pos1[0]
        delta_y = pos2[1] - pos1[1]
        theta = WheelChair.tan_inv(delta_x, delta_y)
        d = (delta_x**2 + delta_y**2)**.5
        return d, theta

    @staticmethod
    def tan_inv(delta_x, delta_y):
        """
        to calculate inverse of delta_y / delta_x
        :param delta_x: difference in x coordinates
        :param delta_y: difference in y coordinates
        :return: angle in radians, 0 < angle < 2*pi
        """
        if delta_x == 0:  # y-axis
            if delta_y < 0:  # negative y-axis
                return 3 * math.pi / 2
            else:
                return math.pi / 2  # positive y-axis
        angle = math.atan(delta_y / delta_x)
        if delta_x > 0:
            if delta_y < 0:  # IV quadrant
                return 2 * math.pi + angle
            else:  # I quadrant
                return angle
        else:  # II and III quadrants
            return math.pi + angle


if __name__ == '__main__':
    room = RectangularRoom(width=10, height=10)
    angular_speed = 2 * 30 / 180.0 * math.pi
    wheel_chair = WheelChair(room=room, speed=1.0, angular_speed=angular_speed)
    test_wheelchair_movement(wheel_chair, room)

