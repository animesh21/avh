import math
import ps2_visualize


def testRobotMovement(robot_type, room_type, delay = 0.4):
    """
    Runs a simulation of a single robot of type robot_type in a 5x5 room.
    """
    angular_speed = 30 / 180.0 * math.pi
    room = room_type(20, 20)
    robot = robot_type(room, 1.0, angular_speed)
    anim = ps2_visualize.RobotVisualization(1, 10, 10, delay)
    while room.getNumCleanedTiles() < room.getNumTiles():
        robot.updatePositionAndClean()
        anim.update(room, [robot])

    anim.done()


def test_wheelchair_movement(wheelchair, room, delay=.2):
    """
    Runs a simulation of a single wheelchair
    :param wheelchair: WheelChair object
    :param room: Room object of the room
    :param delay: delay between two motions
    :return: None
    """
    print 'simulator function called'
    anim = ps2_visualize.WheelChairVisualization(1, 10, 10, delay)
    print 'animator created and calling move()'
    wheelchair.move(anim, room)
    anim.done()
