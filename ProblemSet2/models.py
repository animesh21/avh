import datetime

from mongokit import Document, Connection


connection = Connection()


@connection.register
class WheelChairModel(Document):
    __database__ = 'anveshana'
    __collection__ = 'wheelchairs'

    structure = {
        'name': basestring,
        'speed': float,
        'angular_speed': float,
        'room': list,
        'direction': float,
        'path_tracked': list,
        'status': int,
        'initial_time': datetime.datetime,
        'final_time': datetime.datetime
    }

    required_fields = ['room', 'path_tracked']
