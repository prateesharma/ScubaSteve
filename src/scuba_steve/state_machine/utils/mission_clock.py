####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import rospy
import time


class MissionClock:
    """Singleton object that initializes and provides functionality for the
    mission clock."""
    _instance = None
    _time = None

    def __init__(self):
        """Initializes the mission clock, or does nothing if the class has
        already been initialized.
        """
        if MissionClock._instance is None:
            rospy.loginfo("Initializing MissionClock class")
            MissionClock._instance = self
            MissionClock._time = time.time()
        else:
            raise Exception("Cannot create another MissionClock class")

    @staticmethod
    def get_instance():
        """Fetches the current instance."""
        if not MissionClock._instance:
            MissionClock()
        return MissionClock._instance

    @classmethod
    def get_minutes(cls):
        """Returns the time since the mission clock was started in minutes."""
        return (time.time() - cls._time) / 60

    @classmethod
    def get_seconds(cls):
        """Returns the time since the mission clock was started in seconds."""
        return time.time() - cls._time

    @classmethod
    def get_time(cls):
        """Returns the time since the mission clock was started in minutes."""
        return cls.get_minutes()

    @classmethod
    def get_mission_state(cls):
        """Returns the mission state based on the time since the mission clock
        was started.
        """
        time = cls.get_time()
        state='EXPLORE'
        if (15 <= time < 20) or (35 <= time < 40) or (55 <= time < 60):
            state='COMMS'
        elif time >= 60:
            state='POWERDOWN'            
        return state

    @classmethod
    def reset(cls):
        """Resets the mission clock to zero."""
        _time = time.time()
