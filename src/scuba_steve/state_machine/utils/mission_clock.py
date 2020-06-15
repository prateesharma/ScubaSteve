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

    # Default mission schedule
    _schedule = [
        ( 0, 15, 'EXPLORE'),
        (15, 20, 'COMMS'),
        (20, 35, 'EXPLORE'),
        (35, 40, 'COMMS'),
        (40, 55, 'EXPLORE'),
        (55, 60, 'COMMS'),
        (60, 90, 'POWERDOWN')
    ]

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
    def reset(cls):
        """Resets the mission clock to zero."""
        cls._time = time.time()

    @classmethod
    def set_mission_schedule(cls, schedule):
        """Overrides the current (or default) mission schedule."""
        cls._schedule = schedule

    @classmethod
    def get_mission_state(cls):
        """Returns the mission state based on the time since the mission clock
        was started.
        """
        time = cls.get_time()
        state = 'POWERDOWN'
        for window in cls._schedule:
            if window[0] <= time < window[1]:
                state = window[2]
        return state
