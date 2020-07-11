####################################
# 2019-2020, Lockheed Martin Space #
# TDC Team #2, Scuba Squad         #
####################################

import steve_auv.mission_manager as mm
import time
import unittest

from mm.utils.mission_clock import MissionClock


class TestMissionClock(unittest.TestCase):

    def setUp(self):
        self.mc = MissionClock.get_instance()
        self.mc.reset()

    def test_get_instance(self):
        mc_instance = MissionClock.get_instance()
        self.assertEqual(self.mc, mc_instance)

    def test_get_minutes(self):
        time.sleep(3.0)
        self.assertAlmostEqual(self.mc.get_minutes(), 0.05, 2)

    def test_get_seconds(self):
        time.sleep(3.0)
        self.assertAlmostEqual(self.mc.get_seconds(), 3.0, 1)

    def test_get_time(self):
        time.sleep(3.0)
        self.assertAlmostEqual(self.mc.get_time(), 0.05, 2)

    def test_reset(self):
        time.sleep(3.0)
        self.mc.reset()
        self.assertAlmostEqual(self.mc.get_time(), 0.00, 2)

    def test_set_mission_schedule(self):
        schedule = [
            ( 0, 30, 'POWERDOWN'),
            (30, 60, 'COMMS'),
            (60, 90, 'EXPLORE')
        ]
        self.mc.set_mission_schedule(schedule)
        self.assertEqual(self.mc.get_mission_state(), 'POWERDOWN')

    def test_get_mission_state_explore(self):
        MissionClock._time -= 1 * 60
        self.assertEqual(self.mc.get_mission_state(), 'EXPLORE')

    def test_get_mission_state_comms(self):
        MissionClock._time -= 15 * 60
        self.assertEqual(self.mc.get_mission_state(), 'COMMS')

    def test_get_mission_state_powerdown(self):
        MissionClock._time -= 60 * 60
        self.assertEqual(self.mc.get_mission_state(), 'POWERDOWN')


if __name__ == "__main__":
    unittest.main()
