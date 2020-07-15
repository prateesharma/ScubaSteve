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

    def test_set_mission_schedule(self):
        schedule = [
            ( 0, 'POWERDOWN'),
            (30, 'COMMS'),
            (60, 'EXPLORE')
        ]
        self.mc.set_mission_schedule(schedule)
        self.assertEqual(self.mc.get_mission_state(), 'POWERDOWN')

    def test_next_state(self):
        self.assertEqual(self.mc._current_state, 0)
        self.mc.next_state()
        self.assertEqual(self.mc._current_state, 1)

    def test_reset(self):
        time.sleep(3.0)
        self.mc.reset()
        self.assertAlmostEqual(self.mc.get_time(), 0.00, 2)

    def test_get_minutes(self):
        time.sleep(3.0)
        self.assertAlmostEqual(self.mc.get_minutes(), 0.05, 2)

    def test_get_seconds(self):
        time.sleep(3.0)
        self.assertAlmostEqual(self.mc.get_seconds(), 3.0, 1)

    def test_get_time(self):
        time.sleep(3.0)
        self.assertAlmostEqual(self.mc.get_time(), 0.05, 2)

    def test_get_mission_state(self):
        self.mc._current_state = 3
        self.assertEqual(self.mc.get_mission_state(), 3)

    def test_get_time_until_next_state(self):
        schedule = [
            ( 0, 'COMMS'),
            (30, 'EXPLORE'),
            (60, 'POWERDOWN'),
        ]
        self.mc.set_mission_schedule(schedule)
        MissionClock._time = 15 * 60
        self.assertEqual(self.mc.get_time_until_next_state(), 15 * 60)
        MissionClock.next_state()
        self.assertEqual(self.mc.get_time_until_next_comms(), 45 * 60)
        MissionClock.next_state()
        self.assertEqual(self.mc.get_time_until_next_comms(), None)


if __name__ == "__main__":
    unittest.main()
