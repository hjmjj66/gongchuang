"""Startup, target handoff and real stream failure through the shared gate."""

from pathlib import Path
import unittest

from agv_perception.vision_gate import VisionGate
from agv_perception.raw_control import RawTask, read_raw_config
from agv_perception.work_control import Observation, Pose


class VisionGateTests(unittest.TestCase):
    def setUp(self):
        p = read_raw_config(Path(__file__).parents[1]/'config/raw.yaml')
        self.task = RawTask(p, 'cold-start', (3, 5, 4), 0.)
        self.task._enter('WAIT_REQUIRED', 2.)
        self.task.visual_started = 2.
        self.gate = VisionGate()

    def observation(self, at, target=0):
        return Observation('material', round(at*100), at, (1280, 720), {}, (), geometry_target=target)

    def test_slow_first_inference_waits_then_accepts_new_capture(self):
        # Same timeline as the reported failure: work pose completes, then cold inference.
        obs = self.gate.filter(self.task, self.observation(2.1), 3., 'raw')
        self.assertIsNone(obs)
        self.task.tick(3., Pose(0., 0., 0., 3.), obs)
        self.assertEqual(self.task.state, 'WAIT_REQUIRED')
        obs = self.observation(3.1)
        self.assertIs(self.gate.filter(self.task, obs, 3.2, 'raw'), obs)

    def test_missing_material_with_fresh_frames_has_no_business_timeout(self):
        for now in range(3, 100):
            obs = self.gate.filter(self.task, self.observation(now-.1), now, 'raw')
            self.task.tick(now, Pose(0., 0., 0., now), obs)
            self.assertEqual(self.task.state, 'WAIT_REQUIRED')
            self.assertEqual(self.task.arm_velocity, (0., 0.))

    def test_established_stream_pauses_on_late_frame_and_recovers(self):
        obs = self.observation(3.)
        self.gate.filter(self.task, obs, 3.1, 'raw')
        self.task.arm_velocity, self.task.servo_enabled = (0.2, 10.), True
        self.assertIsNone(self.gate.filter(self.task, obs, 3.503, 'raw'))
        self.assertEqual(self.task.arm_velocity, (0., 0.))
        self.assertFalse(self.task.servo_enabled)
        fresh = self.observation(3.6)
        self.assertIs(self.gate.filter(self.task, fresh, 3.7, 'raw'), fresh)
        for now in (13.7, 3600., 86400.):
            self.assertIsNone(self.gate.filter(self.task, None, now, 'raw'))
            self.assertEqual(self.task.arm_velocity, (0., 0.))
        recovered = self.observation(86400.1)
        self.assertIs(self.gate.filter(self.task, recovered, 86400.2, 'raw'), recovered)

    def test_no_first_frame_waits_indefinitely_then_recovers(self):
        self.gate.filter(self.task, self.observation(2.1), 3., 'raw')
        for now in (13.1, 3600.):
            self.assertIsNone(self.gate.filter(self.task, self.observation(2.1), now, 'raw'))
            self.task.tick(now, Pose(0., 0., 0., now), None)
            self.assertEqual(self.task.state, 'WAIT_REQUIRED')
        fresh = self.observation(3600.1)
        self.assertIs(self.gate.filter(self.task, fresh, 3600.2, 'raw'), fresh)

    def test_target_switch_drops_previous_target_zero_and_uses_new_startup_wait(self):
        old = self.observation(3.)
        self.gate.filter(self.task, old, 3.1, 'raw')
        self.task._enter('WAIT_REQUIRED', 4.)
        self.assertIsNone(self.gate.filter(self.task, old, 4.1, 'raw'))
        new = self.observation(4.2, 3)
        self.assertIs(self.gate.filter(self.task, new, 4.3, 'raw'), new)
