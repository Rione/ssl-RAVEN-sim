import sys, pathlib, unittest, math
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools'))
sys.dont_write_bytecode = True
import measure_sim_spec as m

class ReboundTest(unittest.TestCase):
    def frames(self, body_speed=0, angle=0):
        # Known collision followed by deceleration > old 75 mm/s threshold each frame.
        speeds = [-2000, -2000, -2000, 200, 1600, 1500, 1400, 1300, 1200, 1100, 1000]
        dt, x = 1 / 60, 100
        frames = []
        for i in range(len(speeds) + 1):
            t = i * dt
            rx, ry = body_speed * t * math.cos(angle), body_speed * t * math.sin(angle)
            frames.append(m.Frame(t, (rx + x * math.cos(angle), ry + x * math.sin(angle), 21),
                                  {0: (rx, ry, angle)}, {}))
            if i < len(speeds): x += speeds[i] * dt
        return frames

    def test_floor_deceleration_not_rebound_loss(self):
        r = m.analyze_rebound(self.frames(), 'blue')
        self.assertAlmostEqual(r['normal_ratio'], .8)

    def test_moving_rotated_body(self):
        r = m.analyze_rebound(self.frames(1000, .6), 'blue')
        self.assertAlmostEqual(r['normal_ratio'], .8)

    def test_missing_samples(self):
        self.assertIn('error', m.analyze_rebound([], 'blue'))

if __name__ == '__main__': unittest.main()
