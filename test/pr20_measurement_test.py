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

class KickSlideTest(unittest.TestCase):
    """A ball kicked along +x at v0 from the mouth, sampled at 60 Hz: slide at a_slide down to k * v0, then roll."""
    def frames(self, v0=3000.0, a_slide=3000.0, a_roll=260.0, k=0.64, slides=True):
        dt, x, v, t = 1 / 60, 95.0, 0.0, 0.0
        frames = [m.Frame(t, (x, 0.0, 21.0), {}, {0: (0.0, 0.0, 0.0)})]
        for i in range(150):
            if i == 3:
                v = v0
            elif v > 0:
                a = a_slide if slides and v > k * v0 else a_roll
                v = max(0.0, v - a * dt)
            x += v * dt
            t += dt
            frames.append(m.Frame(t, (x, 0.0, 21.0), {}, {0: (0.0, 0.0, 0.0)}))
        return frames

    def test_sliding_kick(self):
        # 4 m/s slides for 0.48 s, past the 0.1-0.4 s window
        r = m.analyze_kick_slide(None, self.frames(v0=4000.0), 4000.0)
        self.assertAlmostEqual(r['decel_0_1_to_0_4_s_mm_s2'], 3000.0, delta=100.0)
        self.assertAlmostEqual(r['slide_end_speed_mm_s'], 2560.0, delta=60.0)

    def test_kick_that_starts_rolling(self):
        r = m.analyze_kick_slide(None, self.frames(v0=4000.0, slides=False), 4000.0)
        self.assertAlmostEqual(r['decel_0_1_to_0_4_s_mm_s2'], 260.0, delta=30.0)

    def test_no_kick(self):
        self.assertIn('error', m.analyze_kick_slide(None, self.frames(v0=0.0), 3000.0))



class ChipHopsTest(unittest.TestCase):
    """A chip at 45 deg along +x, sampled at 60 Hz. Each floor bounce keeps first / later of the horizontal speed
    and 0.47 of the vertical one; hops below 200 mm/s stop."""
    def frames(self, first, later, v=4000.0):
        g, r, sub = 9810.0, 21.0, 10
        dt = 1 / 600
        vx = vz = v / 2 ** 0.5
        x, z, t, bounces = 95.0, r, 0.0, 0
        frames = [m.Frame(-3 / 60 + i / 60, (x, 0.0, z), {}, {0: (0.0, 0.0, 0.0)}) for i in range(3)]
        for i in range(1, 600 * 3):
            vz -= g * dt if vz != 0.0 else 0.0
            x += vx * dt
            z += vz * dt
            if z < r:
                z = r
                up = -0.47 * vz
                vx *= first if bounces == 0 else later
                bounces += 1
                vz = up if up > 200.0 else 0.0
            t += dt
            if i % sub == 0:
                frames.append(m.Frame(t, (x, 0.0, z), {}, {0: (0.0, 0.0, 0.0)}))
        return frames

    def test_kept_fractions(self):
        r = m.chip_hops(self.frames(0.55, 0.88))
        self.assertAlmostEqual(r['first_landing_kept'], 0.55, delta=0.02)
        self.assertTrue(r['later_landings_kept'])
        for k in r['later_landings_kept']:
            self.assertAlmostEqual(k, 0.88, delta=0.03)

    def test_frictionless_floor(self):
        r = m.chip_hops(self.frames(1.0, 1.0))
        self.assertAlmostEqual(r['first_landing_kept'], 1.0, delta=0.01)

    def test_not_chipped(self):
        flat = [m.Frame(i / 60, (95.0 + 50.0 * i, 0.0, 21.0), {}, {0: (0.0, 0.0, 0.0)}) for i in range(60)]
        self.assertIn('error', m.chip_hops(flat))


if __name__ == '__main__': unittest.main()
