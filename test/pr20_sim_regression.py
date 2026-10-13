"""Run with measure_sim_spec's --exe/--config arguments; uses isolated ports and config copies."""
import sys, pathlib, time, math
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools'))
sys.dont_write_bytecode = True
import measure_sim_spec as m


def wait_physics(ctx, seconds):
    end = ctx.now() + seconds
    deadline = time.monotonic() + max(15, seconds * 10)
    while ctx.now() < end:
        if time.monotonic() > deadline:
            raise RuntimeError('physics clock stopped')
        time.sleep(.01)


def soft_kick(ctx):
    ctx.park_all()
    out = {}
    for team, y, key in (('blue', 1500, 'b0'), ('yellow', -1500, 'y0')):
        held = ctx.give_ball(team, 0, -3000, y)
        assert held, f'{team}: initial catch failed'
        # RAVEN の遅延・一次遅れが落ち着くまで、物理時刻で停止を待つ。
        wait_physics(ctx, 1.0)
        log = len(ctx.sim.log_text())
        ctx.cmd.set(team, 0, kick=.1, dribble=True)
        wait_physics(ctx, .05)
        ctx.cmd.set(team, 0, dribble=True)
        wait_physics(ctx, 1.5)
        first = ctx.kicks_fired(log, key)
        ctx.cmd.set(team, 0, kick=2, dribble=True)
        wait_physics(ctx, .05)
        ctx.cmd.set(team, 0, dribble=True)
        wait_physics(ctx, .6)
        total = ctx.kicks_fired(log, key)
        distance = m.ball_mouth_distance(ctx.vision.latest(), team, 0)
        assert first == 1 and total == 2, (team, first, total, distance)
        assert distance > 100, (team, distance)
        out[team] = dict(soft_kicks=first, total_kicks=total, distance_after_second=distance)
        ctx.cmd.set(team, 0)
    return out


def vectors(ctx):
    ctx.park_all()
    out = {}
    for team, y in (('blue', 2000), ('yellow', -2000)):
        for deg in (0, 45, 90):
            a = math.radians(deg)
            ctx.place(team, 0, -4000, -2500 if deg else y, 0)
            mark = ctx.vision.mark()
            t0 = ctx.now()
            ctx.cmd.set(team, 0, forward=4 * math.cos(a), left=4 * math.sin(a))
            wait_physics(ctx, 1.5)
            release = ctx.now()
            ctx.cmd.set(team, 0)
            wait_physics(ctx, 1)
            result = m.step_response([(t, v) for t, v, _, _ in m.speed_series(m.track(ctx.vision.since(mark), team, 0))], t0, release)
            assert abs(result['top'] - 4000) < 50, result
            assert abs(result['rise_slope'] - 3500) < 180, result
            assert abs(result['fall_slope'] - 6000) < 350, result
            out[f'{team}_{deg}'] = result
    return out

m.SCENES['soft_kick'] = soft_kick
m.SCENES['vectors'] = vectors
# Small representative stationary set. Moving cases require actual RAVEN trajectory alignment.
m.direct_trials = lambda: [(k, 0, v, a, 0) for k in (0, 3) for v in (2000, 4000) for a in (0, 30)]
if __name__ == '__main__': sys.exit(m.main())
