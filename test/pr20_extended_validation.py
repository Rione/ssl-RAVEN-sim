"""Finite full-factorial kick matrix and a 60-second 22-robot endurance run.
Uses measure_sim_spec's temporary configuration and dedicated test ports.
"""
import itertools, math, pathlib, statistics, subprocess, sys, time
sys.dont_write_bytecode = True
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'tools'))
import measure_sim_spec as m


def velocity(frames, team, rid):
    samples = m.track(frames, team, rid)
    if len(samples) < 3: raise RuntimeError('missing robot velocity samples')
    a, b = samples[-3], samples[-1]
    dt = b[0] - a[0]
    return ((b[1] - a[1]) / dt, (b[2] - a[2]) / dt)


def held_matrix(ctx):
    results = []
    # All 96 combinations per config; speed 10 exercises straight/chip clamps.
    cases = list(itertools.product(('blue', 'yellow'), (0, 90), ('stop', 'forward', 'left'), (2., 10.), (0., 30., 45., 60.)))
    ctx.park_all()
    for number, (team, heading_deg, motion, kick, angle) in enumerate(cases, 1):
        heading = math.radians(heading_deg)
        x, y = -2500., -2000.
        ctx.cmd.set(team, 0)
        ctx.cmd.teleport(robots=[(team, 0, x, y, heading)], ball=(x + 95*math.cos(heading), y + 95*math.sin(heading)))
        ctx.wait(.12)
        ctx.cmd.set(team, 0, dribble=True)
        ctx.wait(.2)
        caught = m.ball_mouth_distance(ctx.vision.latest(), team, 0)
        forward, left = (1.,0.) if motion == 'forward' else (0.,1.) if motion == 'left' else (0.,0.)
        ctx.cmd.set(team, 0, forward=forward, left=left, dribble=True)
        before = ctx.vision.mark()
        ctx.wait(.7)
        pre = ctx.vision.since(before)
        vx, vy = velocity(pre, team, 0)
        current = ctx.vision.latest()
        h = current.robot(team, 0)[2]
        mark = ctx.vision.mark()
        log = len(ctx.sim.log_text())
        ctx.cmd.set(team, 0, forward=forward, left=left, dribble=True, kick=kick, kick_angle_deg=angle)
        ctx.wait(.06)
        ctx.cmd.set(team, 0, forward=forward, left=left, dribble=True)
        # Include the apex of the fastest 60-degree chip.
        ctx.wait(.95)
        frames = ctx.vision.since(mark)
        rates, pts = m.ball_speed_series(frames)
        k = min(kick * 1000, ctx.max_straight if angle == 0 else ctx.max_chip) * ctx.kicker_friction
        # Match actual center velocity, not the commanded 1 m/s (RAVEN has gain/delay).
        expected = math.hypot(vx + k*math.cos(math.radians(angle))*math.cos(h), vy + k*math.cos(math.radians(angle))*math.sin(h))
        expected_apex = 25 + (k*math.sin(math.radians(angle)))**2/(2*9800)
        observed = max((v for _,v,_,_ in rates), default=0)
        height = max((p[3] for p in pts), default=0)
        fired = ctx.kicks_fired(log, 'b0' if team == 'blue' else 'y0')
        errors = []
        if caught is None or caught > 3: errors.append('initial catch')
        if fired != 1: errors.append(f'fires={fired}')
        if abs(observed - expected) > max(200, expected*.08): errors.append('horizontal speed')
        if angle and abs(height - expected_apex) > max(50, expected_apex*.12): errors.append('chip apex')
        if not all(math.isfinite(v) for f in frames for v in (f.ball or ())): errors.append('nonfinite')
        results.append(dict(team=team, heading_deg=heading_deg, motion=motion, kick_m_s=kick, angle_deg=angle,
                            expected_horizontal=expected, observed_horizontal=observed, expected_apex=expected_apex,
                            observed_apex=height, fired=fired, errors=errors))
        ctx.cmd.set(team, 0)
        # The other robot stays out of subsequent shots.
        ctx.cmd.teleport(robots=[(team, 0, -5000, -4300 if team=='blue' else 4300, 0)])
        if number % 12 == 0:
            print(f'matrix {number}/{len(cases)}, failures={sum(bool(r["errors"]) for r in results)}', flush=True)
        # Bound memory between trials; retain the returned numerical report.
        with ctx.vision.lock: ctx.vision.frames[:] = ctx.vision.frames[-120:]
    return dict(cases=len(results), failures=sum(bool(r['errors']) for r in results), trials=results)


def endurance(ctx):
    # 22 independent moving circles + repeated ball placements, straight kicks and chips.
    positions = {}
    for team in ('blue', 'yellow'):
        for rid in range(11):
            # Two rows per team, pitch wide enough for the small circles.
            col, row = rid % 6, rid // 6
            positions[team,rid] = (-4700 + col*1750, (-2800 + row*1200) if team=='blue' else (1100 + row*1200))
    ctx.cmd.zero_all(11)
    ctx.cmd.teleport(ball=(0,0), robots=[(team,rid,x,y,0) for (team,rid),(x,y) in positions.items()])
    ctx.wait(.4)
    for (team,rid) in positions:
        ctx.cmd.set(team,rid,forward=.35,angular=1.2,dribble=True,kick=2,kick_angle_deg=45 if rid%2 else 0)
    start = time.monotonic()
    start_sim = ctx.now()
    seconds_seen = set()
    frames_count = 0
    invalid = []
    missing = 0
    max_gap = 0
    prev = None
    rss = []
    kicks_from = len(ctx.sim.log_text())
    initial = ctx.vision.latest()
    moved = set()
    mark = ctx.vision.mark()
    next_place = 0
    cycle = 0
    # Wall-clock endurance time requested by the user (one minute).
    while time.monotonic()-start < 60:
        if ctx.sim.proc.poll() is not None: raise RuntimeError('simulator exited during endurance')
        elapsed = time.monotonic()-start
        if elapsed >= next_place:
            team = 'blue' if cycle%2==0 else 'yellow'
            rid = (cycle//2)%11
            fr = ctx.vision.latest(); r = fr.robot(team,rid)
            ctx.cmd.teleport(ball=m.mouth_point(r))
            next_place += .6; cycle += 1
        time.sleep(.05)
        new = ctx.vision.since(mark); mark += len(new)
        if new: seconds_seen.add(int(elapsed))
        for f in new:
            frames_count += 1
            if len(f.blue)!=11 or len(f.yellow)!=11: missing += 1
            if prev is not None: max_gap = max(max_gap, f.t-prev)
            prev=f.t
            for team in ('blue','yellow'):
                for rid in range(11):
                    r=f.robot(team,rid); a=initial.robot(team,rid)
                    if r is None: continue
                    if a and math.hypot(r[0]-a[0],r[1]-a[1])>100: moved.add((team,rid))
                    if not all(math.isfinite(v) for v in r) or abs(r[0])>7000 or abs(r[1])>5500:
                        invalid.append(dict(t=f.t,team=team,id=rid,pose=r))
            if f.ball and (not all(math.isfinite(v) for v in f.ball) or abs(f.ball[0])>10000 or abs(f.ball[1])>8000):
                invalid.append(dict(t=f.t,ball=f.ball))
        if int(elapsed)%10==0 and (not rss or rss[-1][0]!=int(elapsed)):
            mem=subprocess.run(['ps','-o','rss=','-p',str(ctx.sim.proc.pid)],capture_output=True,text=True)
            rss.append((int(elapsed),int(mem.stdout.strip() or 0)))
        # No unbounded frame history during long runs.
        if mark>500:
            with ctx.vision.lock: ctx.vision.frames[:mark-120]=[]
            mark=120
    log=ctx.sim.log_text()
    severe=[s for s in log.splitlines() if any(e in s for e in ('TypeError:','ReferenceError:','Segmentation','ASSERT:'))]
    return dict(wall_seconds=time.monotonic()-start,physics_seconds=ctx.now()-start_sim,
                frames=frames_count,seconds_with_vision=len(seconds_seen),missing_robot_frames=missing,
                moved_robots=len(moved),max_physics_gap=max_gap,invalid=invalid[:20],invalid_count=len(invalid),
                rss_kib=rss,kicks_blue=ctx.kicks_fired(kicks_from,'b') if False else sum('[kick] b' in l and ' fires ' in l for l in log[kicks_from:].splitlines()),
                kicks_yellow=sum('[kick] y' in l and ' fires ' in l for l in log[kicks_from:].splitlines()),errors=severe)

m.SCENES['held_matrix']=held_matrix
m.SCENES['endurance']=endurance
if __name__=='__main__':sys.exit(m.main())
