#!/usr/bin/env python3
"""sim の台と球が設定どおりに動くかを、sim を 1 つ起動して vision で測る。

RAVEN も Sumatra も起動しない。指令は 2 つの口から送る:
  青 = ssl-simulation-protocol の RobotControl (Sumatra が使う口)
  黄 = mocSim (RAVEN が使う口)
測るのは vision に出る姿勢と球の位置、それに sim の標準出力の蹴りの行だけ。sim の中の値は読まない。

sim は一時ディレクトリに作った置き場から起動する。設定はポートだけを試験用にずらした写しを置く。
並行して動いている別の sim とぶつからないためと、sim が起動のたびに設定ファイルを書き戻しても
リポジトリの config_v2.ini に響かないため。

  python3 tools/measure_sim_spec.py --exe build/bin/m2-Sim
  python3 tools/measure_sim_spec.py --exe build/bin/m2-Sim --only accel,receive --json out.json

--repo には src/ と assets/ と config/ と proto/ のある木を渡す (既定はこのファイルのある木)。
protoc が PATH に要る。受ける場面の組は RECEIVE_CASES=0:2500,1000:4500 (台の x 方向の速さ:球の速さ、
mm/s) で差し替えられる。

遅れ (delay_s) は指令を出した時点で最後に届いていたフレームから数えるので、±2 フレームほどの幅がある。
"""
import argparse
import configparser
import json
import math
import os
import pathlib
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

HERE_REPO = pathlib.Path(__file__).resolve().parent.parent

# 試験用のポート。ふだんの sim (10694 / 20694 / 10301 / 10302 / 16941) と重ならない値。
TEST_PORTS = {
    'commandListenPort': 20794,
    'visionMulticastPort': 10794,
    'blueTeamControlPort': 10401,
    'yellowTeamControlPort': 10402,
    'FeedbackPort': 16999,
}

# 口の前の球の位置 [mm]。sim は持っている球を機体の中心から向きの方へこの距離に見せる。
MOUTH_MM = 95.0

PROTOS = [
    'ssl_vision_detection.proto', 'ssl_vision_geometry.proto', 'ssl_vision_wrapper.proto',
    'ssl_simulation_robot_control.proto',
    'mocSim_Commands.proto', 'mocSim_Replacement.proto', 'mocSim_Packet.proto',
]


def compile_protos(proto_dir, out_dir):
    subprocess.run(['protoc', f'-I{proto_dir}', f'--python_out={out_dir}', *PROTOS], check=True)
    sys.path.insert(0, str(out_dir))


def read_ini(path):
    cp = configparser.ConfigParser(strict=False, interpolation=None,
                                   comment_prefixes=(';', '#'), inline_comment_prefixes=(';',))
    cp.optionxform = str
    cp.read(path, encoding='utf-8')
    return cp


def ini_float(cp, section, key, default):
    try:
        return float(cp.get(section, key))
    except (configparser.Error, ValueError):
        return default


def write_test_ini(src, dst):
    text = pathlib.Path(src).read_text(encoding='utf-8', errors='replace')
    for key, port in TEST_PORTS.items():
        text, n = re.subn(rf'^{key}=.*$', f'{key}={port}', text, flags=re.M)
        if n == 0:
            section = 'Encoder' if key == 'FeedbackPort' else 'Network'
            text += f'\n[{section}]\n{key}={port}\n'
    pathlib.Path(dst).write_text(text, encoding='utf-8')


# ---------------------------------------------------------------- sim の起動と停止

class SimProcess:
    """一時の置き場に実行ファイルの写しを置いて起動する。止めるのは自分が起動したものだけ。"""

    def __init__(self, exe, repo, config):
        self.root = pathlib.Path(tempfile.mkdtemp(prefix='measure-sim-'))
        (self.root / 'build' / 'bin').mkdir(parents=True)
        (self.root / 'config').mkdir()
        shutil.copy2(exe, self.root / 'build' / 'bin' / 'm2-Sim')
        os.symlink(pathlib.Path(repo, 'src').resolve(), self.root / 'src')
        os.symlink(pathlib.Path(repo, 'assets').resolve(), self.root / 'assets')
        write_test_ini(config, self.root / 'config' / 'config_v2.ini')
        legacy = pathlib.Path(repo, 'config', 'config.ini')
        if legacy.is_file():
            shutil.copy2(legacy, self.root / 'config' / 'config.ini')
        self.log_path = self.root / 'sim.log'
        self.proc = None

    def start(self):
        env = dict(os.environ, QT_QPA_PLATFORM='offscreen', QSG_RENDER_LOOP='basic',
                   QT_QUICK_CONTROLS_STYLE='Basic')
        self.log = open(self.log_path, 'w')
        self.proc = subprocess.Popen([str(self.root / 'build' / 'bin' / 'm2-Sim')],
                                     cwd=self.root / 'build', env=env,
                                     stdout=self.log, stderr=subprocess.STDOUT,
                                     start_new_session=True)

    def log_text(self):
        return self.log_path.read_text(errors='replace')

    def stop(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait(timeout=5)
        self.proc = None


# ---------------------------------------------------------------- vision と指令

class Frame:
    __slots__ = ('t', 'ball', 'blue', 'yellow')

    def __init__(self, t, ball, blue, yellow):
        self.t = t
        self.ball = ball
        self.blue = blue
        self.yellow = yellow

    def robot(self, team, rid):
        return (self.blue if team == 'blue' else self.yellow).get(rid)


class Vision(threading.Thread):
    def __init__(self, group, port, iface):
        super().__init__(daemon=True)
        import ssl_vision_wrapper_pb2
        self.pb = ssl_vision_wrapper_pb2
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.bind(('', port))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                             struct.pack('4s4s', socket.inet_aton(group), socket.inet_aton(iface)))
        self.sock.settimeout(0.2)
        self.frames = []
        self.lock = threading.Lock()
        self.running = True

    def run(self):
        while self.running:
            try:
                data = self.sock.recv(65535)
            except socket.timeout:
                continue
            pkt = self.pb.SSL_WrapperPacket()
            try:
                pkt.ParseFromString(data)
            except Exception:
                continue
            if not pkt.HasField('detection'):
                continue
            d = pkt.detection
            ball = (d.balls[0].x, d.balls[0].y, d.balls[0].z) if d.balls else None
            blue = {r.robot_id: (r.x, r.y, r.orientation) for r in d.robots_blue}
            yellow = {r.robot_id: (r.x, r.y, r.orientation) for r in d.robots_yellow}
            with self.lock:
                self.frames.append(Frame(d.t_capture, ball, blue, yellow))

    def mark(self):
        with self.lock:
            return len(self.frames)

    def since(self, mark):
        with self.lock:
            return list(self.frames[mark:])

    def latest(self):
        with self.lock:
            return self.frames[-1] if self.frames else None

    def wait_frames(self, n, timeout):
        start = self.mark()
        end = time.time() + timeout
        while time.time() < end:
            if self.mark() - start >= n:
                return True
            time.sleep(0.02)
        return False


class Commander(threading.Thread):
    """両チームの指令を 100 Hz で送り続ける。sim は最後に届いた指令を持ち続けるが、
    Sumatra も RAVEN も毎周期送るので、それに合わせる。"""

    def __init__(self, blue_port, command_port):
        super().__init__(daemon=True)
        import mocSim_Packet_pb2
        import ssl_simulation_robot_control_pb2
        self.moc = mocSim_Packet_pb2
        self.ctl = ssl_simulation_robot_control_pb2
        self.blue_addr = ('127.0.0.1', blue_port)
        self.command_addr = ('127.0.0.1', command_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.cmds = {'blue': {}, 'yellow': {}}
        self.lock = threading.Lock()
        self.running = True

    def set(self, team, rid, forward=0.0, left=0.0, angular=0.0, kick=0.0, kick_angle_deg=0.0,
            dribble=False):
        with self.lock:
            self.cmds[team][rid] = dict(forward=forward, left=left, angular=angular, kick=kick,
                                        kick_angle_deg=kick_angle_deg, dribble=dribble)

    def zero_all(self, count):
        for rid in range(count):
            self.set('blue', rid)
            self.set('yellow', rid)

    def _send(self):
        with self.lock:
            blue = dict(self.cmds['blue'])
            yellow = dict(self.cmds['yellow'])
        if blue:
            msg = self.ctl.RobotControl()
            for rid, c in blue.items():
                rc = msg.robot_commands.add()
                rc.id = rid
                v = rc.move_command.local_velocity
                v.forward = c['forward']
                v.left = c['left']
                v.angular = c['angular']
                if c['kick'] > 0:
                    rc.kick_speed = c['kick']
                    rc.kick_angle = c['kick_angle_deg']
                rc.dribbler_speed = 1000.0 if c['dribble'] else 0.0
            self.sock.sendto(msg.SerializeToString(), self.blue_addr)
        if yellow:
            pkt = self.moc.mocSim_Packet()
            pkt.commands.timestamp = time.time()
            pkt.commands.isteamyellow = True
            for rid, c in yellow.items():
                rc = pkt.commands.robot_commands.add()
                rc.id = rid
                a = math.radians(c['kick_angle_deg'])
                rc.kickspeedx = c['kick'] * math.cos(a)
                rc.kickspeedz = c['kick'] * math.sin(a)
                rc.veltangent = c['forward']
                rc.velnormal = c['left']
                rc.velangular = c['angular']
                rc.spinner = c['dribble']
                rc.wheelsspeed = False
            self.sock.sendto(pkt.SerializeToString(), self.command_addr)

    def teleport(self, ball=None, robots=()):
        """ball = (x, y[, vx, vy]) [mm, mm/s]、robots = [(team, id, x, y, dir_rad)]。"""
        pkt = self.moc.mocSim_Packet()
        rep = pkt.replacement
        if ball is not None:
            rep.ball.x = ball[0] / 1000.0
            rep.ball.y = ball[1] / 1000.0
            if len(ball) >= 4:
                rep.ball.vx = ball[2] / 1000.0
                rep.ball.vy = ball[3] / 1000.0
        for team, rid, x, y, d in robots:
            rr = rep.robots.add()
            rr.id = rid
            rr.x = x / 1000.0
            rr.y = y / 1000.0
            rr.dir = d
            rr.yellowteam = team == 'yellow'
            rr.turnon = True
        self.sock.sendto(pkt.SerializeToString(), self.command_addr)

    def run(self):
        while self.running:
            try:
                self._send()
            except OSError:
                pass
            time.sleep(0.01)


# ---------------------------------------------------------------- 解析の道具

def track(frames, team, rid):
    out = []
    for f in frames:
        r = f.robot(team, rid)
        if r is not None:
            out.append((f.t, r[0], r[1], r[2]))
    return out


def unwrap(angles):
    out = []
    prev = None
    acc = 0.0
    for a in angles:
        if prev is not None:
            d = a - prev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            acc += d
        else:
            acc = a
        out.append(acc)
        prev = a
    return out


def central_rates(ts, values):
    out = []
    for i in range(1, len(ts) - 1):
        dt = ts[i + 1] - ts[i - 1]
        if dt > 0:
            out.append((ts[i], (values[i + 1] - values[i - 1]) / dt))
    return out


def speed_series(tr):
    ts = [p[0] for p in tr]
    vx = central_rates(ts, [p[1] for p in tr])
    vy = central_rates(ts, [p[2] for p in tr])
    return [(a[0], math.hypot(a[1], b[1]), a[1], b[1]) for a, b in zip(vx, vy)]


def omega_series(tr):
    ts = [p[0] for p in tr]
    return central_rates(ts, unwrap([p[3] for p in tr]))


def fit_line(points):
    n = len(points)
    if n < 2:
        return None
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    if sxx <= 0:
        return None
    slope = sum((p[0] - mx) * (p[1] - my) for p in points) / sxx
    return slope, my - slope * mx


def median(values):
    s = sorted(values)
    if not s:
        return None
    m = len(s) // 2
    return s[m] if len(s) % 2 else 0.5 * (s[m - 1] + s[m])


def step_response(series, t_cmd, t_release):
    """速さ (または角速度) の段差応答から、頭打ちの値・立ち上がりの傾き・遅れ・止まるときの傾きを出す。
    傾きは頭打ちの 20〜80 % の区間の直線あてはめ。遅れはその直線が 0 を切る時刻 − 指令を出した時刻。"""
    run = [(t, v) for t, v in series if t_cmd <= t <= t_release]
    if not run:
        return {}
    top = median([v for t, v in run if t >= t_release - 0.4])
    peak = max(v for t, v in run)
    rise = []
    for t, v in run:
        if v >= 0.9 * top:
            break
        if 0.2 * top <= v <= 0.8 * top:
            rise.append((t, v))
    after = [(t, v) for t, v in series if t > t_release]
    fall = []
    started = False
    for t, v in after:
        if v < 0.9 * top:
            started = True
        if started and 0.2 * top <= v <= 0.8 * top:
            fall.append((t, v))
        if started and v < 0.2 * top:
            break
    out = {'top': top, 'peak': peak}
    up = fit_line(rise)
    if up and up[0] > 0:
        out['rise_slope'] = up[0]
        out['delay_s'] = -up[1] / up[0] - t_cmd
    first = next((t for t, v in run if v > 0.05 * top), None)
    if first is not None:
        out['first_motion_s'] = first - t_cmd
    down = fit_line(fall)
    if down:
        out['fall_slope'] = -down[0]
    return out


def mouth_point(robot):
    x, y, th = robot
    return x + MOUTH_MM * math.cos(th), y + MOUTH_MM * math.sin(th)


def ball_mouth_distance(frame, team, rid):
    r = frame.robot(team, rid)
    if r is None or frame.ball is None:
        return None
    mx, my = mouth_point(r)
    return math.hypot(frame.ball[0] - mx, frame.ball[1] - my)


def ball_speed_series(frames):
    pts = [(f.t, f.ball[0], f.ball[1], f.ball[2]) for f in frames if f.ball is not None]
    return speed_series([(t, x, y, 0.0) for t, x, y, z in pts]), pts


# ---------------------------------------------------------------- 試験の場面

class Ctx:
    def __init__(self, vision, cmd, sim, ini, robots_per_team, trace_dir=None):
        self.trace_dir = trace_dir
        self.vision = vision
        self.cmd = cmd
        self.sim = sim
        self.ini = ini
        self.n = robots_per_team
        self.ball_slide = abs(ini_float(ini, 'BallModel', 'AccSlideMmS2', 2159.32))
        self.ball_roll = abs(ini_float(ini, 'BallModel', 'AccRollMmS2', 213.61))
        self.ball_switch = ini_float(ini, 'BallModel', 'KSwitch', 2.0 / 3.0)

    def dump(self, name, frames, team='blue'):
        """--trace のとき、場面の生の vision を 1 行 1 フレームで書く (t, 球 x y z, 台 x y 向き)。"""
        if not self.trace_dir:
            return
        with open(pathlib.Path(self.trace_dir, name + '.csv'), 'w') as fh:
            fh.write('t,bx,by,bz,rx,ry,rth\n')
            for f in frames:
                b = f.ball or (float('nan'),) * 3
                r = f.robot(team, 0) or (float('nan'),) * 3
                fh.write(f'{f.t:.4f},{b[0]:.1f},{b[1]:.1f},{b[2]:.1f},{r[0]:.1f},{r[1]:.1f},{r[2]:.4f}\n')

    def now(self):
        f = self.vision.latest()
        return f.t if f else 0.0

    def park_all(self):
        """全台を下の外周に並べて止める。試験に使う台は場面ごとに置き直す。"""
        self.cmd.zero_all(self.n)
        robots = []
        for k in range(2 * self.n):
            team = 'blue' if k < self.n else 'yellow'
            rid = k % self.n
            robots.append((team, rid, -5000.0 + 450.0 * k, -4550.0, math.pi / 2))
        self.cmd.teleport(ball=(5500.0, 4000.0), robots=robots)
        time.sleep(0.3)

    def place(self, team, rid, x, y, heading):
        self.cmd.set(team, rid)
        self.cmd.teleport(robots=[(team, rid, x, y, heading)])
        time.sleep(0.15)

    def kicks_fired(self, log_from, key):
        text = self.sim.log_text()[log_from:]
        return len(re.findall(rf'\[kick\] {key} fires', text))

    def give_ball(self, team, rid, x, y, heading=0.0):
        """台を置き、口の少し先に球を置いて、ゆっくり進んで持たせる。持てたら True。"""
        self.place(team, rid, x, y, heading)
        bx = x + (MOUTH_MM + 30.0) * math.cos(heading)
        by = y + (MOUTH_MM + 30.0) * math.sin(heading)
        self.cmd.teleport(ball=(bx, by))
        time.sleep(0.1)
        self.cmd.set(team, rid, forward=0.3, dribble=True)
        held = wait_until_held(self, team, rid, 2.0)
        self.cmd.set(team, rid, dribble=True)
        time.sleep(0.25)
        return held


def wait_until_held(ctx, team, rid, timeout, need=3):
    end = time.time() + timeout
    mark = ctx.vision.mark()
    while time.time() < end:
        frames = ctx.vision.since(mark)
        run = 0
        for f in frames:
            d = ball_mouth_distance(f, team, rid)
            run = run + 1 if d is not None and d < 2.0 else 0
            if run >= need:
                return True
        time.sleep(0.03)
    return False


def scene_accel(ctx):
    """4 m/s 前進の段差 → 加速・頭打ち・遅れ・止まるときの減速。"""
    ctx.park_all()
    ctx.place('blue', 0, -5500.0, 2000.0, 0.0)
    ctx.place('yellow', 0, -5500.0, -2000.0, 0.0)
    time.sleep(0.3)
    mark = ctx.vision.mark()
    t_cmd = ctx.now()
    ctx.cmd.set('blue', 0, forward=4.0)
    ctx.cmd.set('yellow', 0, forward=4.0)
    time.sleep(2.6)
    t_release = ctx.now()
    ctx.cmd.set('blue', 0)
    ctx.cmd.set('yellow', 0)
    time.sleep(1.2)
    frames = ctx.vision.since(mark)
    out = {}
    for team in ('blue', 'yellow'):
        series = [(t, v) for t, v, vx, vy in speed_series(track(frames, team, 0))]
        r = step_response(series, t_cmd, t_release)
        out[team] = {
            'top_speed_mm_s': r.get('top'),
            'accel_mm_s2': r.get('rise_slope'),
            'decel_mm_s2': r.get('fall_slope'),
            'delay_s': r.get('delay_s'),
            'first_motion_s': r.get('first_motion_s'),
        }
    return out


def scene_spin(ctx):
    """25 rad/s の回転の段差 → 頭打ち・角加速度・遅れ。"""
    ctx.park_all()
    ctx.place('blue', 0, 0.0, 2000.0, 0.0)
    ctx.place('yellow', 0, 0.0, -2000.0, 0.0)
    time.sleep(0.3)
    mark = ctx.vision.mark()
    t_cmd = ctx.now()
    ctx.cmd.set('blue', 0, angular=25.0)
    ctx.cmd.set('yellow', 0, angular=25.0)
    time.sleep(2.0)
    t_release = ctx.now()
    ctx.cmd.set('blue', 0)
    ctx.cmd.set('yellow', 0)
    time.sleep(1.0)
    frames = ctx.vision.since(mark)
    out = {}
    for team in ('blue', 'yellow'):
        r = step_response(omega_series(track(frames, team, 0)), t_cmd, t_release)
        out[team] = {
            'top_omega_rad_s': r.get('top'),
            'angular_accel_rad_s2': r.get('rise_slope'),
            'angular_decel_rad_s2': r.get('fall_slope'),
            'delay_s': r.get('delay_s'),
        }
    return out


def scene_carry(ctx):
    """持ったまま 3 m/s で走る・20 rad/s で回る・走りながら回る。球が口から離れたフレームを数える。"""
    ctx.park_all()
    out = {}
    for team, y in (('blue', 2000.0), ('yellow', -2000.0)):
        held = ctx.give_ball(team, 0, -5000.0, y, 0.0)
        res = {'held_at_start': held}
        phases = [
            ('run_3mps', dict(forward=3.0), 1.6),
            ('spin_20radps', dict(angular=20.0), 1.5),
            ('run_and_spin', dict(forward=3.0, angular=20.0), 1.5),
        ]
        for name, kw, secs in phases:
            mark = ctx.vision.mark()
            ctx.cmd.set(team, 0, dribble=True, **kw)
            time.sleep(secs)
            ctx.cmd.set(team, 0, dribble=True)
            time.sleep(0.5)
            frames = ctx.vision.since(mark)
            dists = [ball_mouth_distance(f, team, 0) for f in frames]
            away = sum(1 for d in dists if d is None or d > 10.0)
            sp = speed_series(track(frames, team, 0))
            om = omega_series(track(frames, team, 0))
            res[name] = {
                'frames': len(frames),
                'frames_ball_off_mouth': away,
                'max_speed_mm_s': max((v for t, v, a, b in sp), default=None),
                'max_omega_rad_s': max((abs(w) for t, w in om), default=None),
            }
        out[team] = res
    return out


def ball_model_v0(ctx, gap_mm, v_robot, v_target):
    """台が v_robot で動き続けるとき、口に着く瞬間の球の速さが v_target になる初速を二分法で求める。
    球の減速は試験する設定の [BallModel] で計算する。"""
    def arrival(v0):
        dt = 0.001
        t = 0.0
        x = 0.0
        v = v0
        while t < 3.0:
            if x - v_robot * t >= gap_mm:
                return v
            a = ctx.ball_slide if v > ctx.ball_switch * v0 else ctx.ball_roll
            v = max(0.0, v - a * dt)
            x += v * dt
            t += dt
            if v <= 0:
                return 0.0
        return 0.0
    lo, hi = v_target, v_target + 3000.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if arrival(mid) < v_target:
            lo = mid
        else:
            hi = mid
    return hi


RECEIVE_CASES = [
    # (台の x 方向の速さ [mm/s] (+ は後退), 口に着くときの球の速さの狙い [mm/s])
    (0.0, 2500.0), (0.0, 3500.0), (0.0, 4500.0),
    (1000.0, 2500.0), (1000.0, 3500.0), (1000.0, 4500.0), (1000.0, 5500.0),
    (-1000.0, 3500.0),
]


def scene_receive(ctx, team='blue'):
    """−x を向いた台が +x へ下がりながら (または止まって・向かって)、+x へ転がる球を受ける。"""
    out = []
    cases = RECEIVE_CASES
    if os.environ.get('RECEIVE_CASES'):
        cases = [tuple(float(x) for x in c.split(':')) for c in os.environ['RECEIVE_CASES'].split(',')]
    for v_robot, v_target in cases:
        ctx.park_all()
        x0 = -2500.0 if v_robot >= 0 else 1500.0
        ctx.place(team, 0, x0, 1500.0, math.pi)
        # 機体の前は −x なので、+x へ下がるのは前進の指令が負。
        ctx.cmd.set(team, 0, forward=-v_robot / 1000.0, dribble=True)
        time.sleep(0.8)
        f = ctx.vision.latest()
        rx, ry, rth = f.robot(team, 0)
        gap = 300.0
        v0 = ball_model_v0(ctx, gap + MOUTH_MM - 110.0, v_robot, v_target)
        mark = ctx.vision.mark()
        ctx.cmd.teleport(ball=(rx - MOUTH_MM - gap, ry, v0, 0.0))
        time.sleep(0.9)
        ctx.cmd.set(team, 0, dribble=True)
        time.sleep(0.2)
        frames = ctx.vision.since(mark)
        launch_x = rx - MOUTH_MM - gap
        # 置き直しが効く前のフレーム (前の場面の球の位置) を外す。着いた後の刻みで球は v0 で
        # 進んでいるので、置いた場所から前へ 2 刻みぶんまでを「着いた」とみなす。
        first = next((i for i, fr in enumerate(frames)
                      if fr.ball is not None and launch_x - 30.0 < fr.ball[0] < launch_x + v0 * 0.04
                      and abs(fr.ball[1] - ry) < 60.0), len(frames))
        frames = frames[first:]
        ctx.dump(f'receive_{int(v_robot)}_{int(v_target)}', frames, team)
        caught = False
        run = 0
        for fr in frames:
            d = ball_mouth_distance(fr, team, 0)
            run = run + 1 if d is not None and d < 2.0 else 0
            if run >= 8:
                caught = True
                break
        # 口に着く直前 (球が台の中心から 130 mm より遠い最後の 4 フレーム = 3 刻み) の速さ。
        before = []
        for fr in frames:
            r = fr.robot(team, 0)
            if fr.ball is None or r is None:
                continue
            if math.hypot(fr.ball[0] - r[0], fr.ball[1] - r[1]) < 130.0:
                break
            before.append(fr)
        tail = before[-4:]
        v_ball = v_rob = None
        if len(tail) >= 2:
            dt = tail[-1].t - tail[0].t
            v_ball = (tail[-1].ball[0] - tail[0].ball[0]) / dt
            v_rob = (tail[-1].robot(team, 0)[0] - tail[0].robot(team, 0)[0]) / dt
        out.append({
            'robot_vx_cmd_mm_s': v_robot,
            'ball_target_mm_s': v_target,
            'ball_v0_mm_s': round(v0),
            'ball_at_mouth_mm_s': v_ball,
            'robot_vx_mm_s': v_rob,
            'relative_mm_s': (v_ball - v_rob) if v_ball is not None else None,
            'caught': caught,
        })
    return out


def scene_kick_recatch(ctx):
    """持った球を 2 m/s で蹴り、ドリブラを回したまま 0.6 s 待つ。蹴った球を口に吸い戻さないか。"""
    ctx.park_all()
    out = {}
    for team, y, key in (('blue', 2500.0, 'b0'), ('yellow', -2500.0, 'y0')):
        held = ctx.give_ball(team, 0, -3000.0, y, 0.0)
        log_from = len(ctx.sim.log_text())
        mark = ctx.vision.mark()
        ctx.cmd.set(team, 0, kick=2.0, dribble=True)
        time.sleep(0.05)
        ctx.cmd.set(team, 0, dribble=True)
        time.sleep(0.6)
        frames = ctx.vision.since(mark)
        sp, pts = ball_speed_series(frames)
        end_d = ball_mouth_distance(frames[-1], team, 0) if frames else None
        out[team] = {
            'held_before_kick': held,
            'kicks_fired': ctx.kicks_fired(log_from, key),
            'ball_max_speed_mm_s': max((v for t, v, a, b in sp), default=None),
            'ball_to_mouth_after_0_6s_mm': end_d,
            'recaught': end_d is not None and end_d < 2.0,
        }
        ctx.cmd.set(team, 0)
    return out


def scene_kick_repeat(ctx):
    """蹴りの指令を出し続け、0.25 s ごとに球を口の前へ置き直す。2 s で何回蹴れるか。"""
    ctx.park_all()
    out = {}
    for team, y, key in (('blue', 1000.0, 'b0'), ('yellow', -1000.0, 'y0')):
        x = -3000.0
        ctx.place(team, 0, x, y, 0.0)
        log_from = len(ctx.sim.log_text())
        ctx.cmd.set(team, 0, kick=2.0)
        placements = 8
        for _ in range(placements):
            ctx.cmd.teleport(ball=(x + 105.0, y))
            time.sleep(0.25)
        ctx.cmd.set(team, 0)
        out[team] = {'placements': placements, 'kicks_fired': ctx.kicks_fired(log_from, key)}
    return out


def scene_kick_limit(ctx):
    """10 m/s のストレートと 8 m/s・45° のチップ。出た球の水平の速さと高さ。"""
    ctx.park_all()
    out = {}
    for team, y in (('blue', 3000.0), ('yellow', -3000.0)):
        res = {}
        for name, speed, angle in (('straight_10', 10.0, 0.0), ('chip_8_45deg', 8.0, 45.0)):
            held = ctx.give_ball(team, 0, -4500.0, y, 0.0)
            mark = ctx.vision.mark()
            ctx.cmd.set(team, 0, kick=speed, kick_angle_deg=angle, dribble=True)
            time.sleep(0.05)
            ctx.cmd.set(team, 0)
            time.sleep(0.5)
            frames = ctx.vision.since(mark)
            sp, pts = ball_speed_series(frames)
            res[name] = {
                'held_before_kick': held,
                'ball_max_horizontal_mm_s': max((v for t, v, a, b in sp), default=None),
                'ball_max_height_mm': max((p[3] for p in pts), default=None),
            }
        out[team] = res
    return out


def fit_two_phase(points):
    """速さの時系列を 2 本の直線 (滑り → 転がり) で最小二乗あてはめし、折れ点を探す。"""
    best = None
    for k in range(5, len(points) - 5):
        a = fit_line(points[:k])
        b = fit_line(points[k:])
        if not a or not b:
            continue
        err = sum((v - (a[0] * t + a[1])) ** 2 for t, v in points[:k])
        err += sum((v - (b[0] * t + b[1])) ** 2 for t, v in points[k:])
        if best is None or err < best[0]:
            best = (err, k, a, b)
    return best


def scene_ball_decel(ctx):
    """球を 4 m/s で置いて転がし、滑り・転がりの減速と切り替えの割合を測る。"""
    ctx.park_all()
    mark = ctx.vision.mark()
    v0 = 4000.0
    t0 = ctx.now()
    ctx.cmd.teleport(ball=(-5500.0, 3500.0, v0, 0.0))
    time.sleep(3.5)
    frames = ctx.vision.since(mark)
    ctx.dump('ball_decel', frames)
    sp, pts = ball_speed_series(frames)
    # 置いた直後の 3 フレームは置き直しの跳びを含むので外す。
    series = [(t, v) for t, v, a, b in sp if t > t0 + 0.06 and v > 50.0]
    best = fit_two_phase(series)
    if best is None:
        return {}
    err, k, slide, roll = best
    t_switch = (roll[1] - slide[1]) / (slide[0] - roll[0])
    v_switch = slide[0] * t_switch + slide[1]
    return {
        'launch_mm_s': v0,
        'slide_decel_mm_s2': -slide[0],
        'roll_decel_mm_s2': -roll[0],
        'switch_speed_mm_s': v_switch,
        'switch_ratio': v_switch / v0,
    }


SCENES = {
    'accel': scene_accel,
    'spin': scene_spin,
    'carry': scene_carry,
    'receive': scene_receive,
    'kick_recatch': scene_kick_recatch,
    'kick_repeat': scene_kick_repeat,
    'kick_limit': scene_kick_limit,
    'ball_decel': scene_ball_decel,
}


def rounded(obj):
    if isinstance(obj, float):
        return round(obj, 3)
    if isinstance(obj, dict):
        return {k: rounded(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [rounded(v) for v in obj]
    return obj


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--exe', required=True, help='測る m2-Sim の実行ファイル')
    ap.add_argument('--repo', default=str(HERE_REPO), help='src/ assets/ config/ proto/ のある木')
    ap.add_argument('--config', help='使う config_v2.ini (既定は --repo の config/config_v2.ini)')
    ap.add_argument('--only', help='走らせる場面をカンマ区切りで (' + ','.join(SCENES) + ')')
    ap.add_argument('--json', help='結果を JSON で書く先')
    ap.add_argument('--iface', default='127.0.0.1', help='vision のマルチキャストに参加する口の IP')
    ap.add_argument('--keep-root', action='store_true', help='一時の置き場を消さずに残す')
    ap.add_argument('--trace', help='場面の生の vision を CSV で書く先のディレクトリ')
    args = ap.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    config = pathlib.Path(args.config) if args.config else repo / 'config' / 'config_v2.ini'
    ini = read_ini(config)
    names = args.only.split(',') if args.only else list(SCENES)

    pb_dir = pathlib.Path(tempfile.mkdtemp(prefix='measure-sim-pb-'))
    compile_protos(repo / 'proto' / 'pb_src', pb_dir)

    sim = SimProcess(args.exe, repo, config)
    group = ini.get('Network', 'visionMulticastAddress', fallback='224.5.23.2')
    vision = Vision(group, TEST_PORTS['visionMulticastPort'], args.iface)
    vision.start()
    cmd = Commander(TEST_PORTS['blueTeamControlPort'], TEST_PORTS['commandListenPort'])
    results = {}
    try:
        sim.start()
        if not vision.wait_frames(30, 30.0):
            print('vision が届かない。sim の出力:\n' + sim.log_text()[-2000:], file=sys.stderr)
            return 2
        time.sleep(1.0)
        cmd.start()
        n = int(ini_float(ini, 'Robot', 'blueRobotCount', 11))
        if args.trace:
            pathlib.Path(args.trace).mkdir(parents=True, exist_ok=True)
        ctx = Ctx(vision, cmd, sim, ini, n, args.trace)
        for name in names:
            print(f'--- {name}', flush=True)
            results[name] = rounded(SCENES[name](ctx))
            print(json.dumps(results[name], ensure_ascii=False, indent=1), flush=True)
    finally:
        cmd.running = False
        vision.running = False
        sim.stop()
        if not args.keep_root:
            shutil.rmtree(sim.root, ignore_errors=True)
        shutil.rmtree(pb_dir, ignore_errors=True)
    if args.json:
        pathlib.Path(args.json).write_text(json.dumps(results, ensure_ascii=False, indent=1), encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
