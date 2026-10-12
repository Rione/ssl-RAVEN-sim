#!/usr/bin/env python3
"""sim の SimulatorControl の口 (ssl-simulation-protocol の SimulatorCommand) が、台と球を決まりどおりに動かすかを確かめる。

sim を 1 つ試験用の口で起動し (measure_sim_spec.py と同じ置き場の作り方)、SimulatorCommand を送って、返事と vision を見る。
RAVEN も Sumatra も起動しない。全部の台には 100 Hz で速度 0 の指令を送り続ける (試合と同じく、場に居ない台にも指令が来る)。

  python3 tools/check_simulation_control.py --exe build/bin/m2-Sim --config config/config_sumatra.ini

確かめること (どれか 1 つでも外れたら 1 で終わる):
  - 空の SimulatorCommand に空の返事が返る (ハーネスが「この sim は口を持つ」と見分ける手)
  - present = false で台が vision から消え、速度の指令が来ても戻らない。もう一度送っても何もしない
  - 場に居ない台への present 無しの移動は捨てる。present = true で位置が無ければ CREATE_NOPOS_ROBOT
  - present = true と位置・向きで、台がその所・向きで vision に戻る
  - 場に居る台の位置・向きの置き直しと、速度の置き直し (与えた向きへ動き出し、止まっていく)
  - by_force: 置き直さずに目標へ寄っていき、by_force = false で何も書かない命令は止めるだけ (置き直さない)
  - 誤り: x だけ (PARTIAL_COORD)、by_force と速度 (VELOCITY_FORCE)、台数より大きい番号を足す (CREATE_UNSPEC_ROBOT)
  - 球を持った台を出すと、球は口の前に残る
  - mocSim の置き換えの turnon = false でも台が消え、置き直すと戻る
  - 球の置き直し (TeleportBall の x・y)
"""
import argparse
import math
import pathlib
import shutil
import socket
import subprocess
import sys
import tempfile
import time

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.dont_write_bytecode = True  # tools/ に __pycache__ を残さない
import measure_sim_spec as mss  # noqa: E402

CONTROL_PORT = 10400  # 試験用。ふだんの sim の口 (10300) と重ならない
mss.TEST_PORTS['simulationControlPort'] = CONTROL_PORT

PROTOS = mss.PROTOS + ['ssl_gc_common.proto', 'ssl_simulation_error.proto', 'ssl_simulation_config.proto',
                       'ssl_simulation_control.proto']
POS_TOL_MM = 30.0
HEADING_TOL_RAD = 0.05


class Control:
    def __init__(self, port):
        import ssl_simulation_control_pb2
        import ssl_gc_common_pb2
        self.pb = ssl_simulation_control_pb2
        self.common = ssl_gc_common_pb2
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(1.0)
        self.addr = ('127.0.0.1', port)

    def send(self, command):
        """SimulatorCommand を送り、返事 (SimulatorResponse) を返す。返事が来なければ None。"""
        self.sock.sendto(command.SerializeToString(), self.addr)
        try:
            data = self.sock.recv(65535)
        except socket.timeout:
            return None
        response = self.pb.SimulatorResponse()
        response.ParseFromString(data)
        return response

    def robot(self, team, rid, **fields):
        command = self.pb.SimulatorCommand()
        t = command.control.teleport_robot.add()
        t.id.id = rid
        t.id.team = self.common.YELLOW if team == 'yellow' else self.common.BLUE
        for key, value in fields.items():
            setattr(t, key, value)
        return self.send(command)

    def ball(self, **fields):
        command = self.pb.SimulatorCommand()
        for key, value in fields.items():
            setattr(command.control.teleport_ball, key, value)
        return self.send(command)


class Checks:
    def __init__(self):
        self.results = []

    def check(self, name, ok, detail=''):
        self.results.append((name, bool(ok), detail))
        print(f"{'OK ' if ok else 'NG '} {name}" + (f"  ({detail})" if detail else ''), flush=True)

    def failed(self):
        return [r for r in self.results if not r[1]]


def codes(response):
    return [e.code for e in response.errors] if response is not None else None


def wait_robot(ctx, team, rid, present, seconds=1.5, need=5):
    """台が need フレーム続けて vision に居る (present) / 居ない まで待つ。待てたら最後のフレーム、待てなければ None。"""
    end = time.time() + seconds
    mark = ctx.vision.mark()
    while time.time() < end:
        frames = ctx.vision.since(mark)
        run = 0
        for f in frames:
            seen = f.robot(team, rid) is not None
            run = run + 1 if seen == present else 0
            if run >= need:
                return f
        time.sleep(0.02)
    return None


def pose_error(robot, x, y, heading):
    d = math.hypot(robot[0] - x, robot[1] - y)
    a = abs(math.atan2(math.sin(robot[2] - heading), math.cos(robot[2] - heading)))
    return d, a


def run_checks(ctx, ctl, n):
    c = Checks()

    # 口があるか
    probe = ctl.send(ctl.pb.SimulatorCommand())
    c.check('空の命令に空の返事', probe is not None and len(probe.errors) == 0, f'errors={codes(probe)}')

    ctx.park_all()
    ctx.place('yellow', 3, -1000.0, 1000.0, 0.0)
    ctx.place('blue', 2, 1000.0, 1000.0, math.pi)
    time.sleep(0.3)

    # 出す
    r = ctl.robot('yellow', 3, present=False)
    gone = wait_robot(ctx, 'yellow', 3, False)
    others = len(gone.yellow) if gone else None
    c.check('present=false で黄 3 が vision から消える', r is not None and not r.errors and gone is not None,
            f'errors={codes(r)} 残りの黄={others}')
    c.check('他の黄は残る', others == n - 1, f'黄の台数={others}')
    ctx.cmd.set('yellow', 3, forward=1.0)
    ctx.wait(1.0)
    f = ctx.vision.latest()
    c.check('場に居ない台は指令が来ても映らない', f.robot('yellow', 3) is None)
    ctx.cmd.set('yellow', 3)
    r = ctl.robot('yellow', 3, present=False)
    c.check('もう一度 present=false は何もしない', r is not None and not r.errors and wait_robot(ctx, 'yellow', 3, False),
            f'errors={codes(r)}')
    r = ctl.robot('yellow', 3, x=0.0, y=0.0, orientation=0.0)
    ctx.wait(0.3)
    c.check('場に居ない台の present 無しの移動は捨てる', r is not None and not r.errors and ctx.vision.latest().robot('yellow', 3) is None,
            f'errors={codes(r)}')
    r = ctl.robot('yellow', 3, present=True)
    ctx.wait(0.3)
    c.check('位置の無い present=true は CREATE_NOPOS_ROBOT', codes(r) == ['CREATE_NOPOS_ROBOT']
            and ctx.vision.latest().robot('yellow', 3) is None, f'errors={codes(r)}')

    # 戻す (ssl-simulation-controller が足すときと同じ形: タッチラインの上・向き 0)
    r = ctl.robot('yellow', 3, present=True, x=0.1, y=4.5, orientation=0.0)
    back = wait_robot(ctx, 'yellow', 3, True)
    if back is not None:
        ctx.wait(0.2)
        d, a = pose_error(ctx.vision.latest().robot('yellow', 3), 100.0, 4500.0, 0.0)
    else:
        d = a = float('nan')
    c.check('present=true で黄 3 が (0.1, 4.5) 向き 0 に戻る', r is not None and not r.errors and back is not None
            and d < POS_TOL_MM and a < HEADING_TOL_RAD, f'errors={codes(r)} ずれ={d:.1f} mm {a:.3f} rad')
    c.check('黄が全部そろう', back is not None and len(ctx.vision.latest().yellow) == n,
            f'黄の台数={len(ctx.vision.latest().yellow)}')

    # 場に居る台の置き直し
    r = ctl.robot('yellow', 3, x=-1.0, y=2.0, orientation=math.pi / 2)
    ctx.wait(0.4)
    d, a = pose_error(ctx.vision.latest().robot('yellow', 3), -1000.0, 2000.0, math.pi / 2)
    c.check('位置と向きの置き直し', r is not None and not r.errors and d < POS_TOL_MM and a < HEADING_TOL_RAD,
            f'ずれ={d:.1f} mm {a:.3f} rad')
    r = ctl.robot('yellow', 3, orientation=-math.pi / 2)
    ctx.wait(0.4)
    d, a = pose_error(ctx.vision.latest().robot('yellow', 3), -1000.0, 2000.0, -math.pi / 2)
    c.check('向きだけの置き直しは位置を保つ', r is not None and not r.errors and d < POS_TOL_MM and a < HEADING_TOL_RAD,
            f'ずれ={d:.1f} mm {a:.3f} rad')

    # 速度の置き直し: +x へ 1 m/s を与えると、その向きへ動き出して止まる
    r = ctl.robot('yellow', 3, x=-2.0, y=0.0, orientation=math.pi / 2, v_x=1.0, v_y=0.0)
    mark = ctx.vision.mark()
    ctx.wait(1.5)
    tr = mss.track(ctx.vision.since(mark), 'yellow', 3)
    moved = tr[-1][1] - (-2000.0) if tr else float('nan')
    side = abs(tr[-1][2] - 0.0) if tr else float('nan')
    still = abs(tr[-1][1] - tr[-10][1]) if len(tr) > 10 else float('nan')
    c.check('速度を与えると +x へ動いて止まる', r is not None and not r.errors and moved > 30.0 and side < 30.0 and still < 2.0,
            f'進んだ={moved:.0f} mm 横={side:.0f} mm 最後の 10 フレームの動き={still:.1f} mm')

    # by_force: 置き直さずに寄る
    start = ctx.vision.latest().robot('blue', 2)
    r = ctl.robot('blue', 2, x=-0.5, y=-1.0, orientation=0.0, by_force=True)
    mark = ctx.vision.mark()
    ctx.wait(3.0)
    tr = mss.track(ctx.vision.since(mark), 'blue', 2)
    first = tr[1] if len(tr) > 1 else None
    jump = math.hypot(first[1] - start[0], first[2] - start[1]) if first else float('nan')
    d, a = pose_error(ctx.vision.latest().robot('blue', 2), -500.0, -1000.0, 0.0)
    c.check('by_force は飛ばずに寄っていく', r is not None and not r.errors and jump < 200.0, f'最初の 2 フレームで {jump:.0f} mm')
    c.check('by_force で目標に着く', d < POS_TOL_MM and a < HEADING_TOL_RAD, f'ずれ={d:.1f} mm {a:.3f} rad')
    ctx.cmd.set('blue', 2, forward=0.5)
    ctx.wait(0.5)
    d, a = pose_error(ctx.vision.latest().robot('blue', 2), -500.0, -1000.0, 0.0)
    c.check('by_force の間は台の指令で動かない', d < POS_TOL_MM, f'ずれ={d:.1f} mm')
    ctx.cmd.set('blue', 2)
    r = ctl.robot('blue', 2, by_force=False)
    ctx.wait(0.4)
    d, a = pose_error(ctx.vision.latest().robot('blue', 2), -500.0, -1000.0, 0.0)
    c.check('by_force=false で何も書かなければ置き直さずに止める', r is not None and not r.errors and d < POS_TOL_MM,
            f'ずれ={d:.1f} mm')
    ctx.cmd.set('blue', 2, forward=0.5)
    mark = ctx.vision.mark()
    ctx.wait(0.6)
    tr = mss.track(ctx.vision.since(mark), 'blue', 2)
    driven = tr[-1][1] - tr[0][1] if len(tr) > 1 else float('nan')
    c.check('by_force を止めると台の指令で走る', driven > 100.0, f'進んだ={driven:.0f} mm')
    ctx.cmd.set('blue', 2)

    # 誤り
    r = ctl.robot('blue', 2, x=1.0)
    c.check('x だけは PARTIAL_COORD', codes(r) == ['PARTIAL_COORD'], f'errors={codes(r)}')
    r = ctl.robot('blue', 2, x=1.0, y=1.0, v_x=1.0, by_force=True)
    c.check('by_force と速度は VELOCITY_FORCE', codes(r) == ['VELOCITY_FORCE'], f'errors={codes(r)}')
    r = ctl.robot('yellow', n + 1, present=True, x=0.0, y=0.0, orientation=0.0)
    c.check('台数より大きい番号は CREATE_UNSPEC_ROBOT', codes(r) == ['CREATE_UNSPEC_ROBOT'], f'errors={codes(r)}')

    # 球を持った台を出す: 球は口の前に残る
    held = ctx.give_ball('yellow', 5, 2000.0, -2000.0, 0.0)
    holder = ctx.vision.latest().robot('yellow', 5)
    mouth = (holder[0] + mss.MOUTH_MM * math.cos(holder[2]), holder[1] + mss.MOUTH_MM * math.sin(holder[2]))
    r = ctl.robot('yellow', 5, present=False)
    gone = wait_robot(ctx, 'yellow', 5, False)
    ctx.wait(0.5)
    ball = ctx.vision.latest().ball
    off = math.hypot(ball[0] - mouth[0], ball[1] - mouth[1]) if ball else float('nan')
    c.check('球を持った台を出すと球は口の前に残る', held and gone is not None and off < POS_TOL_MM,
            f'持てた={held} 球と口のずれ={off:.0f} mm')
    ctl.robot('yellow', 5, present=True, x=2.0, y=-3.0, orientation=0.0)
    c.check('出した台を戻せる', wait_robot(ctx, 'yellow', 5, True) is not None)

    # mocSim の置き換え: turnon=false で出し、置き直すと戻る
    pkt = ctx.cmd.moc.mocSim_Packet()
    rr = pkt.replacement.robots.add()
    rr.id, rr.x, rr.y, rr.dir, rr.yellowteam, rr.turnon = 4, 0.0, 0.0, 0.0, False, False
    ctx.cmd.sock.sendto(pkt.SerializeToString(), ctx.cmd.command_addr)
    c.check('置き換えの turnon=false で青 4 が消える', wait_robot(ctx, 'blue', 4, False) is not None)
    ctx.cmd.teleport(robots=[('blue', 4, 3000.0, 3000.0, 0.0)])
    back = wait_robot(ctx, 'blue', 4, True)
    if back is not None:
        ctx.wait(0.2)
        d, a = pose_error(ctx.vision.latest().robot('blue', 4), 3000.0, 3000.0, 0.0)
    else:
        d = float('nan')
    c.check('置き換えで青 4 が戻る', back is not None and d < POS_TOL_MM, f'ずれ={d:.1f} mm')
    r = ctl.robot('blue', 4, present=False)
    c.check('置き換えで戻した台を present=false で出せる', r is not None and not r.errors
            and wait_robot(ctx, 'blue', 4, False) is not None)
    ctl.robot('blue', 4, present=True, x=3.0, y=3.0, orientation=0.0)

    # 球の置き直し
    r = ctl.ball(x=0.5, y=0.5)
    ctx.wait(0.4)
    ball = ctx.vision.latest().ball
    off = math.hypot(ball[0] - 500.0, ball[1] - 500.0) if ball else float('nan')
    c.check('球の置き直し', r is not None and not r.errors and off < POS_TOL_MM, f'ずれ={off:.1f} mm')
    return c


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--exe', required=True, help='確かめる m2-Sim の実行ファイル')
    ap.add_argument('--repo', default=str(mss.HERE_REPO), help='src/ assets/ config/ proto/ のある木')
    ap.add_argument('--config', default=str(mss.HERE_REPO / 'config' / 'config_sumatra.ini'), help='使う設定ファイル')
    ap.add_argument('--iface', default='127.0.0.1', help='vision のマルチキャストに参加する口の IP')
    args = ap.parse_args()

    repo = pathlib.Path(args.repo).resolve()
    config = pathlib.Path(args.config)
    ini = mss.read_ini(config)
    n = int(mss.ini_float(ini, 'Robot', 'yellowRobotCount', 11))
    pb_dir = pathlib.Path(tempfile.mkdtemp(prefix='check-simctl-pb-'))
    subprocess.run(['protoc', f'-I{repo / "proto" / "pb_src"}', f'--python_out={pb_dir}', *PROTOS], check=True)
    sys.path.insert(0, str(pb_dir))

    sim = mss.SimProcess(args.exe, repo, config, pass_config=True)
    group = ini.get('Network', 'visionMulticastAddress', fallback='224.5.23.2')
    vision = mss.Vision(group, mss.TEST_PORTS['visionMulticastPort'], args.iface)
    vision.start()
    cmd = mss.Commander(mss.TEST_PORTS['blueTeamControlPort'], mss.TEST_PORTS['commandListenPort'])
    try:
        sim.start(headless=True)
        if not vision.wait_frames(30, 30.0):
            print('vision が届かない。sim の出力:\n' + sim.log_text()[-2000:], file=sys.stderr)
            return 2
        time.sleep(1.0)
        cmd.start()
        cmd.zero_all(n)
        ctx = mss.Ctx(vision, cmd, sim, ini, n)
        checks = run_checks(ctx, Control(CONTROL_PORT), n)
    finally:
        cmd.running = False
        vision.running = False
        sim.stop()
        log = sim.log_text()
        shutil.rmtree(sim.root, ignore_errors=True)
        shutil.rmtree(pb_dir, ignore_errors=True)
    errors = [line for line in log.splitlines() if 'qml' in line.lower() and ('error' in line.lower() or 'TypeError' in line)]
    if errors:
        print('sim の QML の誤り:\n  ' + '\n  '.join(errors[:20]))
    failed = checks.failed()
    print(f'\n{len(checks.results) - len(failed)}/{len(checks.results)} 通った')
    return 1 if failed or errors else 0


if __name__ == '__main__':
    sys.exit(main())
