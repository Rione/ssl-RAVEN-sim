#!/usr/bin/env python3
"""実機同定モデルの表から、sim と RAVEN の設定を両方生成する。

sim の `[RobotModel.<id>]` と RAVEN の `system_model_sim_ID<n>.yaml` は同じ台を指していないと
意味がない。ずれると MPC と EKF が存在しない台に向けて働く (それが 0918 の「sim で変な挙動」)。
だから両方をここ 1 箇所から出す。手で片方だけ直さないこと。

  python3 tools/gen_robot_models.py --print-ini          # sim の [RobotModel.*] を標準出力へ
  python3 tools/gen_robot_models.py --write-raven <dir>  # RAVEN の app/config へ yaml を書く
  python3 tools/gen_robot_models.py --ball <dir>         # 球のモデルを実機の測定から 3 か所へ配る

球のモデル (減速・跳ね返り) も 3 か所にある: RAVEN の実機ベース system_model_real.yaml・
RAVEN の sim ベース system_model_sim.yaml・sim の config_v2.ini [BallModel]。測るのは実機
(--kick-cal が BallFrictionAnalysis の結果を system_model_real.yaml に書く) の 1 回だけなので、
そこを源にして残り 2 つへ写す。sim は<b>実機の球を真似る</b>ので同じ値でよい。

値の出どころ: ssl-RAVEN app/config/system_model_real_<mac>_ID<n>.yaml (実機 3 台の同定値)。
ID 2/4/11 は同定値そのままなので sim の ini にだけ出し、RAVEN 側は実機ファイルを直接使う
(Config.findSimOverlayFileName の 2 段目)。それ以外の ID は 3 世代のどれかを母体にした
クローンで、id とキー名から決まる固定のばらつきを掛ける (実行のたびに変わらない)。
"""
import argparse
import hashlib
import pathlib
import re
import sys

# --- 実機 3 台の同定値 (RAVEN app/config/system_model_real_*_ID*.yaml の robot 節) ---
BASE = {
    'A': dict(src='ID2 (d83add4cb8bd) 旧基板 Pi4',
              TauVxSec=0.04326035519210209, TauVySec=0.05136164374674227, TauOmegaSec=0.038527985065899314,
              DeadTimeSec=0.087,
              TractionAccelXMmS2=5549.0, TractionAccelYMmS2=3740.0,
              TractionDecelXMmS2=4192.0, TractionDecelYMmS2=4736.0,
              GainVx=0.756, GainVy=0.638, GainVyFromUx=-0.057, GainVxFromUy=-0.094,
              GainOmega=0.937, MaxAngularVelRadS=8.42,
              WheelRimSpeedBudgetMmS=2250.0),
    'B': dict(src='ID4 (d83add1a09be)',
              TauVxSec=0.0366767807706137, TauVySec=0.03829272063369836, TauOmegaSec=0.0300531941413449,
              DeadTimeSec=0.11707112787164525,
              TractionAccelXMmS2=3971.0, TractionAccelYMmS2=1715.0,
              TractionDecelXMmS2=4735.0, TractionDecelYMmS2=3142.0,
              GainVx=0.942, GainVy=0.569, GainVyFromUx=0.0, GainVxFromUy=0.0,
              GainOmega=1.0, MaxAngularVelRadS=10.0,
              WheelRimSpeedBudgetMmS=2250.0),
    'C': dict(src='ID11 (e0d55de88825)',
              TauVxSec=0.030093206349125306, TauVySec=0.025223797520654442, TauOmegaSec=0.02157840321679048,
              DeadTimeSec=0.11,
              TractionAccelXMmS2=3434.0, TractionAccelYMmS2=1215.0,
              TractionDecelXMmS2=3850.0, TractionDecelYMmS2=1735.0,
              GainVx=0.928, GainVy=1.0, GainVyFromUx=-0.031, GainVxFromUy=0.0,
              GainOmega=1.0, MaxAngularVelRadS=10.0,
              WheelRimSpeedBudgetMmS=2000.0),
    'D': dict(src='ID12 (20bd1dd3e050) 新基板',
              TauVxSec=0.0366767807706137, TauVySec=0.03829272063369836, TauOmegaSec=0.0300531941413449,
              DeadTimeSec=0.063,
              TractionAccelXMmS2=3778.0, TractionAccelYMmS2=2060.0,
              TractionDecelXMmS2=2053.0, TractionDecelYMmS2=924.0,
              GainVx=0.983, GainVy=0.776, GainVyFromUx=-0.02, GainVxFromUy=0.002,
              GainOmega=0.956, MaxAngularVelRadS=9.72,
              WheelRimSpeedBudgetMmS=2000.0),
}

# 回転の角加速度だけ全機共通。60 Hz の vision では立ち上がりが 2〜3 フレームで終わり分解できないので、
# 保守側の計画上限を置く (0918 実機の実測は id 2 が 339・id 12 が 129 rad/s² で、どちらも 35 より速い)。
# 回転のゲインと最大角速度は BASE で世代ごとに持つ (0918 に --traction で測った)。
COMMON = dict(MaxAngularAccelRadS2=35.0)

# ばらつきの幅 (±)。実機 3 台の世代差より十分小さく、個体差として妥当な範囲に収める。
SPREAD = {
    'TauVxSec': 0.10, 'TauVySec': 0.10, 'TauOmegaSec': 0.10,
    'DeadTimeSec': 0.08,
    'TractionAccelXMmS2': 0.12, 'TractionAccelYMmS2': 0.12,
    'TractionDecelXMmS2': 0.12, 'TractionDecelYMmS2': 0.12,
    'GainVx': 0.05, 'GainVy': 0.05, 'GainVyFromUx': 0.15, 'GainVxFromUy': 0.15,
    'GainOmega': 0.03, 'MaxAngularVelRadS': 0.08,
    'WheelRimSpeedBudgetMmS': 0.04,
}

# id -> 母体の世代。2/4/11 はその実機そのもの。
GEN = ['A', 'B', 'A', 'B', 'B', 'C', 'D', 'C', 'B', 'D', 'C', 'C', 'D', 'B', 'D', 'A']
EXACT = {2: 'A', 4: 'B', 11: 'C', 12: 'D'}

ORDER = ['TauVxSec', 'TauVySec', 'TauOmegaSec', 'DeadTimeSec',
         'TractionAccelXMmS2', 'TractionAccelYMmS2', 'TractionDecelXMmS2', 'TractionDecelYMmS2',
         'GainVx', 'GainVy', 'GainVyFromUx', 'GainVxFromUy', 'GainOmega', 'WheelRimSpeedBudgetMmS',
         'MaxAngularVelRadS', 'MaxAngularAccelRadS2']

# sim の ini のキー -> RAVEN の system_model robot 節のキー。
RAVEN_KEY = {
    'TauVxSec': 'tau_vx', 'TauVySec': 'tau_vy', 'TauOmegaSec': 'tau_omega',
    'DeadTimeSec': 'input_dead_time_sec',
    'TractionAccelXMmS2': 'traction_accel_x_mm_s2', 'TractionAccelYMmS2': 'traction_accel_y_mm_s2',
    'TractionDecelXMmS2': 'traction_decel_x_mm_s2', 'TractionDecelYMmS2': 'traction_decel_y_mm_s2',
    'GainVx': 'gain_vx', 'GainVy': 'gain_vy',
    'GainVyFromUx': 'gain_vy_from_ux', 'GainVxFromUy': 'gain_vx_from_uy',
    'GainOmega': 'gain_omega',
    'WheelRimSpeedBudgetMmS': 'wheel_rim_speed_budget_mm_s',
    'MaxAngularVelRadS': 'max_angular_velocity',
    'MaxAngularAccelRadS2': 'max_angular_acceleration',
}


def jitter(robot_id, key):
    """id と key から決まる [-1, 1] の値。実行のたびに変わらない。"""
    h = hashlib.sha256(f"{robot_id}:{key}".encode()).digest()
    return (int.from_bytes(h[:4], 'big') / 0xFFFFFFFF) * 2.0 - 1.0


def model_for(robot_id):
    base = BASE[GEN[robot_id]]
    if robot_id in EXACT:
        vals = {k: base[k] for k in ORDER if k in base}
    else:
        vals = {k: base[k] * (1.0 + SPREAD[k] * jitter(robot_id, k)) for k in ORDER if k in base}
    vals.update(COMMON)   # 回転の上限は未計測なので世代もばらつきも付けない
    return base, vals


def fmt(v):
    return f"{v:.6g}"


def emit_ini():
    out = []
    for rid in range(16):
        base, vals = model_for(rid)
        out.append(f"[RobotModel.{rid}]")
        if rid in EXACT:
            out.append(f"; ID{rid}: RAVEN {base['src']} の同定値そのまま")
        else:
            out.append(f"; ID{rid}: {base['src']} 世代のクローン (id で決まる固定のばらつき)")
        for k in ORDER:
            out.append(f"{k}={fmt(vals[k])}")
        out.append("")
    return "\n".join(out)


def emit_raven(config_dir):
    """RAVEN の per-robot オーバーレイを書く。

    EXACT の ID は書かない — RAVEN は ID が一致する実機ファイルを直接読むので、
    同じ値を 2 箇所に置くと片方だけ直されてずれる。書かないだけでなく<b>消す</b>:
    RAVEN は system_model_sim_ID<n>.yaml を実機ファイルより先に見るので、以前の生成が
    残っていると実測値が影に入る (0918: ID12 が世代 D になった後も 10:42 のクローンが
    残っていて、sim は実測 0.983 で動くのに RAVEN は 0.728 を仮定していた)。
    """
    written = []
    removed = []
    for rid in EXACT:
        stale = config_dir / f"system_model_sim_ID{rid}.yaml"
        if stale.exists():
            stale.unlink()
            removed.append(stale.name)
    for rid in range(16):
        if rid in EXACT:
            continue
        base, vals = model_for(rid)
        lines = [
            f"# 自動生成: ssl-RAVEN-sim tools/gen_robot_models.py — 手で編集しない。",
            f"# sim の config_v2.ini [RobotModel.{rid}] と同じ台。{base['src']} 世代のクローン。",
            f"# 重なるのは robot 節だけ (Config.simSystemModel)。encoder/ball_model/kicker は",
            f"# system_model_sim.yaml のまま。",
            "robot:",
        ]
        for k in ORDER:
            lines.append(f"  {RAVEN_KEY[k]}: {fmt(vals[k])}")
        # RAVEN は牽引の 4 値が揃っていればそちらを使うが、max_* も別経路で読まれる。
        # 実機ファイルと同じく「弱いほうの軸」を入れて辻褄を合わせる。
        lines.append(f"  max_accel_mm_s2: {fmt(vals['TractionAccelYMmS2'])}")
        lines.append(f"  max_decel_mm_s2: {fmt(vals['TractionDecelYMmS2'])}")
        path = config_dir / f"system_model_sim_ID{rid}.yaml"
        path.write_text("\n".join(lines) + "\n")
        written.append(path.name)
    return written, removed


# --- 球のモデル: 実機の測定 (RAVEN の system_model_real.yaml) を源に 3 か所へ配る ---

# RAVEN の yaml のキー -> sim の ini のキー。ここに無い欄は写さない。
BALL_KEY = {
    'acc_slide_mm_s2': 'AccSlideMmS2',
    'acc_roll_mm_s2': 'AccRollMmS2',
    'k_switch': 'KSwitch',
    'direct_kick.normal_restitution': 'DirectKickNormalRestitution',
    'direct_kick.tangent_retention': 'DirectKickTangentRetention',
}


def read_ball_model(yaml_path):
    """yaml の ball_model 節を {キー: 値の文字列} で返す。入れ子は '親.子' で平らにする。

    yaml の丸ごとの読み書きはしない (PyYAML を要らなくする・他の節に触らないため)。
    機械が書いたファイルなので、2 空白の字下げだけを見れば足りる。
    """
    out = {}
    depth = None
    parent = None
    for line in yaml_path.read_text(encoding='utf-8').splitlines():
        if line.strip().startswith('#') or not line.strip():
            continue
        if not line.startswith(' '):
            depth = 0 if line.startswith('ball_model:') else None
            parent = None
            continue
        if depth is None:
            continue
        m = re.match(r'^( +)([A-Za-z0-9_]+):\s*(.*)$', line)
        if not m:
            continue
        indent, key, value = len(m.group(1)), m.group(2), m.group(3).strip()
        if indent == 2:
            parent = None
            if value:
                out[key] = value
            else:
                parent = key
        elif indent == 4 and parent and value:
            out[f'{parent}.{key}'] = value
    return out


def write_ball_model(yaml_path, ball):
    """yaml の ball_model 節を置き換える。他の節には触らない。"""
    lines = yaml_path.read_text(encoding='utf-8').splitlines()
    out, i, replaced = [], 0, False
    while i < len(lines):
        if lines[i].startswith('ball_model:'):
            out.extend(ball_yaml_block(ball))
            i += 1
            while i < len(lines) and (lines[i].startswith(' ') or not lines[i].strip()):
                i += 1
            replaced = True
            continue
        out.append(lines[i])
        i += 1
    if not replaced:
        out.extend(ball_yaml_block(ball))
    yaml_path.write_text('\n'.join(out) + '\n', encoding='utf-8')


def ball_yaml_block(ball):
    lines = ['ball_model:']
    for key in ('acc_slide_mm_s2', 'acc_roll_mm_s2', 'k_switch'):
        if key in ball:
            lines.append(f'  {key}: {ball[key]}')
    nested = {k: v for k, v in ball.items() if '.' in k}
    for parent in sorted({k.split('.')[0] for k in nested}):
        lines.append(f'  {parent}:')
        for k, v in sorted(nested.items()):
            if k.startswith(parent + '.'):
                lines.append(f'    {k.split(".")[1]}: {v}')
    return lines


def write_ball_ini(ini_path, ball):
    """sim の ini の [BallModel] 節を置き換える。他の節には触らない。"""
    lines = ini_path.read_text(encoding='utf-8', errors='replace').splitlines()
    block = ['[BallModel]'] + [f'{BALL_KEY[k]}={ball[k]}' for k in BALL_KEY if k in ball] + ['']
    out, i, replaced = [], 0, False
    while i < len(lines):
        if lines[i].strip() == '[BallModel]':
            out.extend(block)
            i += 1
            while i < len(lines) and not lines[i].strip().startswith('['):
                i += 1
            replaced = True
            continue
        out.append(lines[i])
        i += 1
    if not replaced:
        out.extend([''] + block)
    ini_path.write_text('\n'.join(out) + '\n', encoding='utf-8')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--print-ini", action="store_true", help="sim の [RobotModel.*] を標準出力へ")
    ap.add_argument("--write-raven", metavar="CONFIG_DIR", help="RAVEN の app/config へ yaml を書く")
    ap.add_argument("--ball", metavar="CONFIG_DIR",
                    help="球のモデルを RAVEN の system_model_real.yaml から sim ベースと config_v2.ini へ配る")
    args = ap.parse_args()
    if not args.print_ini and not args.write_raven and not args.ball:
        ap.print_help()
        return 1
    if args.print_ini:
        print(emit_ini())
    if args.write_raven:
        d = pathlib.Path(args.write_raven)
        if not d.is_dir():
            print(f"config ディレクトリが無い: {d}", file=sys.stderr)
            return 2
        written, removed = emit_raven(d)
        for name in removed:
            print(f"removed {d / name} (実機ファイルを影に入れていた)")
        for name in written:
            print(f"wrote {d / name}")
    if args.ball:
        d = pathlib.Path(args.ball)
        source = d / "system_model_real.yaml"
        if not source.is_file():
            print(f"球の測定が無い: {source}", file=sys.stderr)
            return 2
        ball = read_ball_model(source)
        if not ball:
            print(f"{source} に ball_model 節が無い", file=sys.stderr)
            return 2
        print("球のモデル (源: " + str(source) + ")")
        for k, v in ball.items():
            print(f"  {k}: {v}")
        sim_base = d / "system_model_sim.yaml"
        if sim_base.is_file():
            write_ball_model(sim_base, ball)
            print(f"wrote {sim_base}")
        ini = pathlib.Path(__file__).resolve().parent.parent / "config" / "config_v2.ini"
        if ini.is_file():
            write_ball_ini(ini, ball)
            print(f"wrote {ini}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
