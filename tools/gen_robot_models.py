#!/usr/bin/env python3
"""実機同定モデルの表から、sim と RAVEN の設定を両方生成する。

sim の `[RobotModel.<id>]` と RAVEN の `system_model_sim_ID<n>.yaml` は同じ台を指していないと
意味がない。ずれると MPC と EKF が存在しない台に向けて働く (それが 0918 の「sim で変な挙動」)。
だから両方をここ 1 箇所から出す。手で片方だけ直さないこと。

  python3 tools/gen_robot_models.py --print-ini          # sim の [RobotModel.*] を標準出力へ
  python3 tools/gen_robot_models.py --write-raven <dir>  # RAVEN の app/config へ yaml を書く
  python3 tools/gen_robot_models.py --ball <dir>         # 球のモデルを実機の測定から 3 か所へ配る
  python3 tools/gen_robot_models.py --model sumatra --write-ini   # sim の機体と球を Sumatra の前提にする

--model は台の表を選ぶ。raven-id2 (既定) は実機 ID 2 の同定値を ID ごとの節に、sumatra は
TIGERs の Sumatra が sim の機体に前提にしている値を全 ID 共通の 1 節に出す。--write-ini は
config_v2.ini の台・蹴り・捕る・球の節をその表で書き換え、ほかの行には触らない。

球のモデル (減速・跳ね返り) も 3 か所にある: RAVEN の実機ベース system_model_real.yaml・
RAVEN の sim ベース system_model_sim.yaml・sim の config_v2.ini [BallModel]。測るのは実機
(--kick-cal が BallFrictionAnalysis の結果を system_model_real.yaml に書く) の 1 回だけなので、
そこを源にして残り 2 つへ写す。sim は<b>実機の球を真似る</b>ので同じ値でよい。

値の出どころ: ssl-RAVEN app/config/system_model_real_d83add4cb8bd_ID2.yaml にある ID 2 の実機同定値。
sim では全 ID に同じ基準モデルを適用する。まず全機で ID 2 の動きを再現できる状態を基準にし、
個体差を導入する場合は別途実機計測の根拠を用意する。RAVEN の sim 用 overlay は全 ID 分生成する。
"""
import argparse
import pathlib
import re
import sys

# --- 実機 ID 2 の現在の設定値 (RAVEN 個体設定 + system_model_real.yaml の共通値) ---
BASE = {
    'ID2': dict(src='ID2 (d83add4cb8bd)',
                TauVxSec=0.04326035519210209, TauVySec=0.05136164374674227,
                TauOmegaSec=0.038527985065899314, DeadTimeSec=0.087,
                TractionAccelXMmS2=4269.0, TractionAccelYMmS2=3740.0,
                # ID2 の減速上限は未計測。RAVEN の共通既定値 6000 を暫定反映する。
                TractionDecelXMmS2=6000.0, TractionDecelYMmS2=6000.0,
                GainVx=0.88, GainVy=0.83, GainVyFromUx=-0.036, GainVxFromUy=-0.028,
                GainOmega=0.937, MaxAngularVelRadS=8.42,
                WheelRimSpeedBudgetMmS=2250.0),
}

# 回転の角加速度だけ全 ID 共通。60 Hz の vision では立ち上がりが 2〜3 フレームで終わり分解できないので、
# 保守側の計画上限を置く (0918 実機の実測は id 2 が 339・id 12 が 129 rad/s² で、どちらも 35 より速い)。
# 回転のゲインと最大角速度は ID 2 の実測値を全 ID に適用する。
COMMON = dict(MaxAngularAccelRadS2=35.0)
RAVEN_MAX_ACCEL_MM_S2 = 3500.0  # system_model_real.yaml の現在の共通設定

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


# --- Sumatra (TIGERs) が sim の機体に前提にしている値 ---
# Sumatra は sim に機体の能力を送らず、自分の表 (config/botParamsDatabase.json の "Simulation") で
# 計画する。sim の機体がそれより弱いと、Sumatra の計画が sim の都合で外れる。両チーム同じ機体にする。
# 行番号は Sumatra の Release 2025 (0bb4c653)。値は (sim の値, 注釈)。
SUMATRA_BOT = 'Sumatra config/botParamsDatabase.json'
SUMATRA_NO_LAG = f'Sumatra は指令どおりに動く台で計画する ({SUMATRA_BOT}:98-120 に遅れ・ゲインの項が無い)'
SUMATRA_MODEL = {
    'TauVxSec': (0.0, SUMATRA_NO_LAG),
    'TauVySec': (0.0, None),
    'TauOmegaSec': (0.0, None),
    'DeadTimeSec': (0.0, None),
    'GainVx': (1.0, None),
    'GainVy': (1.0, None),
    'GainVyFromUx': (0.0, None),
    'GainVxFromUy': (0.0, None),
    'GainOmega': (1.0, None),
    'TractionAccelXMmS2': (3500.0, f'accMaxFast 3.5 m/s^2 ({SUMATRA_BOT}:108)。ふだんの accMax 3.0 は AI が自分で抑える'),
    'TractionAccelYMmS2': (3500.0, None),
    'TractionDecelXMmS2': (6000.0, f'brkMax 6.0 m/s^2 ({SUMATRA_BOT}:102)'),
    'TractionDecelYMmS2': (6000.0, None),
    'WheelRimSpeedBudgetMmS': (0.0, '予算なし。Sumatra の movementLimits は並進と回転を別々に縛るだけ'),
    'MaxAngularVelRadS': (20.0, f'velMaxW 20 rad/s ({SUMATRA_BOT}:104)'),
    'MaxAngularAccelRadS2': (50.0, f'accMaxW 50 rad/s^2 ({SUMATRA_BOT}:105)'),
    'MaxLinearVelMmS': (4000.0, f'velMaxFast 4.0 m/s ({SUMATRA_BOT}:107)。ふだんの velMax 3.0 は AI が自分で抑える'),
}

# 蹴る・捕るの機体の能力 ([Physics]) と球の減速 ([BallModel])。値は (sim の値, 注釈)。
SUMATRA_PHYSICS = {
    'MaxLinearKickSpeed': (7.5, f'maxAbsoluteStraightVelocity 7.5 m/s ({SUMATRA_BOT}:118)'),
    'MaxChipKickSpeed': (5.5, f'maxAbsoluteChipVelocity 5.5 m/s、3 次元の速さ ({SUMATRA_BOT}:117)'),
    'KickerFriction': (1.0, 'Sumatra は指令した初速で球が出る前提'),
    'KickerRechargeSec': (0.0, 'Sumatra の sim の機体はいつも満充電 (SumatraSimBot.java:95 withKickerLevel(1.0))'),
    'DribblerCatchMaxSpeedMmS': (4000.0, 'Sumatra のパスは受ける側で最大 3.2 m/s (PassFactory.java:24)。'
                                         '実機も 3 m/s 程度は捕れる見立てなので余裕を持たせる'),
}
SUMATRA_BALL_SRC = 'Sumatra BallParameters.java (moduli-geometry)'
SUMATRA_BALL = {
    'AccSlideMmS2': (-3000.0, f'{SUMATRA_BALL_SRC}:26。sim の geometry は球のモデルを送らないので Sumatra はこの値で予測する'),
    'AccRollMmS2': (-260.0, f'{SUMATRA_BALL_SRC}:33'),
    'KSwitch': (0.64, f'{SUMATRA_BALL_SRC}:40'),
    # 口の板での跳ね返りと止めずに蹴る球 (GameObjects.qml directKickVelocity・botMaterial)。
    # Sumatra (simulation_match は environment SIMULATOR・simulation なし) は止めずに蹴る球を
    # ConstantLossRedirectConsultant でこの 2 つから予測する。spezis は 6 つで、SIMULATOR はその 6 つめ
    # (defValueSpezis の 6 つめ。7 つめの値は使われない)。
    'DirectKickNormalRestitution': (0.55, f'redirectRestitutionCoefficient の SIMULATOR ({SUMATRA_BALL_SRC}:59)。'
                                          '実機の板は 0.47 (RAVEN system_model_real.yaml の ball_model.direct_kick)'),
    'DirectKickTangentRetention': (0.35, f'redirectSpinFactor の SIMULATOR ({SUMATRA_BALL_SRC}:52)。'
                                         '実機の板は 1.0 (RAVEN の既定、測っていない)'),
}

MODELS = ('raven-id2', 'sumatra')


def model_for(robot_id, model='raven-id2'):
    # robot_id は出力先のセクション選択にだけ使い、物理パラメータは全 ID 共通。
    if model == 'sumatra':
        vals = {k: v for k, (v, _) in SUMATRA_MODEL.items()}
        return {'src': 'Sumatra Simulation'}, vals
    base = BASE['ID2']
    vals = {k: base[k] for k in ORDER if k in base}
    vals.update(COMMON)   # 回転角加速度の計画上限は全 ID 共通
    return base, vals


def fmt(v):
    return f"{v:.9g}"


def commented(key, value, note):
    lines = [f"; {note}"] if note else []
    return lines + [f"{key}={fmt(value)}"]


def emit_ini(model='raven-id2'):
    if model == 'sumatra':
        # 全番号・両チーム共通の 1 節。番号ごとの節は書かない (書くと番号ごとに上書きされる)。
        out = ["[RobotModel]", "; 全番号・両チーム共通。Sumatra (TIGERs) が sim の機体に前提にしている値", "Enabled=true"]
        for k, (v, note) in SUMATRA_MODEL.items():
            out += commented(k, v, note)
        return "\n".join(out) + "\n"
    out = []
    for rid in range(16):
        base, vals = model_for(rid, model)
        out.append(f"[RobotModel.{rid}]")
        out.append(f"; ID{rid}: RAVEN ID2 ({base['src']}) の同一モデル")
        for k in ORDER:
            out.append(f"{k}={fmt(vals[k])}")
        out.append("")
    return "\n".join(out)


def split_sections(lines):
    """ini の行を [(節の名前 or None, [行])] に分ける。最初の節より前の行は名前 None。"""
    out = [(None, [])]
    for line in lines:
        m = re.match(r'^\[(.+)\]\s*$', line.strip())
        if m:
            out.append((m.group(1), [line]))
        else:
            out[-1][1].append(line)
    return out


def set_keys(body, items):
    """節の行 (先頭は [名前]) の中で、items の鍵を注釈つきで置き換える。無い鍵は節の末尾に足す。
    置き換える鍵の直前にある ; の行 (前の注釈) は捨てて書き直す。"""
    keep = []
    for line in body[1:]:
        m = re.match(r'^([A-Za-z0-9_]+)\s*=', line)
        if m and m.group(1) in items:
            while keep and keep[-1].lstrip().startswith(';'):
                keep.pop()
            continue
        keep.append(line)
    while keep and not keep[-1].strip():
        keep.pop()
    for k, (v, note) in items.items():
        keep += commented(k, v, note)
    return [body[0]] + keep + [""]


def write_ini(ini_path, model):
    """config_v2.ini の台 ([RobotModel*])・蹴る捕る ([Physics] の該当の鍵)・球 ([BallModel] の減速) を
    表の値で書き換える。ほかの節と行はそのまま残す。"""
    if model != 'sumatra':
        raise SystemExit("--write-ini は今は --model sumatra だけ (raven-id2 は --print-ini の出力を貼る)")
    sections = split_sections(ini_path.read_text(encoding='utf-8', errors='replace').splitlines())
    out = []
    for name, body in sections:
        if name is not None and (name == 'RobotModel' or name.startswith('RobotModel.')):
            continue
        if name == 'Physics':
            body = set_keys(body, SUMATRA_PHYSICS)
        elif name == 'BallModel':
            body = set_keys(body, SUMATRA_BALL)
        out += body
    while out and not out[-1].strip():
        out.pop()
    out += [""] + emit_ini(model).splitlines()
    ini_path.write_text("\n".join(out) + "\n", encoding='utf-8')


def emit_raven(config_dir, model='raven-id2'):
    """sim の駆動系 overlay を全 ID 分書く。real 用の個体ファイルには触れない。

    sim 用を欠かすと RAVEN は ID の一致する real 用ファイルを使う。real と sim の値が
    別々に更新された場合でも silent fallback しないよう、実測 ID も含めて必ず sim 専用を置く。
    """
    written = []
    for rid in range(16):
        base, vals = model_for(rid, model)
        if model == 'sumatra':
            source = "Sumatra (TIGERs) が sim の機体に前提にしている値 (全 ID 共通の [RobotModel])"
            section = "[RobotModel]"
        else:
            source = f"RAVEN ID2 ({base['src']}) と同一の基準モデル"
            section = f"[RobotModel.{rid}]"
        lines = [
            f"# 自動生成: ssl-RAVEN-sim tools/gen_robot_models.py — 手で編集しない。",
            f"# sim の config_v2.ini {section} と同じ台。{source}。",
            f"# 重なるのは robot 節だけ (Config.simSystemModel)。encoder/ball_model/kicker は",
            f"# system_model_sim.yaml のまま。",
            "robot:",
        ]
        for k in ORDER:
            lines.append(f"  {RAVEN_KEY[k]}: {fmt(vals[k])}")
        # RAVEN は max_* も別経路で読む。max_accel は共通設定を保ち、減速は
        # 現在の共通既定値を使う (ID2 の減速限界は未計測)。
        max_accel = vals['TractionAccelXMmS2'] if model == 'sumatra' else RAVEN_MAX_ACCEL_MM_S2
        lines.append(f"  max_accel_mm_s2: {fmt(max_accel)}")
        lines.append(f"  max_decel_mm_s2: {fmt(vals['TractionDecelYMmS2'])}")
        path = config_dir / f"system_model_sim_ID{rid}.yaml"
        path.write_text("\n".join(lines) + "\n")
        written.append(path.name)
    return written


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
    ap.add_argument("--model", choices=MODELS, default='raven-id2', help="台の表 (既定 raven-id2)")
    ap.add_argument("--write-ini", action="store_true",
                    help="config_v2.ini の台・蹴る捕る・球の節を --model の表で書き換える (今は sumatra だけ)")
    args = ap.parse_args()
    if not args.print_ini and not args.write_raven and not args.ball and not args.write_ini:
        ap.print_help()
        return 1
    if args.print_ini:
        print(emit_ini(args.model))
    if args.write_ini:
        ini = pathlib.Path(__file__).resolve().parent.parent / "config" / "config_v2.ini"
        write_ini(ini, args.model)
        print(f"wrote {ini}")
    if args.write_raven:
        d = pathlib.Path(args.write_raven)
        if not d.is_dir():
            print(f"config ディレクトリが無い: {d}", file=sys.stderr)
            return 2
        written = emit_raven(d, args.model)
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
