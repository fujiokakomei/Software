# ct_reader.py -- Cognitive Target の値（5リングの合計）を読むモジュール
#
# main.py での使い方:
#     import ct_reader
#     ...
#     uart.write(str(ct_reader.decide(sensor.snapshot)) + "\n")   # 0 / 1 / 2 / -1
#
# 戻り値は「ロボットが投下するレスキューキット数」と同じ:
#     合計 0 (Unharmed) -> 0,  合計 1 (Stable) -> 1,  合計 2 (Harmed) -> 2
#     合計が 0/1/2 以外 / 読み取り失敗 -> -1   (victimではない: 識別してはいけない)
#
# 方針（YOLOのbboxには頼らない）
#   1. 画像中心(またはhint)から外向きにレイを飛ばして外周を検出 -> 円フィットで中心と半径Rを求める
#   2. リング k(0..4) の中央 r = (0.2k + 0.1) * R を全周サンプリング
#   3. 壁(白)で正規化した色を 黒/赤/黄/緑/青 に分類 -> リングごとに多数決
#   4. 5リングの値を合計。隣り合う同色リングも統合せず、5層それぞれを数える
#   5. 複数フレームで投票

import math
import gc

DEBUG = True

# ---------------- チューニング用パラメータ ----------------
N_RAYS = 16          # 外周検出のレイ本数
WALL_RUN = 4         # 壁の画素がこの px 数連続したら外周とみなす
SAT_FG = 0.28        # 壁で正規化した彩度がこれ以上 -> 色つき（=ターゲット）
V_FG_LO = 0.40       # 壁に対する明るさ比がこれ未満 -> 黒（=ターゲット）
FIT_RMS = 0.10       # 円フィット残差(RMS) / R の上限
R_MIN = 12           # 許容する外半径[px]
R_MAX = 105
N_ANG = 12           # 各リングで何方向サンプルするか
AGREE = 0.6          # リング内で同じ色が占める割合の下限
V_BLACK = 0.38       # 壁に対する明るさ比がこれ未満 -> 黒
S_COLOR = 0.28       # 彩度がこれ未満（かつ黒でない）-> 判定不能
# 色相の境界[deg]: 赤 | 黄 | 緑 | 青 | 赤
H_RY, H_YG, H_GB, H_BR = 28.0, 80.0, 170.0, 290.0

VAL = {'k': -2, 'r': -1, 'y': 0, 'g': 1, 'b': 2}

_DIRS = [(math.cos(2 * math.pi * i / N_RAYS), math.sin(2 * math.pi * i / N_RAYS))
         for i in range(N_RAYS)]


def _log(*a):
    if DEBUG:
        print("[ct]", *a)


# ---------------- 画素アクセス ----------------
# ctx = (img, W, H, inv)  inv は壁色の逆数 (白で正規化するため)

def _wall_ref(img, W, H):
    # 画面を8x8でサンプルし、明るい上位1/4の平均を「壁の色」とする
    pts = []
    for iy in range(8):
        for ix in range(8):
            p = img.get_pixel(int((ix + 0.5) * W / 8), int((iy + 0.5) * H / 8))
            pts.append((p[0] + p[1] + p[2], p[0], p[1], p[2]))
    pts.sort()
    top = pts[-16:]
    n = len(top)
    r = sum(t[1] for t in top) / n
    g = sum(t[2] for t in top) / n
    b = sum(t[3] for t in top) / n
    return max(r, 1.0), max(g, 1.0), max(b, 1.0)


def _get1(ctx, x, y):
    img, W, H, inv = ctx
    if x < 0 or y < 0 or x >= W or y >= H:
        return None
    p = img.get_pixel(x, y)
    return (min(p[0] * inv[0], 1.3), min(p[1] * inv[1], 1.3), min(p[2] * inv[2], 1.3))


def _get5(ctx, x, y):
    # 十字5点平均
    img, W, H, inv = ctx
    if x < 1 or y < 1 or x >= W - 1 or y >= H - 1:
        return None
    r = g = b = 0
    for dx, dy in ((0, 0), (1, 0), (-1, 0), (0, 1), (0, -1)):
        p = img.get_pixel(x + dx, y + dy)
        r += p[0]
        g += p[1]
        b += p[2]
    return (min(r * inv[0] / 5, 1.3), min(g * inv[1] / 5, 1.3), min(b * inv[2] / 5, 1.3))


# ---------------- 色分類 ----------------

def _is_fg(c):
    # 壁(白)以外 = ターゲットの画素
    v = max(c)
    if v < V_FG_LO:
        return True
    return (v - min(c)) / v > SAT_FG


def _classify(c):
    r, g, b = c
    v = max(r, g, b)
    if v < V_BLACK:
        return 'k'
    d = v - min(r, g, b)
    if d / v < S_COLOR:
        return None
    if v == r:
        h = 60.0 * (((g - b) / d) % 6.0)
    elif v == g:
        h = 60.0 * ((b - r) / d + 2.0)
    else:
        h = 60.0 * ((r - g) / d + 4.0)
    if h < H_RY or h >= H_BR:
        return 'r'
    if h < H_YG:
        return 'y'
    if h < H_GB:
        return 'g'
    return 'b'


# ---------------- 円フィット ----------------

def _det3(a, b, c, d, e, f, g, h, i):
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def _fit(pts):
    # 代数的円フィット (Kasa法): x^2+y^2 = A x + B y + C
    n = len(pts)
    sx = sy = sxx = syy = sxy = sz = szx = szy = 0.0
    for x, y in pts:
        z = x * x + y * y
        sx += x
        sy += y
        sxx += x * x
        syy += y * y
        sxy += x * y
        sz += z
        szx += z * x
        szy += z * y
    D = _det3(sxx, sxy, sx, sxy, syy, sy, sx, sy, n)
    if abs(D) < 1e-6:
        return None
    A = _det3(szx, sxy, sx, szy, syy, sy, sz, sy, n) / D
    B = _det3(sxx, szx, sx, sxy, szy, sy, sx, sz, n) / D
    C = _det3(sxx, sxy, szx, sxy, syy, szy, sx, sy, sz) / D
    cx = A / 2
    cy = B / 2
    r2 = C + cx * cx + cy * cy
    if r2 <= 0:
        return None
    return cx, cy, math.sqrt(r2)


def _residuals(pts, cx, cy, R):
    return [abs(math.sqrt((x - cx) * (x - cx) + (y - cy) * (y - cy)) - R) for x, y in pts]


def _fit_robust(pts):
    # 外れ点（壁のシミ・影など）を落としながら2回フィット。戻り値: (cx, cy, R, rms)
    f = _fit(pts)
    if f is None:
        return None
    for _ in range(2):
        res = _residuals(pts, f[0], f[1], f[2])
        med = sorted(res)[len(res) // 2]
        thr = max(1.5, 2.5 * med)
        keep = [p for p, e in zip(pts, res) if e <= thr]
        if len(keep) < 6 or len(keep) == len(pts):
            break
        f2 = _fit(keep)
        if f2 is None:
            break
        pts = keep
        f = f2
    res = _residuals(pts, f[0], f[1], f[2])
    rms = math.sqrt(sum(e * e for e in res) / len(res))
    return f[0], f[1], f[2], rms


# ---------------- 外周検出 ----------------

def _find_start(ctx, cx0, cy0):
    # 色つき画素（=ターゲット内部）をひとつ見つける。まず中心、次に周囲を探す
    for rad in (0, 5, 10, 15):
        n = 1 if rad == 0 else 8
        for i in range(n):
            a = 2 * math.pi * i / n
            x = int(cx0 + rad * math.cos(a) + 0.5)
            y = int(cy0 + rad * math.sin(a) + 0.5)
            c = _get1(ctx, x, y)
            if c is not None and _is_fg(c):
                return x, y
    return None


def _ray(ctx, sx, sy, dx, dy, rmax):
    # 開始点から外向きに進み、壁がWALL_RUN画素続いたら「最後に色つきだった半径」を返す
    last = 0
    run = 0
    for r in range(1, rmax + 1):
        c = _get1(ctx, int(sx + dx * r + 0.5), int(sy + dy * r + 0.5))
        if c is None:
            return None            # 画像端に到達 -> 不明
        if _is_fg(c):
            last = r
            run = 0
        else:
            run += 1
            if run >= WALL_RUN:
                return last if last >= 3 else None
    return None


# ---------------- リング読み取り ----------------

def _read_rings(ctx, cx, cy, R):
    cols = []
    for k in range(5):                      # k=0: 中心の円, k=4: 一番外のリング
        r = (0.2 * k + 0.1) * R
        cnt = {}
        n = 0
        for i in range(N_ANG):
            a = 2 * math.pi * i / N_ANG
            c = _get5(ctx, int(cx + r * math.cos(a) + 0.5), int(cy + r * math.sin(a) + 0.5))
            if c is None:
                continue
            n += 1
            cl = _classify(c)
            if cl is not None:
                cnt[cl] = cnt.get(cl, 0) + 1
        best = None
        bn = 0
        for key in cnt:
            if cnt[key] > bn:
                best = key
                bn = cnt[key]
        if n == 0 or best is None or bn < AGREE * n:
            _log("ring", k, "unsure", cnt, "of", n)
            return None
        cols.append(best)
    return cols


# ---------------- 公開API ----------------

def read_once(img, hint=None):
    """1フレームから値(0/1/2)を返す。読めない/victimでない場合は None。
    hint=(x, y, w, h) はYOLOのbbox（任意）。無ければ画像中心を使う。"""
    W = img.width()
    H = img.height()
    ref = _wall_ref(img, W, H)
    ctx = (img, W, H, (1.0 / ref[0], 1.0 / ref[1], 1.0 / ref[2]))

    if hint:
        hx, hy, hw, hh = hint
        cx0 = hx + hw / 2
        cy0 = hy + hh / 2
        rmax = int(0.9 * max(hw, hh)) + 4
    else:
        cx0 = W / 2
        cy0 = H / 2
        rmax = min(W, H) // 2 - 2

    st = _find_start(ctx, cx0, cy0)
    if st is None:
        _log("NG: no colored area near center")
        return None
    sx, sy = st

    pts = []
    for dx, dy in _DIRS:
        r = _ray(ctx, sx, sy, dx, dy, rmax)
        if r:
            pts.append((dx * r, dy * r))
    if len(pts) < 8:
        _log("NG: too few edge points", len(pts))
        return None

    fit = _fit_robust(pts)
    if fit is None:
        _log("NG: circle fit failed")
        return None
    cx = sx + fit[0]
    cy = sy + fit[1]
    R = fit[2]
    rms = fit[3]
    if DEBUG:
        img.draw_circle(int(cx), int(cy), int(R))
    if R < R_MIN or R > R_MAX or rms > FIT_RMS * R:
        _log("NG: not circle-like R=%d rms=%.1f" % (int(R), rms))
        return None
    if hint and not (0.5 < 2 * R / max(hw, hh) < 1.6):
        _log("NG: size mismatch with hint", int(R), hw, hh)
        return None

    cols = _read_rings(ctx, cx, cy, R)
    if cols is None:
        return None
    s = sum(VAL[c] for c in cols)
    _log("rings(center->out)", "".join(cols), "sum", s,
         "R", int(R), "ctr", int(cx), int(cy), "rms %.1f" % rms)
    if s in (0, 1, 2):
        return s
    return None          # 合計が0/1/2以外: 健康状態を表さないターゲット -> victimではない


def decide(snap, hint=None, n_frames=5, min_agree=3):
    """snap: 画像を返す関数 (sensor.snapshot)。
    最大n_frames枚読み、min_agree枚以上一致した値(0/1/2)を返す。合意できなければ -1。"""
    votes = [0, 0, 0]
    for i in range(n_frames):
        v = read_once(snap(), hint)
        gc.collect()
        if v is not None:
            votes[v] += 1
            if votes[v] >= min_agree:
                return v
        if max(votes) + (n_frames - 1 - i) < min_agree:
            break
    _log("decide: no consensus", votes)
    return -1


# 単体テスト用: MaixPy IDEでこのファイルを直接実行すると、カメラ画像で読み取りを繰り返す
if __name__ == "__main__":
    import sensor
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing((224, 224))
    sensor.skip_frames(time=2000)
    print(sensor.snapshot().get_pixel(112, 112))   # (r, g, b) のタプルが出ることを確認
    while True:
        print("value:", read_once(sensor.snapshot()))
