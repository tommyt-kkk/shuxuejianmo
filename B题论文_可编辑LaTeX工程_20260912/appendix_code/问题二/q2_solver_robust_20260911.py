#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026 国赛 B 题 问题二：带接收保证的第二个检测点选择（离散近似实现）

论文公式对应关系
    (21) C1 = D  intersect  W1  intersect  B(S1,1500)      -> candidate_source_region()
    (24) Q_det = {q: max|q-G| <= 1000}  -> reception_feasible()
    (35)-(37) P2 = C1  intersect  W2(q, theta2)       -> second_region()
    连续 D_wc 的角域离散代理             -> sampled_worst_case_loss()
    相对容差分层优化                      -> optimize_second_point()
    J_phi 采样后验诊断                    -> min_cross_angle_quality()

只用标准库；画图需要 matplotlib（可选）。

用法
    python q2_solver_robust_20260911.py --x1 0 --y1 0 --svd 0
    python q2_solver_robust_20260911.py --x1 400 --y1 -300 --svd 37.5 --plot
    python q2_solver_robust_20260911.py --convergence
    python q2_solver_robust_20260911.py --selftest
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

# ----------------------------------------------------------------------------
# 题目常数（题目正文 + 附件）
# ----------------------------------------------------------------------------
EPS_DEG = 1.0                            # 示向度误差上界 +/-1 deg
EPS = math.radians(EPS_DEG)
R_MIN = 1000.0                           # 有效接收半径下界
R_MAX = 1500.0                           # 有效接收半径上界
R_ARENA = 1800.0                         # 目标圆域半径
NEAR_TH = 5.0                            # 近距离阈值

CLIP_TOL = 1e-9                          # 半平面裁剪容差（仅数值用）
POINT_TOL = 1e-7                         # 几何点去重/边界判断容差（m）
AREA_REL_TOL = 1e-12                     # 退化多边形相对面积容差
OBJECTIVE_ABS_TOL = 1e-6                 # 目标值为 0 时的数值保护（m）

Vec = Tuple[float, float]
Poly = List[Vec]
HalfPlane = Tuple[float, float, float]   # n*x <= c 即 n[0]*x + n[1]*y <= c


# ----------------------------------------------------------------------------
# 基础几何
# ----------------------------------------------------------------------------
def wrap_pi(a: float) -> float:
    """把角度差归一到 [-pi, pi)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def dist(p: Vec, q: Vec) -> float:
    return math.hypot(p[0] - q[0], p[1] - q[1])


def polygon_centroid(poly: Poly) -> Vec:
    return (sum(p[0] for p in poly) / len(poly),
            sum(p[1] for p in poly) / len(poly))


def polygon_area2(poly: Poly) -> float:
    """返回多边形有向面积的两倍；点或线段返回 0。"""
    n = len(poly)
    if n < 3:
        return 0.0
    return sum(poly[i][0] * poly[(i + 1) % n][1]
               - poly[(i + 1) % n][0] * poly[i][1]
               for i in range(n))


def normalize_polygon(poly: Poly, tol: float = POINT_TOL) -> Poly:
    """按原顺序删除重复/近重复顶点，保留点、线段等退化交集。"""
    out: Poly = []
    for p in poly:
        if not any(dist(p, q) <= tol for q in out):
            out.append(p)
    return out


def clip_polygon(poly: Poly, hp: HalfPlane) -> Poly:
    """Sutherland-Hodgman：用半平面 n*x <= c 裁剪凸多边形"""
    nx, ny, c = hp
    out: Poly = []
    m = len(poly)
    for i in range(m):
        a = poly[i]
        b = poly[(i + 1) % m]
        fa = nx * a[0] + ny * a[1] - c
        fb = nx * b[0] + ny * b[1] - c
        ina = fa <= CLIP_TOL
        inb = fb <= CLIP_TOL
        if ina:
            out.append(a)
        if ina != inb:                      # 边跨越边界 -> 求交点
            t = fa / (fa - fb)
            out.append((a[0] + t * (b[0] - a[0]),
                        a[1] + t * (b[1] - a[1])))
    return normalize_polygon(out)


def intersect_halfplanes(hps: Sequence[HalfPlane], box: Poly) -> Poly:
    """半平面交：用一个足够大的凸多边形作起点逐条裁剪"""
    poly = list(box)
    for hp in hps:
        poly = clip_polygon(poly, hp)
        if not poly:
            return []
    return poly


def circle_outer_halfplanes(center: Vec, r: float, n: int) -> List[HalfPlane]:
    """半径 r 的圆的外接正 n 边形 = n 条切线半平面（外近似，保守）"""
    hps = []
    cx, cy = center
    for k in range(n):
        th = 2.0 * math.pi * k / n
        nx, ny = math.cos(th), math.sin(th)
        hps.append((nx, ny, nx * cx + ny * cy + r))
    return hps


def cone_halfplanes(apex: Vec, theta: float, eps: float) -> List[HalfPlane]:
    """
    从 apex 出发、中心方向 theta、半角 eps 的角形区域（论文式 (6)）
        u^- x (X-S) >= 0  ->  ( u^-_y, -u^-_x ) * X <= u^-_y Sx - u^-_x Sy
        u^+ x (X-S) <= 0  ->  (-u^+_y,  u^+_x ) * X <= u^+_x Sy - u^+_y Sx
    注意第二式的常数项顺序，容易写反（S!=原点时才会暴露）。
    """
    ax, ay = apex
    um = (math.cos(theta - eps), math.sin(theta - eps))
    up = (math.cos(theta + eps), math.sin(theta + eps))
    hp1 = (um[1], -um[0], um[1] * ax - um[0] * ay)
    hp2 = (-up[1], up[0], up[0] * ay - up[1] * ax)
    return [hp1, hp2]


def polygon_diameter_bruteforce(poly: Poly) -> float:
    """凸多边形直径：顶点对枚举 O(n^2)，作为基准/小规模使用"""
    best = 0.0
    n = len(poly)
    for i in range(n):
        for j in range(i + 1, n):
            best = max(best, dist(poly[i], poly[j]))
    return best


def polygon_diameter_calipers(poly: Poly) -> float:
    """凸多边形直径：旋转卡壳 O(n)（论文 5.4 的算法）"""
    poly = normalize_polygon(poly)
    n = len(poly)
    if n < 3:
        return polygon_diameter_bruteforce(poly)
    scale = max(1.0, max(abs(x) for p in poly for x in p))
    area2 = polygon_area2(poly)
    if abs(area2) <= AREA_REL_TOL * scale * scale:
        return polygon_diameter_bruteforce(poly)
    # 统一为逆时针
    pts = poly if area2 > 0 else list(reversed(poly))
    k = 1
    best = 0.0
    for i in range(n):
        ni = (i + 1) % n
        ax, ay = pts[i]
        bx, by = pts[ni]
        def area(j: int) -> float:
            px, py = pts[j % n]
            return abs((bx - ax) * (py - ay) - (px - ax) * (by - ay))
        # 推进对踵点（用 >= 处理共线，避免漏掉点对）
        while area(k + 1) >= area(k):
            k += 1
            if k - i > n:            # 防御性上限
                break
        best = max(best, dist(pts[i], pts[k % n]), dist(pts[ni], pts[k % n]))
    return best


# ----------------------------------------------------------------------------
# 论文模型
# ----------------------------------------------------------------------------
def initial_box(S1: Vec) -> Poly:
    """覆盖 B(S1,R_MAX)  union  B(O,R_ARENA) 的正方形，作为半平面交的起点"""
    r = max(dist(S1, (0.0, 0.0)) + R_MAX, R_ARENA) + 100.0
    return [(-r, -r), (r, -r), (r, r), (-r, r)]


def candidate_source_region(S1: Vec, theta1: float, n_arc: int = 180) -> Poly:
    """
    式 (21)：C1 的外近似（全为半平面）
        圆域用外接正多边形近似 -> 得到的 C_hat1 superset of C1，接收保证与直径都偏保守
    """
    hps: List[HalfPlane] = []
    hps += cone_halfplanes(S1, theta1, EPS)               # 第一次示向约束 W1
    hps += circle_outer_halfplanes(S1, R_MAX, n_arc)      # |G-S1| <= 1500
    hps += circle_outer_halfplanes((0.0, 0.0), R_ARENA, n_arc)   # 目标圆域 D
    return intersect_halfplanes(hps, initial_box(S1))


def boundary_samples(poly: Poly, k: int = 2) -> Poly:
    """凸多边形边界采样：顶点 + 每条边 k 个内点"""
    pts: Poly = []
    n = len(poly)
    for i in range(n):
        a = poly[i]
        b = poly[(i + 1) % n]
        pts.append(a)
        for t in range(1, k + 1):
            s = t / (k + 1)
            pts.append((a[0] + s * (b[0] - a[0]), a[1] + s * (b[1] - a[1])))
    return pts


def reception_feasible(q: Vec, C1: Poly, margin: float = 0.0) -> Tuple[bool, float]:
    """
    式 (24)：q  in  Q_det iff max_{G in C1} |q-G| <= R_MIN
    凸函数在凸多边形的最大值必在顶点取到，因此只需遍历顶点。
    """
    if not C1:
        return False, math.inf
    dmax = max(dist(q, v) for v in C1)
    return (dmax <= R_MIN + margin, dmax)


def point_in_convex_polygon(q: Vec, poly: Poly,
                            tol: float = POINT_TOL) -> int:
    """判断点与非退化凸多边形的位置：1 内部，0 边界，-1 外部。"""
    pts = normalize_polygon(poly, tol)
    if len(pts) < 3 or abs(polygon_area2(pts)) <= AREA_REL_TOL:
        return -1
    sign = 1.0 if polygon_area2(pts) > 0.0 else -1.0
    on_boundary = False
    for i, a in enumerate(pts):
        b = pts[(i + 1) % len(pts)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        cross = sign * (ex * (q[1] - a[1]) - ey * (q[0] - a[0]))
        cross_tol = tol * max(1.0, math.hypot(ex, ey))
        if cross < -cross_tol:
            return -1
        if abs(cross) <= cross_tol:
            on_boundary = True
    return 0 if on_boundary else 1


def bearing_interval(q: Vec, C1: Poly, k_edge: int = 2) -> Tuple[float, float]:
    """
    返回从 q 看凸集 C1 的连续方位角覆盖区间 [beta_min, beta_max]。

    q 严格位于 C1 内部时覆盖完整 2pi；边界或外部情形则对所有顶点
    方向寻找最大循环空隙，取其补弧。参数 k_edge 仅为旧接口兼容保留。
    注意：该区间基于扩大的 C1，而非去除了两个 5 m 邻域的物理源集，
    因而用于构造保守的扩大角域，不代表排除邻域后的精确可达角集。
    """
    del k_edge
    pts = normalize_polygon(C1)
    if not pts:
        raise ValueError("C1 为空，无法计算方位角区间")
    if point_in_convex_polygon(q, pts) == 1:
        return -math.pi, math.pi

    angles = sorted(math.atan2(p[1] - q[1], p[0] - q[0]) % (2.0 * math.pi)
                    for p in pts if dist(p, q) > POINT_TOL)
    if not angles:
        return 0.0, 0.0
    if len(angles) == 1:
        return angles[0], angles[0]

    gaps = [angles[i + 1] - angles[i] for i in range(len(angles) - 1)]
    gaps.append(angles[0] + 2.0 * math.pi - angles[-1])
    j = max(range(len(gaps)), key=gaps.__getitem__)
    start = angles[(j + 1) % len(angles)]
    end = angles[j]
    if end < start:
        end += 2.0 * math.pi
    return start, end


def second_region(q: Vec, theta2: float, C1: Poly) -> Poly:
    """式 (37)：P2 = C1  intersect  W2(q, theta2)（用外近似 C_hat1 裁剪，结果为 P2 superset of P2）"""
    poly = list(C1)
    for hp in cone_halfplanes(q, theta2, EPS):
        poly = clip_polygon(poly, hp)
        if not poly:
            return []
    return normalize_polygon(poly)


def sampled_worst_case_loss(q: Vec, C1: Poly, n_theta: int = 121,
                            k_edge: int = 2) -> Tuple[float, float]:
    """
    式 (38) 连续最坏损失的角域扩大、有限采样近似。

    P2 只通过 theta2 依赖 (G,e2)，代码在由扩大集合 C1 得到的方位角覆盖上
    进行一维采样。角域扩大本身偏保守，但有限采样可能漏过连续峰值，
    因此返回值只是离散近似，不能称为连续 D_wc 的严格上界或精确值。
    返回 (采样最大直径, 对应 theta2)。
    """
    if n_theta < 3:
        raise ValueError("n_theta 必须至少为 3")
    lo, hi = bearing_interval(q, C1, k_edge)
    full_circle = hi - lo >= 2.0 * math.pi - 1e-10
    if full_circle:
        theta_values = [lo + 2.0 * math.pi * i / n_theta
                        for i in range(n_theta)]
    else:
        t0, t1 = lo - EPS, hi + EPS
        theta_values = [t0 + (t1 - t0) * i / (n_theta - 1)
                        for i in range(n_theta)]

    best, best_t = 0.0, theta_values[0]
    for t in theta_values:
        P2 = second_region(q, t, C1)
        if not P2:
            continue
        d = polygon_diameter_calipers(P2)
        if d > best:
            best, best_t = d, t
    return best, best_t


def min_cross_angle_quality(q: Vec, S1: Vec, C1: Poly, k_edge: int = 2) -> float:
    """式 (42)：J_phi(q) = min_{G} |sin phi(G,q)|，仅作搜索引导/复核

    phi 是两条示向线（S1->G 与 q->G）的夹角，故取向量 (G-S1) 与 (G-q)。
    注意：G 与 S1 重合（或距离 <= 5 m）时，S1->G 方向未定义，应排除；
    同理 G 与 q 距离 <= 5 m 时 q->G 未定义（该情形由"距离过近->直接清除"处理）。
    """
    best = None
    for g in boundary_samples(C1, k_edge):
        if dist(q, g) <= NEAR_TH or dist(S1, g) <= NEAR_TH:
            continue
        v1 = (g[0] - S1[0], g[1] - S1[1])
        v2 = (g[0] - q[0], g[1] - q[1])
        ang = abs(math.sin(math.atan2(v1[0] * v2[1] - v1[1] * v2[0],
                                     v1[0] * v2[0] + v1[1] * v2[1])))
        best = ang if best is None else min(best, ang)
    return 0.0 if best is None else best


# ----------------------------------------------------------------------------
# 分层优化（式 (39)-(41)）
# ----------------------------------------------------------------------------
def adaptive_ab_range(S1: Vec, theta1: float, C1: Poly,
                      radius: float = R_MIN,
                      margin: float = POINT_TOL) -> Tuple[Tuple[float, float],
                                                          Tuple[float, float]]:
    """由所有 C1 顶点的接收圆约束构造不漏解的 (a,b) 必要包围盒。"""
    u = (math.cos(theta1), math.sin(theta1))
    nrm = (-math.sin(theta1), math.cos(theta1))
    local = []
    for p in C1:
        dx, dy = p[0] - S1[0], p[1] - S1[1]
        local.append((dx * u[0] + dy * u[1],
                      dx * nrm[0] + dy * nrm[1]))
    if not local:
        raise ValueError("C1 为空，无法构造搜索包围盒")

    a_lo = max(a - radius for a, _ in local) - margin
    a_hi = min(a + radius for a, _ in local) + margin
    b_lo = max(b - radius for _, b in local) - margin
    b_hi = min(b + radius for _, b in local) + margin
    if a_lo > a_hi or b_lo > b_hi:
        raise RuntimeError("接收保证区域的必要包围盒为空")
    return (a_lo, a_hi), (b_lo, b_hi)


def optimize_second_point(S1: Vec, theta1: float,
                          a_range: Optional[Tuple[float, float]] = None,
                          b_range: Optional[Tuple[float, float]] = None,
                          coarse: int = 16, refine: int = 3,
                          n_theta: int = 121, n_arc: int = 180,
                          decision_rel_tol: float = 0.05,
                          refine_rel_tol: float = 0.10) -> dict:
    if coarse < 2:
        raise ValueError("coarse 必须至少为 2")
    if refine < 0:
        raise ValueError("refine 不能为负数")
    if n_theta < 3:
        raise ValueError("n_theta 必须至少为 3")
    if n_arc < 8:
        raise ValueError("n_arc 必须至少为 8")
    if decision_rel_tol < 0.0 or refine_rel_tol < 0.0:
        raise ValueError("相对容差不能为负数")

    C1 = candidate_source_region(S1, theta1, n_arc)
    if len(C1) < 3 or abs(polygon_area2(C1)) <= AREA_REL_TOL:
        raise RuntimeError("C1 为空，请检查输入")

    auto_a_range, auto_b_range = adaptive_ab_range(S1, theta1, C1)
    a_range = auto_a_range if a_range is None else a_range
    b_range = auto_b_range if b_range is None else b_range
    if a_range[0] >= a_range[1] or b_range[0] >= b_range[1]:
        raise ValueError("搜索范围上下界无效")

    u = (math.cos(theta1), math.sin(theta1))
    nrm = (-math.sin(theta1), math.cos(theta1))

    def to_xy(a: float, b: float) -> Vec:
        return (S1[0] + a * u[0] + b * nrm[0],
                S1[1] + a * u[1] + b * nrm[1])

    evaluated = []                       # (D_sample, maxdist, a, b, theta2)
    seen = set()

    def scan(a0, a1, b0, b1, na, nb):
        for ia in range(na + 1):
            a = a0 + (a1 - a0) * ia / na
            for ib in range(nb + 1):
                b = b0 + (b1 - b0) * ib / nb
                key = (round(a, 10), round(b, 10))
                if key in seen:
                    continue
                seen.add(key)
                q = to_xy(a, b)
                ok, dmax = reception_feasible(q, C1)
                if not ok:
                    continue
                d_sample, th2 = sampled_worst_case_loss(q, C1, n_theta)
                evaluated.append((d_sample, dmax, a, b, th2))

    # 第一阶段：粗网格
    scan(a_range[0], a_range[1], b_range[0], b_range[1], coarse, coarse)
    if not evaluated:
        raise RuntimeError("候选网格中没有任何满足接收约束的点")

    # 第二阶段：在最优解附近逐级加密
    for it in range(refine):
        evaluated.sort(key=lambda t: t[0])
        d_best = evaluated[0][0]
        refine_allowance = max(OBJECTIVE_ABS_TOL, refine_rel_tol * abs(d_best))
        pool = [e for e in evaluated if e[0] <= d_best + refine_allowance]
        a_lo = min(e[2] for e in pool); a_hi = max(e[2] for e in pool)
        b_lo = min(e[3] for e in pool); b_hi = max(e[3] for e in pool)
        step_a = (a_range[1] - a_range[0]) / (coarse * (2 ** (it + 1)))
        step_b = (b_range[1] - b_range[0]) / (coarse * (2 ** (it + 1)))
        a_lo = max(a_range[0], a_lo - 4 * step_a)
        a_hi = min(a_range[1], a_hi + 4 * step_a)
        b_lo = max(b_range[0], b_lo - 4 * step_b)
        b_hi = min(b_range[1], b_hi + 4 * step_b)
        scan(a_lo, a_hi, b_lo, b_hi, coarse, coarse)

    evaluated.sort(key=lambda t: t[0])
    d_star = evaluated[0][0]
    # 相对容差的离散近似最优集；零目标值用极小绝对容差作数值保护。
    decision_allowance = max(OBJECTIVE_ABS_TOL, decision_rel_tol * abs(d_star))
    pool = [e for e in evaluated if e[0] <= d_star + decision_allowance]
    pool.sort(key=lambda e: math.hypot(e[2], e[3]))     # 移动时间  proportional to  |q-S1|/5
    best_move = pool[0]
    q_star = to_xy(best_move[2], best_move[3])

    return {
        "C1": C1,
        "q_star": q_star,
        "a_star": best_move[2],
        "b_star": best_move[3],
        "D_sample_star": best_move[0],
        "reception_maxdist": best_move[1],
        "reception_slack": R_MIN - best_move[1],
        "theta2_sample_worst_deg": math.degrees(best_move[4]),
        "D_sample_min": d_star,
        "move_time_s": math.hypot(best_move[2], best_move[3]) / 5.0,
        "J_phi": min_cross_angle_quality(q_star, S1, C1),
        "n_evaluated": len(evaluated),
        "search_a_range": a_range,
        "search_b_range": b_range,
        "decision_rel_tol": decision_rel_tol,
        "refine_rel_tol": refine_rel_tol,
        "n_theta": n_theta,
        "n_arc": n_arc,
        "coarse": coarse,
        "refine": refine,
        "top": evaluated[:10],
    }


def convergence_check(S1: Vec, theta1: float, q_ref: Vec) -> dict:
    """分别检查角度、圆弧和候选点网格离散；不把稳定性等同于严格收敛证明。"""
    theta_rows = []
    C1_theta = candidate_source_region(S1, theta1, 180)
    for n_theta in (61, 121, 241, 481):
        value, th = sampled_worst_case_loss(q_ref, C1_theta, n_theta)
        theta_rows.append({"n_theta": n_theta, "D_sample": value,
                           "theta_deg": math.degrees(th)})

    arc_rows = []
    for n_arc in (60, 90, 180, 360):
        C1_arc = candidate_source_region(S1, theta1, n_arc)
        ok, dmax = reception_feasible(q_ref, C1_arc)
        value, th = sampled_worst_case_loss(q_ref, C1_arc, 241)
        arc_rows.append({"n_arc": n_arc, "D_sample": value,
                         "theta_deg": math.degrees(th),
                         "reception_feasible": ok,
                         "reception_maxdist": dmax})

    # 搜索网格只用较低的固定几何精度生成候选，再统一在高精度设置下复核。
    grid_rows = []
    C1_eval = candidate_source_region(S1, theta1, 180)
    for coarse in (8, 12, 16):
        res = optimize_second_point(
            S1, theta1, coarse=coarse, refine=1, n_theta=61, n_arc=90,
            decision_rel_tol=0.0, refine_rel_tol=0.10)
        q = res["q_star"]
        ok, dmax = reception_feasible(q, C1_eval)
        value, th = sampled_worst_case_loss(q, C1_eval, 241)
        grid_rows.append({"coarse": coarse, "q": q,
                          "D_sample_rechecked": value,
                          "theta_deg_rechecked": math.degrees(th),
                          "reception_feasible_rechecked": ok,
                          "reception_maxdist_rechecked": dmax})
    return {"theta_sampling": theta_rows,
            "arc_approximation": arc_rows,
            "candidate_grid": grid_rows}


# ----------------------------------------------------------------------------
# 自检
# ----------------------------------------------------------------------------
def selftest() -> None:
    # 1) 角形区域：中心方向上的点应在内，反向点应在外
    apex = (0.0, 0.0)
    hps = cone_halfplanes(apex, 0.0, EPS)
    for p, expect_in in [((100.0, 0.0), True), ((100.0, 3.0), False),
                         ((-100.0, 0.0), False), ((1000.0, 17.0), True)]:
        inside = all(h[0] * p[0] + h[1] * p[1] <= h[2] + 1e-9 for h in hps)
        assert inside == expect_in, (p, inside, expect_in)
    # 2) 平移后仍正确（常量项符号检查）
    apex2 = (900.0, 0.0)
    hps2 = cone_halfplanes(apex2, math.radians(3.5), EPS)
    inside_self = all(h[0] * apex2[0] + h[1] * apex2[1] <= h[2] + 1e-9 for h in hps2)
    assert inside_self, "锥顶必须在角形区域内"
    assert not all(h[0] * 0.0 + h[1] * 0.0 <= h[2] + 1e-9 for h in hps2), "原点不应在该角形内"
    # 3) 直径算法一致性
    poly = [(0.0, 0.0), (1500.0, -26.18), (1500.0, 26.18),
            (900.0, 200.0), (-300.0, 40.0)]
    a = polygon_diameter_bruteforce(poly)
    b = polygon_diameter_calipers(poly)
    assert abs(a - b) < 1e-6, (a, b)
    # 4) 重复点表示的退化线段仍应得到正确直径
    line = [(0.0, 0.0), (3.0, 4.0), (3.0, 4.0), (0.0, 0.0)]
    assert abs(polygon_diameter_calipers(line) - 5.0) < 1e-9
    # 5) 让第二角形与正方形下边重合，交集应为长度 1 的线段
    square = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    segment = second_region((-1.0, 0.0), -EPS, square)
    assert len(segment) == 2, segment
    assert abs(polygon_diameter_calipers(segment) - 1.0) < 1e-8, segment
    # 6) 内部、边内部、顶点、外部四类方位角覆盖
    lo, hi = bearing_interval((0.5, 0.5), square)
    assert abs((hi - lo) - 2.0 * math.pi) < 1e-9
    lo, hi = bearing_interval((0.5, 0.0), square)
    assert abs((hi - lo) - math.pi) < 1e-9, (lo, hi)
    lo, hi = bearing_interval((0.0, 0.0), square)
    assert abs((hi - lo) - math.pi / 2.0) < 1e-9, (lo, hi)
    lo, hi = bearing_interval((-1.0, 0.5), square)
    assert 0.0 < hi - lo < math.pi, (lo, hi)
    # 7) 理论构造点 q0 应满足保守接收约束
    C1 = candidate_source_region((0.0, 0.0), 0.0, n_arc=60)
    q0 = (750.0, 0.0)
    ok, dmax = reception_feasible(q0, C1)
    assert ok, dmax
    # 8) 离散最坏损失基本调用
    d_sample, _ = sampled_worst_case_loss(q0, C1, n_theta=21)
    assert d_sample > 0.0 and math.isfinite(d_sample), d_sample
    print("[selftest] 几何裁剪、退化直径、方位角覆盖与接收约束检查通过")


# ----------------------------------------------------------------------------
# 主程序
# ----------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser(description="问题二：第二个检测点选择（离散近似）")
    ap.add_argument("--x1", type=float, default=0.0, help="第一个检测点 x")
    ap.add_argument("--y1", type=float, default=0.0, help="第一个检测点 y")
    ap.add_argument("--svd", type=float, default=0.0, help="第一次测得的示向度（度）")
    ap.add_argument("--n-theta", type=int, default=121, help="theta2 采样数")
    ap.add_argument("--n-arc", type=int, default=180, help="圆域外接多边形边数")
    ap.add_argument("--coarse", type=int, default=16, help="粗网格每边分段数")
    ap.add_argument("--refine", type=int, default=3, help="加密轮数")
    ap.add_argument("--delta", type=float, default=0.05,
                    help="最终离散近似最优集的相对容差 delta_D")
    ap.add_argument("--refine-delta", type=float, default=0.10,
                    help="逐级加密候选池的相对容差")
    ap.add_argument("--output-dir", type=Path,
                    default=Path(__file__).resolve().parent / "outputs" /
                    "问题二_鲁棒几何修订_20260911",
                    help="JSON 与图表输出目录")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--convergence", action="store_true",
                    help="执行角度、圆弧和候选网格的分层稳定性检查")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        selftest()
        return

    S1 = (args.x1, args.y1)
    theta1 = math.radians(args.svd)
    res = optimize_second_point(S1, theta1, n_theta=args.n_theta, n_arc=args.n_arc,
                                coarse=args.coarse, refine=args.refine,
                                decision_rel_tol=args.delta,
                                refine_rel_tol=args.refine_delta)
    res["input"] = {"S1": S1, "theta1_deg": args.svd}

    q = res["q_star"]
    print("=== C1 顶点（外近似） ===")
    for v in res["C1"]:
        print("   (%.2f, %.2f)" % v)
    print("=== 推荐第二检测点 ===")
    print("   q* = (%.2f, %.2f)" % q)
    print("   a = %.2f m (沿中心示向), b = %.2f m (侧向)" % (res["a_star"], res["b_star"]))
    print("   移动时间 = %.2f s" % res["move_time_s"])
    print("   离散最坏定位损失近似值 = %.2f m (采样最坏 theta2 = %.1f deg)" %
          (res["D_sample_star"], res["theta2_sample_worst_deg"]))
    print("   到 C1 外近似连续边界的最大距离 = %.2f m；接收松弛量 = %.2f m" %
          (res["reception_maxdist"], res["reception_slack"]))
    print("   采样交会角质量（后验诊断）= %.3f" % res["J_phi"])
    print("=== 前 10 个候选（按离散损失升序）===")
    for d_sample, dmax, a, b, th2 in res["top"]:
        print("   a=%7.2f b=%8.2f  D_sample=%8.2f  maxdist=%7.2f  theta2=%7.1f" %
              (a, b, d_sample, dmax, math.degrees(th2)))

    if args.convergence:
        print("正在执行分层稳定性检查......")
        res["convergence"] = convergence_check(S1, theta1, q)

    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "q2_result.json"
    with json_path.open("w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=2, allow_nan=False)
    print("结果已保存：%s" % json_path)

    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:                       # pragma: no cover
            print("matplotlib 不可用，跳过绘图：", exc)
            return
        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        xs = [v[0] for v in res["C1"]] + [res["C1"][0][0]]
        ys = [v[1] for v in res["C1"]] + [res["C1"][0][1]]
        plt.figure(figsize=(6, 6))
        plt.plot(xs, ys, "-", label=r"候选源区域 $\widehat C_1$")
        plt.plot([S1[0]], [S1[1]], "ko", label=r"第一检测点 $S_1$")
        plt.plot([q[0]], [q[1]], "r*", markersize=14, label=r"推荐点 $q^*$")
        for d_sample, dmax, a, b, th2 in res["top"]:
            qq = (S1[0] + a * math.cos(theta1) - b * math.sin(theta1),
                  S1[1] + a * math.sin(theta1) + b * math.cos(theta1))
            plt.plot([qq[0]], [qq[1]], "b.", alpha=0.4)
        plt.gca().set_aspect("equal")
        plt.xlabel("横坐标 / m")
        plt.ylabel("纵坐标 / m")
        plt.grid(alpha=0.3)
        plt.legend()
        pdf_path = output_dir / "q2_result.pdf"
        png_path = output_dir / "q2_result.png"
        plt.savefig(pdf_path, bbox_inches="tight")
        plt.savefig(png_path, dpi=180, bbox_inches="tight")
        plt.close()
        print("图已保存：%s；%s" % (pdf_path, png_path))


if __name__ == "__main__":
    main()
