"""差速底盘教学示例。单位：m、s、rad。仅使用 Python 标准库。

运行：python diff_drive.py
保存轨迹：python diff_drive.py --csv trajectory.csv
模型假设：两轮半径相同，纯滚动，无侧滑，参考点为轮轴中点。
omega_l / omega_r 为车轮角速度，不是减速器前的电机轴角速度。
"""

from math import cos, sin, isfinite
import argparse
import csv


def check_geometry(r, L):
    if not (isfinite(r) and isfinite(L) and r > 0 and L > 0):
        raise ValueError("r and L must be finite and positive")


def forward(omega_l, omega_r, r, L):
    """由左右车轮角速度得到底盘前进速度 v 和左转角速度 Omega。"""
    check_geometry(r, L)
    v = r * (omega_r + omega_l) / 2
    Omega = r * (omega_r - omega_l) / L
    return v, Omega


def inverse(v, Omega, r, L):
    """由目标底盘速度得到左右车轮角速度，返回顺序为左、右。"""
    check_geometry(r, L)
    omega_l = (v - L * Omega / 2) / r
    omega_r = (v + L * Omega / 2) / r
    return omega_l, omega_r


def step(x, y, theta, v, Omega, dt):
    """显式欧拉法。x 和 y 都使用更新前的 theta。"""
    if not isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    x_new = x + v * cos(theta) * dt
    y_new = y + v * sin(theta) * dt
    theta_new = theta + Omega * dt
    return x_new, y_new, theta_new


def simulate(omega_l=5.0, omega_r=7.0, r=0.03, L=0.16,
             dt=0.01, steps=200):
    """恒定轮速下从原点朝世界 x 轴出发，返回包含初始状态的轨迹。"""
    v, Omega = forward(omega_l, omega_r, r, L)
    x, y, theta = 0.0, 0.0, 0.0
    rows = [(0.0, x, y, theta)]
    for k in range(steps):
        x, y, theta = step(x, y, theta, v, Omega, dt)
        rows.append(((k + 1) * dt, x, y, theta))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", help="可选：保存 t,x,y,theta 四列轨迹")
    args = parser.parse_args()
    v, Omega = forward(5.0, 7.0, 0.03, 0.16)
    print(f"v={v:.3f} m/s, Omega={Omega:.3f} rad/s")
    omega_l, omega_r = inverse(v, Omega, 0.03, 0.16)
    print(f"inverse: omega_l={omega_l:.3f}, omega_r={omega_r:.3f} rad/s")
    rows = simulate()
    t, x, y, theta = rows[-1]
    print(f"Euler at t={t:.2f}s: x={x:.6f}, y={y:.6f}, theta={theta:.6f}")
    # 仅用于这个恒定 v、Omega 且初始位姿为零的例题。
    exact_x = v / Omega * sin(Omega * t)
    exact_y = v / Omega * (1 - cos(Omega * t))
    print(f"Exact circular motion: x={exact_x:.6f}, y={exact_y:.6f}")
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(("t_s", "x_m", "y_m", "theta_rad"))
            writer.writerows(rows)
        print(f"Saved {len(rows)} rows to {args.csv}")


if __name__ == "__main__":
    main()
