"""Independent reference for checks.toml (LandingGearAssembly harness).

Evaluates the FIRE CompliantPointLeg law (penalty normal force, viscous
friction capped at mu*N) for the asymmetric 4-leg layout of world.toml, at the
prescribed CG kinematics of each harness.c phase, and prints the expected
per-leg and summed values. Run: python3 ref.py  (numpy only).
Inputs are a zero-order hold: the trace row at t reflects inputs at t - 1 ms.
"""
import numpy as np

k, c, ct, mu = 3000.0, 150.0, 25.0, 0.6
R_LEG = np.array([[0.20, 0.15, 0.10], [-0.12, 0.18, 0.12],
                  [-0.15, -0.16, 0.10], [0.18, -0.10, 0.11]])


def Rzyx(yaw, pitch, roll):
    cz, sz, cy, sy, cx, sx = (np.cos(yaw), np.sin(yaw), np.cos(pitch),
                              np.sin(pitch), np.cos(roll), np.sin(roll))
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    return Rz @ Ry @ Rx


LEVEL = (np.zeros(3), np.array([0, 0, -1.0]), np.zeros(3))
# Sloped, moving terrain of the 2.30-2.70 s phase.
INC, AZ = np.radians(8.0), np.radians(-40.0)
SLOPE_N = np.array([-np.sin(INC) * np.cos(AZ), -np.sin(INC) * np.sin(AZ), -np.cos(INC)])
SLOPE_P = np.array([1.5, -0.8, 0.4])
SLOPE_V = np.array([-0.2, 0.15, 0.05])
SLOPE_R = Rzyx(-0.5, 0.1, -0.12)
SLOPE_H = 0.085  # CG height above the plane along n
SLOPE_CG = SLOPE_P + np.array([0.25, -0.1, 0.0]) - (np.array([0.25, -0.1, 0.0]) @ SLOPE_N) * SLOPE_N \
    + SLOPE_H * SLOPE_N


def gear(p, v, w, R, terrain=LEVEL):
    tP, n, tv = terrain
    F = np.zeros(3); M = np.zeros(3); legs = []
    for r in R_LEG:
        tip = p + R @ r
        rel = R @ (v + np.cross(w, r)) - tv
        gap = n @ (tip - tP); vn = n @ rel
        contact = gap <= 0
        N = max(0.0, k * max(0.0, -gap) - c * vn) if contact else 0.0
        vt = rel - vn * n; sp = np.linalg.norm(vt)
        fm = min(ct * sp, mu * N) if (contact and N > 0) else 0.0
        fw = N * n - fm * vt / max(sp, 1e-6); fb = R.T @ fw
        F += fb; M += np.cross(r, fb)
        legs.append(dict(gap=gap, N=N, contact=contact, fm=fm, cap=mu * N, sp=sp,
                         power=(fw - N * n) @ vt, fw=fw))
    return F, M, legs


I = np.eye(3)
Z = np.zeros(3)
PHASES = [
    ("0.575", (np.array([0, 0, -0.3 + 0.5 * 0.374]), np.array([0, 0, .5]), Z, I)),
    ("0.61", (np.array([0, 0, -0.3 + 0.5 * 0.409]), np.array([0, 0, .5]), Z, I)),
    ("0.8", (np.array([0, 0, -0.095]), Z, Z, I)),
    ("1.1", (np.array([0, 0, -0.095]), Z, Z, Rzyx(0, -0.04, 0.06))),
    ("1.5", (np.array([0, 0, -0.095]), np.array([.3, -.5, 0]), Z, Rzyx(np.pi / 6, 0, 0))),
    ("1.85", (np.array([0, 0, -0.095]), Z, np.array([.5, -.8, .3]), I)),
    ("2.5", (SLOPE_CG, np.array([0.6, 0.5, 0.05]), np.array([0.2, 0.3, -0.4]), SLOPE_R,
             (SLOPE_P, SLOPE_N, SLOPE_V))),
]

if __name__ == "__main__":
    np.set_printoptions(precision=12)
    for name, args in PHASES:
        F, M, legs = gear(*args)
        print(f"t={name}  F_b={F.tolist()}  M_b={M.tolist()}")
        for i, L in enumerate(legs, 1):
            print(f"   leg{i} gap={L['gap']!r} N={L['N']!r} contact={L['contact']} "
                  f"|Ft|={L['fm']!r} cap={L['cap']!r} power={L['power']!r}")
    print("slope CG", SLOPE_CG.tolist(), "n", SLOPE_N.tolist())
