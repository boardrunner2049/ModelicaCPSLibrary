"""Independent rigid-body reference for the MultirotorFrames harness.

Integrates the free 6-DOF motion under the constant body wrench the model
asserts (F_b = (1.6,0,0.4) N, M_b = (0.004,0.036,-0.186) N.m about the CG,
m = 1.5 kg, zero gravity), from p = (0,0,-1), q_wb = (cos45,0,0,sin45)
(scalar first, yaw 90 deg), v = omega = 0, with scipy DOP853 at rtol 1e-13.
Prints the 1 s state for the intended inertia diag(0.02,0.02,0.04) and, as a
cross-check of this script, for the inertia the FMU currently computes
(Rumoca bug A adds products -m r_j r_k with r = (0.03,-0.02,0.01)); the latter
matches the FMU and native `rumoca sim` to ~2e-9. Run: python3 ref_traj.py
"""
import numpy as np
from scipy.integrate import solve_ivp

m = 1.5
F_b = np.array([1.6, 0.0, 0.4])
M_b = np.array([0.004, 0.036, -0.186])


def R_of(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def run(J, T=1.0):
    Ji = np.linalg.inv(J)

    def f(t, s):
        p, v, q, w = s[0:3], s[3:6], s[6:10], s[10:13]
        qd = 0.5 * np.array([-q[1] * w[0] - q[2] * w[1] - q[3] * w[2],
                             q[0] * w[0] + q[2] * w[2] - q[3] * w[1],
                             q[0] * w[1] + q[3] * w[0] - q[1] * w[2],
                             q[0] * w[2] + q[1] * w[1] - q[2] * w[0]])
        return np.concatenate([v, R_of(q / np.linalg.norm(q)) @ F_b / m, qd,
                               Ji @ (M_b - np.cross(w, J @ w))])

    s0 = np.concatenate([[0, 0, -1], [0, 0, 0], [np.sqrt(0.5), 0, 0, np.sqrt(0.5)], [0, 0, 0]])
    s = solve_ivp(f, (0, T), s0, method="DOP853", rtol=1e-13, atol=1e-14).y[:, -1]
    return dict(p_w=s[0:3], v_w=s[3:6], q_wb=s[6:10], omega_b=s[10:13])


J_INTENDED = np.diag([0.02, 0.02, 0.04])
r = np.array([0.03, -0.02, 0.01])
J_FMU = J_INTENDED - m * (np.outer(r, r) - np.diag(np.diag(np.outer(r, r))))

if __name__ == "__main__":
    for name, J in [("intended diag(0.02,0.02,0.04)", J_INTENDED), ("FMU (bug A)", J_FMU)]:
        print(name)
        for k, v in run(J).items():
            print(f"  {k} = {v.tolist()}")
