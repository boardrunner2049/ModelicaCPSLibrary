"""Independent rigid-body reference for the NedFrdContract harness.

Integrates oldBody (legacy ENU world / FLU body) from the model's own
parameters: m = 2 kg, J = [2,0.3,-0.2;0.3,3,0.4;-0.2,0.4,4], constant body
force (1,2,4) N and moment (0.1,-0.2,0.3) N.m, gravity (0,0,-9.80665) ENU,
p(0) = (1,2,3), body velocity (0.2,-0.1,0.3), q(0) from ZYX Euler
(roll 0.3, pitch -0.2, yaw 0.7), body rate (0.1,0.2,-0.3); scipy DOP853 at
rtol 1e-13. The NED/FRD body must equal W p, W v and B omega of this
trajectory. Run: python3 ref_traj.py
"""
import numpy as np
from scipy.integrate import solve_ivp

m = 2.0
J = np.array([[2, 0.3, -0.2], [0.3, 3, 0.4], [-0.2, 0.4, 4.0]])
F_b = np.array([1.0, 2.0, 4.0])
M_b = np.array([0.1, -0.2, 0.3])
g_w = np.array([0, 0, -9.80665])
W = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]])
B = np.diag([1.0, -1.0, -1.0])


def q_from_euler(roll, pitch, yaw):
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def R_of(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def f(t, s):
    p, v, q, w = s[0:3], s[3:6], s[6:10], s[10:13]
    qd = 0.5 * np.array([-q[1] * w[0] - q[2] * w[1] - q[3] * w[2],
                         q[0] * w[0] + q[2] * w[2] - q[3] * w[1],
                         q[0] * w[1] + q[3] * w[0] - q[1] * w[2],
                         q[0] * w[2] + q[1] * w[1] - q[2] * w[0]])
    return np.concatenate([v, R_of(q / np.linalg.norm(q)) @ F_b / m + g_w, qd,
                           np.linalg.solve(J, M_b - np.cross(w, J @ w))])


Q0 = q_from_euler(0.3, -0.2, 0.7)


def run(T=2.0):
    s0 = np.concatenate([[1, 2, 3], R_of(Q0) @ np.array([0.2, -0.1, 0.3]), Q0, [0.1, 0.2, -0.3]])
    s = solve_ivp(f, (0, T), s0, method="DOP853", rtol=1e-13, atol=1e-14).y[:, -1]
    return dict(p_w=s[0:3], v_w=s[3:6], q_wb=s[6:10], omega_b=s[10:13])


if __name__ == "__main__":
    print("q0", Q0.tolist(), "v_w0", (R_of(Q0) @ np.array([0.2, -0.1, 0.3])).tolist())
    r = run()
    for k, v in r.items():
        print(f"oldBody {k} = {v.tolist()}")
    print("body p_w = W p =", (W @ r["p_w"]).tolist())
    print("body v_w = W v =", (W @ r["v_w"]).tolist())
    print("body omega_b = B w =", (B @ r["omega_b"]).tolist())
