#!/usr/bin/env python3
"""Compare C++ solver/decode/observations to the supplied upstream Python source.
No ROS/flight commands. Requires NumPy and SciPy; does not import Torch.
"""
import argparse
import ast
import subprocess
import tempfile
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation

p = argparse.ArgumentParser()
p.add_argument('--probe', required=True)
p.add_argument('--yopo-root', required=True, type=Path)
a = p.parse_args()
rng = np.random.default_rng(1928)
ns = {'np': np}
tree = ast.parse((a.yopo_root / 'policy/poly_solver.py').read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MincoTraj')
exec(compile(ast.Module(body=[cls], type_ignores=[]), 'upstream_solver', 'exec'), ns)
Minco = ns['MincoTraj']
tree = ast.parse((a.yopo_root / 'policy/state_transform.py').read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'StateTransform')
methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
           ('pred_to_traj_params_cpu', 'prepare_input_cpu', 'normalize_obs_cpu')]
exec(compile(ast.Module(body=methods, type_ignores=[]), 'upstream_decode', 'exec'), ns)

with tempfile.TemporaryDirectory(prefix='minco_numeric_') as temp:
    fixture = Path(temp) / 'input.txt'
    def call(mode, data):
        np.savetxt(fixture, np.asarray(data).reshape(-1), fmt='%.17g')
        return np.fromstring(subprocess.check_output([a.probe, mode, str(fixture)], text=True), sep=' ')

    max_coef = max_sample = max_decode = max_obs = 0.
    for index in range(24):
        durations = rng.uniform(.2, 3., 2) if index < 16 else rng.uniform(8., 35., 2)
        head, tail, inner = rng.normal(size=(3, 3)), rng.normal(size=(3, 3)), rng.normal(size=3)
        times = np.r_[0., durations[0]-1e-7, durations[0], durations[0]+1e-7,
                      np.linspace(0, durations.sum(), 19)]
        result = call('solve', np.r_[head.ravel(), tail.ravel(), inner, durations, times])
        ref = Minco().solve(head, tail, inner, durations)
        coeff = result[:36].reshape(12, 3)
        np.testing.assert_allclose(coeff, ref.coeffs[0], rtol=2e-7, atol=2e-7)
        max_coef = max(max_coef, np.max(np.abs(coeff-ref.coeffs[0])))
        samples = result[36:].reshape(-1, 5, 3)
        for derivative in range(5):
            if derivative < 3:
                expected = ref._eval(times, derivative)
            else:
                # Upstream exposes PVA only; derive jerk/snap from its coefficients.
                expected=[]
                for t in times:
                    piece=int(t>=durations[0]);local=t-piece*durations[0]
                    factors=np.zeros(6)
                    for k in range(derivative,6):
                        factors[k]=np.prod(np.arange(k-derivative+1,k+1))*local**(k-derivative)
                    expected.append(factors@ref.coeffs[0,piece*6:(piece+1)*6])
                expected=np.asarray(expected)
            np.testing.assert_allclose(samples[:, derivative], expected, rtol=2e-6, atol=2e-6)
            max_sample = max(max_sample, np.max(np.abs(samples[:, derivative]-expected)))
        # Endpoint constraints and C4 at the join, including jerk/snap extension.
        np.testing.assert_allclose(samples[0, :3], head, rtol=1e-6, atol=1e-6)
        np.testing.assert_allclose(samples[-1, :3], tail, rtol=1e-6, atol=1e-6)
        np.testing.assert_allclose(samples[2, 0], inner, rtol=1e-6, atol=1e-6)

    for speed in (.3, .5, 1.):
        angles = np.array([[(j-2)*np.deg2rad(90)/5, (i-1)*np.deg2rad(60)/3] for i in range(3) for j in range(5)], dtype=np.float32)
        rotations = Rotation.from_euler('ZYX', np.c_[angles[:, 0], -angles[:, 1], np.zeros(15)]).as_matrix().astype(np.float32)[::-1]
        lp = SimpleNamespace(yaw_diff=np.deg2rad(15), pitch_diff=np.deg2rad(15), vel_max=speed,
                             acc_max=(speed/6)**2*6, vertical_num=3, horizon_num=5)
        obj = SimpleNamespace(lattice_primitive=lp, _yaw_lattice_np=angles[:, 0], _pitch_lattice_np=angles[:, 1],
                              _radio_max_lattice_np=np.full(15, 10.), _pd_init_lattice_np=np.full((15, 2), 10/speed/2),
                              tail_yaw_half=np.deg2rad(45), tail_pitch_half=np.deg2rad(30), tail_radio_max=10., duration_min=.1,
                              _Rbp_image_grid_np=rotations, goal_length=10.)
        for case in range(8):
            pred = rng.uniform(-1, 1, (14, 15)).astype(np.float32)
            if case == 0: pred[12:] = -1  # exercise duration floor
            actual = call('decode', np.r_[speed, pred.ravel()]).reshape(15, 14)
            inner, tail, duration = ns['pred_to_traj_params_cpu'](obj, pred.T, np.arange(14, -1, -1))
            expected = np.c_[inner, tail.reshape(15, 9), duration]
            np.testing.assert_allclose(actual, expected, rtol=2e-6, atol=3e-6)
            max_decode = max(max_decode, np.max(np.abs(actual-expected)))
            head = rng.normal(size=(3, 3))
            rot = Rotation.random(random_state=rng).as_matrix()
            goal = rng.normal(size=3) * (20 if case % 2 else 1)
            actual = call('obs', np.r_[speed, head.ravel(), rot.ravel(), goal])
            g = goal-head[0]
            if np.linalg.norm(g)>10:
                gz=np.clip(g[2],-10,10)
                g=np.r_[g[:2]/(np.linalg.norm(g[:2])+1e-9)*np.sqrt(100-gz*gz),gz]
            raw=np.r_[rot.T@head[1],rot.T@head[2],rot.T@g][None]
            expected=ns['prepare_input_cpu'](obj,ns['normalize_obs_cpu'](obj,raw)).ravel()
            np.testing.assert_allclose(actual,expected,rtol=3e-6,atol=2e-5)
            max_obs=max(max_obs,np.max(np.abs(actual-expected)))
    subprocess.run([a.probe,'guards',str(fixture)],check=True)
print(dict(solver_cases=24,decode_cases=24,observation_cases=24,max_coefficient_abs_error=max_coef,
           max_sample_abs_error=max_sample,max_decode_abs_error=max_decode,max_observation_abs_error=max_obs))
