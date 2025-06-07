
import subprocess
import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt


proc = subprocess.Popen(['bash', 'build.sh', 'clean', 'build'])
proc.wait()

target = 10000
def run_sim(kp, ki, kd):
    proc = subprocess.Popen(['bash', 'build.sh', 'test'],
                        stdin=subprocess.PIPE,
                        text=True)

    proc.stdin.write(f'{kp} {ki} {kd}\n')
    proc.stdin.flush()

    proc.stdin.close()
    proc.wait()

    A, V, T = [], [], []
    with open('stats', 'r') as file:
        lines = file.readlines()
        for line in lines:
            a, v, t = map(float, line.strip().split(' '))

            A.append(a)
            V.append(v)
            T.append(t)

    return (A, V, T)

def compute_metrics(kpid):
    kp, ki, kd = kpid
    print(f'Running kp={kp:.4f} ki={ki:.4f} kd={kd:.4f}...')
    A, V, T = map(np.array, run_sim(kp, ki, kd))

    error = np.mean(np.abs(A - target))
    rmse = np.sqrt(np.mean((A - target)**2))
    overshoot = max(A) - target

    within_band = np.abs(A - target) < 0.005 * target
    settling_time = len(A)
    for i in range(len(within_band)):
        if all(within_band[i:]):
            settling_time = i
            break
    
    jerk = np.diff(T)
    jerk_metric = np.mean(np.abs(jerk))

    thrust_effort = np.mean(T)
    thrust_variation = np.std(T)

    score = 2 * error + \
            2 * rmse + \
            1 * settling_time + \
            1.5 * jerk_metric + \
            2 * thrust_effort + \
            3 * thrust_variation + \
            0.05 * overshoot

    return score


initial_guess = [ 1.2282, 0.2210, 3.1814 ]
bounds = [(0.0, 5.0), (0.0, 5.0), (0.0, 5.0)]

result = minimize(compute_metrics, initial_guess, bounds=bounds, method='L-BFGS-B')
best_kp, best_ki, best_kd = result.x
print(f'\n=== Best PID ===\nkp={best_kp:.4f}, ki={best_ki:.4f}, kd={best_kd:.4f}')

A, V, T = run_sim(best_kp, best_ki, best_kd)
TS = np.array(range(len(A))) / 1000

fig,axes = plt.subplots(3, 1, figsize=(10, 8))
plt.title(f'kp={best_kp:.4f} ki={best_ki:.4f} kd={best_kd:.4f}')

axes[0].plot(TS, A)
axes[0].set_title('Altitude over Time')
axes[0].set_ylabel('Altitude (m)')

axes[1].plot(TS, V)
axes[1].set_title('Velocity over Time')
axes[1].set_ylabel('Velocity (m/s)')

axes[2].plot(TS, T)
axes[2].set_title('Thrust over Time')
axes[2].set_ylabel('Thrust (m/ss)')

plt.tight_layout()
plt.show()
