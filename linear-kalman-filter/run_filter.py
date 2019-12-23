"""
Based on the work of Marko Cotra.

References: 
* https://gist.github.com/cotramarko
* https://towardsdatascience.com/wtf-is-sensor-fusion-part-2-the-good-old-kalman-filter-3642f321440

"""
import numpy as np
import matplotlib.pyplot as plt

from kalman_filter import LinearKalmanFilter
from models import simulate_system, create_model_parameters

np.random.seed(0)
(A, H, Q, R) = create_model_parameters()
simulation_time = 20

# Initial state
x0 = np.array([0, 0.1, 0, 0.1])
P0 = 0 * np.eye(4)

state, measurements = simulate_system(simulation_time, x0)
kalman_filter = LinearKalmanFilter(A, H, Q, R, x0, P0)

estimated_state = np.zeros((simulation_time, 4))
estimation_covariance = np.zeros((simulation_time, 4, 4))

for k in range(simulation_time):
    kalman_filter.predict()
    kalman_filter.update(measurements[k, :])

    estimated_state[k, :] = kalman_filter.estimated_state
    estimation_covariance[k, ...] = kalman_filter.estimation_covariance

plt.figure(figsize=(7, 5))
plt.plot(state[:, 0], state[:, 2], '-bo')
plt.plot(estimated_state[:, 0], estimated_state[:, 2], '-ko')
plt.plot(measurements[:, 0], measurements[:, 1], ':rx')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['true state', 'inferred state', 'observed measurement'])
plt.show()
