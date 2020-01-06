"""
Based on the work of Marko Cotra.

References: 
* https://gist.github.com/cotramarko
* https://towardsdatascience.com/wtf-is-sensor-fusion-part-2-the-good-old-kalman-filter-3642f321440

"""
import numpy as np
import matplotlib.pyplot as plt

from particle_filter import ParticleFilter
from models import simulate_system, create_model_parameters

np.random.seed(0)
(A, H, Q, R) = create_model_parameters()
simulation_time = 20
n_particles = 1000

# Initial state
x0 = np.array([0, 0.1, 0, 0.1])

state, measurements = simulate_system(simulation_time, x0)
particle_filter = ParticleFilter(A, H, Q, R, x0, n_particles)

estimated_state = np.zeros((simulation_time, 4))

for k in range(simulation_time):
    particle_filter.predict()
    particle_filter.update_particle_weights(measurements[k, :])

    estimated_state[k, :] = particle_filter.calculate_estimate()

    particle_filter.resample()

# Plot X-Y
plt.figure()
plt.plot(state[:, 0], state[:, 2], '-bo')
plt.plot(estimated_state[:, 0], estimated_state[:, 2], '-ko')
plt.plot(measurements[:, 0], measurements[:, 1], ':rx')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.legend(['true state', 'inferred state', 'observed measurement'])

# Plot particles
particles_x = [particle[0] for particle in particle_filter.particles]
particles_y = [particle[2] for particle in particle_filter.particles]

plt.figure()
plt.scatter(particles_x, particles_y,
            c=particle_filter.particles_weights, s=30)
plt.colorbar()

plt.show()
