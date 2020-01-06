import numpy as np
import scipy.stats

import math


class ParticleFilter():
    def __init__(self, A, H, Q, R, x0, n_particles):
        # Model parameters
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R

        # Particles
        self.particles = [self._create_particle(
            x0) for _ in range(n_particles)]
        self.particles_weights = [
            1.0 / n_particles for _ in range(n_particles)]

    def predict(self):
        for ind in range(len(self.particles)):
            self.particles[ind] = self._propagate_particle(self.particles[ind])

    def update_particle_weights(self, measurement):
        for ind in range(len(self.particles)):
            # matrix_likelihood = scipy.stats.norm(
            #    self.H @ self.particles[ind], self.R).pdf(measurement)
            # matrix_likelihood = np.nan_to_num(matrix_likelihood)

            # print('Matrix likelihood:', matrix_likelihood)
            x = self.H @ self.particles[ind]
            l1 = scipy.stats.norm(
                x[0],
                math.sqrt(self.R[0][0])).pdf(
                measurement[0])
            l2 = scipy.stats.norm(
                x[1],
                math.sqrt(self.R[1][1])).pdf(
                measurement[1])
            # print('Separate likelihood:', l1, l2)

            #assert abs(matrix_likelihood[0][0] - l1) < 1e-10
            #assert abs(matrix_likelihood[1][1] - l2) < 1e-10

            self.particles_weights[ind] = math.sqrt(l1 ** 2 + l2 ** 2)
            self.particles_weights[ind] += 1e-300  # Avoid round-off to 0

        sum_weights = sum(self.particles_weights)
        self.particles_weights = [1.0 * weight / sum_weights
                                  for weight in self.particles_weights]

    def resample(self):
        # Calculate cumulative distribution function (cdf)
        cdf = [0] * len(self.particles_weights)
        for ind in range(len(cdf)):
            cdf[ind] = sum(self.particles_weights[:ind+1])

        # print(cdf)

        resampled_particles = [None] * len(self.particles)
        # Multinomial resample
        for ind in range(len(self.particles)):
            rand_num = np.random.rand()
            filtered_indexes = [
                index for index, value in enumerate(cdf)
                if value >= rand_num]
            sampled_index = min(filtered_indexes)
            resampled_particles[ind] = self.particles[sampled_index]

        self.particles = resampled_particles
        self.particles_weights = [
            1.0 / len(self.particles_weights)
            for _ in range(len(self.particles_weights))]

    def calculate_estimate(self):
        max_weight_ind = self.particles_weights.index(
            max(self.particles_weights))
        return self.particles[max_weight_ind]

    def calculate_mean(self):
        mean_particle = np.array([0.0, 0.0, 0.0, 0.0])

        for particle in self.particles:
            mean_particle += particle

        return mean_particle / len(self.particles)

    def _create_particle(self, x0):
        #x = np.random.uniform(-1, 1)
        x = np.random.normal(x0[0], math.sqrt(self.Q[0][0]))
        vx = x0[1]
        #y = np.random.uniform(-1, 1)
        y = np.random.normal(x0[2], math.sqrt(self.Q[2][2]))
        vy = x0[3]

        return np.array([x, vx, y, vy])

    def _propagate_particle(self, particle):
        zero_mean = np.zeros(self.Q.shape[0])

        particle = self.A @ particle + \
            np.random.multivariate_normal(mean=zero_mean, cov=self.Q)

        return particle
