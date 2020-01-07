import numpy as np

import math


class ParticleFilter():
    def __init__(self, A, H, Q, R, x0, n_particles):
        # Model parameters
        self.A = A
        self.H = H
        self.Q = Q
        self.R = R

        # Particles
        self.n_particles = n_particles
        self.particles = self._create_particles(x0, self.n_particles)
        self.particles_weights = (1.0 / self.n_particles) * \
            np.ones(self.n_particles)

    def predict(self):
        for ind in range(self.n_particles):
            self.particles[ind] = self.A @ self.particles[ind]

        zero_mean = np.zeros(self.Q.shape[0])
        self.particles += np.random.multivariate_normal(
            mean=zero_mean, cov=self.Q, size=self.n_particles)

    def update_particle_weights(self, measurement):
        for ind in range(self.n_particles):
            likelihood = self._calculate_combined_likelihood(
                self.particles[ind], measurement)
            self.particles_weights[ind] = likelihood
            self.particles_weights[ind] += 1e-300  # Avoid round-off to 0

        self.particles_weights /= sum(self.particles_weights)

    def resample(self):
        cdf = np.cumsum(self.particles_weights)

        # Resample using either multinomial or systematic
        resampled_particles = self._resample_systematic(cdf)

        self.particles = resampled_particles
        self.particles_weights = (1.0 / self.n_particles) * \
            np.ones(self.n_particles)

    def calculate_estimate(self):
        return np.average(
            self.particles, weights=self.particles_weights, axis=0)

    def _create_particles(self, x0, n_particles):
        return np.random.multivariate_normal(
            mean=x0, cov=self.Q, size=n_particles)

    def _calculate_combined_likelihood(self, particle, measurement):
        mu = self.H @ particle

        likelihood_x = self._calculate_likelihood(
            x=measurement[0], mu=mu[0], sigma=math.sqrt(self.R[0][0]))
        likelihood_y = self._calculate_likelihood(
            x=measurement[1], mu=mu[1], sigma=math.sqrt(self.R[1][1]))

        return likelihood_x * likelihood_y

    def _calculate_likelihood(self, x, mu, sigma):
        return (1.0 / (sigma * math.sqrt(2 * math.pi))) * \
            math.exp(-(x - mu) ** 2 / (2 * sigma ** 2))

    def _resample_multinomial(self, cdf):
        resampled_particles = np.copy(self.particles)
        for ind in range(self.n_particles):
            sampled_index = np.argmax(cdf >= np.random.rand())
            resampled_particles[ind] = self.particles[sampled_index]

        return resampled_particles

    def _resample_systematic(self, cdf):
        resampled_particles = np.copy(self.particles)
        r0 = (1.0 / self.n_particles) * np.random.rand()
        for ind in range(self.n_particles):
            sampled_index = np.argmax(cdf >= r0 + ind/self.n_particles)
            resampled_particles[ind] = self.particles[sampled_index]

        return resampled_particles
