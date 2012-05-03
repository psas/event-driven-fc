import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

summary = np.loadtxt('summary_log.txt')
particles = np.loadtxt('particle_log.txt')

timesteps = summary[:,0]
time_min = timesteps[0]
time_max = timesteps[-1]

def show_kde(col, stddev=1):
	ymin = particles[:,col].min()
	ymax = particles[:,col].max()
	ypoints = np.reshape(np.linspace(ymin, ymax, 500), (1, -1))
	extent = [time_min, time_max, ymin, ymax]

	values = []
	rows = particles.shape[0]
	row = 0;
	while row < rows:
		x = particles[row, 0]
		next_row = row + 1
		while next_row < rows and particles[next_row, 0] == x:
			next_row = next_row + 1
		weights = np.reshape(particles[row:next_row, 1], (-1, 1))
		normalized_weights = weights / np.sum(weights)
		means = np.reshape(particles[row:next_row, col], (-1, 1))
		kernels = stats.norm.pdf(ypoints, loc=means, scale=stddev)
		values.append(np.sum(normalized_weights * kernels, axis=0))
		row = next_row

	imgplot = plt.imshow(np.rot90(np.vstack(values)), extent=extent, aspect='auto')
	imgplot.set_cmap('hot')

plt.figure(1)
#plt.subplot(221)
plt.title('Receiver velocity (m/s)')
show_kde(4, 0.25)
plt.plot(timesteps, summary[:,6], label='True')
plt.plot(timesteps, summary[:,9], label='Particle mean')
plt.legend()

plt.figure(2)
#plt.subplot(222)
plt.title('Receiver position (m)')
show_kde(3, 0.25)
plt.plot(timesteps, summary[:,5], label='True')
plt.plot(timesteps, summary[:,8], label='Particle mean')
plt.legend()

plt.figure(3)
#plt.subplot(223)
plt.title('Doppler measurement')
show_kde(2, 1.5e-9)
plt.plot(timesteps, summary[:,3], label='True')
plt.plot(timesteps, summary[:,4], label='Measurement')
plt.plot(timesteps, summary[:,7], label='Particle mean')
plt.legend()

plt.figure(4)
#plt.subplot(224)
plt.title('Effective particles')
plt.plot(timesteps, summary[:,2], '.')
plt.axhline(y=100)

plt.show()
