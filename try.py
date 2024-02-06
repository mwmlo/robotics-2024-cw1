class Visualize:
    def __init__(self, x_std, y_std, theta_std, n_particles):
        self.x_std = x_std
        self.y_std = y_std
        self.theta_std = theta_std
        self.particles = [Particle(0,0,0, 1/n_particles) for p in range(n_particles)]

    def draw_particles(self):
        print("drawParticles:"+str([p.draw() for p in self.particles]))
class Particle:
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight

    def draw(self):
        return (self.x, self.y, self.theta)

v = Visualize(0,0,0, 100)
v.draw_particles()