import random
from Particle import Particle

def normalize(visualizer):
    """Normalize particle weights in visualizer."""
    total_weight = sum([p.weight for p in visualizer.particles])
    for p in visualizer.particles:
        p.weight /= total_weight

def resample(visualizer):
    """Resamples and updates particle set."""
    cumulative_weights = [0]
    curr_total = 0
    for p in visualizer.particles:
        curr_total += p.weight
        cumulative_weights.append(curr_total)
    # Resample particles based on cumulative weights
    resampled_particles = []
    new_w = 1/len(visualizer.particles)
    for _ in range(len(visualizer.particles)):
        sample_num = random.uniform(0, 1)
        # See where the random number intersects with the array values
        for i in range(len(cumulative_weights) - 1):
            if cumulative_weights[i] <= sample_num <= cumulative_weights[i + 1]:
                p = visualizer.particles[i - 1]
                new_p = Particle(p.x, p.y, p.theta, new_w)
                resampled_particles.append(new_p)
                break
    # Overwrite the main arrays when resampling is done
    visualizer.particles = resampled_particles
