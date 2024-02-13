import random

def normalise(particles):
    total_weight = sum([p.weight for p in particles])
    for p in particles:
        p.weight /= total_weight

def resample(particles):
    cumulative_weights = []
    curr_total = 0
    for p in particles:
        curr_total += p.weight
        cumulative_weights.append(curr_total)
    resampled_particles = []
    for _ in range(len(particles)):
        sample_num = random.uniform(0, 1)
        # See where the random number intersects with the array values
        for i, cw in enumerate(cumulative_weights):
            if sample_num >= cw:
                resampled_particles.append(particles[i])
                break
    # TODO: overwrite the main arrays when resampling is done
