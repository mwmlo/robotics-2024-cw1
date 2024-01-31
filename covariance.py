from statistics import mean

locations = [(-3.0, 1.4), (-1.7, 2.0), (-0.6, 2.4), (0.9, 0.6), (0.4, 1.8), (1.7, -0.9), (0.9, 0.6), (0.7, 1.1), (-0.2, 2.4), (0.9, 0.6)] 
# e.g. [(x_1, y_1), (x_2, y_2), ..., (x_10, y_10)]
mean_x = mean([x for x, _ in locations])
mean_y = mean([y for _, y in locations])
cov_xx = mean([(x - mean_x) ** 2 for x, _ in locations])
cov_yy = mean([(y - mean_y) ** 2 for _, y in locations])
cov_xy = mean([(x - mean_x) * (y - mean_y) for x, y in locations])

print(cov_xx, cov_xy, cov_xy, cov_yy)