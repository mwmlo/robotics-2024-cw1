from statistics import mean

locations = [] # e.g. [(x_1, y_1), (x_2, y_2), ..., (x_10, y_10)]
mean_x = mean([x for x, _ in locations])
mean_y = mean([y for _, y in locations])
cov_xx = mean([(x - mean_x) ** 2 for x, _ in locations])
cov_yy = mean([(y - mean_y) ** 2 for _, y in locations])
cov_xy = mean([(x - mean_x) * (y - mean_y) for x, y in locations])

print(cov_xx, cov_xy, cov_xy, cov_yy)