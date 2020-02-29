#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

distances = ["""13
14
16
53.5
56
56
100
108
109""",
"""32
33.5
36
88.5
81
83.5
144
146
146
222
225
229""",
"""55
56
50
128
121
122.5
199
200.5
198.5
274
277.5
272""",
"""55
53
56
114
123
121
209
204
202
272
279
282""",
"""56
54.5
57.5
128
123
129.5
204
203
204.5
280
276
281"""]

distances = [np.array([float(x) for x in row.split()]) for row in distances]
for i in range(len(distances)):
    distances[i] = distances[i].reshape((-1, 3)).mean(-1)
    if len(distances[i]) == 3:
        distances[i] = np.concatenate([distances[i], [np.nan]])
distances = np.stack(distances)

marginal_speeds = np.array([1000, 1500, 2000, 2500])
marginal_hood_angles = np.array([0.0, 0.25, 0.5, 0.75, 1.0])

for i, hood_angle in enumerate(marginal_hood_angles):
    distances_hood_angle = distances[i]
    not_nan = np.logical_not(np.isnan(distances_hood_angle)).nonzero()
    slope, intercept = np.polyfit(distances_hood_angle[not_nan], marginal_speeds[not_nan], 1)
    print(f'y={slope}x+{intercept} for hood angle {hood_angle} where x is distance (in) and y is flywheel speed (rpm)')

fig, (speeds_ax, hood_ax) = plt.subplots(1, 2)

hood_norm = ((distances - distances.mean(0, keepdims=True)) / distances.std(0, keepdims=True)).flatten()
hood_angles = np.repeat(marginal_hood_angles[:, np.newaxis], distances.shape[1], axis=1).flatten()
hood_ax.scatter(hood_angles, hood_norm, c='r')
hood_valid_indices = np.argwhere(np.logical_not(np.isnan(hood_norm)))
hood_norm = hood_norm[hood_valid_indices][:, 0]
hood_angles = hood_angles[hood_valid_indices][:, 0]
slope, intercept = np.polyfit(hood_angles, hood_norm, 1)
line_x = np.linspace(hood_angles.min(), hood_angles.max(), 500)
line_y = (line_x * slope) + intercept
hood_ax.plot(line_x, line_y, c='b', label=f'$y={slope:.4f}x{intercept:.4f}$')
hood_ax.set_xlabel('Hood angle (proportion of max)')
hood_ax.set_ylabel('Distance (normalized)')
hood_ax.set_title('Distance by hood angle')
hood_ax.legend()

speeds_norm = ((distances - distances.mean(1, keepdims=True)) / distances.std(1, keepdims=True)).T.flatten()
speeds = np.repeat(marginal_speeds[np.newaxis, :], distances.shape[0], axis=0).T.flatten()
speeds_valid_indices = np.argwhere(np.logical_not(np.isnan(speeds_norm)))
speeds_norm = speeds_norm[speeds_valid_indices][:, 0]
speeds = speeds[speeds_valid_indices][:, 0]
speeds_ax.scatter(speeds, speeds_norm, c='r')
slope, intercept = np.polyfit(speeds, speeds_norm, 1)
line_x = np.linspace(speeds.min(), speeds.max(), 500)
line_y = (line_x * slope) + intercept
speeds_ax.plot(line_x, line_y, c='b', label=f'$y={slope:.4f}x{intercept:.4f}$')
speeds_ax.set_xlabel('Flywheel speed (rpm)')
speeds_ax.set_ylabel('Distance (normalized)')
speeds_ax.set_title('Distance by flywheel speed')
speeds_ax.legend()

fig.tight_layout(rect=(0, 0.05, 1, 0.95))
plt.show()
