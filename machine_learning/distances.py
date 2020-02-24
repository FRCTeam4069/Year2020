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

fig, (speeds_ax, hood_ax) = plt.subplots(1, 2)
for hood_angle, row in zip(marginal_hood_angles, (distances - distances.mean(0, keepdims=True)) / distances.std(0, keepdims=True)):
    hood_ax.scatter([hood_angle] * len(row), row)
hood_ax.set_xlabel('Hood angle')
hood_ax.set_ylabel('Distance')
hood_ax.set_title('Distance by hood angle')

for speed, row in zip(marginal_speeds, ((distances - distances.mean(1, keepdims=True)) / distances.std(1, keepdims=True)).T):
    speeds_ax.scatter([speed] * len(row), row)
speeds_ax.set_xlabel('Flywheel speed')
speeds_ax.set_ylabel('Distance')
speeds_ax.set_title('Distance by flywheel speed')

plt.show()
