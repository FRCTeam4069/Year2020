import matplotlib.pyplot as plt
import numpy as np

t, V = np.loadtxt("ShooterVoltage.csv", delimiter=",", unpack=True)
t2, v = np.loadtxt("ShooterVelocity.csv", delimiter=",", unpack=True)

plt.figure()
plt.plot(t, V, label = "Voltage")
plt.xlabel("Time")
plt.ylabel("Voltage")

plt.figure()
plt.plot(t2, v, label = "Velocity")
plt.xlabel("Time")
plt.ylabel("Velocity")


plt.legend()
plt.show()
