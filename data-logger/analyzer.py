from sys import argv
import csv
import matplotlib.pyplot as plt

#files = argv[1:]

files = ["KF Velocity.csv", "Velocity.csv"]

kf_file = open(files[0], "r")
v_file = open(files[1], "r")

plt.figure()
kf_ts, kf_xs = zip(*[(float(l[0]), float(l[1])) for l in csv.reader(kf_file)])
v_ts, v_xs = zip(*[(float(l[0]), float(l[1])) for l in csv.reader(v_file)])

plt.plot(kf_ts, v_xs, label="Measured Encoder Velocity")
plt.plot(kf_ts, kf_xs, label="Kalman Filtered Velocity")
plt.xlabel("Time")
plt.ylabel("Velocity")
plt.legend()

#for f in files:
#    with open(f, "r", newline="") as csv_file:
#        ts, xs = zip(*[(float(l[0]), float(l[1])) for l in csv.reader(csv_file)])
#        plt.figure()
#        plt.plot(ts, xs, label=f)
#        plt.xlabel("Time")
#        plt.ylabel(f)
#        plt.legend()

plt.show()
