#!/usr/bin/python3

import zmq
import matplotlib.pyplot as plt
import csv

# time will always be logged
ts = []
topics = {}

print("Making ZMQ context")
ctx = zmq.Context()
sock = ctx.socket(zmq.PULL)
sock.connect("tcp://10.40.69.2:5802")

print("Socket connected.")
while True:
    print("Loop")
    obj = sock.recv_json()
    print("Received")

    if obj["enabled"]:
        for topic in obj["topicNames"]:
            if topic not in topics:
                topics[topic] = [obj["topics"][topic]]
            else:
                topics[topic] = topics[topic] + [obj["topics"][topic]]
        ts += [obj["time"]]
    else:
        if ts is not []:
            print("Should display graphs")
            for (name, topic) in topics.items():
                plt.figure()
                plt.plot(ts, topic, label=name)

                plt.xlabel("Time")
                plt.ylabel(name)
                plt.legend()
                with open(f"{name}.csv", "w") as f:
                    wr = csv.writer(f, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    for (t, value) in zip(ts, topic):
                        wr.writerow([t, value])

            plt.show()
            ts.clear()
            topics.clear()
