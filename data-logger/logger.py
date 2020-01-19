#!/usr/bin/python3

import zmq
import matplotlib.pyplot as plt

# time will always be logged
t = []
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
        t += [obj["time"]]
    else:
        if t is not []:
            print("Should display graphs")
            for (name, topic) in topics.items():
                ref = []
                if "Velocity" in name:
                    ref = [150 for _x in topic]
                plt.figure()
                plt.plot(t, topic, label=name)
                if ref:
                    plt.plot(t, ref, label="Reference")

                plt.xlabel("Time")
                plt.ylabel(name)
                plt.legend()

            plt.show()
            t.clear()
            topics.clear()
