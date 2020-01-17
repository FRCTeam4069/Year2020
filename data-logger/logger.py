#!/usr/bin/python3

import zmq
import matplotlib.pyplot as plt

# time will always be logged
t = []
topics = {}

ctx = zmq.Context()
sock = ctx.socket(zmq.PULL)
sock.connect("tcp://10.40.69.2:5802")

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
                plt.figure()
                plt.plot(t, topic, label=name)
                plt.xlabel("Time")
                plt.ylabel(name)
                plt.legend()

            plt.show()
            t.clear()
            topics.clear()
