#!/usr/bin/env python3

import sys

if "--noninteractive" in sys.argv:
    import matplotlib as mpl

    mpl.use("svg")

import frccontrol as fct
import matplotlib.pyplot as plt
import numpy as np


class Flywheel(fct.System):
    def __init__(self, dt):
        """Flywheel subsystem.
        Keyword arguments:
        dt -- time between model/controller updates
        """
        state_labels = [("Angular velocity", "rad/s"), ("Input Error", "V")]
        u_labels = [("Voltage", "V")]
        self.set_plot_labels(state_labels, u_labels)

        fct.System.__init__(
            self,
            np.array([[-12.0]]),
            np.array([[12.0]]),
            dt,
            np.zeros((1, 1)),
            np.zeros((1, 1)),
        )

    def create_model(self, states, inputs):
        # Number of motors
        num_motors = 1.0
        # Flywheel moment of inertia in kg-m^2
        J = 0.00289
        # Gear ratio
        G = 1.0

        MOTOR_FALCON500 = fct.models.DcBrushedMotor(12.0, 4.69, 257.0, 1.5, 6380.0)
        MOTOR_FALCON500.Kv -= 15.0
        new_free_speed = MOTOR_FALCON500.Kv * (12 - MOTOR_FALCON500.free_current * MOTOR_FALCON500.R)
        new_free_speed = new_free_speed / (2*np.pi) * 60
        print(new_free_speed)
        MOTOR_FALCON500 = fct.models.DcBrushedMotor(12.0, 4.69, 257.0, 1.5, new_free_speed)

        return fct.models.flywheel(MOTOR_FALCON500, num_motors, J, G)

    def design_controller_observer(self):
        q = [50.0]
        r = [12.0]
        self.design_lqr(q, r)
        # self.place_controller_poles([0.87])
        self.design_two_state_feedforward()

        q_vel = 0.05
        r_vel = 7.5
        self.design_kalman_filter([q_vel], [r_vel])
        # self.place_observer_poles([0.3])


def main():
    dt = 0.01
    flywheel = Flywheel(dt)
    flywheel.export_java_coeffs("Flywheel")

    # Set up graphing
    l0 = 0.1
    l1 = l0 + 10.0
    l2 = l1 + 0.1
    t = np.arange(0, l2 + 5.0, dt)

    refs = []

    # Generate references for simulation
    for i in range(len(t)):
        if t[i] < l0:
            r = np.array([[0]])
        elif t[i] < l1:
            r = np.array([[1000.0]])
        else:
            r = np.array([[0]])
        refs.append(r)

    x_rec, ref_rec, u_rec, y_rec = flywheel.generate_time_responses(t, refs)
    flywheel.plot_time_responses(t, x_rec, ref_rec, u_rec)
    if "--noninteractive" in sys.argv:
        plt.savefig("flywheel_response.svg")
    else:
        plt.show()


if __name__ == "__main__":
    main()
