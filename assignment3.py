import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = True

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 15
        self.dt = 0.2

        self.L = 2.5 # Car base [m]

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 0, 0]
        self.reference2 = None

        self.x_obs = 7
        self.y_obs = 0.1

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t += np.cos(psi_t) * v_t * dt
        y_t += np.sin(psi_t) * v_t * dt

        a_t = pedal
        v_t += a_t * dt - v_t/25

        psi_t += v_t * dt * np.tan(steering)/self.L

        return [x_t, y_t, psi_t, v_t]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(self.horizon):
            speed = state[3]
            heading = state[2]

            state = self.plant_model(state, self.dt, u[i*2], u[i*2 + 1])

            distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)
            distance_to_obstacle = np.sqrt((self.x_obs - state[0])**2 + (self.y_obs - state[1])**2)

            # Position cost
            cost +=  distance_to_goal

            # Obstacle cost
            if distance_to_obstacle < 1.5:
                cost += 3.5/distance_to_obstacle

            # Heading cost
            cost += 10 * (heading - state[2])**2

            cost +=  2 * (ref[2] - state[2])**2

            # Acceleration cost
            if abs(u[2*i]) > 0.2:
                cost += (speed - state[3])**2

        return cost

sim_run(options, ModelPredictiveControl)
