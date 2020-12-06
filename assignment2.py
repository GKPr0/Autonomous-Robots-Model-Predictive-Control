import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]
options['OBSTACLES'] = False

class ModelPredictiveControl:
    def __init__(self):
        self.horizon = 20
        self.dt = 0.2

        self.L = 2.5 # Car base [m]

        # Reference or set point the controller will achieve.
        self.reference1 = [10, 10, 0]
        self.reference2 = [10, 2, 3 * 3.14/2]

    def plant_model(self,prev_state, dt, pedal, steering):
        x_t = prev_state[0]
        y_t = prev_state[1]
        psi_t = prev_state[2]
        v_t = prev_state[3]

        x_t += np.cos(psi_t) * v_t * dt 
        y_t += np.sin(psi_t) * v_t * dt 

        psi_t += v_t * dt* (np.tan(steering)/self.L)
    
        a_t = pedal
        v_t += a_t * dt - v_t/25

        return [x_t, y_t, psi_t, v_t]

    def cost_function(self,u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        for i in range(self.horizon):
            speed = state[3]
            heading = state[2]
            state = self.plant_model(state, self.dt, u[2*i], u[2*i + 1])
            
            # Position cost
            distance_to_goal = np.sqrt((ref[0] - state[0])**2 + (ref[1] - state[1])**2)
            cost += distance_to_goal

            # Heading cost
            cost += 10 * (heading - state[2])**2

            cost +=  2 * (ref[2] - state[2])**2

            # Aceleration cost
            if abs(u[2*i]) > 0.2:
                cost += (speed - state[3])**2

        return cost

sim_run(options, ModelPredictiveControl)
