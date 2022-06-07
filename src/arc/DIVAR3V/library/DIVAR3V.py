import numpy as np
from pydrake.all import (
    RotationMatrix
)
from problems.DIVAR3V.library.sim_funcs import *

# DIVA-R1.1.3-V

class DIVAR3V:
    def __init__(self, dv_samples):
        self.var = dv_samples
        self.problem_name = 'DIVA-R1.1.3-V'
        self._compute_commons()
        self.problem_description = 'Top-down development of a humanoid robot arm for manufacturing applications'
        # a1, lua, a23_distr, lfa, a45_distr, rho, s2, s4, K, taumax
        # self.plotter = np.array([[2,0],[4,5],[1,6],[1,9],[3,9],[1,8],[4,8], [0,7]])
        self.plotter = np.array([[2,0],[1,9],[3,9],[1,8],[4,8]])
        # self.plotter = np.array([[2,0]])

    def _compute_commons(self):
        self.results = Parallel(n_jobs=-1, verbose=1, backend="threading")(map(delayed(compute_bottom_up_mappings), self.var.values.tolist()))
        self.results_array = np.array(self.results)
        pass

    def reachability(self):
        self.var['reachability'] = self.results_array[:, 0]
        pass

    def applied_force(self):
        self.var['applied_force'] = self.results_array[:, 1]

    def payload(self):
        self.var['payload'] = self.results_array[:, 2]
        pass

    def tcycle(self):
        self.var['tcycle'] = self.results_array[:, 3]
        # 8/(2*np.sqrt(self.var['K']))
        pass

    def motion_feasibility(self):
        self.var['motion_feasibility'] = self.results_array[:, 4]
        pass

    # There are still quantities of interest
    # left: Payload and max torque during simulation
    # Instead of max payload some other dynamic condition
    # like lifting in less than gives seconds can be done
