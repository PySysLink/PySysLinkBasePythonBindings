import pysyslink_base

class euler_forward(pysyslink_base.IOdeStepSolver):
    def __init__(self):
        pass

    def solve_step(self, system, states_0, currentTime, timeStep):
        return states_0


ef = pysyslink_base.EulerForwardStepSolver()

efpy = euler_forward()


print(isinstance(ef, pysyslink_base.IOdeStepSolver))
print(isinstance(efpy, pysyslink_base.IOdeStepSolver))
