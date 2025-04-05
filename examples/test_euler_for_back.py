import math
import pysyslink_base
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg') 

# Set the default logger configuration and log level
pysyslink_base.SpdlogManager.configure_default_logger()
pysyslink_base.SpdlogManager.set_log_level(pysyslink_base.LogLevel.debug)

# Create a BlockEventsHandler
block_events_handler = pysyslink_base.BlockEventsHandler()

# Load the plugins using BlockTypeSupportPluginLoader
plugin_loader = pysyslink_base.BlockTypeSupportPluginLoader()
block_factories = plugin_loader.load_plugins("/usr/local/lib")

solvers = [{"default": {
        "Type": "odeint",
        "ControlledSolver": "runge_kutta_fehlberg78",
        "AbsoluteTolerance": 1e-12,
        "RelativeTolerance": 1e-12,
        "ActivateEvents": True
    }},
    {"default": {
         "Type": "EulerForward",
         "FirstTimeStep": 0.1,
         "ActivateEvents": False
     }},
     {"default": {
         "Type": "EulerBackward",
         "FirstTimeStep": 0.1,
         "ActivateEvents": False}}]

plt.figure()

for i in range(3):
    # Parse the simulation model from a YAML file
    simulation_model = pysyslink_base.ModelParser.parse_from_yaml(
                            "system_euler_for_back.yaml", block_factories, block_events_handler
                        )


    # Get the direct block chains and order them
    block_chains = simulation_model.get_direct_block_chains()
    ordered_blocks = simulation_model.order_block_chains_onto_free_order(block_chains)

    # Propagate sample times
    simulation_model.propagate_sample_times()

    # Set up simulation options
    simulation_options = pysyslink_base.SimulationOptions()
    simulation_options.start_time = 0.0
    simulation_options.stop_time = 10.0
    simulation_options.run_in_natural_time = False
    simulation_options.natural_time_speed_multiplier = 1
    simulation_options.block_ids_input_or_output_and_indexes_to_log = [
        ("integrator1", "output", 0),
        ("display1", "input", 0)
    ]
    simulation_options.solvers_configuration = solvers[i]

    # Create a SimulationManager and run the simulation
    simulation_manager = pysyslink_base.SimulationManager(simulation_model, simulation_options)
    simulation_output = simulation_manager.run_simulation()

    # Access and print continuous values for integrator2/output
    continuous_values = simulation_output.signals["LoggedSignals"]["integrator1/output/0"].try_cast_to_typed().values
    continuous_times = simulation_output.signals["LoggedSignals"]["integrator1/output/0"].try_cast_to_typed().times


    plt.plot(continuous_times, continuous_values, label="Solver: {}".format(solvers[i]["default"]["Type"]))



# Python calculation to check that it is correct

# Differential equation parameters
h = 0.1          # step size
steps = 100       # number of steps (for t from 0 to 1)
x0 = 1.0         # initial condition

# Euler Forward: x_{n+1} = x_n + h * f(x_n)
def euler_forward(x0, h, steps):
    x_values = [x0]
    for i in range(steps):
        x_current = x_values[-1]
        x_next = x_current + h * (-20 * x_current)  # f(x) = -20*x
        x_values.append(x_next)
    return x_values

# Euler Backward: x_{n+1} = x_n + h * f(x_{n+1})
# For f(x) = -20*x, the implicit equation is:
#   x_{n+1} = x_n - 20*h*x_{n+1}  ->  x_{n+1}(1 + 20*h) = x_n
#   Thus, x_{n+1} = x_n / (1 + 20*h)
def euler_backward(x0, h, steps):
    x_values = [x0]
    for i in range(steps):
        x_current = x_values[-1]
        x_next = x_current / (1 + 20 * h)
        x_values.append(x_next)
    return x_values

# Analytical solution: x(t) = exp(-20*t)
def analytical_solution(t):
    return math.exp(-20 * t)

# Compute the solutions
t_values = [i * h for i in range(steps + 1)]
x_forward = euler_forward(x0, h, steps)
x_backward = euler_backward(x0, h, steps)
x_analytical = [analytical_solution(t) for t in t_values]
plt.plot(t_values, x_analytical, 'ko-', label='Analytical')
plt.plot(t_values, x_forward, 'bs--', label='Euler Forward')
plt.plot(t_values, x_backward, 'r^--', label='Euler Backward')


plt.legend()
plt.show()
