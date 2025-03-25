import pysyslink_base
import time
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


# Parse the simulation model from a YAML file
simulation_model = pysyslink_base.ModelParser.parse_from_yaml(
                        "system1_continuous.yaml", block_factories, block_events_handler
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
    ("const1", "output", 0),
    ("integrator2", "output", 0),
    ("display1", "input", 0)
]
simulation_options.solvers_configuration = {
    "default": {
        "Type": "odeint",
        "ControlledSolver": "runge_kutta_fehlberg78",
        "AbsoluteTolerance": 1e-12,
        "RelativeTolerance": 1e-12 
    }
}

# Create a SimulationManager and run the simulation
simulation_manager = pysyslink_base.SimulationManager(simulation_model, simulation_options)

initial_time = time.time()
next_time_hit = 0.0
value_updated = False
try:
    while True:
        if time.time() - initial_time >= next_time_hit:
            next_time_hit = simulation_manager.run_simulation_step()
        if time.time() - initial_time >= 10.0 and not value_updated:
            value_updated = True
            pysyslink_base.ISimulationBlock.find_block_by_id("const1", simulation_model.simulation_blocks).try_update_configuration_value("Value", 9.0)
            # pysyslink_base.ISimulationBlock.find_block_by_id("sum1", simulation_model.simulation_blocks).try_update_configuration_value("Gains", [1.0, -2.0, -2.0])
except KeyboardInterrupt:
    pass

simulation_output = simulation_manager.get_simulation_output()

# Access and print continuous values for integrator2/output
continuous_values = simulation_output.signals["LoggedSignals"]["integrator2/output/0"].try_cast_to_typed().values
continuous_times = simulation_output.signals["LoggedSignals"]["integrator2/output/0"].try_cast_to_typed().times

for time, value in zip(continuous_times, continuous_values):
    print(f"{time}: {value}")

# Access and print continuous values for display1/input
continuous_values_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].try_cast_to_typed().values
continuous_times_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].try_cast_to_typed().times

for time, value in zip(continuous_times_log, continuous_values_log):
    print(f"{time}: {value}")

plt.figure()
plt.plot(continuous_times, continuous_values)
plt.show()
