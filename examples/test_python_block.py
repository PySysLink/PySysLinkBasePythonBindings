import pysyslink_base


# Set the default logger configuration and log level
pysyslink_base.SpdlogManager.configure_default_logger()
pysyslink_base.SpdlogManager.set_log_level(pysyslink_base.LogLevel.debug)

# Create a BlockEventsHandler
block_events_handler = pysyslink_base.BlockEventsHandler()

# Load the plugins using BlockTypeSupportPluginLoader
plugin_loader = pysyslink_base.BlockTypeSupportPluginLoader()

plugin_configuration = {"BasicCppSupport/libraryPluginPath": "/usr/local/lib/pysyslink_plugins"}

block_factories = plugin_loader.load_plugins("/usr/local/lib/pysyslink_plugins/block_type_supports/", plugin_configuration)


# Parse the simulation model from a YAML file
simulation_model = pysyslink_base.ModelParser.parse_from_yaml(
                        "system_python_block.yaml", block_factories, block_events_handler
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
    ("integrator1", "output", 0)
]
simulation_options.solvers_configuration = {
    "default": {
        # "Type": "EulerBackward",
        # "FirstTimeStep": 0.1,
        # "ActivateEvents": False
        "Type": "odeint",
        "ControlledSolver": "rosenbrock4_controller",
        "AbsoluteTolerance": 1e-8,
        "RelativeTolerance": 1e-8 
    }
}

# Create a SimulationManager and run the simulation
simulation_manager = pysyslink_base.SimulationManager(simulation_model, simulation_options)
simulation_output = simulation_manager.run_simulation()

def print_display(block_id):
    """Print the values of a Display block."""
    disp = simulation_output.signals["Displays"][block_id].try_cast_to_typed()
    times = disp.times
    values = disp.values

    print(f"\n=== Display {block_id} ===")
    for t, v in zip(times, values):
        print(f"{t:.4f}   {v}")
    print(f"Total samples: {len(values)}")


def print_logged(signal_name):
    """Print values of a logged internal signal (e.g. integrator1/output/0)."""
    log = simulation_output.signals["LoggedSignals"][signal_name].try_cast_to_typed()
    times = log.times
    values = log.values

    print(f"\n=== Logged signal {signal_name} ===")
    for t, v in zip(times, values):
        print(f"{t:.4f}   {v}")
    print(f"Total samples: {len(values)}")


# --------------------------------------
# 1. Main display (output of integrator2)
# --------------------------------------
print_display("display1")

# --------------------------------------
# 2. Logged output of integrator1
# --------------------------------------
print_logged("integrator1/output/0")

# --------------------------------------
# 3. Python block outputs displayed
# --------------------------------------
print_display("display2")   # Python output #1
print_display("display3")   # Python output #2
