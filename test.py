import pysyslink_base

# Set the default logger configuration and log level
pysyslink_base.SpdlogManager.configure_default_logger()
pysyslink_base.SpdlogManager.set_log_level(pysyslink_base.LogLevel.debug)

# Create a BlockEventsHandler
block_events_handler = pysyslink_base.BlockEventsHandler()

# Load the plugins using BlockTypeSupportPlugingLoader
pluging_loader = pysyslink_base.BlockTypeSupportPlugingLoader()
block_factories = pluging_loader.load_plugins("/usr/local/lib")

# Parse the simulation model from a YAML file
simulation_model = pysyslink_base.SimulationModel(
    pysyslink_base.ModelParser.ParseFromYaml(
        "/home/pello/PySysLink/Tests/system1_multirate.yaml", block_factories, block_events_handler
    )
)

# Get the direct block chains and order them
block_chains = simulation_model.GetDirectBlockChains()
ordered_blocks = simulation_model.OrderBlockChainsOntoFreeOrder(block_chains)

# Propagate sample times
simulation_model.PropagateSampleTimes()

# Set up simulation options
simulation_options = pysyslink_base.SimulationOptions()
simulation_options.startTime = 0.0
simulation_options.stopTime = 10.0
simulation_options.runInNaturalTime = True
simulation_options.naturalTimeSpeedMultiplier = 1
simulation_options.blockIdsInputOrOutputAndIndexesToLog = [
    {"const1", "output", 0},
    {"integrator2", "output", 0},
    {"display1", "input", 0}
]
simulation_options.solversConfiguration = {
    "default": {
        "Type": pysyslink_base.ConfigurationValue("odeint"),
        "ControlledSolver": pysyslink_base.ConfigurationValue("runge_kutta_fehlberg78")
    }
}

# Create a SimulationManager and run the simulation
simulation_manager = pysyslink_base.SimulationManager(simulation_model, simulation_options)
simulation_output = simulation_manager.RunSimulation()

# Access and print continuous values for integrator2/output
continuous_values = simulation_output.signals["LoggedSignals"]["integrator2/output/0"].TryCastToTyped("double").values
continuous_times = simulation_output.signals["LoggedSignals"]["integrator2/output/0"].TryCastToTyped("double").times

for time, value in zip(continuous_times, continuous_values):
    print(f"{time}: {value}")

# Access and print continuous values for display1/input
continuous_values_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].TryCastToTyped("double").values
continuous_times_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].TryCastToTyped("double").times

for time, value in zip(continuous_times_log, continuous_values_log):
    print(f"{time}: {value}")
