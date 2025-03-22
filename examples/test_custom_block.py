from typing import Any, Dict
import pysyslink_base


class Saturation(pysyslink_base.ISimulationBlock):
    def __init__(self, block_configuration: Dict[str, Any], block_events_handler: pysyslink_base.IBlockEventsHandler):
        self.sample_time = pysyslink_base.SampleTime(pysyslink_base.SampleTimeType.inherited, supported_sample_time_types_for_inheritance=[pysyslink_base.SampleTimeType.continuous, pysyslink_base.SampleTimeType.discrete])
        self.input_ports = [pysyslink_base.InputPort(has_direct_feedthrough=True, value=pysyslink_base.SignalValue_double(0.0))]
        self.output_ports = [pysyslink_base.OutputPort(pysyslink_base.SignalValue_double(0.0))]
        
        self.high_limit = block_configuration["high_limit"]
        self.low_limit = block_configuration["low_limit"]

        super().__init__(block_configuration, block_events_handler)

    def get_sample_time(self):
        return self.sample_time
    
    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    def get_input_ports(self):
        return self.input_ports

    def get_output_ports(self):
        return self.output_ports
    
    def _compute_outputs_of_block(self, sample_time, current_time: float, is_minor_step: bool = False) -> list[pysyslink_base.OutputPort]:   
        value = self.get_input_ports()[0].get_value().try_cast_to_typed().get_payload()
        print("value: {}".format(value))
        if value > self.high_limit:
            value = self.high_limit
        elif value < self.low_limit:
            value = self.low_limit
        self.get_output_ports()[0].set_value(pysyslink_base.SignalValue_double(value))
        return self.get_output_ports()


class CustomBlockFactory(pysyslink_base.IBlockFactory):
    def __init__(self):
        super().__init__()

    def create_block(self, block_configuration, block_events_handler):
        if block_configuration["BlockClass"] == "Custom/Saturation":
            return Saturation(block_configuration, block_events_handler)
        else:
            raise Exception("BlockClass {} not supported".format(block_configuration["BlockClass"]))



# Set the default logger configuration and log level
pysyslink_base.SpdlogManager.configure_default_logger()
pysyslink_base.SpdlogManager.set_log_level(pysyslink_base.LogLevel.debug)


# Create a BlockEventsHandler
block_events_handler = pysyslink_base.BlockEventsHandler()

# Test that saturation block works
saturation = Saturation({"Name" : "saturation_1", "Id" : "saturation_1", "high_limit": 10, "low_limit": -10}, block_events_handler)

print(saturation)
print(saturation.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())

saturation.get_input_ports()[0].set_value(pysyslink_base.SignalValue_double(9.0))
saturation.compute_outputs_of_block(saturation.get_sample_time(), 0.0)
print(saturation.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())


saturation.get_input_ports()[0].set_value(pysyslink_base.SignalValue_double(12.0))
saturation.compute_outputs_of_block(saturation.get_sample_time(), 0.0)
print(saturation.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())
print(saturation.is_block_free_source())

# Load the plugins using BlockTypeSupportPluginLoader
plugin_loader = pysyslink_base.BlockTypeSupportPluginLoader()
block_factories = plugin_loader.load_plugins("/usr/local/lib")
block_factories["Custom"] = CustomBlockFactory()

simulation_model = pysyslink_base.ModelParser.parse_from_yaml(
                        "system_saturation.yaml", block_factories, block_events_handler
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
simulation_options.run_in_natural_time = True
simulation_options.natural_time_speed_multiplier = 1
simulation_options.block_ids_input_or_output_and_indexes_to_log = [
    ("sum2", "output", 0),
    ("accumulator1", "output", 0),
    ("display1", "input", 0)
]
# simulation_options.solvers_configuration = {
#     "default": {
#         "Type": "odeint",
#         "ControlledSolver": "runge_kutta_fehlberg78"
#     }
# }

# Create a SimulationManager and run the simulation
simulation_manager = pysyslink_base.SimulationManager(simulation_model, simulation_options)
simulation_output = simulation_manager.run_simulation()

# Access and print continuous values for integrator2/output
continuous_values = simulation_output.signals["LoggedSignals"]["sum2/output/0"].try_cast_to_typed().values
continuous_times = simulation_output.signals["LoggedSignals"]["sum2/output/0"].try_cast_to_typed().times

for time, value in zip(continuous_times, continuous_values):
    print(f"{time}: {value}")

# Access and print continuous values for display1/input
continuous_values_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].try_cast_to_typed().values
continuous_times_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].try_cast_to_typed().times

for time, value in zip(continuous_times_log, continuous_values_log):
    print(f"{time}: {value}")
    