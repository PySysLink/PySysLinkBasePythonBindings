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
    
class SecondOrderSystem(pysyslink_base.ISimulationBlockWithContinuousStates):
    def __init__(self, block_configuration: Dict[str, Any], block_events_handler: pysyslink_base.IBlockEventsHandler):
        continuous_sample_time_group = block_configuration.get("ContinuousSampleTimeGroup", 0)
        self.sample_time = pysyslink_base.SampleTime(pysyslink_base.SampleTimeType.continuous, continuous_sample_time_group=continuous_sample_time_group)
        self.input_ports = [pysyslink_base.InputPort(has_direct_feedthrough=False, value=pysyslink_base.SignalValue_double(0.0))]
        self.output_ports = [pysyslink_base.OutputPort(pysyslink_base.SignalValue_double(0.0))]
        
        self.Ts = block_configuration["Ts"]
        self.tau = block_configuration["tau"]
        self.Kp = block_configuration["Kp"]

        self.x1 = 0
        self.x2 = 0

        super().__init__(block_configuration, block_events_handler)

    def get_sample_time(self):
        return self.sample_time
    
    def set_sample_time(self, sample_time):
        self.sample_time = sample_time

    def get_input_ports(self):
        return self.input_ports

    def get_output_ports(self):
        return self.output_ports
    
    def get_continuous_state_derivatives(self, sample_time, current_time):
        u = self.get_input_ports()[0].get_value().try_cast_to_typed().get_payload()
        dx1 = self.x2
        dx2 = -1/(self.Ts**2) * self.x1 - 2 * self.tau/self.Ts * self.x2 + self.Kp/(self.Ts**2) * u
        return [dx1, dx2]
    
    def get_continuous_states(self):
        return [self.x1, self.x2]
    
    def set_continuous_states(self, new_states):
        self.x1 = new_states[0]
        self.x2 = new_states[1]
    
    def get_events(self, sample_time, event_time, event_time_states):
        return [(event_time_states[0], float('nan')), (event_time_states[1], float('nan'))]
    
    def _compute_outputs_of_block(self, sample_time, current_time: float, is_minor_step: bool = False) -> list[pysyslink_base.OutputPort]:   
        print("kaixo barnealdetik")
        y = self.x1

        self.get_output_ports()[0].set_value(pysyslink_base.SignalValue_double(y))
        return self.get_output_ports()


class CustomBlockFactory(pysyslink_base.IBlockFactory):
    def __init__(self):
        super().__init__()

    def create_block(self, block_configuration, block_events_handler):
        print("Creating block {}".format(block_configuration["Id"]))
        if block_configuration["BlockClass"] == "Custom/Saturation":
            return Saturation(block_configuration, block_events_handler)
        elif block_configuration["BlockClass"] == "Custom/SecondOrderSystem":
            return SecondOrderSystem(block_configuration, block_events_handler)
        else:
            raise Exception("BlockClass {} not supported".format(block_configuration["BlockClass"]))



# Set the default logger configuration and log level
pysyslink_base.SpdlogManager.configure_default_logger()
pysyslink_base.SpdlogManager.set_log_level(pysyslink_base.LogLevel.debug)


# Create a BlockEventsHandler
block_events_handler = pysyslink_base.BlockEventsHandler()

# Test that saturation block works
second_order_system = SecondOrderSystem({"Name" : "second_order_1", "Id" : "second_order_1", "Ts": 1, "Kp": 1, "tau": 1}, block_events_handler)

print(second_order_system)
print(second_order_system.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())

second_order_system.get_input_ports()[0].set_value(pysyslink_base.SignalValue_double(9.0))
print("kaixo")
# second_order_system.compute_outputs_of_block(second_order_system.get_sample_time(), 0.0)
second_order_system.set_continuous_states([1,1])
print(second_order_system.get_continuous_states())

print(second_order_system.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())

print("wololo")
print(second_order_system.get_id())
second_order_system.get_input_ports()[0].set_value(pysyslink_base.SignalValue_double(12.0))
second_order_system.compute_outputs_of_block(second_order_system.get_sample_time(), 0.0)
print(second_order_system.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())

# Load the plugins using BlockTypeSupportPluginLoader
plugin_loader = pysyslink_base.BlockTypeSupportPluginLoader()
block_factories = plugin_loader.load_plugins("/usr/local/lib")
block_factories["Custom"] = CustomBlockFactory()

simulation_model = pysyslink_base.ModelParser.parse_from_yaml(
                        "system_second_order_custom.yaml", block_factories, block_events_handler
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
    