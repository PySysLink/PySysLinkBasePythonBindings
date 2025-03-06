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


# Set the default logger configuration and log level
pysyslink_base.SpdlogManager.configure_default_logger()
pysyslink_base.SpdlogManager.set_log_level(pysyslink_base.LogLevel.debug)

# solver_config = pysyslink_base.SimulationOptions()
# solver_config.solvers_configuration = {"kaixo" : 1.2}

# Create a BlockEventsHandler
block_events_handler = pysyslink_base.BlockEventsHandler()

signal_value = pysyslink_base.SignalValue_double(0.0)
print(issubclass(pysyslink_base.SignalValue_double, pysyslink_base.UnknownTypeSignal))
print(isinstance(signal_value, pysyslink_base.UnknownTypeSignal))
# Load the plugins using BlockTypeSupportPlugingLoader
pluging_loader = pysyslink_base.BlockTypeSupportPlugingLoader()
block_factories = pluging_loader.load_plugins("/usr/local/lib")

saturation = Saturation({"Name" : "saturation_1", "Id" : "saturation_1", "high_limit": 10, "low_limit": -10}, block_events_handler)

print(saturation)
print(saturation.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())

saturation.get_input_ports()[0].set_value(pysyslink_base.SignalValue_double(9.0))
saturation.compute_outputs_of_block(saturation.get_sample_time(), 0.0)
print(saturation.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())


saturation.get_input_ports()[0].set_value(pysyslink_base.SignalValue_double(12.0))
saturation.compute_outputs_of_block(saturation.get_sample_time(), 0.0)
print(saturation.get_output_ports()[0].get_value().try_cast_to_typed().get_payload())
