# If you want to use mypy or pyright, you may have to ignore some errors, like below:

# mypy: disable-error-code="type-arg"

from typing import Dict, overload, List
import numpy as np

NumberType = (int, float, np.number)

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  AUTOGENERATED CODE !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# <litgen_stub> // Autogenerated code below! Do not edit!
####################    <generated_from:test.h>    ####################
# #ifndef AMALGAMATED_HEADER_H
#

class SampleTimeType(enum.Enum):
    continuous = enum.auto()  # (= 0)
    discrete = enum.auto()  # (= 1)
    constant = enum.auto()  # (= 2)
    inherited = enum.auto()  # (= 3)
    multirate = enum.auto()  # (= 4)

class SampleTime:
    @overload
    def __init__(
        self,
        sample_time_type: SampleTimeType,
        discrete_sample_time: float,
        continuous_sample_time_group: int,
        supported_sample_time_types_for_inheritance: List[SampleTimeType],
        multirate_sample_times: List[SampleTime],
        input_multirate_sample_time_index: int = -1,
        output_multirate_sample_time_index: int = -1,
    ) -> None:
        pass

    @overload
    def __init__(self, sample_time_type: SampleTimeType) -> None:
        pass

    @overload
    def __init__(
        self, sample_time_type: SampleTimeType, discrete_sample_time: float
    ) -> None:
        pass

    @overload
    def __init__(
        self, sample_time_type: SampleTimeType, continuous_sample_time_group: int
    ) -> None:
        pass

    @overload
    def __init__(
        self,
        sample_time_type: SampleTimeType,
        supported_sample_time_types_for_inheritance: List[SampleTimeType],
    ) -> None:
        pass

    @overload
    def __init__(
        self, sample_time_type: SampleTimeType, multirate_sample_times: List[SampleTime]
    ) -> None:
        pass

    @overload
    def __init__(
        self,
        sample_time_type: SampleTimeType,
        multirate_sample_times: List[SampleTime],
        input_multirate_sample_time_index: int,
        output_multirate_sample_time_index: int,
    ) -> None:
        pass

    def get_sample_time_type(self) -> SampleTimeType:
        pass

    def get_discrete_sample_time(self) -> float:
        pass

    def get_continuous_sample_time_group(self) -> int:
        pass

    def get_supported_sample_time_types_for_inheritance(self) -> List[SampleTimeType]:
        pass

    def get_multirate_sample_times(self) -> List[SampleTime]:
        pass

    def set_multirate_sample_time_in_index(
        self, multirate_sample_time: SampleTime, index: int
    ) -> None:
        pass

    def has_multirate_inherited_sample_time(self) -> bool:
        pass

    def get_input_multirate_sample_time_index(self) -> int:
        pass

    def get_output_multirate_sample_time_index(self) -> int:
        pass

    def is_input_multirate_inherited(self) -> bool:
        pass

    def is_output_multirate_inherited(self) -> bool:
        pass

    @staticmethod
    def sample_time_type_string(sample_time_type: SampleTimeType) -> str:
        pass

# End header: SampleTime.h

class UnknownTypeSignalValue:
    def get_type_id(self) -> str:  # overridable (pure virtual)
        pass
    #  ------------------------------------------------------------------------
    #      <template specializations for function TryCastToTyped>
    @overload
    def try_cast_to_typed(self) -> SignalValue[float]:
        pass

    @overload
    def try_cast_to_typed(self) -> SignalValue[std.complex[float]]:
        pass
    #      </template specializations for function TryCastToTyped>
    #  ------------------------------------------------------------------------

    def clone(self) -> UnknownTypeSignalValue:  # overridable (pure virtual)
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: UnknownTypeSignalValue.h

class ConfigurationValueManager:
    #  ------------------------------------------------------------------------
    #      <template specializations for function TryGetConfigurationValue>
    @staticmethod
    @overload
    def try_get_configuration_value(
        key_name: str, configuration_values: Dict[str, ConfigurationValue]
    ) -> bool:
        pass

    @staticmethod
    @overload
    def try_get_configuration_value(
        key_name: str, configuration_values: Dict[str, ConfigurationValue]
    ) -> int:
        pass

    @staticmethod
    @overload
    def try_get_configuration_value(
        key_name: str, configuration_values: Dict[str, ConfigurationValue]
    ) -> float:
        pass

    @staticmethod
    @overload
    def try_get_configuration_value(
        key_name: str, configuration_values: Dict[str, ConfigurationValue]
    ) -> std.complex[float]:
        pass
    #      </template specializations for function TryGetConfigurationValue>
    #  ------------------------------------------------------------------------
    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: ConfigurationValue.h

# namespace PySysLinkBase

# End header: FullySupportedSignalValue.h

class LogLevel(enum.Enum):
    off = enum.auto()  # (= 0)
    debug = enum.auto()  # (= 1)
    info = enum.auto()  # (= 2)
    warning = enum.auto()  # (= 3)
    error = enum.auto()  # (= 4)
    critical = enum.auto()  # (= 5)

class SpdlogManager:
    @staticmethod
    def configure_default_logger() -> None:
        pass

    @staticmethod
    def set_log_level(log_level: LogLevel) -> None:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: SpdlogManager.h

class UnknownTypeSignal:
    id: str
    times: List[float]

    def get_type_id(self) -> str:  # overridable (pure virtual)
        pass
    #  ------------------------------------------------------------------------
    #      <template specializations for function TryCastToTyped>
    @overload
    def try_cast_to_typed(self) -> Signal[float]:
        pass

    @overload
    def try_cast_to_typed(self) -> Signal[std.complex[float]]:
        pass
    #      </template specializations for function TryCastToTyped>
    #  ------------------------------------------------------------------------

    #  ------------------------------------------------------------------------
    #      <template specializations for function TryInsertValue>
    @overload
    def try_insert_value(self, time: float, value: float) -> None:
        pass

    @overload
    def try_insert_value(self, time: float, value: std.complex[float]) -> None:
        pass
    #      </template specializations for function TryInsertValue>
    #  ------------------------------------------------------------------------
    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

#  ------------------------------------------------------------------------
#      <template specializations for class Signal>
class Signal_double(UnknownTypeSignal):  # Python specialization for Signal<double>
    values: List[float]

    def get_type_id(self) -> str:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

class Signal_complex_double(
    UnknownTypeSignal
):  # Python specialization for Signal<std::complex<double>>
    values: List[std.complex[float]]

    def get_type_id(self) -> str:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

#      </template specializations for class Signal>
#  ------------------------------------------------------------------------

class SimulationOutput:
    signals: Dict[str, Dict[str, UnknownTypeSignal]]
    def __init__(self) -> None:
        """Auto-generated default constructor"""
        pass

# namespace PySysLinkBase

# End header: SimulationOutput.h

class BlockEvent:
    event_type_id: str

    def __init__(self, event_type_id: str) -> None:
        pass

# namespace PySysLinkBase

# End header: BlockEvent.h

class IBlockEventsHandler:
    def block_event_callback(  # overridable (pure virtual)
        self, block_event: BlockEvent
    ) -> None:
        pass

    def register_value_update_block_event_callback(  # overridable (pure virtual)
        self, callback: Callable[[ValueUpdateBlockEvent], None]
    ) -> None:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: IBlockEventsHandler.h

class IOdeStepSolver:
    def solve_step(  # overridable (pure virtual)
        self,
        system: Callable[[List[float], float], List[float]],
        states_0: List[float],
        current_time: float,
        time_step: float,
    ) -> Tuple[bool, List[float], float]:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: IOdeStepSolver.h

class ValueUpdateBlockEvent(BlockEvent):
    simulation_time: float
    value_id: str
    value: FullySupportedSignalValue
    def __init__(
        self, simulation_time: float, value_id: str, value: FullySupportedSignalValue
    ) -> None:
        pass

# namespace PySysLinkBase
# End header: ValueUpdateBlockEvent.h

class ISimulationBlock:
    def get_id(self) -> str:
        pass

    def get_name(self) -> str:
        pass

    def __init__(
        self,
        block_configuration: Dict[str, ConfigurationValue],
        block_events_handler: IBlockEventsHandler,
    ) -> None:
        pass

    def get_sample_time(self) -> SampleTime:  # overridable (pure virtual)
        pass

    def set_sample_time(  # overridable (pure virtual)
        self, sample_time: SampleTime
    ) -> None:
        pass

    def get_input_ports(self) -> List[InputPort]:  # overridable (pure virtual)
        pass

    def get_output_ports(self) -> List[OutputPort]:  # overridable (pure virtual)
        pass

    def compute_outputs_of_block(
        self, sample_time: SampleTime, current_time: float, is_minor_step: bool = False
    ) -> List[OutputPort]:
        pass

    def _compute_outputs_of_block(  # overridable (pure virtual)
        self, sample_time: SampleTime, current_time: float, is_minor_step: bool = False
    ) -> List[OutputPort]:
        pass

    def is_block_free_source(self) -> bool:
        pass

    def is_input_direct_block_chain_end(self, input_index: int) -> bool:
        pass

    def notify_event(self, block_event: BlockEvent) -> None:
        pass

    def try_update_configuration_value(  # overridable (pure virtual)
        self, key_name: str, value: ConfigurationValue
    ) -> bool:
        pass

    @staticmethod
    def find_block_by_id(
        id: str, blocks_to_find: List[ISimulationBlock]
    ) -> ISimulationBlock:
        pass

    def register_read_inputs_callbacks(
        self, callback: Callable[[str, List[InputPort], SampleTime, float], None]
    ) -> None:
        pass

    def register_calculate_output_callbacks(
        self, callback: Callable[[str, List[OutputPort], SampleTime, float], None]
    ) -> None:
        pass

    def get_events(  # overridable
        self,
        sample_time: SampleTime,
        event_time: float,
        event_time_states: List[float],
        include_known_events: bool = False,
    ) -> List[Tuple[float, float]]:
        pass

    def get_known_events(  # overridable
        self,
        resolved_sample_time: SampleTime,
        simulation_start_time: float,
        simulation_end_time: float,
    ) -> List[float]:
        pass

# End header: ISimulationBlock.h

class BlockEventsHandler(IBlockEventsHandler):
    def __init__(self) -> None:
        pass

    def block_event_callback(self, block_event: BlockEvent) -> None:
        pass

    def register_value_update_block_event_callback(
        self, callback: Callable[[ValueUpdateBlockEvent], None]
    ) -> None:
        pass

# namespace PySysLinkBase

# End header: BlockEventsHandler.h

# namespace PySysLinkBase

# End header: OdeintStepSolver.h

class EulerForwardStepSolver(IOdeStepSolver):
    def solve_step(  # overridable
        self,
        system: Callable[[List[float], float], List[float]],
        states_0: List[float],
        current_time: float,
        time_step: float,
    ) -> Tuple[bool, List[float], float]:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: EulerForwardStepSolver.h

class Port:
    def __init__(self, value: UnknownTypeSignalValue) -> None:
        pass

    def try_copy_value_to_port(self, other_port: Port) -> None:
        pass

    def set_value(self, value: UnknownTypeSignalValue) -> None:
        pass

    def get_value(self) -> UnknownTypeSignalValue:
        pass

    def __eq__(self, rhs: Port) -> bool:
        pass

# End header: Port.h

class SimulationOptions:
    def __init__(self) -> None:
        pass
    start_time: float
    stop_time: float

    run_in_natural_time: bool = False
    natural_time_speed_multiplier: float = 1.0

    block_ids_input_or_output_and_indexes_to_log: List[Tuple[str, str, int]] = List[
        Tuple[str, str, int]
    ]()

    solvers_configuration: Dict[str, Dict[str, ConfigurationValue]]

# namespace PySysLinkBase

# End header: SimulationOptions.h

class SolverFactory:
    @staticmethod
    def create_ode_step_solver(
        solver_configuration: Dict[str, ConfigurationValue],
    ) -> IOdeStepSolver:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: SolverFactory.h

#  ------------------------------------------------------------------------
#      <template specializations for class SignalValue>
class SignalValue_double(
    UnknownTypeSignalValue
):  # Python specialization for SignalValue<double>
    @overload
    def __init__(self, initial_payload: float) -> None:
        pass

    @overload
    def __init__(self, other: SignalValue) -> None:
        pass

    def clone(self) -> UnknownTypeSignalValue:
        pass

    def get_type_id(self) -> str:
        pass

    def get_payload(self) -> float:
        pass

    def set_payload(self, new_payload: float) -> None:
        pass

class SignalValue_complex_double(
    UnknownTypeSignalValue
):  # Python specialization for SignalValue<std::complex<double>>
    @overload
    def __init__(self, initial_payload: std.complex[float]) -> None:
        pass

    @overload
    def __init__(self, other: SignalValue) -> None:
        pass

    def clone(self) -> UnknownTypeSignalValue:
        pass

    def get_type_id(self) -> str:
        pass

    def get_payload(self) -> std.complex[float]:
        pass

    def set_payload(self, new_payload: std.complex[float]) -> None:
        pass

#      </template specializations for class SignalValue>
#  ------------------------------------------------------------------------

# namespace PySysLinkBase

# End header: SignalValue.h

class OutputPort(Port):
    def __init__(self, value: UnknownTypeSignalValue) -> None:
        pass

# End header: OutputPort.h

class ISimulationBlockWithContinuousStates(ISimulationBlock):
    def __init__(
        self,
        block_configuration: Dict[str, ConfigurationValue],
        block_events_handler: IBlockEventsHandler,
    ) -> None:
        pass

    def get_continuous_states(self) -> List[float]:  # overridable (pure virtual)
        pass

    def set_continuous_states(  # overridable (pure virtual)
        self, new_states: List[float]
    ) -> None:
        pass

    def get_continuous_state_derivatives(  # overridable (pure virtual)
        self, sample_time: SampleTime, current_time: float
    ) -> List[float]:
        pass

    def get_continuous_state_jacobians(  # overridable
        self, sample_time: SampleTime, current_time: float
    ) -> List[List[float]]:
        pass

# namespace PySysLinkBase

# End header: ISimulationBlockWithContinuousStates.h

class InputPort(Port):
    def __init__(
        self, has_direct_feedthrough: bool, value: UnknownTypeSignalValue
    ) -> None:
        pass

    def has_direct_feedthrough(self) -> bool:
        pass

# End header: InputPort.h

class IBlockFactory:
    def create_block(  # overridable (pure virtual)
        self,
        block_configuration: Dict[str, ConfigurationValue],
        block_events_handler: IBlockEventsHandler,
    ) -> ISimulationBlock:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# End header: IBlockFactory.h

class PortLink:
    def __init__(
        self,
        origin_block: ISimulationBlock,
        sink_block: ISimulationBlock,
        origin_block_port_index: int,
        sink_block_port_index: int,
    ) -> None:
        pass
    origin_block: ISimulationBlock
    sink_block: ISimulationBlock
    origin_block_port_index: int
    sink_block_port_index: int

    @staticmethod
    def parse_from_config(
        link_configuration: Dict[str, ConfigurationValue],
        blocks: List[ISimulationBlock],
    ) -> PortLink:
        pass

# End header: PortLink.h

class BlockTypeSupportPluginLoader:
    def load_plugins(self, plugin_directory: str) -> Dict[str, IBlockFactory]:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: BlockTypeSupportPluginLoader.h

class SimulationModel:
    simulation_blocks: List[ISimulationBlock]
    port_links: List[PortLink]
    block_events_handler: IBlockEventsHandler

    def __init__(
        self,
        simulation_blocks: List[ISimulationBlock],
        port_links: List[PortLink],
        block_events_handler: IBlockEventsHandler,
    ) -> None:
        pass

    def get_connected_ports(
        self, origin_block: ISimulationBlock, output_port_index: int
    ) -> List[InputPort]:
        pass

    def get_connected_blocks(
        self, origin_block: ISimulationBlock, output_port_index: int
    ) -> Tuple[List[ISimulationBlock], List[int]]:
        pass

    def get_origin_block(
        self, sink_block: ISimulationBlock, input_port_index: int
    ) -> ISimulationBlock:
        pass

    def get_direct_block_chains(self) -> List[List[ISimulationBlock]]:
        pass

    def order_block_chains_onto_free_order(
        self, direct_block_chains: List[List[ISimulationBlock]]
    ) -> List[ISimulationBlock]:
        pass

    def propagate_sample_times(self) -> None:
        pass

# End header: SimulationModel.h

class BasicOdeSolver:
    first_time_step: float

    def system_model(self, states: List[float], time: float) -> List[float]:
        pass

    def __init__(
        self,
        ode_step_solver: IOdeStepSolver,
        simulation_model: SimulationModel,
        simulation_blocks: List[ISimulationBlock],
        sample_time: SampleTime,
        simulation_options: SimulationOptions,
        first_time_step: float = 1e-6,
        activate_events: bool = True,
        event_tolerance: float = 1e-2,
    ) -> None:
        pass

    def update_states_to_next_time_hits(self) -> None:
        pass

    def do_step(self, current_time: float, time_step: float) -> None:
        pass

    def compute_major_outputs(self, current_time: float) -> None:
        pass

    def get_next_time_hit(self) -> float:
        pass

    def get_next_suggested_time_step(self) -> float:
        pass

# namespace PySysLinkBase

# End header: BasicOdeSolver.h

class ModelParser:
    @staticmethod
    def parse_from_yaml(
        filename: str,
        block_factories: Dict[str, IBlockFactory],
        block_events_handler: IBlockEventsHandler,
    ) -> SimulationModel:
        pass

    def __init__(self) -> None:
        """Autogenerated default constructor"""
        pass

# namespace PySysLinkBase

# End header: ModelParser.h

class SimulationManager:
    def __init__(
        self, simulation_model: SimulationModel, simulation_options: SimulationOptions
    ) -> None:
        pass

    def run_simulation(self) -> SimulationOutput:
        pass

    def run_simulation_step(self) -> float:
        pass

    def get_simulation_output(self) -> SimulationOutput:
        pass

# End header: SimulationManager.h

# #endif ####################    </generated_from:test.h>    ####################

# </litgen_stub>
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!  AUTOGENERATED CODE END !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
