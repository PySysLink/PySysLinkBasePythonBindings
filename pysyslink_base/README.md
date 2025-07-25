# PySysLinkBase Python Bindings

The PySysLinkBase Python Bindings provide a Python interface to the core PySysLinkBase C++ library. These bindings allow users to build and simulate models—similar to Simulink—but using standard programming languages (such as Python or C++) without requiring a custom language or environment.

> **Note:** The Python bindings are only one part of the overall PySysLink project. They depend on the systemwide installation of PySysLinkBase (and any additional BlockTypeSupport plug‑ins you wish to use) to function properly.

## Overview

PySysLinkBase is an open-source Simulink-like tool that lets you:
- Describe simulation models in YAML.
- Simulate models consisting of interconnected blocks (e.g., constants, summators, integrators, displays).
- Extend functionality via plug‑ins that provide support for different block types.
- Interface with Python to allow scripting, rapid prototyping, and integration into other Python-based scientific or control engineering workflows.

The Python bindings are auto‑generated using [litgen](https://pthom.github.io/litgen), based on the [litgen_template](https://github.com/pthom/litgen_template) repository. This template uses either pybind11 or nanobind (configurable via an environment variable) to produce a native Python module. The bindings for PySysLinkBase have been created using nanobind, and they perform ~6 times slower than the original C++ implementation (which is not bad at all).

## Installation

To install the Python bindings, you must first have a working installation of PySysLinkBase and any desired BlockTypeSupport plug‑ins. Then, you can install the Python package as follows:

1. **Install PySysLinkBase and plug‑ins**  
   Follow the instructions in the respective repositories (e.g., clone, build, and install using CMake):
   ```bash
   git clone https://github.com/PySysLink/PySysLinkBase.git
   cd PySysLinkBase
   mkdir build && cd build
   cmake ..
   make
   sudo make install
   ```
   Repeat similar steps for any BlockTypeSupport plug‑ins you need.

2. **Install the Python Bindings**  
   Clone the Python bindings repository (e.g., `PySysLinkBasePythonBindings`) and install via pip (using a venv is recomended):
   ```bash
   git clone --recursive https://github.com/PySysLink/PySysLinkBasePythonBindings.git
   cd PySysLinkBasePythonBindings
   pip install nanobind
   pip install -v pysyslink_base
   ```

## Using the Bindings

Once installed, you can import the module in Python and interact with the simulation API. For example:

```python
import pysyslink_base


# Set the default logger configuration and log level
pysyslink_base.SpdlogManager.configure_default_logger()
pysyslink_base.SpdlogManager.set_log_level(pysyslink_base.LogLevel.debug)

# Create a BlockEventsHandler
block_events_handler = pysyslink_base.BlockEventsHandler()

print(block_events_handler is pysyslink_base.IBlockEventsHandler)

# Load the plugins using BlockTypeSupportPlugingLoader
pluging_loader = pysyslink_base.BlockTypeSupportPlugingLoader()
block_factories = pluging_loader.load_plugins("/usr/local/lib")


# Parse the simulation model from a YAML file
simulation_model = pysyslink_base.ModelParser.parse_from_yaml(
                        "system1.yaml", block_factories, block_events_handler
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
    ("acumulator1", "output", 0),
    ("display1", "input", 0)
]

# Create a SimulationManager and run the simulation
simulation_manager = pysyslink_base.SimulationManager(simulation_model, simulation_options)
simulation_output = simulation_manager.run_simulation()

# Access and print values for integrator2/output
continuous_values = simulation_output.signals["LoggedSignals"]["acumulator1/output/0"].try_cast_to_typed().values
continuous_times = simulation_output.signals["LoggedSignals"]["acumulator1/output/0"].try_cast_to_typed().times

for time, value in zip(continuous_times, continuous_values):
    print(f"{time}: {value}")

# Access and print values for display1/input
continuous_values_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].try_cast_to_typed().values
continuous_times_log = simulation_output.signals["LoggedSignals"]["display1/input/0"].try_cast_to_typed().times

for time, value in zip(continuous_times_log, continuous_values_log):
    print(f"{time}: {value}")
```

## Regenerating the Bindings

If you make changes to the C++ code in PySysLinkBase, you can regenerate the Python bindings using litgen. From the root directory of the PySysLinkBase repository, run:

```bash
cd pysyslink_base
export LITGEN_USE_NANOBIND=ON  # Or leave unset to use pybind11
python tools/autogenerate_bindings.py
```

This script will:
- Parse all relevant C++ header files.
- Generate a C++ binding file (in either `_pydef_pybind11/` or `_pydef_nanobind/`, depending on your configuration).
- Update Python stub files for type hints.

After regenerating, rebuild the Python bindings package (using pip in editable mode or a fresh installation) to incorporate the changes.

