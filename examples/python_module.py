# my_python_block.py

class MyPythonBlock:
    """
    Minimal Python block compatible with SimulationBlockPython<T>.

    - inputs: a list of Python floats or complex numbers
    - t: current simulation time (float)

    Returns:
        list of outputs (floats or complex numbers)
    """

    def __init__(self, config):
        """
        config comes from the C++ blockConfiguration dictionary.
        Example:
            {
                "PythonModule": "my_python_block",
                "PythonClass": "MyPythonBlock",
                "NumInputs": 1,
                "NumOutputs": 1,
                "Gain": 2.0
            }
        """
        self.config = config
        self.gain = config.get("Gain", 1.0)

        print(f"[MyPythonBlock] Initialized with gain={self.gain}")

    def initialize(self):
        """
        Optional method — called by C++ block if present.
        """
        print("[MyPythonBlock] initialize() called")

    def compute(self, inputs, t):
        """
        inputs: list of float or complex
        t: float (current time)
        """
        # Example: multiply each input by gain
        outputs = [x * self.gain for x in inputs]

        # Could also depend on time, e.g. adding a sin(t) factor, etc.
        return outputs
