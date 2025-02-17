import glob
import re
import litgen
import os


if "LITGEN_USE_NANOBIND" in os.environ and os.environ["LITGEN_USE_NANOBIND"] == "ON":
    LITGEN_USE_NANOBIND = True
else:
    LITGEN_USE_NANOBIND = False


def postprocess_pydef(code: str) -> str:
    print("Kaixo")
    code = code.replace(">>", "> >")
    code = code.replace("std::pair<double, double>", "PairOfDoubles")
    code = code.replace("std::tuple<bool, std::vector<double>, double>", "OdeStepReturnType")
    return code

def my_litgen_options() -> litgen.LitgenOptions:
    # configure your options here
    options = litgen.LitgenOptions()

    if LITGEN_USE_NANOBIND:
        options.bind_library = litgen.BindLibraryType.nanobind

    # ///////////////////////////////////////////////////////////////////
    #  Root namespace
    # ///////////////////////////////////////////////////////////////////
    # The namespace PySysLinkBase is the C++ root namespace for the generated bindings
    # (i.e. no submodule will be generated for it in the python bindings)
    options.namespaces_root = ["PySysLinkBase"]
    
    options.srcmlcpp_options.header_filter_acceptable__regex = "_H$|^SRC_"

    options.class_override_virtual_methods_in_python__regex = "^IBlock|^ISim|^IOde|^BlockEvent|^Unknown"

    options.postprocess_pydef_function = postprocess_pydef


    options.fn_template_options.add_specialization("^TryGetConfigurationValue$", ["bool", "int", "double", "std::complex<double>"], add_suffix_to_function_name=False)
    options.fn_template_options.add_specialization("^TryCastToTyped$", ["double", "std::complex<double>"], add_suffix_to_function_name=False)
    options.fn_template_options.add_specialization("^TryInsertValue$", ["double", "std::complex<double>"], add_suffix_to_function_name=False)

    options.class_template_options.add_specialization("Signal", ["double", "std::complex<double>"])
    # options.class_template_options.add_specialization("SignalValue", ["double", "std::complex<double>"])
    # options.class_template_options.add_specialization("OdeintStepSolver", ["double"])

    # Set to True if you want the stub file to be formatted with black
    options.python_run_black_formatter = True

    return options


def autogenerate() -> None:
    repository_dir = os.path.realpath(os.path.dirname(__file__) + "/../")

    include_dir = os.path.join(repository_dir, "src/cpp_libraries/PySysLinkBase/src")

    # Recursively find all .h files
    header_files = glob.glob(include_dir + "/**/*.h", recursive=True)

    print("Found header files:")
    for header in header_files:
        print(header)

    output_cpp_pydef_file = (
        repository_dir + "/_pydef_nanobind/nanobind_PySysLinkBase.cpp" if LITGEN_USE_NANOBIND
        else repository_dir + "/_pydef_pybind11/pybind_PySysLinkBase.cpp"
    )

    litgen.write_generated_code_for_files(
        options=my_litgen_options(),
        input_cpp_header_files=header_files,
        output_cpp_pydef_file=output_cpp_pydef_file,
        output_stub_pyi_file=repository_dir + "/_stubs/pysyslink_base/__init__.pyi",
    )

    with open(output_cpp_pydef_file, "r") as file:
        code = file.read()

        # Ensure '>>' is replaced with '> >' everywhere
        code = code.replace(">>", "> >")
        code = code.replace("_complex<double>", "_complex_double")

        # Function to replace 'py::init<const PySysLinkBase::SignalValue &>' with the specific type
        
        change_strings = ["double", "std::complex<double>"]
        
        for change_string in change_strings:
            code = code.replace('py::init<const PySysLinkBase::SignalValue &>', 'py::init<const PySysLinkBase::SignalValue<{} > &>'.format(change_string), 1)


        # Replace std::pair and std::tuple, but ignore first occurrence
        def replace_after_first_occurrence(text, old, new):
            first_index = text.find(old)
            if first_index != -1:
                first_index = text.find(old, first_index + 1)  # Find the second occurrence
                if first_index != -1:
                    text = text[:first_index] + text[first_index:].replace(old, new)
            return text

        code = replace_after_first_occurrence(code, "std::pair<double, double>", "PairOfDoubles")
        code = replace_after_first_occurrence(code, "std::tuple<bool, std::vector<double>, double>", "OdeStepReturnType")

    # Write back the modified code
    with open(output_cpp_pydef_file, "w") as file:
        file.write(code)

        print("Post-processing completed.")


if __name__ == "__main__":
    autogenerate()
