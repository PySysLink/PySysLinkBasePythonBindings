from collections import defaultdict, deque
import glob
import re
import litgen
import os
from codemanip import amalgamated_header


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

def preprocess_code(code: str) -> str:
    code = code.replace("public virtual", "public")
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
    options.srcmlcpp_options.code_preprocess_function = preprocess_code
    options.srcmlcpp_options.header_filter_acceptable__regex = "_H$|^SRC_|^AMALGAMATED_HEADER_H"

    options.class_override_virtual_methods_in_python__regex = "^IBlock|^ISim|^IOde|^BlockEvent|^Unknown|^IBlockEventHandler$|^EulerForwardStepSolver$"

    options.postprocess_pydef_function = postprocess_pydef


    options.fn_template_options.add_specialization("^TryGetConfigurationValue$", ["bool", "int", "double", "std::complex<double>"], add_suffix_to_function_name=False)
    options.fn_template_options.add_specialization("^TryCastToTyped$", ["double", "std::complex<double>"], add_suffix_to_function_name=False)
    options.fn_template_options.add_specialization("^TryInsertValue$", ["double", "std::complex<double>"], add_suffix_to_function_name=False)

    options.class_template_options.add_specialization("Signal", ["double", "std::complex<double>"])
    options.class_template_options.add_specialization("SignalValue", ["double", "std::complex<double>"])

    options.class_exclude_by_name__regex = "^ISimulationBlockWithContinuousStates$"
    # options.class_template_options.add_specialization("OdeintStepSolver", ["double"])

    # Set to True if you want the stub file to be formatted with black
    options.python_run_black_formatter = True

    return options

def find_header_dependencies(header_file):
    """Extract header dependencies from #include "..." statements."""
    dependencies = []
    include_pattern = re.compile(r'^\s*#\s*include\s+"([^"]+)"')
    with open(header_file, "r", encoding="utf-8") as f:
        for line in f:
            match = include_pattern.match(line)
            if match:
                dependencies.append(match.group(1))
    return dependencies


def topological_sort(headers, dependency_graph):
    """
    Sort headers so that a header appears only after all its dependencies.
    headers: list of header full paths.
    dependency_graph: dict mapping each header to a list of headers (full paths) it depends on.
    """
    print(dependency_graph)
    # Start with a copy of the dependency graph, so we can modify it.
    dep_graph = {header: list(dependency_graph.get(header, [])) for header in headers}
    # Set of headers that still need to be ordered.
    remaining = set(headers)
    sorted_list = []
    
    while remaining:
        # Find headers with no remaining dependencies among those still remaining.
        no_dep = [h for h in remaining if all(dep not in remaining for dep in dep_graph[h])]
        if not no_dep:
            print("Warning: There may be cyclic dependencies among headers!")
            break
        
        # Add headers with no dependencies to the sorted list and remove them from 'remaining'.
        for header in no_dep:
            sorted_list.append(header)
            remaining.remove(header)
            
    return sorted_list

def amalgamate_headers(header_dir, output_file):
    # Find all .h files in header_dir (recursively)
    header_files = glob.glob(os.path.join(header_dir, '**', '*.h'), recursive=True)
    header_files = [os.path.abspath(h) for h in header_files]

    # Build a mapping from header basenames to full paths for dependency resolution.
    header_map = {os.path.basename(h): h for h in header_files}

    # Build the dependency graph.
    dependency_graph = defaultdict(list)
    for header in header_files:
        deps = find_header_dependencies(header)
        full_deps = []
        for dep in deps:
            if dep in header_map:
                full_deps.append(header_map[dep])
        dependency_graph[header] = full_deps

    # Topologically sort the headers.
    sorted_headers = topological_sort(header_files, dependency_graph)
    print(sorted_headers)

    defined_guards = set()
    combined_lines = []
    combined_lines.append("#ifndef AMALGAMATED_HEADER_H")
    combined_lines.append("#define AMALGAMATED_HEADER_H")
    combined_lines.append("")

    # Regex to match include guard start lines (e.g. "#ifndef GUARD_NAME")
    guard_regex = re.compile(r'^\s*#\s*ifndef\s+(\S+)')

    # First pass: collect all unique guard macros from the headers.
    for header in sorted_headers:
        with open(header, "r", encoding="utf-8") as f:
            for line in f:
                match = guard_regex.match(line)
                if match:
                    defined_guards.add(match.group(1))
                    break  # assume first match is the guard for this header.

    # Write the collected guard macros.
    for guard in defined_guards:
        combined_lines.append(f"#define {guard}")
    combined_lines.append("")

    # Second pass: process each header in dependency order.
    for header in sorted_headers:
        combined_lines.append(f"// Begin header: {os.path.basename(header)}")
        with open(header, "r", encoding="utf-8") as f:
            for line in f:
                # Skip include guard start and end lines.
                if guard_regex.match(line):
                    continue
                if re.match(r'^\s*#\s*endif', line):
                    continue
                combined_lines.append(line.rstrip())
        combined_lines.append(f"// End header: {os.path.basename(header)}")
        combined_lines.append("")
    
    combined_lines.append("#endif // AMALGAMATED_HEADER_H")

    with open(output_file, "w", encoding="utf-8") as out:
        out.write("\n".join(combined_lines))
    
    print(f"Amalgamated header written to {output_file}")

def autogenerate(header_file: str) -> None:
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
        input_cpp_header_files=[header_file],
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

        for change_string in change_strings:
            code = code.replace('nb::init<const PySysLinkBase::SignalValue &>', 'nb::init<const PySysLinkBase::SignalValue<{} > &>'.format(change_string), 1)
        
        for change_string in change_strings:
            code = code.replace('PySysLinkBase::SignalValue::clone', 'PySysLinkBase::SignalValue<{} >::clone'.format(change_string), 1)

        code = code.replace("void py_init_module_pysyslink_base(nb::module_& m)\n{", "void py_init_module_pysyslink_base(nb::module_& m)\n{\nusing namespace PySysLinkBase;", 1)
        code = code.replace("NB_TRAMPOLINE(ISimulationBlockWithContinuousStates, 12);", "ISimulationBlockWithContinuousStates_trampoline(\nstd::map<std::string, ConfigurationValue> blockConfiguration,\nstd::shared_ptr<IBlockEventsHandler> blockEventsHandler)\n: ISimulationBlock(blockConfiguration, blockEventsHandler),\nISimulationBlockWithContinuousStates(blockConfiguration, blockEventsHandler) {}\n\nNB_TRAMPOLINE(ISimulationBlockWithContinuousStates, 12);")

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
    repository_dir = os.path.realpath(os.path.dirname(__file__) + "/../")
    include_dir = os.path.join(repository_dir, "src/cpp_libraries/PySysLinkBase/src")
    print(include_dir)
    amalgamate_headers(header_dir=include_dir, output_file="test.h")
    autogenerate("test.h")
