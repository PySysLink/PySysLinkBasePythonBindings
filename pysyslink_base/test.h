#ifndef AMALGAMATED_HEADER_H
#define AMALGAMATED_HEADER_H

#define PY_SYS_LINK_BASE_PORTS_AND_SIGNAL_VALUES_UNKNOWN_TYPE_SIGNAL_VALUE
#define SRC_PY_SYS_LINK_BASE_MODEL_PARSER
#define SRC_EULER_FORWARD_STEP_SOLVER
#define SRC_EULER_BACKWARD_STEP_SOLVER
#define PYSYSLINK_BASE_SRC_CPP_LIBRARIES_PY_SYS_LINK_BASE_SRC_CONTINUOUS_AND_ODE_ISIMULATION_BLOCK_WITH_CONTINUOUS_STATES
#define SRC_PY_SYS_LINK_BASE_PORTS_AND_SIGNAL_VALUES_SIGNAL_VALUE
#define SRC_PORTS_AND_SIGNAL_VALUES_PORT
#define SRC_CONTINUOUS_AND_ODE_SOLVER_FACTORY
#define SRC_CONTINUOUS_AND_ODE_IODE_STEP_SOLVER
#define SRC_PY_SYS_LINK_BASE_PORT_LINK
#define SRC_PY_SYS_LINK_BASE_IBLOCK_FACTORY
#define SRC_BLOCK_EVENTS_HANDLER
#define SRC_SIMULATION_OUTPUT
#define SRC_BASIC_ODE_SOLVER
#define SRC_IBLOCK_EVENTS_HANDLER
#define SRC_PY_SYS_LINK_BASE_BLOCK_TYPE_SUPPORT_PLUGING_LOADER
#define SRC_PORTS_AND_SIGNAL_VALUES_OUTPUT_PORT
#define SRC_BLOCK_EVENTS_BLOCK_EVENT
#define PY_SYS_LINK_BASE_SAMPLE_TIME
#define SRC_SPDLOG_MANAGER
#define SRC_PORTS_AND_SIGNAL_VALUES_INPUT_PORT
#define SRC_ISIMULATION_BLOCK
#define SRC_CONFIGURATION_VALUE
#define SRC_CONTINUOUS_AND_ODE_ODEINT_STEP_SOLVER
#define SRC_SIMULATION_OPTIONS
#define SRC_FULLY_SUPPORTED_SIGNAL_VALUE
#define SRC_BLOCK_EVENTS_VALUE_UPDATE_BLOCK_EVENT
#define SRC_SIMULATION_MODEL
#define SRC_SIMULATION_MANAGER
#define SRC_CONTINUOUS_AND_ODE_ODEINT_IMPLICIT_STEP_SOLVER

// Begin header: BlockEvent.h
#define SRC_BLOCK_EVENTS_BLOCK_EVENT

#include <string>

namespace PySysLinkBase
{
    class BlockEvent
    {
        public:
        std::string eventTypeId;

        BlockEvent(std::string eventTypeId) : eventTypeId(eventTypeId) {}

        virtual ~BlockEvent() = default; // Ensures the class is polymorphic
    };
} // namespace PySysLinkBase


// End header: BlockEvent.h

// Begin header: SpdlogManager.h
#define SRC_SPDLOG_MANAGER

namespace PySysLinkBase
{
    enum LogLevel
    {
        off,
        debug,
        info,
        warning,
        error,
        critical
    };

    class SpdlogManager
    {
        public:
        static void ConfigureDefaultLogger();
        static void SetLogLevel(LogLevel logLevel);
    };
} // namespace PySysLinkBase


// End header: SpdlogManager.h

// Begin header: FullySupportedSignalValue.h
#define SRC_FULLY_SUPPORTED_SIGNAL_VALUE


#include <string>
#include <variant>
#include <vector>
#include <memory>
#include <map>
#include <stdexcept>
#include <complex>

#include "PortsAndSignalValues/UnknownTypeSignalValue.h"
#include "PortsAndSignalValues/SignalValue.h"

namespace PySysLinkBase
{
    using FullySupportedSignalValue = std::variant<
    int,
    double,
    bool,
    std::complex<double>,
    std::string
>;

FullySupportedSignalValue ConvertToFullySupportedSignalValue(const std::shared_ptr<UnknownTypeSignalValue>& unknownValue);

} // namespace PySysLinkBase



// End header: FullySupportedSignalValue.h

// Begin header: SampleTime.h
#define PY_SYS_LINK_BASE_SAMPLE_TIME

#include <cmath>
#include <vector>
#include <limits>
#include <string>
#include <memory>

namespace PySysLinkBase
{
    enum SampleTimeType
    {
        continuous,
        discrete,
        constant,
        inherited,
        multirate
    };

    class SampleTime
    {
        private:
            SampleTimeType sampleTimeType;
            double discreteSampleTime;
            int continuousSampleTimeGroup = -1;
            std::vector<SampleTimeType> supportedSampleTimeTypesForInheritance = std::vector<SampleTimeType>{};
            std::vector<std::shared_ptr<SampleTime>> multirateSampleTimes = {};
            int inputMultirateSampleTimeIndex = -1;
            int outputMultirateSampleTimeIndex = -1;
        public:
            SampleTime(SampleTimeType sampleTimeType,
                        double discreteSampleTime, int continuousSampleTimeGroup, std::vector<SampleTimeType> supportedSampleTimeTypesForInheritance, std::vector<std::shared_ptr<SampleTime>> multirateSampleTimes,
                        int inputMultirateSampleTimeIndex = -1, int outputMultirateSampleTimeIndex = -1);

            SampleTime(SampleTimeType sampleTimeType) : SampleTime(sampleTimeType, std::numeric_limits<double>::quiet_NaN(), -1, std::vector<SampleTimeType>{}, {}){}
            SampleTime(SampleTimeType sampleTimeType, double discreteSampleTime) : SampleTime(sampleTimeType, discreteSampleTime, -1, std::vector<SampleTimeType>{}, {}){}
            SampleTime(SampleTimeType sampleTimeType, int continuousSampleTimeGroup) : SampleTime(sampleTimeType, std::numeric_limits<double>::quiet_NaN(), continuousSampleTimeGroup, std::vector<SampleTimeType>{}, {}){}
            SampleTime(SampleTimeType sampleTimeType, std::vector<SampleTimeType> supportedSampleTimeTypesForInheritance) : SampleTime(sampleTimeType, std::numeric_limits<double>::quiet_NaN(), -1, supportedSampleTimeTypesForInheritance, {}){}
            SampleTime(SampleTimeType sampleTimeType, std::vector<std::shared_ptr<SampleTime>> multirateSampleTimes) : SampleTime(sampleTimeType, std::numeric_limits<double>::quiet_NaN(), -1, {}, multirateSampleTimes){}
            SampleTime(SampleTimeType sampleTimeType, std::vector<std::shared_ptr<SampleTime>> multirateSampleTimes, int inputMultirateSampleTimeIndex, int outputMultirateSampleTimeIndex)
            : SampleTime(sampleTimeType, std::numeric_limits<double>::quiet_NaN(), -1, {}, multirateSampleTimes, inputMultirateSampleTimeIndex, outputMultirateSampleTimeIndex){}

            const SampleTimeType& GetSampleTimeType() const;
            const double GetDiscreteSampleTime() const;
            const int GetContinuousSampleTimeGroup() const;
            const std::vector<SampleTimeType> GetSupportedSampleTimeTypesForInheritance() const;
            const std::vector<std::shared_ptr<SampleTime>> GetMultirateSampleTimes() const;
            const void SetMultirateSampleTimeInIndex(std::shared_ptr<SampleTime> multirateSampleTime, int index);
            const bool HasMultirateInheritedSampleTime() const;
            const int GetInputMultirateSampleTimeIndex() const;
            const int GetOutputMultirateSampleTimeIndex() const;
            const bool IsInputMultirateInherited() const;
            const bool IsOutputMultirateInherited() const;

            static std::string SampleTimeTypeString(SampleTimeType sampleTimeType);
    };
}

// End header: SampleTime.h

// Begin header: IBlockEventsHandler.h
#define SRC_IBLOCK_EVENTS_HANDLER

#include "BlockEvents/BlockEvent.h"
#include "BlockEvents/ValueUpdateBlockEvent.h"
#include <memory>
#include <functional>

namespace PySysLinkBase
{
    class IBlockEventsHandler
    {
        public:
        virtual ~IBlockEventsHandler() = default;

        virtual void BlockEventCallback(const std::shared_ptr<BlockEvent> blockEvent) const = 0;
        virtual void RegisterValueUpdateBlockEventCallback(std::function<void (std::shared_ptr<ValueUpdateBlockEvent>)> callback) = 0;

    };
} // namespace PySysLinkBase



// End header: IBlockEventsHandler.h

// Begin header: UnknownTypeSignalValue.h
#define PY_SYS_LINK_BASE_PORTS_AND_SIGNAL_VALUES_UNKNOWN_TYPE_SIGNAL_VALUE

#include <string>
#include <memory>
#include <stdexcept>

namespace PySysLinkBase
{
    template <typename T>
    class SignalValue; // Forward declaration

    class UnknownTypeSignalValue
    {

        public:

            virtual ~UnknownTypeSignalValue() = default;
            virtual const std::string GetTypeId() const = 0;

            template <typename T>
            std::unique_ptr<SignalValue<T>> TryCastToTyped()
            {
                SignalValue<T>* typedPtr = dynamic_cast<SignalValue<T>*>(this);

                if (!typedPtr) throw std::bad_cast();

                return std::make_unique<SignalValue<T>>(*typedPtr);
            }

            virtual std::unique_ptr<UnknownTypeSignalValue> clone() const = 0;
    };
} // namespace PySysLinkBase

// End header: UnknownTypeSignalValue.h

// Begin header: ConfigurationValue.h
#define SRC_CONFIGURATION_VALUE

#include <string>
#include <variant>
#include <vector>
#include <memory>
#include <map>
#include <stdexcept>
#include <complex>

namespace PySysLinkBase
{
    using ConfigurationValuePrimitive  = std::variant<
        int,
        double,
        bool,
        std::complex<double>,
        std::string,
        std::vector<int>,
        std::vector<double>,
        std::vector<bool>,
        std::vector<std::complex<double>>,
        std::vector<std::string>>;

    using ConfigurationValue = std::variant<
        int,
        double,
        bool,
        std::complex<double>,
        std::string,
        std::vector<int>,
        std::vector<double>,
        std::vector<bool>,
        std::vector<std::complex<double>>,
        std::vector<std::string>,
        ConfigurationValuePrimitive,
        std::vector<ConfigurationValuePrimitive>>;

    class ConfigurationValueManager
    {
        public:
        template <typename T>
        static T TryGetConfigurationValue(std::string keyName, std::map<std::string, ConfigurationValue> configurationValues)
        {
            ConfigurationValue foundValue;
            auto it = configurationValues.find(keyName);
            if (it == configurationValues.end()) {
                throw std::out_of_range("Key name: " + keyName + " not found in configuration.");
            } else {
                foundValue = it->second;
            }
            try
            {
                return std::get<T>(foundValue);
            }
            catch (std::bad_variant_access const& ex)
            {
                throw std::invalid_argument("Configuration option of key: " + keyName + " was not of expected type: " + ex.what());
            }
        }
    };
} // namespace PySysLinkBase



// End header: ConfigurationValue.h

// Begin header: IOdeStepSolver.h
#define SRC_CONTINUOUS_AND_ODE_IODE_STEP_SOLVER

#include <tuple>
#include <vector>
#include <functional>
#include <stdexcept>

namespace PySysLinkBase
{
    class IOdeStepSolver
    {
        public:
            virtual bool IsJacobianNeeded() const
            {
                return false;
            }
            virtual std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> system,
                                                                    std::vector<double> states_0, double currentTime, double timeStep) = 0;
            virtual std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> systemDerivatives,
                                                                    std::function<std::vector<std::vector<double>>(std::vector<double>, double)> systemJacobian,
                                                                    std::vector<double> states_0, double currentTime, double timeStep)
            {
                throw std::runtime_error("Jacobian not needed");
            }
    };
} // namespace PySysLinkBase


// End header: IOdeStepSolver.h

// Begin header: SignalValue.h
#define SRC_PY_SYS_LINK_BASE_PORTS_AND_SIGNAL_VALUES_SIGNAL_VALUE

#include <string>
#include "UnknownTypeSignalValue.h"
#include <memory>

namespace PySysLinkBase
{
    template <typename T>
    class SignalValue : public UnknownTypeSignalValue
    {
        private:
            T payload;
        public:
            SignalValue(T initialPayload) : payload(initialPayload) {}

            SignalValue(const SignalValue& other) = default;

            std::unique_ptr<UnknownTypeSignalValue> clone() const override {
                return std::make_unique<SignalValue<T>>(*this);
            }

            const std::string GetTypeId() const
            {
                return std::to_string(typeid(T).hash_code()) + typeid(T).name();
            }

            const T GetPayload() const
            {
                return this->payload;
            }

            void SetPayload(T newPayload)
            {
                this->payload = newPayload;
            }
    };
} // namespace PySysLinkBase


// End header: SignalValue.h

// Begin header: SimulationOptions.h
#define SRC_SIMULATION_OPTIONS

#include <vector>
#include <utility>
#include <string>
#include <tuple>
#include "ConfigurationValue.h"

namespace PySysLinkBase
{
    class SimulationOptions
    {
        public:
        SimulationOptions() = default;

        double startTime;
        double stopTime;

        bool runInNaturalTime = false;
        double naturalTimeSpeedMultiplier = 1.0;

        std::vector<std::tuple<std::string, std::string, int>> blockIdsInputOrOutputAndIndexesToLog = {};

        std::map<std::string, std::map<std::string, ConfigurationValue>> solversConfiguration;

        std::string hdf5FileName = "";
        bool saveToFileContinuously = false;

        bool saveToVectors = true;
    };
} // namespace PySysLinkBase


// End header: SimulationOptions.h

// Begin header: OdeintStepSolver.h
#define SRC_CONTINUOUS_AND_ODE_ODEINT_STEP_SOLVER


#include <tuple>
#include <vector>
#include <functional>
#include "IOdeStepSolver.h"
#include <boost/numeric/odeint.hpp>

namespace PySysLinkBase
{
    template <typename T>
    class OdeintStepSolver : public IOdeStepSolver
    {
        private:
            T controlledStepper;
        public:
            OdeintStepSolver(T controlledStepper)
            {
                this->controlledStepper = controlledStepper;
            }
            std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> system,
                                                                    std::vector<double> states_0, double currentTime, double timeStep)
            {
                // Define the system function in the format expected by ODEINT
                auto systemFunction = [&system](const std::vector<double> &x, std::vector<double> &dxdt, double t) {
                    std::vector<double> gradient = system(x, t);
                    dxdt = gradient; // Assign the computed derivative
                };


                // Create the stepper
                // Stepper stepper;

                // Integrate a single step
                std::vector<double> newStates = states_0; // Initial state
                double dt = timeStep;

                boost::numeric::odeint::controlled_step_result result = this->controlledStepper.try_step(systemFunction, newStates, currentTime, dt);
                // controlled_step_result result = stepper.try_step(systemFunction, newStates, currentTime, dt);

                system(states_0, currentTime); // Set initial states again, may be optimized

                // Debug log output
                if (result == boost::numeric::odeint::success)
                {
                    return {true, newStates, dt};
                }
                else
                {
                    return {false, newStates, dt};
                }
            }
    };
} // namespace PySysLinkBase

// End header: OdeintStepSolver.h

// Begin header: SimulationOutput.h
#define SRC_SIMULATION_OUTPUT

#include <vector>
#include <memory>
#include <map>
#include <string>
#include <unordered_map>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <typeinfo>
#include <typeindex>

#include "PortsAndSignalValues/UnknownTypeSignalValue.h"
#include "FullySupportedSignalValue.h"

namespace PySysLinkBase
{
    static std::string escapeJson(const std::string& s) {
        std::ostringstream o;
        for (char c : s) {
            switch (c) {
                case '\"': o << "\\\""; break;
                case '\\': o << "\\\\"; break;
                case '\b': o << "\\b";  break;
                case '\f': o << "\\f";  break;
                case '\n': o << "\\n";  break;
                case '\r': o << "\\r";  break;
                case '\t': o << "\\t";  break;
                default:
                    if (static_cast<unsigned char>(c) < 0x20) {
                        // control characters → \u00XX
                        o << "\\u"
                          << std::hex << std::setw(4) << std::setfill('0')
                          << (int)(unsigned char)c;
                    } else {
                        o << c;
                    }
            }
        }
        return o.str();
    }

    template <typename T>
    class Signal; // Forward declaration

    template <typename T>
    class Signal; // Forward declaration

    class UnknownTypeSignal
    {
    public:
        virtual ~UnknownTypeSignal() = default;
        std::string id;
        std::vector<double> times;

        // Reserve capacity upfront
        UnknownTypeSignal() {
            times.reserve(4096); // Pre-allocate memory
        }

        virtual const std::string GetTypeId() const = 0;

        template <typename T>
        std::unique_ptr<Signal<T>> TryCastToTyped()
        {
            Signal<T>* typedPtr = dynamic_cast<Signal<T>*>(this);
            if (!typedPtr) throw std::bad_cast();
            return std::make_unique<Signal<T>>(*typedPtr);
        }

        template <typename T>
        void TryInsertValue(double time, T value)
        {
            Signal<T>* typedPtr = dynamic_cast<Signal<T>*>(this);
            if (!typedPtr) throw std::bad_cast();

            typedPtr->times.push_back(time);
            typedPtr->values.push_back(std::move(value)); // Use move semantics
        }
    };

    template <typename T>
    class Signal : public UnknownTypeSignal
    {
    public:
        std::vector<T> values;

        Signal() {
            values.reserve(4096); // Pre-allocate memory
        }

        // Cache type ID to avoid repeated allocations
        const std::string GetTypeId() const override {
            const std::string typeId =
                std::to_string(typeid(T).hash_code()) + typeid(T).name();
            return typeId;
        }
    };

    struct WriteTask {
        std::string datasetPath;
        int currentIndex = 0;
        std::vector<double> times;
        std::vector<std::shared_ptr<FullySupportedSignalValue>> values;

        WriteTask() : times(1024, 0.0), values(1024, nullptr) {}
    };

    template<typename Task>
    class TaskQueue {
    std::queue<Task> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool done_ = false;
    public:
    void push(Task const& t) {
        {
        std::lock_guard<std::mutex> lk(mutex_);
        queue_.push(t);
        }
        cv_.notify_one();
    }
    // Consumer pop; returns false if shutting down and queue empty
    bool pop(Task& out) {
        std::unique_lock<std::mutex> lk(mutex_);
        cv_.wait(lk, [&]{ return done_ || !queue_.empty(); });
        if (queue_.empty()) return false;
        out = std::move(queue_.front());
        queue_.pop();
        return true;
    }
    void shutdown() {
        {
        std::lock_guard<std::mutex> lk(mutex_);
        done_ = true;
        }
        cv_.notify_all();
    }
    };

    class SimulationOutput
    {
    private:
        std::thread ioThread;
        TaskQueue<WriteTask> taskQueue;
        bool saveToVectors;
        bool saveToFileContinuously;
        std::string hdf5FileName;
        std::shared_ptr<void> hdf5File; // opaque pointer, actual type in .cpp
        void* dumpOptions; // opaque pointer, actual type in .cpp

        std::unordered_map<std::string, std::size_t> lastFlushedIndex;
        std::unordered_map<std::string, WriteTask> writeTasks;

    public:
        SimulationOutput(bool saveToVectors=true, bool saveToFileContinuously=false, std::string hdf5FileName="");
        ~SimulationOutput();

        std::map<std::string, std::map<std::string, std::shared_ptr<UnknownTypeSignal>>> signals;

        template<typename T>
        void InsertValueTyped(const std::string& signalType, const std::string& signalId, T value, double currentTime);

        void InsertUnknownValue(
            const std::string& signalType,
            const std::string& signalId,
            const std::shared_ptr<PySysLinkBase::UnknownTypeSignalValue>& value,
            double currentTime);

        void InsertFullySupportedValue(
            const std::string& signalType,
            const std::string& signalId,
            const FullySupportedSignalValue& value,
            double currentTime);

        void WriteJson(const std::string& filename) const;
    };

    template<typename T>
    void SimulationOutput::InsertValueTyped(const std::string& signalType, const std::string& signalId, T value, double currentTime)
    {
        auto& signalMap = signals[signalType];
        auto& signalPtr = signalMap[signalId];

        if (!signalPtr) {
            // Use make_shared directly with concrete type
            signalPtr = std::make_shared<Signal<T>>();
            signalPtr->id = signalId;
        }

        // Add the value
        signalPtr->times.push_back(currentTime);
        static_cast<Signal<T>*>(signalPtr.get())->values.push_back(std::move(value));
    }
} // namespace PySysLinkBase


// End header: SimulationOutput.h

// Begin header: EulerForwardStepSolver.h
#define SRC_EULER_FORWARD_STEP_SOLVER


#include <tuple>
#include <vector>
#include <functional>
#include "IOdeStepSolver.h"

namespace PySysLinkBase
{
    class EulerForwardStepSolver : public IOdeStepSolver
    {
        public:
            virtual std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> system,
                                                                    std::vector<double> states_0, double currentTime, double timeStep);
            virtual bool IsJacobianNeeded() const
            {
                return false;
            }
            virtual std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> systemDerivatives,
                                                                    std::function<std::vector<std::vector<double>>(std::vector<double>, double)> systemJacobian,
                                                                    std::vector<double> states_0, double currentTime, double timeStep)
            {
                throw std::runtime_error("Jacobian not needed");
            }
    };
} // namespace PySysLinkBase

// End header: EulerForwardStepSolver.h

// Begin header: ValueUpdateBlockEvent.h
#define SRC_BLOCK_EVENTS_VALUE_UPDATE_BLOCK_EVENT

#include <string>
#include "../FullySupportedSignalValue.h"
#include "BlockEvent.h"

namespace PySysLinkBase
{
    class ValueUpdateBlockEvent : public BlockEvent
    {
        public:
        double simulationTime;
        std::string valueId;
        FullySupportedSignalValue value;
        ValueUpdateBlockEvent(double simulationTime, std::string valueId, FullySupportedSignalValue value) : simulationTime(simulationTime), valueId(valueId), value(value), BlockEvent("ValueUpdate") {}

        ~ValueUpdateBlockEvent() = default;
    };
} // namespace PySysLinkBase
// End header: ValueUpdateBlockEvent.h

// Begin header: SolverFactory.h
#define SRC_CONTINUOUS_AND_ODE_SOLVER_FACTORY

#include <memory>
#include "IOdeStepSolver.h"
#include <map>
#include "../ConfigurationValue.h"

namespace PySysLinkBase
{
    class SolverFactory
    {
        public:
            static std::shared_ptr<IOdeStepSolver> CreateOdeStepSolver(std::map<std::string, ConfigurationValue> solverConfiguration);
    };
} // namespace PySysLinkBase


// End header: SolverFactory.h

// Begin header: EulerBackwardStepSolver.h
#define SRC_EULER_BACKWARD_STEP_SOLVER


#include <tuple>
#include <vector>
#include <functional>
#include <sstream>
#include "IOdeStepSolver.h"
#include <cmath>
#include <stdexcept>
#include <iostream>

namespace PySysLinkBase
{
    class EulerBackwardStepSolver : public IOdeStepSolver
    {
        public:
            EulerBackwardStepSolver(double maximumIterations = 50, double tolerance = 1e-6)
                : maximumIterations(maximumIterations), tolerance(tolerance)
            {
            };
            virtual std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> system,
                                                                    std::vector<double> states_0, double currentTime, double timeStep)
            {
                throw std::runtime_error("Jacobian needed for implicit Euler method");
            }
            virtual std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> systemDerivatives,
                                                                    std::function<std::vector<std::vector<double>>(std::vector<double>, double)> systemJacobian,
                                                                    std::vector<double> states_0, double currentTime, double timeStep);
            virtual bool IsJacobianNeeded() const
            {
                return true;
            }

        private:
            double maximumIterations;
            double tolerance;
            std::vector<double> ComputeNewtonStep(const std::vector<std::vector<double>>& systemJacobianEnd,
                const std::vector<double>& systemDerivativesEnd, const std::vector<double>& states_0, const std::vector<double>& statesEnd,
                 double timeStep);
    };
} // namespace PySysLinkBase

// End header: EulerBackwardStepSolver.h

// Begin header: BlockEventsHandler.h
#define SRC_BLOCK_EVENTS_HANDLER

#include "BlockEvents/BlockEvent.h"
#include "BlockEvents/ValueUpdateBlockEvent.h"
#include "IBlockEventsHandler.h"
#include <memory>
#include <functional>

namespace PySysLinkBase
{
    class BlockEventsHandler : public IBlockEventsHandler
    {
        private:
        std::vector<std::function<void (std::shared_ptr<ValueUpdateBlockEvent>)>> valueUpdateBlockEventCallbacks;

        public:

        BlockEventsHandler();

        void BlockEventCallback(const std::shared_ptr<BlockEvent> blockEvent) const;

        void RegisterValueUpdateBlockEventCallback(std::function<void (std::shared_ptr<ValueUpdateBlockEvent>)> callback);
    };
} // namespace PySysLinkBase

// End header: BlockEventsHandler.h

// Begin header: ISimulationBlock.h
#define SRC_ISIMULATION_BLOCK

#include <string>
#include <vector>
#include <memory>
#include "PortsAndSignalValues/InputPort.h"
#include "PortsAndSignalValues/OutputPort.h"
#include "SampleTime.h"
#include <stdexcept>
#include <map>
#include "ConfigurationValue.h"
#include "BlockEvents/BlockEvent.h"
#include "IBlockEventsHandler.h"

namespace PySysLinkBase
{
    class ISimulationBlock {
    protected:
        std::shared_ptr<IBlockEventsHandler> blockEventsHandler;

        std::string name;
        std::string id;

        std::vector<std::function<void (const std::string, const std::vector<std::shared_ptr<PySysLinkBase::InputPort>>, std::shared_ptr<PySysLinkBase::SampleTime>, double)>> readInputCallbacks;
        std::vector<std::function<void (const std::string, const std::vector<std::shared_ptr<PySysLinkBase::OutputPort>>, std::shared_ptr<PySysLinkBase::SampleTime>, double)>> calculateOutputCallbacks;
        std::vector<std::function<void (const std::string, const std::string, const ConfigurationValue)>> updateConfigurationValueCallbacks;
    public:
        const std::string GetId() const;
        const std::string GetName() const;

        ISimulationBlock(std::map<std::string, ConfigurationValue> blockConfiguration, std::shared_ptr<IBlockEventsHandler> blockEventsHandler);
        virtual ~ISimulationBlock() = default;

        const virtual std::shared_ptr<SampleTime> GetSampleTime() const = 0;
        virtual void SetSampleTime(std::shared_ptr<SampleTime> sampleTime) = 0;

        virtual std::vector<std::shared_ptr<PySysLinkBase::InputPort>> GetInputPorts() const = 0;
        virtual const std::vector<std::shared_ptr<PySysLinkBase::OutputPort>> GetOutputPorts() const = 0;

        const std::vector<std::shared_ptr<PySysLinkBase::OutputPort>> ComputeOutputsOfBlock(const std::shared_ptr<PySysLinkBase::SampleTime> sampleTime, double currentTime, bool isMinorStep=false);
        virtual const std::vector<std::shared_ptr<PySysLinkBase::OutputPort>> _ComputeOutputsOfBlock(const std::shared_ptr<PySysLinkBase::SampleTime> sampleTime, double currentTime, bool isMinorStep=false) = 0;

        bool IsBlockFreeSource() const;
        bool IsInputDirectBlockChainEnd(int inputIndex) const;

        void NotifyEvent(std::shared_ptr<PySysLinkBase::BlockEvent> blockEvent) const;
        bool TryUpdateConfigurationValue(std::string keyName, ConfigurationValue value);
        virtual bool _TryUpdateConfigurationValue(std::string keyName, ConfigurationValue value) = 0;

        static std::shared_ptr<ISimulationBlock> FindBlockById(std::string id, const std::vector<std::shared_ptr<ISimulationBlock>>& blocksToFind);

        void RegisterReadInputsCallbacks(std::function<void (const std::string, const std::vector<std::shared_ptr<PySysLinkBase::InputPort>>, std::shared_ptr<PySysLinkBase::SampleTime>, double)> callback);
        void RegisterCalculateOutputCallbacks(std::function<void (const std::string, const std::vector<std::shared_ptr<PySysLinkBase::OutputPort>>, std::shared_ptr<PySysLinkBase::SampleTime>, double)> callback);
        void RegisterUpdateConfigurationValueCallbacks(std::function<void (const std::string, const std::string, const ConfigurationValue)> callback);

        virtual const std::vector<std::pair<double, double>> GetEvents(const std::shared_ptr<PySysLinkBase::SampleTime> sampleTime, double eventTime, std::vector<double> eventTimeStates, bool includeKnownEvents=false) const
        {
            return {};
        }

        virtual const std::vector<double> GetKnownEvents(const std::shared_ptr<PySysLinkBase::SampleTime> resolvedSampleTime, double simulationStartTime, double simulationEndTime) const
        {
            return {};
        }
    };
}

// End header: ISimulationBlock.h

// Begin header: OdeintImplicitStepSolver.h
#define SRC_CONTINUOUS_AND_ODE_ODEINT_IMPLICIT_STEP_SOLVER


#include <tuple>
#include <vector>
#include <functional>
#include "IOdeStepSolver.h"
#include <boost/numeric/odeint.hpp>
#include <utility>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

namespace PySysLinkBase
{
    template <typename T>
    class OdeintImplicitStepSolver : public IOdeStepSolver
    {
        private:
            std::shared_ptr<T> controlledStepper;

            // Convert std::vector<double> to ublas::vector<double>
            static boost::numeric::ublas::vector<double> stdToUblas(const std::vector<double>& v) {
                boost::numeric::ublas::vector<double> u(v.size());
                for (size_t i = 0; i < v.size(); ++i) {
                    u(i) = v[i];
                }
                return u;
            }

            // Convert ublas::vector<double> to std::vector<double>
            static std::vector<double> ublasToStd(const boost::numeric::ublas::vector<double>& u) {
                std::vector<double> v(u.size());
                for (size_t i = 0; i < u.size(); ++i) {
                    v[i] = u(i);
                }
                return v;
            }

            boost::numeric::ublas::matrix<double> stdToUblasMatrix(const std::vector<std::vector<double>>& mat) {
                size_t rows = mat.size();
                size_t cols = (rows > 0 ? mat[0].size() : 0);
                boost::numeric::ublas::matrix<double> m(rows, cols);
                for (size_t i = 0; i < rows; ++i) {
                    for (size_t j = 0; j < cols; ++j) {
                        m(i, j) = mat[i][j];
                    }
                }
                return m;
            }

        public:
            OdeintImplicitStepSolver(std::shared_ptr<T> controlledStepper)
            {
                this->controlledStepper = controlledStepper;
            }

            virtual bool IsJacobianNeeded() const
            {
                return true;
            }

            virtual std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> system,
                                                                    std::vector<double> states_0, double currentTime, double timeStep)
            {
                throw std::runtime_error("Jacobian needed for implicit odeint method");
            }




            std::tuple<bool, std::vector<double>, double> SolveStep(std::function<std::vector<double>(std::vector<double>, double)> system,
                                                                    std::function<std::vector<std::vector<double>>(std::vector<double>, double)> systemJacobian,
                                                                    std::vector<double> states_0, double currentTime, double timeStep)
            {
                auto systemFunction = [&](const boost::numeric::ublas::vector<double>& x_ublas, boost::numeric::ublas::vector<double>& dxdt_ublas, double t) -> void {
                    std::vector<double> x_std = OdeintImplicitStepSolver::ublasToStd(x_ublas);
                    std::vector<double> dxdt_std = system(x_std, t);
                    if(dxdt_std.size() != static_cast<size_t>(dxdt_ublas.size())) {
                        dxdt_ublas = boost::numeric::ublas::vector<double>(dxdt_std.size());
                    }
                    for (size_t i = 0; i < dxdt_std.size(); ++i) {
                        dxdt_ublas(i) = dxdt_std[i];
                    }
                };

                auto systemJacobianFunction = [&](const boost::numeric::ublas::vector<double>& x,
                    boost::numeric::ublas::matrix<double>& J,
                    double t,
                    boost::numeric::ublas::vector<double>& dfdt) -> void {
                    std::vector<double> x_std = OdeintImplicitStepSolver::ublasToStd(x);
                    std::vector<std::vector<double>> J_std = systemJacobian(x_std, t);
                    // Here you need to compute dfdt (the time derivative of f) or set it to zero if appropriate.
                    // For example, if dfdt is not provided by your original function, you might choose:
                    dfdt = boost::numeric::ublas::vector<double>(x_std.size(), 0.0);
                    J = OdeintImplicitStepSolver::stdToUblasMatrix(J_std);
                };

                typedef boost::numeric::ublas::vector<double> state_type;

                // Assuming state_type is boost::numeric::ublas::vector<double>
                std::function<void(const state_type&, state_type&, double)> boundSystem = systemFunction;
                std::function<void(const state_type&, boost::numeric::ublas::matrix<double>&, double, state_type&)> boundJacobian = systemJacobianFunction;

                // Create the stepper
                // Stepper stepper;

                // Integrate a single step
                std::vector<double> newStates = states_0; // Initial state
                double dt = timeStep;

                boost::numeric::ublas::vector<double> newStates_ublas(newStates.size());
                for (std::size_t i = 0; i < newStates.size(); ++i) {
                    newStates_ublas(i) = newStates[i];
                }

                boost::numeric::odeint::controlled_step_result result = this->controlledStepper->try_step(std::make_pair( boundSystem, boundJacobian ), newStates_ublas, currentTime, dt);

                newStates = OdeintImplicitStepSolver::ublasToStd(newStates_ublas);

                system(states_0, currentTime); // Set initial states again, may be optimized

                // Debug log output
                if (result == boost::numeric::odeint::success)
                {
                    return {true, newStates, dt};
                }
                else
                {
                    return {false, newStates, dt};
                }
            }
    };
} // namespace PySysLinkBase

// End header: OdeintImplicitStepSolver.h

// Begin header: Port.h
#define SRC_PORTS_AND_SIGNAL_VALUES_PORT

#include <string>
#include "UnknownTypeSignalValue.h"
#include <memory>
#include <functional>


namespace PySysLinkBase
{
    class ISimulationBlock;

    class Port {
    protected:
        std::shared_ptr<UnknownTypeSignalValue> value;

    public:
        Port(std::shared_ptr<UnknownTypeSignalValue> value);

        void TryCopyValueToPort(Port& otherPort) const;

        void SetValue(std::shared_ptr<UnknownTypeSignalValue> value);
        std::shared_ptr<UnknownTypeSignalValue> GetValue() const;

        bool operator==(const Port& rhs) const
        {
            return this == &rhs;
        }
    };
}

// End header: Port.h

// Begin header: PortLink.h
#define SRC_PY_SYS_LINK_BASE_PORT_LINK

#include "ISimulationBlock.h"
#include "PortsAndSignalValues/InputPort.h"
#include "PortsAndSignalValues/OutputPort.h"
#include <algorithm>
#include <map>
#include "ConfigurationValue.h"

namespace PySysLinkBase
{
    class PortLink
    {
    public:
        PortLink(std::shared_ptr<ISimulationBlock> originBlock,
                std::shared_ptr<ISimulationBlock> sinkBlock,
                int originBlockPortIndex,
                int sinkBlockPortIndex) : originBlock(originBlock), sinkBlock(sinkBlock),
                                          originBlockPortIndex(originBlockPortIndex), sinkBlockPortIndex(sinkBlockPortIndex) {}

        std::shared_ptr<ISimulationBlock> originBlock;
        std::shared_ptr<ISimulationBlock> sinkBlock;
        int originBlockPortIndex;
        int sinkBlockPortIndex;

        static PortLink ParseFromConfig(std::map<std::string, ConfigurationValue> linkConfiguration, const std::vector<std::shared_ptr<ISimulationBlock>>& blocks);
    };
}

// End header: PortLink.h

// Begin header: InputPort.h
#define SRC_PORTS_AND_SIGNAL_VALUES_INPUT_PORT

#include "Port.h"

namespace PySysLinkBase
{
    class InputPort : public Port {
    private:
        bool hasDirectFeedthrough;
    public:
        InputPort(bool hasDirectFeedthrough, std::shared_ptr<UnknownTypeSignalValue> value);
        const bool HasDirectFeedthrough() const;
    };
}

// End header: InputPort.h

// Begin header: IBlockFactory.h
#define SRC_PY_SYS_LINK_BASE_IBLOCK_FACTORY

#include "ISimulationBlock.h"
#include <vector>
#include "ConfigurationValue.h"
#include <map>
#include <memory>
#include <string>
#include "IBlockEventsHandler.h"

namespace PySysLinkBase
{
   class IBlockFactory {
      public:
         virtual ~IBlockFactory() = default;
         virtual std::shared_ptr<ISimulationBlock> CreateBlock(std::map<std::string, ConfigurationValue> blockConfiguration, std::shared_ptr<IBlockEventsHandler> blockEventsHandler) = 0;
   };
}

// End header: IBlockFactory.h

// Begin header: OutputPort.h
#define SRC_PORTS_AND_SIGNAL_VALUES_OUTPUT_PORT


#include "Port.h"

namespace PySysLinkBase
{
    class OutputPort : public Port {
        public:
            OutputPort(std::shared_ptr<UnknownTypeSignalValue> value);
    };
}

// End header: OutputPort.h

// Begin header: ISimulationBlockWithContinuousStates.h
#define PYSYSLINK_BASE_SRC_CPP_LIBRARIES_PY_SYS_LINK_BASE_SRC_CONTINUOUS_AND_ODE_ISIMULATION_BLOCK_WITH_CONTINUOUS_STATES

#include "../ISimulationBlock.h"
#include <vector>
#include <memory>
#include <stdexcept>
#include <utility>

namespace PySysLinkBase
{
    class ISimulationBlockWithContinuousStates : public ISimulationBlock
    {
        public:
            ISimulationBlockWithContinuousStates(std::map<std::string, ConfigurationValue> blockConfiguration, std::shared_ptr<IBlockEventsHandler> blockEventsHandler)
                                                : ISimulationBlock(blockConfiguration, blockEventsHandler) {}

            virtual ~ISimulationBlockWithContinuousStates() = default;

            virtual const std::vector<double> GetContinuousStates() const = 0;
            virtual void SetContinuousStates(std::vector<double> newStates) = 0;

            virtual const std::vector<double> GetContinuousStateDerivatives(const std::shared_ptr<PySysLinkBase::SampleTime> sampleTime, double currentTime) const = 0;
    };
} // namespace PySysLinkBase


// End header: ISimulationBlockWithContinuousStates.h

// Begin header: SimulationModel.h
#define SRC_SIMULATION_MODEL

#include <vector>
#include "ISimulationBlock.h"
#include "PortLink.h"
#include "PortsAndSignalValues/InputPort.h"
#include "PortsAndSignalValues/OutputPort.h"
#include <optional>
#include "IBlockEventsHandler.h"

namespace PySysLinkBase
{
    class SimulationModel
    {
    public:
        std::vector<std::shared_ptr<ISimulationBlock>> simulationBlocks;
        std::vector<std::shared_ptr<PortLink>> portLinks;
        std::shared_ptr<IBlockEventsHandler> blockEventsHandler;

        SimulationModel(std::vector<std::shared_ptr<ISimulationBlock>> simulationBlocks, std::vector<std::shared_ptr<PortLink>> portLinks, std::shared_ptr<IBlockEventsHandler> blockEventsHandler);

        const std::vector<std::shared_ptr<InputPort>> GetConnectedPorts(const std::shared_ptr<ISimulationBlock> originBlock, int outputPortIndex) const;
        const std::pair<std::vector<std::shared_ptr<ISimulationBlock>>, std::vector<int>> GetConnectedBlocks(const std::shared_ptr<ISimulationBlock> originBlock, int outputPortIndex) const;
        const std::shared_ptr<ISimulationBlock> GetOriginBlock(const std::shared_ptr<ISimulationBlock> sinkBlock, int inputPortIndex) const;

        const std::vector<std::vector<std::shared_ptr<ISimulationBlock>>> GetDirectBlockChains();

        const std::vector<std::shared_ptr<ISimulationBlock>> OrderBlockChainsOntoFreeOrder(const std::vector<std::vector<std::shared_ptr<ISimulationBlock>>> directBlockChains);

        void PropagateSampleTimes();

    private:
        const std::vector<std::shared_ptr<ISimulationBlock>> GetFreeSourceBlocks();

        std::vector<std::vector<std::shared_ptr<ISimulationBlock>>> GetDirectBlockChainsOfSourceBlock(std::shared_ptr<ISimulationBlock> freeSourceBlock);

        void FindChains(std::shared_ptr<ISimulationBlock> currentBlock, std::vector<std::shared_ptr<ISimulationBlock>> currentChain, std::vector<std::vector<std::shared_ptr<ISimulationBlock>>>& resultChains);
    };
}

// End header: SimulationModel.h

// Begin header: BlockTypeSupportPluginLoader.h
#define SRC_PY_SYS_LINK_BASE_BLOCK_TYPE_SUPPORT_PLUGING_LOADER

#include <map>
#include <memory>
#include <string>
#include <dlfcn.h> // For Linux/macOS dynamic linking. Use `windows.h` for Windows.

#include "IBlockFactory.h"

namespace PySysLinkBase {

class BlockTypeSupportPluginLoader {
public:
    std::map<std::string, std::shared_ptr<IBlockFactory>> LoadPlugins(const std::string& pluginDirectory);

private:
    std::vector<std::string> FindSharedLibraries(const std::string& pluginDirectory);
    bool StringEndsWith(const std::string& str, const std::string& suffix);
};

} // namespace PySysLinkBase

// End header: BlockTypeSupportPluginLoader.h

// Begin header: ModelParser.h
#define SRC_PY_SYS_LINK_BASE_MODEL_PARSER

#include "SimulationModel.h"
#include <string>
#include "ConfigurationValue.h"
#include <yaml-cpp/yaml.h>
#include <map>
#include "IBlockFactory.h"

namespace PySysLinkBase
{
    class ModelParser
    {
        private:
            static std::vector<std::shared_ptr<PortLink>> ParseLinks(std::vector<std::map<std::string, ConfigurationValue>> linksConfigurations, const std::vector<std::shared_ptr<ISimulationBlock>>& blocks);
            static std::vector<std::shared_ptr<ISimulationBlock>> ParseBlocks(std::vector<std::map<std::string, ConfigurationValue>> blocksConfigurations, const std::map<std::string, std::shared_ptr<IBlockFactory>>& blockFactories, std::shared_ptr<IBlockEventsHandler> blockEventsHandler);
            static std::complex<double> ParseComplex(const std::string& str);

        public:
            static ConfigurationValue YamlToConfigurationValue(const YAML::Node& node);
            static std::shared_ptr<SimulationModel> ParseFromYaml(std::string filename, const std::map<std::string, std::shared_ptr<IBlockFactory>>& blockFactories, std::shared_ptr<IBlockEventsHandler> blockEventsHandler);
    };
} // namespace PySysLinkBase


// End header: ModelParser.h

// Begin header: BasicOdeSolver.h
#define SRC_BASIC_ODE_SOLVER

#include "IOdeStepSolver.h"
#include "ISimulationBlockWithContinuousStates.h"
#include "../SimulationModel.h"
#include <memory>
#include <vector>
#include "../SimulationOptions.h"

namespace PySysLinkBase
{

    class BasicOdeSolver
    {
        private:
            std::shared_ptr<IOdeStepSolver> odeStepSolver;
            std::shared_ptr<SimulationModel> simulationModel;
            std::vector<std::shared_ptr<ISimulationBlock>> simulationBlocks;
            std::vector<int> continuousStatesInEachBlock;
            int totalStates;

            std::vector<double> knownTimeHits = {};
            int currentKnownTimeHit = 0;
            double nextUnknownTimeHit;
            double nextSuggestedTimeStep;
            std::vector<double> nextTimeHitStates;

            std::shared_ptr<SampleTime> sampleTime;

            void ComputeBlockOutputs(std::shared_ptr<ISimulationBlock> block, std::shared_ptr<SampleTime> sampleTime, double currentTime, bool isMinorStep=false);
            void ComputeMinorOutputs(std::shared_ptr<SampleTime> sampleTime, double currentTime);
            std::vector<double> GetDerivatives(std::shared_ptr<SampleTime> sampleTime, double currentTime);
            std::vector<std::vector<double>> GetJacobian(std::shared_ptr<SampleTime> sampleTime, double currentTime);
            void SetStates(std::vector<double> newStates);
            std::vector<double> GetStates();

            std::tuple<bool, std::vector<double>, double> OdeStepSolverStep(std::function<std::vector<double>(std::vector<double>, double)> systemLambda,
                                                    std::function<std::vector<std::vector<double>>(std::vector<double>, double)> systemJacobianLambda,
                                                    std::vector<double> states_0, double currentTime, double timeStep);

            bool activateEvents;
            double eventTolerance;
            const std::vector<std::pair<double, double>> GetEvents(const std::shared_ptr<PySysLinkBase::SampleTime> sampleTime, double eventTime, std::vector<double> eventTimeStates) const;
        public:
            double firstTimeStep;

            std::vector<double> SystemModel(std::vector<double> states, double time);
            std::vector<std::vector<double>> SystemModelJacobian(std::vector<double> states, double time);

            BasicOdeSolver(std::shared_ptr<IOdeStepSolver> odeStepSolver, std::shared_ptr<SimulationModel> simulationModel,
                            std::vector<std::shared_ptr<ISimulationBlock>> simulationBlocks, std::shared_ptr<SampleTime> sampleTime,
                            std::shared_ptr<SimulationOptions> simulationOptions,
                            double firstTimeStep = 1e-6, bool activateEvents=true, double eventTolerance=1e-2);

            void UpdateStatesToNextTimeHits();
            void DoStep(double currentTime, double timeStep);
            void ComputeMajorOutputs(double currentTime);

            double GetNextTimeHit() const;
            double GetNextSuggestedTimeStep() const;
    };
} // namespace PySysLinkBase


// End header: BasicOdeSolver.h

// Begin header: SimulationManager.h
#define SRC_SIMULATION_MANAGER

#include "SimulationModel.h"
#include "SimulationOptions.h"
#include "ContinuousAndOde/BasicOdeSolver.h"
#include "ContinuousAndOde/IOdeStepSolver.h"
#include "SimulationOutput.h"
#include "BlockEvents/ValueUpdateBlockEvent.h"

#include <tuple>
#include <unordered_map>
#include <functional>

namespace PySysLinkBase
{
    class SimulationManager
    {
        public:
        SimulationManager(std::shared_ptr<SimulationModel> simulationModel, std::shared_ptr<SimulationOptions> simulationOptions);
        std::shared_ptr<SimulationOutput> RunSimulation();

        double RunSimulationStep();
        std::shared_ptr<SimulationOutput> GetSimulationOutput();

        private:
        bool hasRunFullSimulation = false;
        bool isRunningStepByStep = false;
        bool isFirstStepDone = false;
        std::vector<std::shared_ptr<SampleTime>> nextSampleTimesToProcess = {};

        void ClassifyBlocks(std::vector<std::shared_ptr<PySysLinkBase::ISimulationBlock>> orderedBlocks,
                            std::map<std::shared_ptr<SampleTime>, std::vector<std::shared_ptr<ISimulationBlock>>>& blocksForEachDiscreteSampleTime,
                            std::vector<std::shared_ptr<ISimulationBlock>>& blocksWithConstantSampleTime,
                            std::map<std::shared_ptr<SampleTime>, std::vector<std::shared_ptr<ISimulationBlock>>>& blocksForEachContinuousSampleTimeGroup);

        void ProcessBlock(std::shared_ptr<SimulationModel> simulationModel, std::shared_ptr<ISimulationBlock> block, std::shared_ptr<SampleTime> sampleTime, double currentTime, bool isMinorStep=false);

        void GetTimeHitsToSampleTimes(std::shared_ptr<SimulationOptions> simulationOptions, std::map<std::shared_ptr<SampleTime>, std::vector<std::shared_ptr<ISimulationBlock>>> blocksForEachDiscreteSampleTime);

        std::tuple<double, int, std::vector<std::shared_ptr<SampleTime>>> GetNearestTimeHit(int nextDiscreteTimeHitToProcessIndex);
        std::tuple<double, std::vector<std::shared_ptr<SampleTime>>> GetNearestTimeHit(double currentTime);


        std::map<std::shared_ptr<SampleTime>, std::shared_ptr<BasicOdeSolver>> odeSolversForEachContinuousSampleTimeGroup;

        std::map<std::shared_ptr<SampleTime>, std::vector<std::shared_ptr<ISimulationBlock>>> blocksForEachDiscreteSampleTime;
        std::map<std::shared_ptr<SampleTime>, std::vector<std::shared_ptr<ISimulationBlock>>> blocksForEachContinuousSampleTimeGroup;
        std::vector<std::shared_ptr<ISimulationBlock>> blocksWithConstantSampleTime;

        bool IsBlockInSampleTimes(const std::shared_ptr<ISimulationBlock>& block, const std::vector<std::shared_ptr<SampleTime>>& sampleTimes,
                                            const std::map<std::shared_ptr<SampleTime>, std::vector<std::shared_ptr<ISimulationBlock>>>& blockMap);

        std::map<double, std::vector<std::shared_ptr<SampleTime>>> timeHitsToSampleTimes;
        std::vector<double> timeHits;
        double currentTime;

        std::shared_ptr<SimulationModel> simulationModel;
        std::shared_ptr<SimulationOptions> simulationOptions;

        std::shared_ptr<SimulationOutput> simulationOutput;

        void ValueUpdateBlockEventCallback(const std::shared_ptr<ValueUpdateBlockEvent> blockEvent);

        void LogSignalInputReadCallback(const std::string blockId, const std::vector<std::shared_ptr<PySysLinkBase::InputPort>> inputPorts, int inputPortIndex, std::shared_ptr<PySysLinkBase::SampleTime> sampleTime, double currentTime);
        void LogSignalOutputUpdateCallback(const std::string blockId, const std::vector<std::shared_ptr<PySysLinkBase::OutputPort>> outputPorts, int outputPortIndex, std::shared_ptr<PySysLinkBase::SampleTime> sampleTime, double currentTime);
        void UpdateConfigurationValueCallback(const std::string blockId, const std::string keyName, ConfigurationValue value);

        std::unordered_map<const Port*, const Port*> portToLogInToAvoidRepetition = {};
        std::unordered_map<const Port*, std::pair<std::string, int>> loggedPortToCorrespondentBlockIdAndOutputPortIndex = {};

        std::vector<std::shared_ptr<PySysLinkBase::ISimulationBlock>> orderedBlocks;

        void ProcessBlocksInSampleTimes(const std::vector<std::shared_ptr<SampleTime>> sampleTimes, bool isMinorStep=false);
        void MakeFirstSimulationStep();
        void ProcessTimeHit(double time, const std::vector<std::shared_ptr<SampleTime>>& sampleTimesToProcess);

        std::vector<std::shared_ptr<ISimulationBlock>> simulationBlocksForceOutputUpdate = {};
    };
}

// End header: SimulationManager.h

#endif // AMALGAMATED_HEADER_H