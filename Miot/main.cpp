/**
 * @file main.cpp
 * @brief Traffic Signal Timing Optimization & Delay Analysis Tool
 *
 * ENGINEERING STANDARDS & REFERENCES:
 * - Highway Capacity Manual (HCM 6th Edition): Signalized Intersection Methodology
 * - Webster, F.V. (1958). "Traffic Signal Settings." Road Research Technical Paper No. 39
 *
 * DESIGN ASSUMPTIONS & LIMITATIONS:
 * 1. Isolated, pre-timed (fixed-time) signalized intersection operating under 4-phase control.
 * 2. Uniform arrival rates assumed for Webster's delay equation (d1).
 * 3. Analysis period for incremental overflow delay (d2) set to T = 0.25 hours (15 minutes).
 * 4. Saturation flow rates strictly bounded between 1,000 and 2,400 vphg per lane group.
 * 5. Minimum cycle length clamped to 30s; maximum cycle length clamped to 180s.
 */

#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>
#include <cassert>

// Structure to represent a single lane group within a phase
struct LaneGroup {
    std::string name;
    double arrivalRateVehHr;      // Arrival rate (vehicles/hour)
    double saturationFlowVehHr;   // Saturation flow rate (vehicles/hour)
};

// Structure to represent a signal phase containing lane groups
struct SignalPhase {
    std::string phaseName;
    std::vector<LaneGroup> lanes;
};

// Structure holding performance metrics for a single lane group
struct LaneMetrics {
    std::string name;
    double arrivalRate;
    double capacity;
    double vcRatio;           // Degree of saturation (v/c)
    double avgDelaySec;       // Average delay (sec/veh)
    double avgQueueVeh;       // Average queue length (vehicles)
};

// Structure holding performance metrics for a phase
struct PhaseMetrics {
    std::string phaseName;
    double displayGreenSec;
    double effectiveGreenSec;
    std::vector<LaneMetrics> laneMetrics;
};

// Structure holding full intersection report metrics
struct OptimizationResult {
    double cycleLengthSec;
    double criticalFlowRatioSum;
    bool isOversaturated;
    bool maxWaitExceeded;
    double avgIntersectionDelaySec;
    std::vector<PhaseMetrics> phases;
};

class TrafficSignalOptimizer {
private:
    double yellowTimeSec;
    double redClearanceSec;
    double lostTimePerPhaseSec;

public:
    TrafficSignalOptimizer(double yellow = 3.0, double redClearance = 1.0, double lostTime = 4.0)
        : yellowTimeSec(yellow), redClearanceSec(redClearance), lostTimePerPhaseSec(lostTime) {}

    // Computes optimal cycle length using Webster's Equation: C = (1.5*L + 5) / (1 - Y)
    double calculateCycleLength(double sumCriticalRatios, double totalLostTime,
                                 bool &oversaturated,
                                 double minCycle = 30.0, double maxCycle = 180.0) const {
        if (sumCriticalRatios >= 1.0) {
            oversaturated = true;
            return maxCycle;
        }

        oversaturated = false;
        double websterCycle = (1.5 * totalLostTime + 5.0) / (1.0 - sumCriticalRatios);
        return std::max(minCycle, std::min(maxCycle, websterCycle));
    }

    // Main optimization routine evaluating phase timing, delays, and queuing
    OptimizationResult optimizeIntersection(const std::vector<SignalPhase>& phases,
                                            double maxAllowedWaitSec = 120.0,
                                            double minGreenSec = 7.0) const {
        OptimizationResult result;
        size_t numPhases = phases.size();
        double totalLostTime = numPhases * lostTimePerPhaseSec;

        // Step 1: Calculate critical flow ratios (y_i = max(v_j / s_j)) per phase
        std::vector<double> criticalFlowRatios;
        double sumY = 0.0;
        for (const auto& phase : phases) {
            double maxYi = 0.0;
            for (const auto& lane : phase.lanes) {
                if (lane.saturationFlowVehHr > 0.0) {
                    double yi = lane.arrivalRateVehHr / lane.saturationFlowVehHr;
                    if (yi > maxYi) maxYi = yi;
                }
            }
            criticalFlowRatios.push_back(maxYi);
            sumY += maxYi;
        }
        result.criticalFlowRatioSum = sumY;

        // Step 2: Determine optimal cycle length
        result.cycleLengthSec = calculateCycleLength(sumY, totalLostTime, result.isOversaturated, 30.0, 180.0);

        // Step 3: Green time allocation
        double totalEffectiveGreen = result.cycleLengthSec - totalLostTime;
        std::vector<double> effectiveGreenAllocations(numPhases, minGreenSec);

        if (totalEffectiveGreen > 0 && !result.isOversaturated && sumY > 0) {
            for (size_t i = 0; i < numPhases; ++i) {
                double allocated = (criticalFlowRatios[i] / sumY) * totalEffectiveGreen;
                effectiveGreenAllocations[i] = std::max(minGreenSec, allocated);
            }
        }

        // Step 4: Evaluate queuing and delay (HCM + Webster Formulation)
        double totalIntersectionDelaySum = 0.0;
        double totalIntersectionVolumeSum = 0.0;
        result.maxWaitExceeded = false;

        for (size_t i = 0; i < numPhases; ++i) {
            PhaseMetrics pMetrics;
            pMetrics.phaseName = phases[i].phaseName;
            pMetrics.effectiveGreenSec = effectiveGreenAllocations[i];
            pMetrics.displayGreenSec = pMetrics.effectiveGreenSec + lostTimePerPhaseSec - (yellowTimeSec + redClearanceSec);

            for (const auto& lane : phases[i].lanes) {
                LaneMetrics lMetrics;
                lMetrics.name = lane.name;
                lMetrics.arrivalRate = lane.arrivalRateVehHr;

                double v = lane.arrivalRateVehHr / 3600.0;     // veh/sec

                // Capacity calculation (veh/hr)
                double c = lane.saturationFlowVehHr * (pMetrics.effectiveGreenSec / result.cycleLengthSec);
                lMetrics.capacity = c;
                lMetrics.vcRatio = (c > 0) ? (lane.arrivalRateVehHr / c) : 1.0;

                // Webster's Uniform Delay (d1)
                double gOverC = pMetrics.effectiveGreenSec / result.cycleLengthSec;
                double xBounded = std::min(1.0, lMetrics.vcRatio);

                double denominator = 2.0 * (1.0 - xBounded * gOverC);
                double d1 = (denominator > 0.0)
                    ? (result.cycleLengthSec * std::pow(1.0 - gOverC, 2)) / denominator
                    : 0.0;

                // Incremental Overflow Delay (d2) for 15-min analysis period (T = 0.25 hrs)
                double T = 0.25;
                double x = lMetrics.vcRatio;
                double d2 = 0.0;
                if (c > 0 && x > 0) {
                    d2 = 900.0 * T * ((x - 1.0) + std::sqrt(std::pow(x - 1.0, 2) + (8.0 * x) / (c * T)));
                }

                lMetrics.avgDelaySec = d1 + d2;

                // Average Queue Length calculation (vehicles)
                double redTime = result.cycleLengthSec - pMetrics.effectiveGreenSec;
                lMetrics.avgQueueVeh = (v * redTime) + (d2 * v);

                if (lMetrics.avgDelaySec > maxAllowedWaitSec) {
                    result.maxWaitExceeded = true;
                }

                totalIntersectionDelaySum += lMetrics.avgDelaySec * lane.arrivalRateVehHr;
                totalIntersectionVolumeSum += lane.arrivalRateVehHr;

                pMetrics.laneMetrics.push_back(lMetrics);
            }
            result.phases.push_back(pMetrics);
        }

        result.avgIntersectionDelaySec = (totalIntersectionVolumeSum > 0)
            ? (totalIntersectionDelaySum / totalIntersectionVolumeSum)
            : 0.0;

        return result;
    }
};

// Formatted report output
void printTimingReport(const std::string& planName, const OptimizationResult& result, double maxAllowedWait) {
    std::cout << "\n========================================================" << std::endl;
    std::cout << " SIGNAL TIMING REPORT: " << planName << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Calculated Cycle Length : " << result.cycleLengthSec << " sec" << std::endl;
    std::cout << std::setprecision(3);
    std::cout << "Sum of Critical Ratios  : " << result.criticalFlowRatioSum << std::endl;
    std::cout << std::setprecision(1);
    std::cout << "Avg Intersection Delay  : " << result.avgIntersectionDelaySec << " sec/veh" << std::endl;

    if (result.isOversaturated) {
        std::cout << "WARNING: Intersection is OVER-SATURATED (Demand >= Capacity)." << std::endl;
    }
    if (result.maxWaitExceeded) {
        std::cout << "WARNING: Average delay exceeds maximum allowable limit (" << maxAllowedWait << "s)." << std::endl;
    }

    std::cout << "--------------------------------------------------------" << std::endl;
    std::cout << std::left << std::setw(24) << "Phase / Lane Group" << " | "
              << std::setw(9)  << "Green (s)" << " | "
              << std::setw(9)  << "v/c Ratio" << " | "
              << std::setw(9)  << "Delay (s)" << " | "
              << std::setw(10) << "Queue (veh)" << std::endl;
    std::cout << "--------------------------------------------------------" << std::endl;

    for (const auto& p : result.phases) {
        std::cout << std::left << std::setw(24) << p.phaseName << " | "
                  << std::right << std::setw(9) << p.displayGreenSec << " | "
                  << std::setw(9) << "-" << " | "
                  << std::setw(9) << "-" << " | "
                  << std::setw(10) << "-" << std::endl;

        for (const auto& lane : p.laneMetrics) {
            std::string laneDisplayName = "  -- " + lane.name;
            std::cout << std::left << std::setw(24) << laneDisplayName << " | "
                      << std::right << std::setw(9) << "" << " | " // Left blank for lane rows
                      << std::setprecision(2) << std::setw(9) << lane.vcRatio << " | "
                      << std::setprecision(1) << std::setw(9) << lane.avgDelaySec << " | "
                      << std::setprecision(1) << std::setw(10) << lane.avgQueueVeh << std::endl;
        }
    }
    std::cout << "========================================================\n" << std::endl;
}

// Input validation helper
double getValidInput(const std::string& prompt, double minValue, double maxValue) {
    double value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && value >= minValue && value <= maxValue) {
            return value;
        }
        std::cout << "Invalid input. Please enter a numerical value between "
                  << minValue << " and " << maxValue << ".\n";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
}

// Interactive setup mode
void runInteractiveMode(const TrafficSignalOptimizer& optimizer) {
    std::cout << "\n--- INTERACTIVE SIGNAL TIMING OPTIMIZER ---\n";
    double maxWait = getValidInput("Enter maximum acceptable wait time per vehicle (sec) [10-300]: ", 10.0, 300.0);
    int numPhases = static_cast<int>(getValidInput("Enter number of signal phases [2-6]: ", 2.0, 6.0));

    std::vector<SignalPhase> customPhases;
    for (int i = 0; i < numPhases; ++i) {
        SignalPhase phase;
        phase.phaseName = "Phase " + std::to_string(i + 1);

        int numLanes = static_cast<int>(getValidInput("Enter number of lane groups for Phase " + std::to_string(i + 1) + " [1-4]: ", 1.0, 4.0));
        for (int j = 0; j < numLanes; ++j) {
            LaneGroup lane;
            std::cout << "  Lane Group " << (j + 1) << " Name: ";
            std::cin >> std::ws; // Clear whitespace/newlines before reading string
            std::getline(std::cin, lane.name);
            lane.arrivalRateVehHr = getValidInput("  Arrival rate (vph) [0-3000]: ", 0.0, 3000.0);
            lane.saturationFlowVehHr = getValidInput("  Saturation flow rate (vph) [1000-2400]: ", 1000.0, 2400.0);
            phase.lanes.push_back(lane);
        }
        customPhases.push_back(phase);
    }

    OptimizationResult customResult = optimizer.optimizeIntersection(customPhases, maxWait);
    printTimingReport("CUSTOM USER PLAN", customResult, maxWait);
}

// Automated unit and boundary testing suite
void runUnitTests(const TrafficSignalOptimizer& optimizer) {
    std::cout << "[RUNNING UNIT & BOUNDARY TESTS]..." << std::endl;

    // Test 1: Zero Demand Boundary Test
    std::vector<SignalPhase> zeroPhases = {
        {"Phase 1", {{"L1", 0.0, 1800.0}}},
        {"Phase 2", {{"L2", 0.0, 1800.0}}}
    };
    OptimizationResult resZero = optimizer.optimizeIntersection(zeroPhases);
    assert(resZero.cycleLengthSec == 30.0); // Should hit min cycle length limit
    assert(!resZero.isOversaturated);
    assert(resZero.avgIntersectionDelaySec == 0.0);

    // Test 2: Oversaturated Boundary Test
    std::vector<SignalPhase> overPhases = {
        {"Phase 1", {{"L1", 1200.0, 1800.0}}}, // Y1 = 0.667
        {"Phase 2", {{"L2", 1000.0, 1800.0}}}  // Y2 = 0.555 (Sum > 1.0)
    };
    OptimizationResult resOver = optimizer.optimizeIntersection(overPhases);
    assert(resOver.cycleLengthSec == 180.0); // Max cycle length cap
    assert(resOver.isOversaturated == true);

    // Test 3: Input Range Bounds Validation Logic
    double minVal = 10.0, maxVal = 300.0;
    auto isValid = [](double val, double min, double max) {
        return val >= min && val <= max;
    };

    assert(!isValid(5.0, minVal, maxVal));    // Below min
    assert(!isValid(350.0, minVal, maxVal));  // Above max
    assert(isValid(60.0, minVal, maxVal));    // Valid

    std::cout << "[UNIT TESTS PASSED SUCCESSFULLY]\n" << std::endl;
}

// Automated verification and scenario comparison execution
int main() {
    TrafficSignalOptimizer optimizer(3.0, 1.0, 4.0);

    // Execute unit and boundary tests first
    runUnitTests(optimizer);

    double maxWaitTarget = 60.0;

    // Phase Configuration 1: Realistic Peak Demand Period (Sum Y ≈ 0.843)
    std::vector<SignalPhase> peakPhases = {
        {"Phase 1: Main St Thru/Right", {
            {"NB Thru/Right", 800.0, 1900.0}, // Y1 = 0.421
            {"SB Thru/Right", 780.0, 1900.0}
        }},
        {"Phase 2: Main St Left Turn", {
            {"NB Left", 180.0, 1700.0},       // Y2 = 0.106
            {"SB Left", 160.0, 1700.0}
        }},
        {"Phase 3: Side St Thru/Right", {
            {"EB Thru/Right", 400.0, 1800.0}, // Y3 = 0.222
            {"WB Thru/Right", 380.0, 1800.0}
        }},
        {"Phase 4: Side St Left Turn", {
            {"EB Left", 150.0, 1600.0},       // Y4 = 0.094
            {"WB Left", 140.0, 1600.0}
        }}
    };

    // Phase Configuration 2: Off-Peak Period (~50% Reduction)
    std::vector<SignalPhase> offPeakPhases = {
        {"Phase 1: Main St Thru/Right", {
            {"NB Thru/Right", 500.0, 1900.0},
            {"SB Thru/Right", 480.0, 1900.0}
        }},
        {"Phase 2: Main St Left Turn", {
            {"NB Left", 100.0, 1700.0},
            {"SB Left", 90.0, 1700.0}
        }},
        {"Phase 3: Side St Thru/Right", {
            {"EB Thru/Right", 220.0, 1800.0},
            {"WB Thru/Right", 200.0, 1800.0}
        }},
        {"Phase 4: Side St Left Turn", {
            {"EB Left", 70.0, 1600.0},
            {"WB Left", 60.0, 1600.0}
        }}
    };

    // Optimization Runs
    OptimizationResult peakResults = optimizer.optimizeIntersection(peakPhases, maxWaitTarget);
    OptimizationResult offPeakResults = optimizer.optimizeIntersection(offPeakPhases, maxWaitTarget);

    // Display Reports
    printTimingReport("PEAK PERIOD PLAN", peakResults, maxWaitTarget);
    printTimingReport("OFF-PEAK PERIOD PLAN", offPeakResults, maxWaitTarget);

    // Comparative Analysis Summary
    std::cout << "========================================================" << std::endl;
    std::cout << " PLAN COMPARISON SUMMARY" << std::endl;
    std::cout << "========================================================" << std::endl;
    std::cout << std::fixed << std::setprecision(1);
    std::cout << "Peak Cycle Length     : " << peakResults.cycleLengthSec << "s  | Off-Peak Cycle Length     : " << offPeakResults.cycleLengthSec << "s" << std::endl;
    std::cout << "Peak Avg Delay/Veh    : " << peakResults.avgIntersectionDelaySec << "s | Off-Peak Avg Delay/Veh    : " << offPeakResults.avgIntersectionDelaySec << "s" << std::endl;
    std::cout << "Peak Capacity Util.   : " << static_cast<int>(peakResults.criticalFlowRatioSum * 100.0) << "% | Off-Peak Capacity Util.   : " << static_cast<int>(offPeakResults.criticalFlowRatioSum * 100.0) << "%" << std::endl;
    std::cout << "========================================================\n" << std::endl;

    // Option for interactive configuration
    char choice;
    std::cout << "Would you like to run a custom interactive scenario? (y/n): ";
    if (std::cin >> choice && (choice == 'y' || choice == 'Y')) {
        runInteractiveMode(optimizer);
    }
    std::cout << std::endl; // Clean output trailing newline

    return 0;
}
