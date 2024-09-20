//
// Created by Yinghao Qin on 19/09/2024.
//

#ifndef CEVRP_HEURISTIC_INTERFACE_HPP
#define CEVRP_HEURISTIC_INTERFACE_HPP

#include <iostream>
#include <random>
#include <chrono>
#include <utility>

using namespace std;

class HeuristicInterface {
protected:
    string name;
    int seed;
    std::default_random_engine random_engine;
    uniform_real_distribution<double> uniform_real_dist;
public:
    // Constructor to initialize member variables
    HeuristicInterface(string heuristic_name, int seed_value)
            : name(std::move(heuristic_name)),
              seed(seed_value),
              random_engine(seed_value),
              uniform_real_dist(0.0, 1.0) {

    }

    virtual void run() = 0;
    virtual void initialize_heuristic() = 0;
    virtual void run_heuristic() = 0;
    [[nodiscard]] virtual bool stop_criteria_max_evals() const = 0;
    [[nodiscard]] virtual bool stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const = 0;

    virtual ~HeuristicInterface() = default;
};


#endif //CEVRP_HEURISTIC_INTERFACE_HPP
