//
// Created by Yinghao Qin on 19/09/2024.
//

#include "ma.hpp"

Ma::Ma(Case *instance, int seed, int stop_criteria_option, bool enable_logging) : HeuristicInterface("CBMA", seed),
instance(instance), stop_criteria_option(stop_criteria_option), enable_logging(enable_logging) {
    this->route_cap = this->instance->num_vehicle_ * 3;
    this->node_cap = this->instance->num_customer_ + 1;
    this->iter = 0;

    this->pop_size = 100;
    this->elite_ratio = 0.01;
    this->immigrant_ratio = 0.05;
    this->crossover_prob = 1.0;
    this->mutation_prob = 0.5;
    this->mutation_ind_prob = 0.2;
    this->tournament_size = 2;
}

Ma::~Ma() {
    population.clear();
    delete instance;
}

void Ma::run() {
    // Initialize time variables
    start = std::chrono::high_resolution_clock::now();

    if (enable_logging) {
        open_log_for_evolution();  // Open log if logging is enabled
    }

    initialize_heuristic();

    switch (this->stop_criteria_option) {
        case 0:
            while (!stop_criteria_max_evals()) {
                run_heuristic();

                if (enable_logging) {
                    flush_row_into_evol_log();
                }
            }
            break;

        case 1:
            while (!stop_criteria_max_exec_time(duration)) {
                run_heuristic();
                duration = std::chrono::high_resolution_clock::now() - start;

                if (enable_logging) {
                    flush_row_into_evol_log();
                }
            }
            break;
        default:
            std::cerr << "Invalid stop criteria option!" << std::endl;
            break;
    }

    if (enable_logging) {
        close_log_for_evolution();  // Close log if logging is enabled
        save_log_for_solution();    // Save the log if logging is enabled
    }
}

bool Ma::stop_criteria_max_evals() const {
    return instance->get_evals() >= instance->max_evals_;
}

bool Ma::stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const {
    return duration.count() >= instance->max_exec_time_;
}

