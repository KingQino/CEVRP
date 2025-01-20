//
// Created by Yinghao Qin on 17/01/2025.
//

#include <random>
#include "lahc.hpp"



Lahc::Lahc(Case *instance, int seed, int stop_criteria_option, bool enable_logging, int history_length) : HeuristicInterface("LAHC", seed),
instance(instance), stop_criteria_option(stop_criteria_option), enable_logging(enable_logging), history_length(history_length) {

    this->restart_num = 0;
    this->iter = 0;
    this->idle_iter = 0;

    this->uniform_int_dist = std::uniform_int_distribution<int>(0, 2); // for random selection of local search operators

    // hyperparameters for LAHC
    this->history_length = history_length;
    this->history_list = new double[history_length];

    // default parameters
    this->route_cap = this->instance->num_vehicle_ * 3;
    this->node_cap = this->instance->num_customer_ + 1;
}


Lahc::~Lahc() {
    delete[] this->history_list;
//    this->global_best.reset();
}

void Lahc::run() {
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

bool Lahc::stop_criteria_max_evals() const {
    return instance->get_evals() >= instance->max_evals_;
}

bool Lahc::stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const {
    return duration.count() >= instance->max_exec_time_;
}

bool Lahc::stop_criteria_obj_convergence(double current_best_obj) const {
    static int no_change_count = 0; // consecutive no-change count
    static double prev_best_obj = std::numeric_limits<double>::max(); // previous best objective, initialized to infinity

    // calculate the change in objective value
    double obj_change = std::abs(current_best_obj - prev_best_obj);

    // If change is small, increment count; otherwise, reset
    if (obj_change < instance->convergence_epsilon_) {
        no_change_count++;
    } else {
        no_change_count = 0;
    }

    // update previous objective value
    prev_best_obj = current_best_obj;

    // check if max no-change count is reached
    return no_change_count >= instance->max_no_change_count_;
}

void Lahc::initialize_heuristic() {
    vector<vector<int>> routes = routes_constructor_with_hien_method(*instance, random_engine);
    this->current = make_shared<Individual>(route_cap, node_cap, routes,
                                            instance->compute_total_distance(routes),
                                            instance->compute_demand_sum_per_route(routes));
    for(int i = 0; i < history_length; ++i)
        history_list[i] = current->upper_cost;
    this->iter = 0;
    this->idle_iter = 0;

    global_best = make_unique<Individual>(route_cap, node_cap);
    global_best->lower_cost = INFEASIBLE;
}

void Lahc::run_heuristic() {
    do {
        // undergo upper-level optimisation operators or perturbation method on the candidate solution
        std::shared_ptr<Individual> candidate = make_shared<Individual>(*current);

        switch (uniform_int_dist(random_engine)) {
            case 0:
                two_opt_intra_for_individual(*candidate, *instance);
                break;
            case 1:
                two_opt_inter_for_individual(*candidate, *instance);
                break;
            case 2:
                node_relocation_intra_for_individual(*candidate, *instance);
                break;
//            case 3:
//                generalized_double_bridge_for_individual(*candidate, *instance, random_engine);
//                break;
        }

        if (idle_iter >= 30) generalized_double_bridge_for_individual(*candidate, *instance, random_engine);


        candidate->upper_cost >= current->upper_cost ? idle_iter++ : idle_iter = 0;

        auto v = iter % history_length;
        if (candidate->upper_cost < history_list[v] || candidate->upper_cost <= current->upper_cost) {
            current = std::move(candidate); // std::move is used to transfer ownership of the `candidate` pointer to `current`.
        } else {
            // do nothing - keep the current solution
//            this->current = make_shared<Individual>(*current); // keep the current solution
        }

        if (current->upper_cost < history_list[v]) {
            history_list[v] = current->upper_cost;
        }

        iter++;
        duration = std::chrono::high_resolution_clock::now() - start;

        std::unique_ptr<Individual> dummy_current = make_unique<Individual>(*current);
        fix_one_solution(*dummy_current, *instance);
        if (dummy_current->lower_cost < global_best->lower_cost) {
            global_best = std::move(dummy_current); // std::move is used to transfer ownership of the `dummyCurrent` pointer to `globalBest`.
        }

//        cout << *this << endl;
//        cout << "iterationCounter: " << iterationCounter << ", idleIterationCounter: " << idleIterationCounter << ", globalBest: " << globalBest->get_fit() << ", timeUsed: " << duration.count() << endl;
    } while ( (iter < 100'000L || idle_iter < static_cast<long>(iter * 0.2)) && duration.count() < instance->max_exec_time_);
    // the stopping criterion is either the right part is false (the time used exceeds the maximum execution time)
    // , or the left part is false ( after 100,000 iterations, and the idle iteration number exceeds 2% of the total iterations)

}

void Lahc::open_log_for_evolution() {
    string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
    create_directories_if_not_exists(directory);

    string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory + "/" + file_name);
    log_evolution << "iter,restart_num,upper_cost,lower_cost\n";
}

void Lahc::flush_row_into_evol_log() {
    oss_row_evol << iter << "," << restart_num << "," << global_best->upper_cost << "," << global_best->lower_cost << "\n";
}

void Lahc::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Lahc::save_log_for_solution() {
    string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
//    create_directories_if_not_exists(directory);

    string file_name = "solution." + instance->instance_name_ + ".txt";

    log_solution.open(directory + "/" + file_name);
    log_solution << fixed << setprecision(5) << global_best->lower_cost << endl;
    for (int i = 0; i < global_best->num_routes; ++i) {
        for (int j = 0; j < global_best->lower_num_nodes_per_route[i]; ++j) {
            log_solution << global_best->lower_routes[i][j] << ",";
        }
        log_solution << endl;
    }
    log_solution.close();
}

