//
// Created by Yinghao Qin on 19/09/2024.
//

#ifndef CEVRP_MA_HPP
#define CEVRP_MA_HPP

#include <random>
#include <algorithm>
#include <iterator>
#include <deque>
#include "case.hpp"
#include "individual.hpp"
#include "utils.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Ma : public HeuristicInterface, public StatsInterface {
public:

    Ma(Case *instance, int seed, int stop_criteria_option = 0, bool enable_logging = false);
    ~Ma() override;
    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    bool stop_criteria_max_evals() const override;
    bool stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const override;
    void open_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void close_log_for_evolution() override;
    void save_log_for_solution() override;
    shared_ptr<Individual> admit_one_immigrant(const vector<int>& chromosome);


    Case* instance;
    vector<shared_ptr<Individual>> population;

    int stop_criteria_option; // 0 for max-evals, 1 for max-exec-time
    bool enable_logging; // a flag to control logging
    int route_cap;
    int node_cap;
    int iter; // iteration counter

    // hyperparameters: all of them below
    // genetic algorithm
    int pop_size;
    double elite_ratio;
    double immigrant_ratio;
    double crossover_prob;
    double mutation_prob;
    double mutation_ind_prob;

    // confidence-based filter strategy


    // controlled experiments
    int tournament_size;


};

#endif //CEVRP_MA_HPP
