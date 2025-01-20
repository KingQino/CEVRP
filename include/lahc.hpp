//
// Created by Yinghao Qin on 15/01/2025.
//

#ifndef CEVRP_LAHC_HPP
#define CEVRP_LAHC_HPP

#include "case.hpp"
#include "individual.hpp"
#include "utils.hpp"
#include "heuristic_interface.hpp"
#include "stats_interface.hpp"

using namespace std;

class Lahc : public HeuristicInterface, public StatsInterface {
public:
    static const std::string algorithm;

    Lahc(Case *instance, int seed, int stop_criteria_option = 0, bool enable_logging = false, int history_length = 5'000);
    ~Lahc() override;

    void run() override;
    void initialize_heuristic() override;
    void run_heuristic() override;
    bool stop_criteria_max_evals() const override;
    bool stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const override;
    bool stop_criteria_obj_convergence(double current_best_obj) const override;
    void open_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void close_log_for_evolution() override;
    void save_log_for_solution() override;

    Case* instance;
    int stop_criteria_option; // 0 for max-evals, 1 for max-exec-time
    bool enable_logging; // a flag to control logging

    long history_length; // LAHC history length Lh

    // default parameters
    int route_cap;
    int node_cap;

    uniform_int_distribution<int> uniform_int_dist;
    long iter; // iteration counter I
    long idle_iter; // idle iteration counter
    int restart_num; // restart number
    double* history_list; // LAHC history list L, it contains the cost function (fitness) values

    std::shared_ptr<Individual> current; // current solution s
    std::unique_ptr<Individual> global_best;

    friend ostream& operator<<(ostream& os, const Lahc& lahc);

};

#endif //CEVRP_LAHC_HPP
