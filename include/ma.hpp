//
// Created by Yinghao Qin on 19/09/2024.
//

#ifndef CEVRP_MA_HPP
#define CEVRP_MA_HPP

#include <random>
#include <algorithm>
#include <iterator>
#include <deque>
#include <iostream>
#include <vector>
#include <memory>
#include <cassert>
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
    bool stop_criteria_obj_convergence(double current_best_obj) const override;
    void open_log_for_evolution() override;
    void flush_row_into_evol_log() override;
    void close_log_for_evolution() override;
    void save_log_for_solution() override;
    void init_ind_by_chromosome(Individual& ind, const vector<int>& chromosome) const;
    shared_ptr<Individual> admit_one_individual(const vector<int>& chromosome);
    static Individual& select_best_individual_ref(const vector<unique_ptr<Individual>>& individuals);
    static Individual* select_best_individual_ptr(const vector<Individual*>& individuals);
    static Individual* select_best_upper_individual_ptr(const vector<Individual*>& individuals);
    vector<int> get_immigrant_chromosome(std::default_random_engine& rng) const;
    static vector<double> extract_fitness_values(const vector<unique_ptr<Individual>>& individuals);
    static double calculate_diversity_by_normalized_fitness_difference(const vector<double>& fitness_values);
    static double calculate_diversity_by_broken_paris_distance(const vector<unique_ptr<Individual>>& individuals, int num_closest);
    static vector<int> sel_random(int length, int k, std::default_random_engine& rng); // select k random indexes from 0 to length-1
    void cx_partially_matched(vector<int>& parent1, vector<int>& parent2, std::default_random_engine& rng);
    void mut_shuffle_indexes(vector<int>& chromosome, double independent_prob, std::default_random_engine& rng);

    double broken_pairs_distance(const Individual& ind1, const Individual& ind2) const; // distance measure between two individuals, used for diversity calculations
    static double average_broken_pairs_distance_closest(const Individual& ind, int num_closest); // returns the average broken pairs distance of this individual with the nbClosest individuals in the population
    void update_proximate_individuals(); // updates the proximate individuals of the individual in the population
    void update_biased_fitness(); // updates the biased fitness of the individual

    Case* instance;
    vector<unique_ptr<Individual>> population;
    unique_ptr<Individual> global_best;
    unique_ptr<Individual> global_upper_best;
    double diversity;

    uniform_int_distribution<int> uniform_int_dist;
    uniform_int_distribution<int> uniform_swap_dist;
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
    double gammaL; // confidence ratio of local search (>1): the larger it is, the more solutions are selected to carry out local search
    double gammaR; // confidence ratio of recharging (0, 1): the smaller it is, the more solutions are selected to carry out recharging
    int delta;  // confidence interval
    deque<double> P; // list for confidence intervals of local search
    double r; // confidence interval is used to judge whether an upper-level sub-solution should make the charging process

    // diversity
    int num_closest; // the number of closest individuals to calculate the diversity
    int num_elite; // the number of elite individuals to be preserved

    // refine
    double refine_threshold_ratio; // threshold ratio for accepting bad solutions

    // controlled experiments
    int tournament_size;


};

#endif //CEVRP_MA_HPP
