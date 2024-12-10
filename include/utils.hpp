//
// Created by Yinghao Qin on 14/09/2024.
//

#ifndef CEVRP_UTILS_HPP
#define CEVRP_UTILS_HPP

#include <iostream>
#include <vector>
#include <random>
#include <cstring>
#include <numeric>
#include <memory>
#include <unordered_set>
#include <list>
#include <stack>
#include <functional>

#include "case.hpp"
#include "individual.hpp"

using namespace std;

#define INFEASIBLE 1'000'000'000L


// used in 2-opt*, define the hash relationship to make sure one-to-one mapping between route pairs
typedef struct tPairHash
{
    size_t operator() (pair<int, int> const & apair) const {
        return apair.first * 256 + apair.second;
    }
} PairHash;

// This structure is used in the "enumerate stations"
typedef struct tState {
    int m_len, n_len, i, stationIdx; // Current state variables
} State;

// This struct is used to store the charging station information for the given route
typedef struct tChargingMeta {
    double cost; // cost after applying recharging decision
    int num_stations; // num of stations
    vector<int> chosen_pos; // chosen position
    vector<int> chosen_sta; // chosen station
} ChargingMeta;


// split - transfer a chromosome into upper-solution
vector<vector<int>> prins_split(const vector<int>& chromosome, Case& instance);
pair<vector<int>, double> classical_split(const vector<int>& chromosome, Case& instance);
vector<vector<int>> hien_clustering(const Case& instance, std::default_random_engine& rng);
void hien_balancing(vector<vector<int>>& routes, const Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_split(Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_hien_method(const Case& instance, std::default_random_engine& rng);
vector<vector<int>> routes_constructor_with_direct_encoding(const Case& instance, std::default_random_engine& rng);


// upper-level optimisation: local search operators => minimise the upper cost
// due to the observation that the lower cost (CEVRP) is highly positive correlation with the upper cost (CVRP)
// helper functions
unordered_set<pair<int, int>, PairHash> get_route_pairs(int num_routes);
void update_route_pairs(unordered_set<pair<int, int>, PairHash>& route_pairs, int r1, int r2);
void moveItoJ(int* route, int a, int b);
bool contains(const int* array, int size, int element);
// operators
void apply_multiple_operators(Individual& individual, Case& instance, const vector<std::function<void(Individual&, Case&)>>& operators);
void two_opt_for_single_route(int* route, int length, double& cost, Case& instance);
void two_opt_for_individual(Individual& individual, Case& instance); // two-arcs exchange, intra-route
bool two_opt_star_between_two_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, int node_cap, Case& instance);
bool two_opt_move_inter_route_for_individual(Individual& individual, Case& instance); // two-arcs exchange, inter-route
bool two_opt_star_for_individual(Individual& individual, Case& instance); // two-arcs exchange, inter-route
bool node_shift(int* route, int length, double& cost, Case& instance);
void one_point_move_intra_route_for_individual(Individual& individual, Case& instance); // three-arcs exchange, intra-route (One-point move intra-route)
bool node_shift_between_two_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, Case& instance);
bool one_point_move_inter_route_for_individual(Individual& individual, Case& instance); // three-arcs exchange, inter-route (One-point move inter-route)
void two_nodes_swap_for_single_route(int* route, int length, double& cost, Case& instance);
void two_point_move_intra_route_for_individual(Individual& individual, Case& instance); // four-arcs exchange, intra-route
bool two_nodes_swap_between_two_routes(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2, double& cost, Case& instance);
bool two_point_move_inter_route_for_individual(Individual& individual, Case& instance); // four-arcs exchange, inter-route
// accelerated version
void two_opt_for_single_route_acceleration(int* route, int length, double& cost, Case& instance);
bool two_opt_star_between_two_routes_acceleration(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, int node_cap, Case& instance);
bool two_opt_move_inter_route_for_individual_acceleration(Individual& individual, Case& instance);
bool node_shift_acceleration(int* route, int length, double& cost, Case& instance);
bool node_shift_between_two_routes_acceleration(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, Case& instance);
bool one_point_move_inter_route_for_individual_acceleration(Individual& individual, Case& instance);
void two_nodes_swap_for_single_route_acceleration(int* route, int length, double& cost, Case& instance);
bool two_nodes_swap_between_two_routes_acceleration(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2, double& cost, Case& instance);
bool two_point_move_inter_route_for_individual_acceleration(Individual& individual, Case& instance); // four-arcs exchange, inter-route

// neighbors expanding: expand the numbers of promising upper-level solution by exploiting the neighborhood => good at handle coupling issues
// we are going to use the concept `threshold` to accept some bad solutions. `threshold_ratio` is a decimal fraction (>1), `base_cost` is the cost of the best upper-level sub-solution encountered so far
// Note! The `base_cost` either uses the best upper-level sub-solution within each generation, or the best upper sub-solution found globally so far
vector<std::unique_ptr<Individual>> one_point_move_neighbors(Individual& individual, Case& instance, double base_cost, double threshold_ratio);
vector<std::unique_ptr<Individual>> one_point_intra_route_for_individual(Individual& individual, Case& instance, double threshold);
vector<std::unique_ptr<Individual>> one_point_inter_route_for_individual(Individual& individual, Case& instance, double threshold);
vector<std::unique_ptr<Individual>> two_point_move_neighbors(Individual& individual, Case& instance, double base_cost, double threshold_ratio);
vector<std::unique_ptr<Individual>> two_point_intra_route_for_individual(Individual& individual, Case& instance, double threshold);
vector<std::unique_ptr<Individual>> two_point_inter_route_for_individual(Individual& individual, Case& instance, double threshold);
vector<std::unique_ptr<Individual>> two_opt_move_neighbors(Individual& individual, Case& instance, double base_cost, double threshold_ratio);
vector<std::unique_ptr<Individual>> two_opt_intra_route_for_individual(Individual& individual, Case& instance, double threshold);
vector<std::unique_ptr<Individual>> two_opt_inter_route_for_individual(Individual& individual, Case& instance, double threshold);


// lower-level optimisation: make recharging decision => minimise the lower cost
// i.e., insert the optimal/near-optimal charging stations_ into the route
void fix_one_solution(Individual& individual, Case& instance);
double insert_station_by_simple_enumeration_array(int* route, int length, int* repaired_route, int& repaired_length, Case& instance);
double insert_station_by_remove_array(int* route, int length, int* repaired_route, int& repaired_length, Case& instance);
void tryACertainNArray(int m_len, int n_len, int* chosen_pos, int* best_chosen_pos, double& finalfit, int cur_upper_bound, int* route, int length, vector<double>& accumulated_distance, Case& instance);

// refine: neighbourhood expanding + recharging all enumeration
std::unique_ptr<Individual> refine(Individual& individual, Case& instance, double base_cost, double threshold_ratio);
std::unique_ptr<Individual> refine_limited_memory(Individual& individual, Case& instance, double base_cost, double threshold_ratio);
void recharging_by_all_enumeration(Individual& individual, Case& instance);
tuple<double, int*, int> insert_station_by_all_enumeration(int* route, int length, Case& instance);
double insert_station_by_all_enumeration(int* route, int length, int* repaired_route, int& repaired_length, Case& instance);
ChargingMeta try_enumerate_n_stations_to_route(int m_len, int n_len, int* chosen_sta, int* chosen_pos, double& cost,
                                               int cur_upper_bound, int* route, int length, vector<double>& accumulated_distance, Case& instance);



#endif //CEVRP_UTILS_HPP
