//
// Created by Yinghao Qin on 14/09/2024.
//

#ifndef CEVRP_INDIVIDUAL_HPP
#define CEVRP_INDIVIDUAL_HPP

#include <iostream>
#include <vector>
#include <cstring>
#include <set>

using namespace std;

class Individual {
public:
    int route_cap; // route capacity - 3 by MIN_VEHICLES
    int node_cap; // node capacity - NUM_OF_CUSTOMERS + num_of_depot

    // chromosome information
    int chromosome_length;

    // upper-level solution for CVRP
    int** routes;
    int num_routes; // the actual number of routes for the solution
    int* num_nodes_per_route; // the node number of each route
    int* demand_sum_per_route; // the demand_ sum of all customers_ of each route
    double upper_cost;

    // lower-level solution for CEVRP, `num_routes` and `demand_sum_per_route` are the same as the upper-level solution
    int** lower_routes;
    int*  lower_num_nodes_per_route;
    double lower_cost;

    // used for calculate the diversity
    double biased_fitness{}; // the smaller, the better
    std::vector<int> successors;  // for each node, the successor in the solution (can be the depot 0)
    std::vector<int> predecessors;  // for each node, the predecessor in the solution (can be the depot 0)
    std::multiset<std::pair<double, Individual*>> proximate_individuals; // the other individuals in the population, ordered by increasing proximity (the set container follows a natural ordering based on the first value of the pair)

    Individual(const Individual  &ind);
    Individual(int route_cap, int node_cap);
    Individual(int route_cap, int node_cap, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum_per_route);
    ~Individual();

    void start_lower_solution();
    void set_lower_cost(double lower_cost_);
    [[nodiscard]] vector<int> get_chromosome() const;
    void cleanup() const;
    void reset();

    friend ostream& operator<<(ostream& os, const Individual& individual);
};

#endif //CEVRP_INDIVIDUAL_HPP
