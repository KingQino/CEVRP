//
// Created by Yinghao Qin on 14/09/2024.
//

#ifndef CEVRP_INDIVIDUAL_HPP
#define CEVRP_INDIVIDUAL_HPP

#include <iostream>
#include <vector>
#include <cstring>

using namespace std;

class Individual {
public:
    int route_cap; // route capacity - 3 by MIN_VEHICLES
    int node_cap; // node capacity - NUM_OF_CUSTOMERS + num_of_depot
    int** routes;
    int route_num; // the actual number of routes for the solution
    int* node_num; // the node number of each route
    int* demand_sum; // the demand sum of all customers of each route
    double upper_cost;
    double lower_cost;


    Individual(const Individual  &ind);
    Individual(int route_cap, int node_cap);
    Individual(int route_cap, int node_cap, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum);
    ~Individual();

    void set_lower_cost(double lower_cost_);
    void set_routes(const vector<vector<int>>& routes_);

    friend ostream& operator<<(ostream& os, const Individual& individual);
};

#endif //CEVRP_INDIVIDUAL_HPP
