//
// Created by Yinghao Qin on 14/09/2024.
//

#include "individual.hpp"


Individual::Individual(const Individual& ind) {
    this->route_cap = ind.route_cap;
    this->node_cap = ind.node_cap;
    this->route_num = ind.route_num;
    this->upper_cost = ind.upper_cost;
    this->lower_cost = ind.lower_cost;
    this->routes = new int *[ind.route_cap];
    for (int i = 0; i < ind.route_cap; ++i) {
        this->routes[i] = new int[ind.node_cap];
        memcpy(this->routes[i], ind.routes[i], sizeof(int) * ind.node_cap);
    }
    this->node_num = new int[ind.route_cap];
    memcpy(this->node_num, ind.node_num, sizeof(int) * ind.route_cap);
    this->demand_sum = new int[ind.route_cap];
    memcpy(this->demand_sum, ind.demand_sum, sizeof(int) * ind.route_cap);
}

Individual::Individual(int route_cap, int node_cap) {
    this->route_cap = route_cap;
    this->node_cap = node_cap;
    this->routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
    }
    this->route_num = 0;
    this->node_num = new int[route_cap];
    memset(this->node_num, 0, sizeof(int) * route_cap);
    this->demand_sum = new int [route_cap];
    memset(this->demand_sum, 0, sizeof(int) * route_cap);
    this->upper_cost = 0;
    this->lower_cost = 0;
}

Individual::Individual(int route_cap, int node_cap, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum)
        :Individual(route_cap, node_cap) {
    this->upper_cost = upper_cost;
    this->route_num = static_cast<int>(routes.size());
    for (int i = 0; i < this->route_num; ++i) {
        this->node_num[i] = static_cast<int>(routes[i].size());
        for (int j = 0; j < this->node_num[i]; ++j) {
            this->routes[i][j] = routes[i][j];
        }
    }
    for (int i = 0; i < demand_sum.size(); ++i) {
        this->demand_sum[i] = demand_sum[i];
    }
}

Individual::~Individual() {
    for (int i = 0; i < this->route_cap; ++i) {
        delete[] this->routes[i];
    }
    delete[] this->routes;
    delete[] this->node_num;
    delete[] this->demand_sum;
}

std::ostream& operator<<(std::ostream& os, const Individual& individual) {
    os << "Route Capacity: " << individual.route_cap << "\n";
    os << "Node Capacity: " << individual.node_cap << "\n";
    os << "Number of Routes: " << individual.route_num << "\n";
    os << "Upper Cost: " << individual.upper_cost << "\n";
    os << "Lower Cost: " << individual.lower_cost << "\n";

    os << "Number of Nodes per route: ";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << individual.node_num[i] << " ";
    }
    os << "\n";

    os << "Demand sum per route: ";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << individual.demand_sum[i] << " ";
    }
    os << "\n";

    for (int i = 0; i < individual.route_cap; ++i) {
        os << "Route " << i + 1 << ": ";
        for (int j = 0; j < individual.node_cap; ++j) {
            os << individual.routes[i][j] << " ";
        }
        os << "\n";
    }
    os << "\n";

    return os;
}