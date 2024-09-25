//
// Created by Yinghao Qin on 14/09/2024.
//

#include "individual.hpp"


Individual::Individual(const Individual& ind) {
    this->route_cap = ind.route_cap;
    this->node_cap = ind.node_cap;
    this->chromosome_length = ind.chromosome_length;
    this->num_routes = ind.num_routes;
    this->upper_cost = ind.upper_cost;
    this->lower_cost = ind.lower_cost;
    this->routes = new int *[ind.route_cap];
    this->lower_routes = new int *[ind.route_cap];
    for (int i = 0; i < ind.route_cap; ++i) {
        this->routes[i] = new int[ind.node_cap];
        this->lower_routes[i] = new int[ind.node_cap];
        memcpy(this->routes[i], ind.routes[i], sizeof(int) * ind.node_cap);
        memcpy(this->lower_routes[i], ind.lower_routes[i], sizeof(int) * ind.node_cap);
    }
    this->num_nodes_per_route = new int[ind.route_cap];
    this->lower_num_nodes_per_route = new int [ind.route_cap];
    memcpy(this->num_nodes_per_route, ind.num_nodes_per_route, sizeof(int) * ind.route_cap);
    memcpy(this->lower_num_nodes_per_route, ind.lower_num_nodes_per_route, sizeof(int) * ind.route_cap);
    this->demand_sum_per_route = new int[ind.route_cap];
    memcpy(this->demand_sum_per_route, ind.demand_sum_per_route, sizeof(int) * ind.route_cap);
}

Individual::Individual(int route_cap, int node_cap) {
    this->route_cap = route_cap;
    this->node_cap = node_cap;
    this->chromosome_length = this->node_cap - 1;
    this->routes = new int *[route_cap];
    this->lower_routes = new int *[route_cap];
    for (int i = 0; i < route_cap; ++i) {
        this->routes[i] = new int[node_cap];
        this->lower_routes[i] = new int [node_cap];
        memset(this->routes[i], 0, sizeof(int) * node_cap);
        memset(this->lower_routes[i], 0, sizeof(int) * node_cap);
    }
    this->num_routes = 0;
    this->num_nodes_per_route = new int[route_cap];
    this->lower_num_nodes_per_route = new int [route_cap];
    memset(this->num_nodes_per_route, 0, sizeof(int) * route_cap);
    memset(this->lower_num_nodes_per_route, 0, sizeof(int) * route_cap);
    this->demand_sum_per_route = new int [route_cap];
    memset(this->demand_sum_per_route, 0, sizeof(int) * route_cap);
    this->upper_cost = 0;
    this->lower_cost = 0;
}

Individual::Individual(int route_cap, int node_cap, const vector<vector<int>>& routes, double upper_cost, const vector<int>& demand_sum_per_route)
        :Individual(route_cap, node_cap) {
    this->upper_cost = upper_cost;
    this->num_routes = static_cast<int>(routes.size());
    for (int i = 0; i < this->num_routes; ++i) {
        this->num_nodes_per_route[i] = static_cast<int>(routes[i].size());
        for (int j = 0; j < this->num_nodes_per_route[i]; ++j) {
            this->routes[i][j] = routes[i][j];
        }
    }
    for (int i = 0; i < demand_sum_per_route.size(); ++i) {
        this->demand_sum_per_route[i] = demand_sum_per_route[i];
    }
}

Individual::~Individual() {
    for (int i = 0; i < this->route_cap; ++i) {
        delete[] this->routes[i];
        delete[] this->lower_routes[i];
    }
    delete[] this->routes;
    delete[] this->lower_routes;
    delete[] this->num_nodes_per_route;
    delete[] this->lower_num_nodes_per_route;
    delete[] this->demand_sum_per_route;
}

void Individual::cleanup() const {
    memset(this->num_nodes_per_route + this->num_routes, 0, sizeof(int) * (this->route_cap - this->num_routes));
    memset(this->demand_sum_per_route + this->num_routes, 0, sizeof(int) * (this->route_cap - this->num_routes));
    for (int i = 0; i < this->num_routes; ++i) {
        memset(this->routes[i] + this->num_nodes_per_route[i], 0, sizeof(int) * (this->node_cap - this->num_nodes_per_route[i]));
    }
    for (int i = this->num_routes; i < this->route_cap; ++i) {
        memset(this->routes[i], 0, sizeof(int) * this->node_cap);
    }
}

void Individual::start_lower_solution() const {
    for (int i = 0; i < this->route_cap; ++i) {
        memcpy(this->lower_routes[i], this->routes[i], sizeof(int) * this->node_cap);
        this->lower_num_nodes_per_route[i] = this->num_nodes_per_route[i];
    }
}

void Individual::set_lower_cost(double lower_cost_) {
    this->lower_cost = lower_cost_;
}

vector<int> Individual::get_chromosome() const {
    vector<int> chromosome;
    chromosome.reserve(chromosome_length);
    for (int i = 0; i < num_routes; ++i) {
        for (int j = 1; j < num_nodes_per_route[i] - 1; ++j) {
            chromosome.push_back(routes[i][j]);
        }
    }
    return std::move(chromosome);
}

//void Individual::set_lower_routes(const vector<vector<int>> &lower_routes_) {
//    for (int i = 0; i < static_cast<int>(lower_routes_.size()); ++i) {
//        this->lower_num_nodes_per_route[i] = static_cast<int>(lower_routes_[i].size());
//        for (int j = 0; j < this->lower_num_nodes_per_route[i]; ++j) {
//            this->lower_routes[i][j] = lower_routes_[i][j];
//        }
//    }
//}

//void Individual::set_routes(const vector<vector<int>>& routes_) {
//    this->num_routes = static_cast<int>(routes_.size());
//    for (int i = 0; i < this->num_routes; ++i) {
//        this->num_nodes_per_route[i] = static_cast<int>(routes_[i].size());
//        for (int j = 0; j < this->num_nodes_per_route[i]; ++j) {
//            this->routes[i][j] = routes_[i][j];
//        }
//    }
//}

std::ostream& operator<<(std::ostream& os, const Individual& individual) {
    os << "Route Capacity: " << individual.route_cap << "\n";
    os << "Node Capacity: " << individual.node_cap << "\n";
    os << "Number of Routes: " << individual.num_routes << "\n";
    os << "Upper Cost: " << individual.upper_cost << "\n";
    os << "Lower Cost: " << individual.lower_cost << "\n";

    os << "Number of Nodes per route (upper): ";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << individual.num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Number of Nodes per route (lower): ";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << individual.lower_num_nodes_per_route[i] << " ";
    }
    os << "\n";

    os << "Demand sum per route: ";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << individual.demand_sum_per_route[i] << " ";
    }
    os << "\n";

    os << "Upper Routes: \n";
    for (int i = 0; i < individual.route_cap; ++i) {
        os << "Route " << i + 1 << ": ";
        for (int j = 0; j < individual.node_cap; ++j) {
            os << individual.routes[i][j] << " ";
        }
        os << "\n";
    }

    os << "Lower Routes: \n";
    for (int i = 0; i < individual.num_routes; ++i) {
        os << "Route " << i + 1 << ": ";
        for (int j = 0; j < individual.node_cap; ++j) {
            os << individual.lower_routes[i][j] << " ";
        }
        os << "\n";
    }

    return os;
}