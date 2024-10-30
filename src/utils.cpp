//
// Created by Yinghao Qin on 14/09/2024.
//

#include "utils.hpp"



/****************************************************************/
/*                  Get Upper-level Solution                    */
/****************************************************************/

pair<vector<int>, double> classical_split(const vector<int>& chromosome, Case& instance) {
    // Initialize x with an additional 0 at the start
    std::vector<int> x(chromosome.size() + 1, 0);
    std::copy(chromosome.begin(), chromosome.end(), x.begin() + 1);

    // Initialize vv and pp vectors
    std::vector<double> vv(x.size(), std::numeric_limits<double>::max());
    std::vector<int> pp(x.size(), 0);
    vv[0] = 0.0;

    // Calculate the shortest paths
    for (int i = 1; i < x.size(); ++i) {
        int load = 0;
        double cost = 0;
        int j = i;
        do {
            load += instance.get_customer_demand_(x[j]);
            if (i == j) {
                cost = instance.get_distance(instance.depot_, x[j]) * 2;
            } else {
                cost -= instance.get_distance(x[j - 1], instance.depot_);
                cost += instance.get_distance(x[j - 1], x[j]);
                cost += instance.get_distance(instance.depot_, x[j]);
            }

            if (load <= instance.max_vehicle_capa_) {
                if (vv[i - 1] + cost < vv[j]) {
                    vv[j] = vv[i - 1] + cost;
                    pp[j] = i - 1;
                }
                j++;
            }
        } while (!(j >= x.size() || load > instance.max_vehicle_capa_));
    }

    return {std::move(pp), vv[x.size() - 1]};
}

// Prins, C., 2004. A simple and effective evolutionary algorithm for the vehicle routing problem. Computers & operations research, 31(12), pp.1985-2002.
vector<vector<int>> prins_split(const vector<int>& chromosome, Case& instance) {
    vector<int> x(chromosome.size() + 1, 0); // a giant tour starts from 0
    copy(chromosome.begin(), chromosome.end(), x.begin() + 1);

    vector<double> vv(x.size(), std::numeric_limits<double>::max()); // value, the accumulated cost of the shortest path from 0 to i
    vector<int> pp(x.size(), 0); // path, record the split routes of the corresponding shortest path
    vv[0] = 0.0;

    for (int i = 1; i < x.size(); ++i) {
        int load = 0;
        double cost = 0;
        int j = i;
        do
        {
            load += instance.get_customer_demand_(x[j]);
            if (i == j) {
                cost = instance.get_distance(instance.depot_, x[j]) * 2;
            } else {
                cost -= instance.get_distance(x[j -1], instance.depot_);
                cost += instance.get_distance(x[j -1], x[j]);
                cost += instance.get_distance(instance.depot_, x[j]);
            }

            if (load <= instance.max_vehicle_capa_) {
                if (vv[i - 1] + cost < vv[j]) {
                    vv[j] = vv[i - 1] + cost;
                    pp[j] = i - 1;
                }
                j++;
            }
        } while (!(j >= x.size() || load >instance.max_vehicle_capa_));
    }

    vector<vector<int>> all_routes;
    auto j = x.size() - 1;
    while (true) {
        int i = pp[j];
        vector<int> temp(x.begin() + i + 1, x.begin() + j + 1);
        all_routes.push_back(temp);
        j = i;
        if (i == 0) {
            break;
        }
    }

    return all_routes;
}

// Hien et al., "A greedy search based evolutionary algorithm for electric vehicle routing problem", 2023.
vector<vector<int>> hien_clustering(const Case& instance, std::default_random_engine& rng) {
    vector<int> customers_(instance.customers_);

    std::shuffle(customers_.begin(), customers_.end(), rng);

    vector<vector<int>> tours;

    vector<int> tour;
    while (!customers_.empty()) {
        tour.clear();

        int anchor = customers_.front();
        customers_.erase(customers_.begin());
        tour.push_back(anchor);
        int cap = instance.get_customer_demand_(anchor);

        const vector<int> &nearby_customers = instance.customer_to_cluster_map_.at(anchor);
        for (int node: nearby_customers) {
            auto it = find(customers_.begin(), customers_.end(), node);
            if (it == customers_.end()) {
                continue;
            }
            if (cap + instance.get_customer_demand_(node) <= instance.max_vehicle_capa_) {
                tour.push_back(node);
                cap += instance.get_customer_demand_(node);
                customers_.erase(it);
            } else {
                tours.push_back(tour);
                break;
            }
        }
    }

    tours.push_back(tour);

    return std::move(tours);
}

void hien_balancing(vector<vector<int>>& routes, const Case& instance, std::default_random_engine& rng) {
    vector<int>& lastRoute = routes.back();

    uniform_int_distribution<> distribution(0, static_cast<int >(lastRoute.size() -1) );
    int customer = lastRoute[distribution(rng)];  // Randomly choose a customer from the last route

    int cap1 = 0;
    for (int node : lastRoute) {
        cap1 += instance.get_customer_demand_(node);
    }

    for (int x : instance.customer_to_cluster_map_.at(customer)) {
        if (find(lastRoute.begin(), lastRoute.end(), x) != lastRoute.end()) {
            continue;
        }

        auto route2It = find_if(routes.begin(), routes.end(), [x](const vector<int>& route) {
            return find(route.begin(), route.end(), x) != route.end();
        });

        if (route2It != routes.end()) {
            vector<int>& route2 = *route2It;
            int cap2 = 0;
            for (int node : route2) {
                cap2 += instance.get_customer_demand_(node);
            }

            int demand_X = instance.get_customer_demand_(x);

            if (demand_X + cap1 <= instance.max_vehicle_capa_ && abs((cap1 + demand_X) - (cap2 - demand_X)) < abs(cap1 - cap2)) {
                route2.erase(remove(route2.begin(), route2.end(), x), route2.end());
                lastRoute.push_back(x);
                cap1 += demand_X;
            } else {
                break;
            }
        }
    }
}

vector<vector<int>> routes_constructor_with_split(Case& instance, std::default_random_engine& rng) {
    vector<int> a_giant_tour(instance.customers_);

    shuffle(a_giant_tour.begin(), a_giant_tour.end(), rng);

    vector<vector<int>> all_routes = prins_split(a_giant_tour, instance);
    for (auto& route : all_routes) {
        route.insert(route.begin(), instance.depot_);
        route.push_back(instance.depot_);
    }

    return all_routes;
}

vector<vector<int>> routes_constructor_with_hien_method(const Case& instance, std::default_random_engine& rng){
    vector<vector<int>> routes = hien_clustering(instance, rng);
    hien_balancing(routes, instance, rng);

    for (auto& route : routes) {
        route.insert(route.begin(), 0);
        route.push_back(0);
    }

    return std::move(routes);
}

// Jia Ya-Hui, et al., "Confidence-Based Ant Colony Optimization for Capacitated Electric Vehicle Routing Problem With Comparison of Different Encoding Schemes", 2022
vector<vector<int>> routes_constructor_with_direct_encoding(const Case& instance, std::default_random_engine& rng) {
    vector<int> customers_(instance.customers_);

    int vehicle_idx = 0; // vehicle index - starts from the vehicle 0
    int load_of_one_route = 0; // the load of the current vehicle
    vector<int> route = {instance.depot_}; // the first route starts from depot_ 0

    vector<vector<int>> all_routes;
    while(!customers_.empty()) {
        vector<int> all_temp;
        for(int i : customers_) {
            if(instance.get_customer_demand_(i) <= instance.max_vehicle_capa_ - load_of_one_route) {
                all_temp.push_back(i);
            }
        }

        int remain_total_demand_ = accumulate(customers_.begin(), customers_.end(), 0, [&](int total, int i) {
            return total + instance.get_customer_demand_(i);
        });
        if(remain_total_demand_ <= instance.max_vehicle_capa_ * (instance.num_vehicle_ - vehicle_idx - 1) || all_temp.empty()) {
            all_temp.push_back(instance.depot_); // add depot_ node into the all_temp
        }

        int cur = route.back();
        uniform_int_distribution<> distribution(0, static_cast<int>(all_temp.size()) - 1);
        int next = all_temp[distribution(rng)]; // int next = roulette_wheel_selection(all_temp, cur);
        route.push_back(next);

        if (next == instance.depot_) {
            all_routes.push_back(route);
            vehicle_idx += 1;
            route = {0};
            load_of_one_route = 0;
        } else {
            load_of_one_route += instance.get_customer_demand_(next);
            customers_.erase(remove(customers_.begin(), customers_.end(), next), customers_.end());
        }
    }

    route.push_back(instance.depot_);
    all_routes.push_back(route);

    return all_routes;
}

/****************************************************************/
/*                    Local search Operators                    */
/****************************************************************/

void two_opt_for_single_route(int* route, int length, double& cost, Case& instance) {
    if (length < 5) return;

    for (size_t i = 1; i < length - 2; ++i) {
        for (size_t j = i + 1; j < length - 1; ++j) {
            // Calculate the cost difference between the old route and the new route obtained by swapping edges
            double old_cost = instance.get_distance(route[i - 1], route[i]) +
                              instance.get_distance(route[j], route[j + 1]);

            double new_cost = instance.get_distance(route[i - 1], route[j]) +
                              instance.get_distance(route[i], route[j + 1]);

            if (new_cost < old_cost) {
                // The cost variation should be considered
                reverse(route + i, route + j + 1);
                cost += new_cost - old_cost;
            }
        }
    }
}

// Croes, Georges A. "A method for solving traveling-salesman problems." Operations research 6, no. 6 (1958): 791-812.
void two_opt_for_individual(Individual& individual, Case& instance) {
    for (int i = 0; i < individual.num_routes; ++i) {
        two_opt_for_single_route(individual.routes[i],  individual.num_nodes_per_route[i], individual.upper_cost, instance);
    }
}

unordered_set<pair<int, int>, pair_hash> get_route_pairs(int num_routes) {
    unordered_set<pair<int, int>, pair_hash> route_pairs;
    for (int i = 0; i < num_routes - 1; i++) {
        for (int j = i + 1; j < num_routes; j++) {
            route_pairs.insert(make_pair(i, j));
        }
    }

    return route_pairs;
}

// Jia Ya-Hui, et al.
bool two_opt_star_for_individual(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);
    int* tempr = new int[individual.node_cap];
    int* tempr2 = new int[individual.node_cap];
    bool updated = false;
    bool updated2 = false;
    while (!route_pairs.empty())
    {
        updated2 = false;
        int r1 = route_pairs.begin()->first;
        int r2 = route_pairs.begin()->second;
        route_pairs.erase(route_pairs.begin());
        int frdem = 0;
        for (int n1 = 0; n1 < individual.num_nodes_per_route[r1] - 1; n1++) {
            frdem += instance.get_customer_demand_(individual.routes[r1][n1]);
            int srdem = 0;
            for (int n2 = 0; n2 < individual.num_nodes_per_route[r2] - 1; n2++) {
                srdem += instance.get_customer_demand_(individual.routes[r2][n2]);
                if (frdem + individual.demand_sum_per_route[r2] - srdem <= instance.max_vehicle_capa_ && srdem + individual.demand_sum_per_route[r1] - frdem <= instance.max_vehicle_capa_) {
                    double xx1 = instance.get_distance(individual.routes[r1][n1], individual.routes[r1][n1 + 1]) +
                                 instance.get_distance(individual.routes[r2][n2], individual.routes[r2][n2 + 1]);
                    double xx2 = instance.get_distance(individual.routes[r1][n1], individual.routes[r2][n2 + 1]) +
                                 instance.get_distance(individual.routes[r2][n2], individual.routes[r1][n1 + 1]);
                    double change = xx1 - xx2;
                    if (change > 0.00000001) {
                        individual.upper_cost -= change;
                        memcpy(tempr, individual.routes[r1], sizeof(int) * individual.node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2 + 1; i < individual.num_nodes_per_route[r2]; i++) {
                            individual.routes[r1][counter1] = individual.routes[r2][i];
                            counter1++;
                        }
                        int counter2 = n2 + 1;
                        for (int i = n1 + 1; i < individual.num_nodes_per_route[r1]; i++) {
                            individual.routes[r2][counter2] = tempr[i];
                            counter2++;
                        }
                        individual.num_nodes_per_route[r1] = counter1;
                        individual.num_nodes_per_route[r2] = counter2;
                        int newdemsum1 = frdem + individual.demand_sum_per_route[r2] - srdem;
                        int newdemsum2 = srdem + individual.demand_sum_per_route[r1] - frdem;
                        individual.demand_sum_per_route[r1] = newdemsum1;
                        individual.demand_sum_per_route[r2] = newdemsum2;
                        updated = true;
                        updated2 = true;
                        for (int i = 0; i < r1; i++) {
                            route_pairs.insert({i, r1});
                        }
                        for (int i = 0; i < r2; i++) {
                            route_pairs.insert({i, r2});
                        }
                        if (individual.demand_sum_per_route[r1] == 0) {
                            int* tempp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tempp;
                            individual.demand_sum_per_route[r1] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r1] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }
                        if (individual.demand_sum_per_route[r2] == 0) {
                            int* tempp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tempp;
                            individual.demand_sum_per_route[r2] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r2] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }
                        break;
                    }
                }
                else if (frdem + srdem <= instance.max_vehicle_capa_ && individual.demand_sum_per_route[r1] - frdem + individual.demand_sum_per_route[r2] - srdem <= instance.max_vehicle_capa_) {
                    double xx1 = instance.get_distance(individual.routes[r1][n1], individual.routes[r1][n1 + 1])
                                 + instance.get_distance(individual.routes[r2][n2], individual.routes[r2][n2 + 1]);
                    double xx2 = instance.get_distance(individual.routes[r1][n1], individual.routes[r2][n2])
                                 + instance.get_distance(individual.routes[r1][n1 + 1], individual.routes[r2][n2 + 1]);
                    double change = xx1 - xx2;
                    if (change > 0.00000001) {
                        individual.upper_cost -= change;
                        memcpy(tempr, individual.routes[r1], sizeof(int) * individual.node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2; i >= 0; i--) {
                            individual.routes[r1][counter1] = individual.routes[r2][i];
                            counter1++;
                        }
                        int counter2 = 0;
                        for (int i = individual.num_nodes_per_route[r1] - 1; i >= n1 + 1; i--) {
                            tempr2[counter2] = tempr[i];
                            counter2++;
                        }
                        for (int i = n2 + 1; i < individual.num_nodes_per_route[r2]; i++) {
                            tempr2[counter2] = individual.routes[r2][i];
                            counter2++;
                        }
                        memcpy(individual.routes[r2], tempr2, sizeof(int) * individual.node_cap);
                        individual.num_nodes_per_route[r1] = counter1;
                        individual.num_nodes_per_route[r2] = counter2;

                        int newdemsum1 = frdem + srdem;
                        int newdemsum2 = individual.demand_sum_per_route[r1] + individual.demand_sum_per_route[r2] - frdem - srdem;
                        individual.demand_sum_per_route[r1] = newdemsum1;
                        individual.demand_sum_per_route[r2] = newdemsum2;
                        updated = true;
                        updated2 = true;
                        for (int i = 0; i < r1; i++) {
                            route_pairs.insert({i, r1});
                        }
                        for (int i = 0; i < r2; i++) {
                            route_pairs.insert({i, r2});
                        }
                        if (individual.demand_sum_per_route[r1] == 0) {
                            int* tempp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tempp;
                            individual.demand_sum_per_route[r1] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r1] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }
                        if (individual.demand_sum_per_route[r2] == 0) {
                            int* tempp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tempp;
                            individual.demand_sum_per_route[r2] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r2] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }
                        break;
                    }
                }
            }
            if (updated2) break;
        }
    }
    delete[] tempr;
    delete[] tempr2;
    individual.cleanup();
    return updated;
}

bool node_shift(int* route, int length, double& cost, Case& instance) {
    if (length <= 4) return false;

    double old_cost, new_cost;
    for (int i = 1; i < length - 1; i++) {
        for (int j = 1; j < length - 1; j++) {
            if (i == j) continue;

            if (i < j) {
                old_cost = instance.get_distance(route[i - 1], route[i]) + instance.get_distance(route[i], route[i + 1]) + instance.get_distance(route[j], route[j + 1]);
                new_cost = instance.get_distance(route[i - 1], route[i + 1]) + instance.get_distance(route[j], route[i]) + instance.get_distance(route[i], route[j + 1]);
            } else {
                old_cost = instance.get_distance(route[i - 1], route[i]) + instance.get_distance(route[i], route[i + 1]) + instance.get_distance(route[j - 1], route[j]);
                new_cost = instance.get_distance(route[j - 1], route[i]) + instance.get_distance(route[i], route[j]) + instance.get_distance(route[i - 1], route[i + 1]);
            }

            if (new_cost < old_cost) {
                moveItoJ(route, i, j);
                cost -= (old_cost - new_cost);
            }
        }
    }

    return true;
}

void moveItoJ(int* route, int a, int b) {
    int x = route[a];
    if (a < b) {
        for (int i = a; i < b; i++) {
            route[i] = route[i + 1];
        }
        route[b] = x;
    }
    else if (a > b) {
        for (int i = a; i > b; i--) {
            route[i] = route[i - 1];
        }
        route[b] = x;
    }
}

void one_point_move_intra_route_for_individual(Individual& individual, Case& instance) {
    for (int i = 0; i < individual.num_routes; i++) {
        node_shift(individual.routes[i], individual.num_nodes_per_route[i], individual.upper_cost, instance);
    }
}

// node shift between two routes, inter-route operator for One Point move (i.e., customer insertion)
// Toth, Paolo, and Daniele Vigo. "The granular tabu search and its application to the vehicle-routing problem." Informs Journal on computing 15, no. 4 (2003): 333-346.
bool node_shift_between_two_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, Case& instance) {
    if (length1 < 3 || length2 < 3) return false;

    for (int i = 1; i < length1 - 1; i++) {
        // vehicle capacity constraint check
        if (loading2 + instance.get_customer_demand_(route1[i]) > instance.max_vehicle_capa_) continue;

        for (int j = 0; j < length2 - 1; j++) {
            double old_cost = instance.get_distance(route1[i - 1], route1[i]) + instance.get_distance(route1[i], route1[i + 1]) + instance.get_distance(route2[j], route2[j + 1]);
            double new_cost = instance.get_distance(route1[i - 1], route1[i + 1]) + instance.get_distance(route2[j], route1[i]) + instance.get_distance(route1[i], route2[j + 1]);

            if (new_cost < old_cost) {
                int x = route1[i];
                for (int p = i; p < length1 - 1; p++) {
                    route1[p] = route1[p + 1];
                }
                length1--;
                loading1 -= instance.get_customer_demand_(x);
                for (int q = length2; q > j + 1; q--) {
                    route2[q] = route2[q - 1];
                }
                route2[j + 1] = x;
                length2++;
                loading2 += instance.get_customer_demand_(x);
                cost -= (old_cost - new_cost);
                return true;
            }
        }
    }

    return false;
}

bool one_point_move_inter_route_for_individual(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);

    while (!route_pairs.empty())
    {
        int r1 = route_pairs.begin()->first;
        int r2 = route_pairs.begin()->second;
        route_pairs.erase(route_pairs.begin());
        node_shift_between_two_routes(individual.routes[r1], individual.routes[r2], individual.num_nodes_per_route[r1], individual.num_nodes_per_route[r2],
                                      individual.demand_sum_per_route[r1], individual.demand_sum_per_route[r2], individual.upper_cost, instance);
    }

    // iterate the variable "demand_sum_per_route" to remove the empty route, if the demand_sum_per_route is 0, then remove the route
    for (int i = 0; i < individual.num_routes; i++) {
        if (individual.demand_sum_per_route[i] == 0) {
            for (int j = i; j < individual.num_routes - 1; j++) {
                int* temp = individual.routes[j];
                individual.routes[j] = individual.routes[j + 1];
                individual.routes[j + 1] = temp;
                individual.num_nodes_per_route[j] = individual.num_nodes_per_route[j + 1];
                individual.demand_sum_per_route[j] = individual.demand_sum_per_route[j + 1];
            }
            individual.num_routes--;
            i--;
        }
    }

    // update the variable "num_routes" and "route_cap" to remove the empty route
    for (size_t i = individual.num_routes; i < individual.route_cap; ++i) {
        individual.num_nodes_per_route[i] = 0;
        individual.demand_sum_per_route[i] = 0;
    }

    return true;
}

// swap two nodes within a route
void two_nodes_swap_for_single_route(int* route, int length, double& cost, Case& instance) {
    // boundary check
    if (length < 5) return;

    // adjacent nodes do not swap
    for(int i = 1; i < length - 3; i++) {
        for(int j = i + 2; j < length - 1; j++) {
            double old_cost = instance.get_distance(route[i - 1], route[i]) + instance.get_distance(route[i], route[i + 1])
                              + instance.get_distance(route[j - 1], route[j]) + instance.get_distance(route[j], route[j + 1]);
            double new_cost = instance.get_distance(route[i - 1], route[j]) + instance.get_distance(route[j], route[i + 1])
                              + instance.get_distance(route[j - 1], route[i]) + instance.get_distance(route[i], route[j + 1]);
            if (new_cost < old_cost) {
                swap(route[i], route[j]);
                cost -= old_cost - new_cost;
            }
        }
    }
}

void two_point_move_intra_route_for_individual(Individual& individual, Case& instance) {
    for (int i = 0; i < individual.num_routes; i++) {
        two_nodes_swap_for_single_route(individual.routes[i], individual.num_nodes_per_route[i], individual.upper_cost, instance);
    }
}

// swap two nodes between two routes
bool two_nodes_swap_between_two_routes(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2, double& cost, Case& instance) {
    // boundary check
    if (length1 < 3 || length2 < 3) return false;

    // vehicle capacity constraint check and fitness improvement check
    for (int i = 1; i < length1 - 1; i++) {
        for (int j = 1; j < length2 - 1; j++) {
            int demand_I = instance.get_customer_demand_(route1[i]);
            int demand_J = instance.get_customer_demand_(route2[j]);
            if (loading1 - demand_I + demand_J <= instance.max_vehicle_capa_ && loading2 - demand_J + demand_I <= instance.max_vehicle_capa_) {
                double old_cost = instance.get_distance(route1[i - 1], route1[i]) + instance.get_distance(route1[i], route1[i + 1])
                                 + instance.get_distance(route2[j - 1], route2[j]) + instance.get_distance(route2[j], route2[j + 1]);
                double new_cost = instance.get_distance(route1[i - 1], route2[j]) + instance.get_distance(route2[j], route1[i + 1])
                                 + instance.get_distance(route2[j - 1], route1[i]) + instance.get_distance(route1[i], route2[j + 1]);
                if (new_cost < old_cost) {
                    swap(route1[i], route2[j]);
                    loading1 = loading1 - demand_I + demand_J;
                    loading2 = loading2 - demand_J + demand_I;
                    cost -= old_cost - new_cost;
                }
            }
        }
    }

    return true;
}

bool two_point_move_inter_route_for_individual(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);

    while (!route_pairs.empty())
    {
        int r1 = route_pairs.begin()->first;
        int r2 = route_pairs.begin()->second;
        route_pairs.erase(route_pairs.begin());
        two_nodes_swap_between_two_routes(individual.routes[r1], individual.routes[r2], individual.num_nodes_per_route[r1], individual.num_nodes_per_route[r2],
                                          individual.demand_sum_per_route[r1], individual.demand_sum_per_route[r2], individual.upper_cost, instance);
    }

    return true;
}

/****************************************************************/
/*                   Recharging Optimization                    */
/****************************************************************/

double fix_one_solution(Individual &individual, Case& instance) {
    double lower_cost = 0.0;

    individual.start_lower_solution();
    for (int i = 0; i < individual.num_routes; ++i) {
        double cost_SE = insert_station_by_simple_enumeration_array(individual.routes[i], individual.num_nodes_per_route[i], individual.lower_routes[i], individual.lower_num_nodes_per_route[i], instance);

        if (cost_SE == -1) {
            double cost_RE = insert_station_by_remove_array(individual.routes[i], individual.num_nodes_per_route[i], individual.lower_routes[i], individual.lower_num_nodes_per_route[i], instance);
            if (cost_RE == -1) {
                lower_cost += INFEASIBLE;
            } else {
                lower_cost += cost_RE;
            }
        } else {
            lower_cost += cost_SE;
        }

    }

    individual.set_lower_cost(lower_cost);
    return lower_cost;
}

double insert_station_by_simple_enumeration_array(int* route, int length, int* repaired_route, int& repaired_length, Case& instance) {
    vector<double> accumulateDistance(length, 0);
    for (int i = 1; i < length; i++) {
        accumulateDistance[i] = accumulateDistance[i - 1] + instance.get_distance(route[i], route[i - 1]);
    }
    if (accumulateDistance.back() <= instance.max_distance_) {
        return accumulateDistance.back();
    }

    int ub = (int)(accumulateDistance.back() / instance.max_distance_ + 1);
    int lb = (int)(accumulateDistance.back() / instance.max_distance_);
    int* chosenPos = new int[length];
    int* bestChosenPos = new int[length]; // customized variable
    double final_cost = numeric_limits<double>::max();
    double best_cost = final_cost; // customized variable
    for (int i = lb; i <= ub; i++) {
        tryACertainNArray(0, i, chosenPos, bestChosenPos, final_cost, i, route, length, accumulateDistance, instance);

        if (final_cost < best_cost) {
            memset(repaired_route, 0, sizeof(int) * repaired_length);
            int currentIndex = 0;
            int idx = 0;
            for (int j = 0; j < i; ++j) {
                int from = route[bestChosenPos[j]];
                int to = route[bestChosenPos[j] + 1];
                int station = instance.best_station_[from][to];

                int numElementsToCopy = bestChosenPos[j] + 1 - idx;
                memcpy(&repaired_route[currentIndex], &route[idx], numElementsToCopy * sizeof(int));

                currentIndex += numElementsToCopy;

                repaired_route[currentIndex++] = station;
                idx = bestChosenPos[j] + 1;
            }

            int remainingElementsToCopy = length - idx;
            memcpy(&repaired_route[currentIndex], &route[idx], remainingElementsToCopy * sizeof(int));
            repaired_length = currentIndex + remainingElementsToCopy;

            best_cost = final_cost;
        }
    }
    delete[] chosenPos;
    delete[] bestChosenPos;
    if (final_cost !=  numeric_limits<double>::max()) {
        return final_cost;
    }
    else {
        return -1;
    }
}

double insert_station_by_remove_array(int* route, int length, int* repaired_route, int& repaired_length, Case& instance) {
    list<pair<int, int>> stationInserted;
    for (int i = 0; i < length - 1; i++) {
        double allowedDis = instance.max_distance_;
        if (i != 0) {
            allowedDis = instance.max_distance_ - instance.get_distance(stationInserted.back().second, route[i]);
        }
        int onestation = instance.get_best_and_feasible_station(route[i], route[i + 1], allowedDis);
        if (onestation == -1) return -1;
        stationInserted.emplace_back(i, onestation);
    }
    while (!stationInserted.empty())
    {
        bool change = false;
        auto delone = stationInserted.begin();
        double savedis = 0;
        auto itr = stationInserted.begin();
        auto next = itr;
        next++;
        if (next != stationInserted.end()) {
            int endInd = next->first;
            int endstation = next->second;
            double sumdis = 0;
            for (int i = 0; i < endInd; i++) {
                sumdis += instance.get_distance(route[i], route[i + 1]);
            }
            sumdis += instance.get_distance(route[endInd], endstation);
            if (sumdis <= instance.max_distance_) {
                savedis = instance.get_distance(route[itr->first], itr->second)
                          + instance.get_distance(itr->second, route[itr->first + 1])
                          - instance.get_distance(route[itr->first], route[itr->first + 1]);
            }
        }
        else {
            double sumdis = 0;
            for (int i = 0; i < length - 1; i++) {
                sumdis += instance.get_distance(route[i], route[i + 1]);
            }
            if (sumdis <= instance.max_distance_) {
                savedis = instance.get_distance(route[itr->first], itr->second)
                          + instance.get_distance(itr->second, route[itr->first + 1])
                          - instance.get_distance(route[itr->first], route[itr->first + 1]);
            }
        }
        itr++;
        while (itr != stationInserted.end())
        {
            int startInd, endInd;
            next = itr;
            next++;
            auto prev = itr;
            prev--;
            double sumdis = 0;
            if (next != stationInserted.end()) {
                startInd = prev->first + 1;
                endInd = next->first;
                sumdis += instance.get_distance(prev->second, route[startInd]);
                for (int i = startInd; i < endInd; i++) {
                    sumdis += instance.get_distance(route[i], route[i + 1]);
                }
                sumdis += instance.get_distance(route[endInd], next->second);
                if (sumdis <= instance.max_distance_) {
                    double savedistemp = instance.get_distance(route[itr->first], itr->second)
                                         + instance.get_distance(itr->second, route[itr->first + 1])
                                         - instance.get_distance(route[itr->first], route[itr->first + 1]);
                    if (savedistemp > savedis) {
                        savedis = savedistemp;
                        delone = itr;
                    }
                }
            }
            else {
                startInd = prev->first + 1;
                sumdis += instance.get_distance(prev->second, route[startInd]);
                for (int i = startInd; i < length - 1; i++) {
                    sumdis += instance.get_distance(route[i], route[i + 1]);
                }
                if (sumdis <= instance.max_distance_) {
                    double savedistemp = instance.get_distance(route[itr->first], itr->second)
                                         + instance.get_distance(itr->second, route[itr->first + 1])
                                         - instance.get_distance(route[itr->first], route[itr->first + 1]);
                    if (savedistemp > savedis) {
                        savedis = savedistemp;
                        delone = itr;
                    }
                }
            }
            itr++;
        }
        if (savedis != 0) {
            stationInserted.erase(delone);
            change = true;
        }
        if (!change) {
            break;
        }
    }
    double sum = 0;
    for (int i = 0; i < length - 1; i++) {
        sum += instance.get_distance(route[i], route[i + 1]);
    }
    int currentIndex = 0;
    int idx = 0;
    for (auto& e : stationInserted) {
        int pos = e.first;
        int stat = e.second;
        sum -= instance.get_distance(route[pos], route[pos + 1]);
        sum += instance.get_distance(route[pos], stat);
        sum += instance.get_distance(stat, route[pos + 1]);

        int numElementsToCopy = pos + 1 - idx;
        memcpy(&repaired_route[currentIndex], &route[idx], numElementsToCopy * sizeof(int));
        currentIndex += numElementsToCopy;

        repaired_route[currentIndex++] = stat;

        idx = pos + 1;
    }
    int remainingElementsToCopy = length - idx;
    memcpy(&repaired_route[currentIndex], &route[idx], remainingElementsToCopy * sizeof(int));
    repaired_length = currentIndex + remainingElementsToCopy;

    return sum;
}

void tryACertainNArray(int mlen, int nlen, int* chosenPos, int* bestChosenPos, double& finalfit, int curub, int* route, int length, vector<double>& accumulateDis, Case& instance) {
    for (int i = mlen; i <= length - 1 - nlen; i++) {
        if (curub == nlen) {
            double onedis = instance.get_distance(route[i], instance.best_station_[route[i]][route[i + 1]]);
            if (accumulateDis[i] + onedis > instance.max_distance_) {
                break;
            }
        }
        else {
            int lastpos = chosenPos[curub - nlen - 1];
            double onedis = instance.get_distance(route[lastpos + 1], instance.best_station_[route[lastpos]][route[lastpos + 1]]);
            double twodis = instance.get_distance(route[i], instance.best_station_[route[i]][route[i + 1]]);
            if (accumulateDis[i] - accumulateDis[lastpos + 1] + onedis + twodis > instance.max_distance_) {
                break;
            }
        }
        if (nlen == 1) {
            double onedis = accumulateDis.back() - accumulateDis[i + 1] + instance.get_distance(instance.best_station_[route[i]][route[i + 1]], route[i + 1]);
            if (onedis > instance.max_distance_) {
                continue;
            }
        }

        chosenPos[curub - nlen] = i;
        if (nlen > 1) {
            tryACertainNArray(i + 1, nlen - 1, chosenPos,  bestChosenPos, finalfit, curub, route, length, accumulateDis, instance);
        }
        else {
            double disum = accumulateDis.back();
            for (int j = 0; j < curub; j++) {
                int firstnode = route[chosenPos[j]];
                int secondnode = route[chosenPos[j] + 1];
                int thestation = instance.best_station_[firstnode][secondnode];
                disum -= instance.get_distance(firstnode, secondnode);
                disum += instance.get_distance(firstnode, thestation);
                disum += instance.get_distance(secondnode, thestation);
            }
            if (disum < finalfit) {
                finalfit = disum;
                for (int j = 0; j < length; ++j) {
                    bestChosenPos[j] = chosenPos[j];
                }
            }
        }
    }
}

