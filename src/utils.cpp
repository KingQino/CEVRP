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

        int* nearby_customers = instance.sorted_nearby_customers[anchor];
        int length = instance.num_customer_ - 1; // the size of nearby_customers

        for (int i = 0; i < length; ++i) {
            int node = nearby_customers[i];
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

    int size = instance.num_customer_ - 1; // the size of nearby_customers

    for (int i = 0; i < size; ++i) {
        int x = instance.sorted_nearby_customers[customer][i];

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

bool contains(const int* array, int size, int element) {
    for (int i = 0; i < size; i++) {
        if (array[i] == element) {
            return true;  // Element found
        }
    }
    return false;  // Element not found
}

void two_opt_for_single_route_acceleration(int* route, int length, double& cost, Case& instance) {
    if (length < 5) return;
    int size = instance.restricted_candidate_list_size_;

    for (size_t i = 1; i < length - 2; ++i) {
        for (size_t j = i + 1; j < length - 1; ++j) {

            if (!contains(instance.sorted_nearby_customers[route[i - 1]], size, route[j]) ||
                !contains(instance.sorted_nearby_customers[route[i]], size, route[j + 1])) {
                continue; // restricted candidates check
            }

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

void update_route_pairs(unordered_set<pair<int, int>, pair_hash>& route_pairs, int r1, int r2) {
    for (int i = 0; i < r1; i++) route_pairs.insert({i, r1});
    for (int i = 0; i < r2; i++) route_pairs.insert({i, r2});
}

bool two_opt_star_between_two_routes(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, int node_cap, Case& instance) {
    if (length1 < 3 || length2 < 3) return false;

    bool updated = false;
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    int* temp_r1 = new int[node_cap];
    int* temp_r2 = new int[node_cap];

    for (int n1 = 0; n1 < length1 - 1 && !updated; n1++) {
        partial_dem_r1 += instance.get_customer_demand_(route1[n1]);

        int partial_dem_r2 = 0; // the partial demand of route r2
        for (int n2 = 0; n2 < length2 - 1; n2++) {
            partial_dem_r2 += instance.get_customer_demand_(route2[n2]);

            if (partial_dem_r1 + loading2 - partial_dem_r2 <= instance.max_vehicle_capa_ && partial_dem_r2 + loading1 - partial_dem_r1 <= instance.max_vehicle_capa_) {
                double old_cost = instance.get_distance(route1[n1], route1[n1 + 1]) + instance.get_distance(route2[n2], route2[n2 + 1]);
                double new_cost = instance.get_distance(route1[n1], route2[n2 + 1]) + instance.get_distance(route2[n2], route1[n1 + 1]);

                if (new_cost < old_cost) {
                    // update
                    cost -= (old_cost - new_cost);
                    memcpy(temp_r1, route1, sizeof(int) * node_cap);
                    int counter1 = n1 + 1;
                    for (int i = n2 + 1; i < length2; i++) {
                        route1[counter1++] = route2[i];
                    }
                    int counter2 = n2 + 1;
                    for (int i = n1 + 1; i < length1; i++) {
                        route2[counter2++] = temp_r1[i];
                    }
                    length1 = counter1;
                    length2 = counter2;
                    int new_dem_sum_1 = partial_dem_r1 + loading2 - partial_dem_r2;
                    int new_dem_sum_2 = partial_dem_r2 + loading1 - partial_dem_r1;
                    loading1 = new_dem_sum_1;
                    loading2 = new_dem_sum_2;

                    updated = true;
                    break;
                }
            } else if (partial_dem_r1 + partial_dem_r2 <= instance.max_vehicle_capa_ && loading1 - partial_dem_r1 + loading2 - partial_dem_r2 <= instance.max_vehicle_capa_) {
                double old_cost = instance.get_distance(route1[n1], route1[n1 + 1]) + instance.get_distance(route2[n2], route2[n2 + 1]);
                double new_cost = instance.get_distance(route1[n1], route2[n2]) + instance.get_distance(route1[n1 + 1], route2[n2 + 1]);

                if (new_cost < old_cost) {
                    cost -= (old_cost - new_cost);
                    memcpy(temp_r1, route1, sizeof(int) * node_cap);
                    int counter1 = n1 + 1;
                    for (int i = n2; i >= 0; i--) {
                        route1[counter1++] = route2[i];
                    }
                    int counter2 = 0;
                    for (int i = length1 - 1; i >= n1 + 1; i--) {
                        temp_r2[counter2++] = temp_r1[i];
                    }
                    for (int i = n2 + 1; i < length2; i++) {
                        temp_r2[counter2++] = route2[i];
                    }
                    memcpy(route2, temp_r2, sizeof(int) * node_cap);
                    length1 = counter1;
                    length2 = counter2;
                    int new_dem_sum_1 = partial_dem_r1 + partial_dem_r2;
                    int new_dem_sum_2 = loading1 + loading2 - partial_dem_r1 - partial_dem_r2;
                    loading1 = new_dem_sum_1;
                    loading2 = new_dem_sum_2;

                    updated = true;
                    break;
                }
            }
        }
    }

    delete[] temp_r1;
    delete[] temp_r2;

    return updated;
}

bool two_opt_star_between_two_routes_acceleration(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, int node_cap, Case& instance) {
    if (length1 < 3 || length2 < 3) return false;

    int size = instance.restricted_candidate_list_size_;

    bool updated = false;
    int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route
    int* temp_r1 = new int[node_cap];
    int* temp_r2 = new int[node_cap];

    for (int n1 = 0; n1 < length1 - 1 && !updated; n1++) {
        partial_dem_r1 += instance.get_customer_demand_(route1[n1]);

        int partial_dem_r2 = 0; // the partial demand of route r2
        for (int n2 = 0; n2 < length2 - 1; n2++) {
            partial_dem_r2 += instance.get_customer_demand_(route2[n2]);

            if (partial_dem_r1 + loading2 - partial_dem_r2 <= instance.max_vehicle_capa_ && partial_dem_r2 + loading1 - partial_dem_r1 <= instance.max_vehicle_capa_) {
                if (!contains(instance.sorted_nearby_customers[route1[n1]], size, route2[n2 + 1]) ||
                    !contains(instance.sorted_nearby_customers[route2[n2]], size, route1[n1 + 1])) {
                    continue;
                }
                double old_cost = instance.get_distance(route1[n1], route1[n1 + 1]) + instance.get_distance(route2[n2], route2[n2 + 1]);
                double new_cost = instance.get_distance(route1[n1], route2[n2 + 1]) + instance.get_distance(route2[n2], route1[n1 + 1]);

                if (new_cost < old_cost) {
                    // update
                    cost -= (old_cost - new_cost);
                    memcpy(temp_r1, route1, sizeof(int) * node_cap);
                    int counter1 = n1 + 1;
                    for (int i = n2 + 1; i < length2; i++) {
                        route1[counter1++] = route2[i];
                    }
                    int counter2 = n2 + 1;
                    for (int i = n1 + 1; i < length1; i++) {
                        route2[counter2++] = temp_r1[i];
                    }
                    length1 = counter1;
                    length2 = counter2;
                    int new_dem_sum_1 = partial_dem_r1 + loading2 - partial_dem_r2;
                    int new_dem_sum_2 = partial_dem_r2 + loading1 - partial_dem_r1;
                    loading1 = new_dem_sum_1;
                    loading2 = new_dem_sum_2;

                    updated = true;
                    break;
                }
            } else if (partial_dem_r1 + partial_dem_r2 <= instance.max_vehicle_capa_ && loading1 - partial_dem_r1 + loading2 - partial_dem_r2 <= instance.max_vehicle_capa_) {
                if (!contains(instance.sorted_nearby_customers[route1[n1]], size, route2[n2]) ||
                    !contains(instance.sorted_nearby_customers[route1[n1 + 1]], size, route2[n2 + 1])) {
                    continue;
                }
                double old_cost = instance.get_distance(route1[n1], route1[n1 + 1]) + instance.get_distance(route2[n2], route2[n2 + 1]);
                double new_cost = instance.get_distance(route1[n1], route2[n2]) + instance.get_distance(route1[n1 + 1], route2[n2 + 1]);

                if (new_cost < old_cost) {
                    cost -= (old_cost - new_cost);
                    memcpy(temp_r1, route1, sizeof(int) * node_cap);
                    int counter1 = n1 + 1;
                    for (int i = n2; i >= 0; i--) {
                        route1[counter1++] = route2[i];
                    }
                    int counter2 = 0;
                    for (int i = length1 - 1; i >= n1 + 1; i--) {
                        temp_r2[counter2++] = temp_r1[i];
                    }
                    for (int i = n2 + 1; i < length2; i++) {
                        temp_r2[counter2++] = route2[i];
                    }
                    memcpy(route2, temp_r2, sizeof(int) * node_cap);
                    length1 = counter1;
                    length2 = counter2;
                    int new_dem_sum_1 = partial_dem_r1 + partial_dem_r2;
                    int new_dem_sum_2 = loading1 + loading2 - partial_dem_r1 - partial_dem_r2;
                    loading1 = new_dem_sum_1;
                    loading2 = new_dem_sum_2;

                    updated = true;
                    break;
                }
            }
        }
    }

    delete[] temp_r1;
    delete[] temp_r2;

    return updated;
}

bool two_opt_move_inter_route_for_individual_acceleration(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) return false;

    bool flag = false;

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);
    while (!route_pairs.empty()) {
        auto [r1, r2] = *route_pairs.begin();
        route_pairs.erase(route_pairs.begin());

        bool updated = two_opt_star_between_two_routes_acceleration(individual.routes[r1], individual.routes[r2],
                                                       individual.num_nodes_per_route[r1], individual.num_nodes_per_route[r2],
                                                       individual.demand_sum_per_route[r1], individual.demand_sum_per_route[r2],
                                                       individual.upper_cost, individual.node_cap, instance);

        // update route pairs
        if (updated) {
            flag = true;
            update_route_pairs(route_pairs, r1, r2);
        }

        // remove empty routes
        if (individual.demand_sum_per_route[r1] == 0) {
            int* tmp = individual.routes[r1];
            individual.routes[r1] = individual.routes[individual.num_routes - 1];
            individual.routes[individual.num_routes - 1] = tmp;
            individual.demand_sum_per_route[r1] = individual.demand_sum_per_route[individual.num_routes - 1];
            individual.num_nodes_per_route[r1] = individual.num_nodes_per_route[individual.num_routes - 1];
            individual.num_routes--;
            for (int i = 0; i < individual.num_routes; i++) {
                route_pairs.erase({i, individual.num_routes});
            }
        }
        if (individual.demand_sum_per_route[r2] == 0) {
            int* tmp = individual.routes[r2];
            individual.routes[r2] = individual.routes[individual.num_routes - 1];
            individual.routes[individual.num_routes - 1] = tmp;
            individual.demand_sum_per_route[r2] = individual.demand_sum_per_route[individual.num_routes - 1];
            individual.num_nodes_per_route[r2] = individual.num_nodes_per_route[individual.num_routes - 1];
            individual.num_routes--;
            for (int i = 0; i < individual.num_routes; i++) {
                route_pairs.erase({i, individual.num_routes});
            }
        }
    }

    individual.cleanup(); // TODO: check if this is necessary
    return flag;
}

bool two_opt_move_inter_route_for_individual(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) return false;

    bool flag = false;

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);
    while (!route_pairs.empty()) {
        auto [r1, r2] = *route_pairs.begin();
        route_pairs.erase(route_pairs.begin());

        bool updated = two_opt_star_between_two_routes(individual.routes[r1], individual.routes[r2],
                                                  individual.num_nodes_per_route[r1], individual.num_nodes_per_route[r2],
                                                  individual.demand_sum_per_route[r1], individual.demand_sum_per_route[r2],
                                                  individual.upper_cost, individual.node_cap, instance);

        // update route pairs
        if (updated) {
            flag = true;
            update_route_pairs(route_pairs, r1, r2);
        }

        // remove empty routes
        if (individual.demand_sum_per_route[r1] == 0) {
            int* tmp = individual.routes[r1];
            individual.routes[r1] = individual.routes[individual.num_routes - 1];
            individual.routes[individual.num_routes - 1] = tmp;
            individual.demand_sum_per_route[r1] = individual.demand_sum_per_route[individual.num_routes - 1];
            individual.num_nodes_per_route[r1] = individual.num_nodes_per_route[individual.num_routes - 1];
            individual.num_routes--;
            for (int i = 0; i < individual.num_routes; i++) {
                route_pairs.erase({i, individual.num_routes});
            }
        }
        if (individual.demand_sum_per_route[r2] == 0) {
            int* tmp = individual.routes[r2];
            individual.routes[r2] = individual.routes[individual.num_routes - 1];
            individual.routes[individual.num_routes - 1] = tmp;
            individual.demand_sum_per_route[r2] = individual.demand_sum_per_route[individual.num_routes - 1];
            individual.num_nodes_per_route[r2] = individual.num_nodes_per_route[individual.num_routes - 1];
            individual.num_routes--;
            for (int i = 0; i < individual.num_routes; i++) {
                route_pairs.erase({i, individual.num_routes});
            }
        }
    }

//    individual.cleanup(); // TODO: check if this is necessary
    return flag;
}

// Jia Ya-Hui, et al.
bool two_opt_star_for_individual(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);
    int* temp_r1 = new int[individual.node_cap];
    int* temp_r2 = new int[individual.node_cap];
    bool updated = false;

    while (!route_pairs.empty())
    {
        auto [r1, r2] = *route_pairs.begin();
        route_pairs.erase(route_pairs.begin());
        int partial_dem_r1 = 0; // the partial demand of route r1, i.e., the head partial route

        for (int n1 = 0; n1 < individual.num_nodes_per_route[r1] - 1 && !updated; n1++) {
            partial_dem_r1 += instance.get_customer_demand_(individual.routes[r1][n1]);

            int partial_dem_r2 = 0; // the partial demand of route r2
            for (int n2 = 0; n2 < individual.num_nodes_per_route[r2] - 1; n2++) {
                partial_dem_r2 += instance.get_customer_demand_(individual.routes[r2][n2]);

                if (partial_dem_r1 + individual.demand_sum_per_route[r2] - partial_dem_r2 <= instance.max_vehicle_capa_ && partial_dem_r2 + individual.demand_sum_per_route[r1] - partial_dem_r1 <= instance.max_vehicle_capa_) {
                    double old_cost = instance.get_distance(individual.routes[r1][n1], individual.routes[r1][n1 + 1]) +
                                      instance.get_distance(individual.routes[r2][n2], individual.routes[r2][n2 + 1]);
                    double new_cost = instance.get_distance(individual.routes[r1][n1], individual.routes[r2][n2 + 1]) +
                                      instance.get_distance(individual.routes[r2][n2], individual.routes[r1][n1 + 1]);
                    double change = old_cost - new_cost;
                    if (change > 0.000'000'01) {
                        // update individual
                        individual.upper_cost -= change;
                        memcpy(temp_r1, individual.routes[r1], sizeof(int) * individual.node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2 + 1; i < individual.num_nodes_per_route[r2]; i++) {
                            individual.routes[r1][counter1] = individual.routes[r2][i];
                            counter1++;
                        }
                        int counter2 = n2 + 1;
                        for (int i = n1 + 1; i < individual.num_nodes_per_route[r1]; i++) {
                            individual.routes[r2][counter2] = temp_r1[i];
                            counter2++;
                        }
                        individual.num_nodes_per_route[r1] = counter1;
                        individual.num_nodes_per_route[r2] = counter2;
                        int new_dem_sum_1 = partial_dem_r1 + individual.demand_sum_per_route[r2] - partial_dem_r2;
                        int new_dem_sum_2 = partial_dem_r2 + individual.demand_sum_per_route[r1] - partial_dem_r1;
                        individual.demand_sum_per_route[r1] = new_dem_sum_1;
                        individual.demand_sum_per_route[r2] = new_dem_sum_2;

                        // update route pairs
                        update_route_pairs(route_pairs, r1, r2);

                        // remove empty routes
                        if (individual.demand_sum_per_route[r1] == 0) {
                            int* tmp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tmp;
                            individual.demand_sum_per_route[r1] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r1] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }
                        if (individual.demand_sum_per_route[r2] == 0) {
                            int* tmp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tmp;
                            individual.demand_sum_per_route[r2] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r2] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }

                        updated = true;
                        break;
                    }
                }
                else if (partial_dem_r1 + partial_dem_r2 <= instance.max_vehicle_capa_ && individual.demand_sum_per_route[r1] - partial_dem_r1 + individual.demand_sum_per_route[r2] - partial_dem_r2 <= instance.max_vehicle_capa_) {
                    double old_cost = instance.get_distance(individual.routes[r1][n1], individual.routes[r1][n1 + 1]) +
                                      instance.get_distance(individual.routes[r2][n2], individual.routes[r2][n2 + 1]);
                    double new_cost = instance.get_distance(individual.routes[r1][n1], individual.routes[r2][n2]) +
                                      instance.get_distance(individual.routes[r1][n1 + 1], individual.routes[r2][n2 + 1]);
                    double change = old_cost - new_cost;
                    if (change > 0.000'000'01) {
                        individual.upper_cost -= change;
                        memcpy(temp_r1, individual.routes[r1], sizeof(int) * individual.node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2; i >= 0; i--) {
                            individual.routes[r1][counter1] = individual.routes[r2][i];
                            counter1++;
                        }
                        int counter2 = 0;
                        for (int i = individual.num_nodes_per_route[r1] - 1; i >= n1 + 1; i--) {
                            temp_r2[counter2] = temp_r1[i];
                            counter2++;
                        }
                        for (int i = n2 + 1; i < individual.num_nodes_per_route[r2]; i++) {
                            temp_r2[counter2] = individual.routes[r2][i];
                            counter2++;
                        }
                        memcpy(individual.routes[r2], temp_r2, sizeof(int) * individual.node_cap);
                        individual.num_nodes_per_route[r1] = counter1;
                        individual.num_nodes_per_route[r2] = counter2;

                        int new_dem_sum_1 = partial_dem_r1 + partial_dem_r2;
                        int new_dem_sum_2 = individual.demand_sum_per_route[r1] + individual.demand_sum_per_route[r2] - partial_dem_r1 - partial_dem_r2;
                        individual.demand_sum_per_route[r1] = new_dem_sum_1;
                        individual.demand_sum_per_route[r2] = new_dem_sum_2;

                        update_route_pairs(route_pairs, r1, r2);

                        if (individual.demand_sum_per_route[r1] == 0) {
                            int* tmp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tmp;
                            individual.demand_sum_per_route[r1] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r1] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }
                        if (individual.demand_sum_per_route[r2] == 0) {
                            int* tmp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.num_routes - 1];
                            individual.routes[individual.num_routes - 1] = tmp;
                            individual.demand_sum_per_route[r2] = individual.demand_sum_per_route[individual.num_routes - 1];
                            individual.num_nodes_per_route[r2] = individual.num_nodes_per_route[individual.num_routes - 1];
                            individual.num_routes--;
                            for (int i = 0; i < individual.num_routes; i++) {
                                route_pairs.erase({i, individual.num_routes});
                            }
                        }

                        updated = true;
                        break;
                    }
                }
            }
        }
    }
    delete[] temp_r1;
    delete[] temp_r2;
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

bool node_shift_acceleration(int* route, int length, double& cost, Case& instance) {
    if (length <= 4) return false;

    int size = instance.restricted_candidate_list_size_;

    double old_cost, new_cost;
    for (int i = 1; i < length - 1; i++) {
        for (int j = 1; j < length - 1; j++) {
            if (i == j) continue;

            if (i < j) {
                if (!contains(instance.sorted_nearby_customers[route[j]], size, route[i]) ||
                    !contains(instance.sorted_nearby_customers[route[i]], size, route[j + 1])) {
                    continue;
                }
                old_cost = instance.get_distance(route[i - 1], route[i]) + instance.get_distance(route[i], route[i + 1]) + instance.get_distance(route[j], route[j + 1]);
                new_cost = instance.get_distance(route[i - 1], route[i + 1]) + instance.get_distance(route[j], route[i]) + instance.get_distance(route[i], route[j + 1]);
            } else {
                if (!contains(instance.sorted_nearby_customers[route[j - 1]], size, route[i]) ||
                    !contains(instance.sorted_nearby_customers[route[i]], size, route[j])) {
                    continue;
                }
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

bool node_shift_between_two_routes_acceleration(int* route1, int* route2, int& length1, int& length2, int& loading1, int& loading2, double& cost, Case& instance) {
    if (length1 < 3 || length2 < 3) return false;

    for (int i = 1; i < length1 - 1; i++) {
        // vehicle capacity constraint check
        if (loading2 + instance.get_customer_demand_(route1[i]) > instance.max_vehicle_capa_) continue;

        for (int j = 0; j < length2 - 1; j++) {
            if (!contains(instance.sorted_nearby_customers[route2[j]], instance.restricted_candidate_list_size_, route1[i]) ||
                !contains(instance.sorted_nearby_customers[route1[i]], instance.restricted_candidate_list_size_, route2[j + 1])) {
                continue;
            }

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

bool one_point_move_inter_route_for_individual_acceleration(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);

    while (!route_pairs.empty())
    {
        int r1 = route_pairs.begin()->first;
        int r2 = route_pairs.begin()->second;
        route_pairs.erase(route_pairs.begin());
        node_shift_between_two_routes_acceleration(individual.routes[r1], individual.routes[r2],
                                                   individual.num_nodes_per_route[r1], individual.num_nodes_per_route[r2],
                                                   individual.demand_sum_per_route[r1], individual.demand_sum_per_route[r2],
                                                   individual.upper_cost, instance);
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
                cost -= (old_cost - new_cost);
            }
        }
    }
}

void two_nodes_swap_for_single_route_acceleration(int* route, int length, double& cost, Case& instance) {
    // boundary check
    if (length < 5) return;

    // adjacent nodes do not swap
    for(int i = 1; i < length - 3; i++) {
        for(int j = i + 2; j < length - 1; j++) {
            if (!contains(instance.sorted_nearby_customers[route[i - 1]], instance.restricted_candidate_list_size_, route[j]) ||
                !contains(instance.sorted_nearby_customers[route[j]], instance.restricted_candidate_list_size_, route[i + 1]) ||
                !contains(instance.sorted_nearby_customers[route[j - 1]], instance.restricted_candidate_list_size_, route[i]) ||
                !contains(instance.sorted_nearby_customers[route[i]], instance.restricted_candidate_list_size_, route[j + 1])) {
                continue;
            }
            double old_cost = instance.get_distance(route[i - 1], route[i]) + instance.get_distance(route[i], route[i + 1])
                              + instance.get_distance(route[j - 1], route[j]) + instance.get_distance(route[j], route[j + 1]);
            double new_cost = instance.get_distance(route[i - 1], route[j]) + instance.get_distance(route[j], route[i + 1])
                              + instance.get_distance(route[j - 1], route[i]) + instance.get_distance(route[i], route[j + 1]);
            if (new_cost < old_cost) {
                swap(route[i], route[j]);
                cost -= (old_cost - new_cost);
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
                    cost -= (old_cost - new_cost);
                }
            }
        }
    }

    return true;
}

bool two_nodes_swap_between_two_routes_acceleration(int* route1, int* route2, int length1, int length2, int& loading1, int& loading2, double& cost, Case& instance) {
    // boundary check
    if (length1 < 3 || length2 < 3) return false;

    // vehicle capacity constraint check and fitness improvement check
    for (int i = 1; i < length1 - 1; i++) {
        for (int j = 1; j < length2 - 1; j++) {
            if (!contains(instance.sorted_nearby_customers[route1[i - 1]], instance.restricted_candidate_list_size_, route2[j]) ||
                !contains(instance.sorted_nearby_customers[route2[j]], instance.restricted_candidate_list_size_, route1[i + 1]) ||
                !contains(instance.sorted_nearby_customers[route2[j - 1]], instance.restricted_candidate_list_size_, route1[i]) ||
                !contains(instance.sorted_nearby_customers[route1[i]], instance.restricted_candidate_list_size_, route2[j + 1])) {
                continue;
            }

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
                    cost -= (old_cost - new_cost);
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

bool two_point_move_inter_route_for_individual_acceleration(Individual& individual, Case& instance) {
    if (individual.num_routes == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);

    while (!route_pairs.empty())
    {
        int r1 = route_pairs.begin()->first;
        int r2 = route_pairs.begin()->second;
        route_pairs.erase(route_pairs.begin());
        two_nodes_swap_between_two_routes_acceleration(individual.routes[r1], individual.routes[r2],
                                                       individual.num_nodes_per_route[r1], individual.num_nodes_per_route[r2],
                                                       individual.demand_sum_per_route[r1], individual.demand_sum_per_route[r2],
                                                       individual.upper_cost, instance);
    }

    return true;
}

/****************************************************************/
/*                   Neighborhood Expanding                     */
/****************************************************************/

vector<std::unique_ptr<Individual>> one_point_move_neighbors(Individual& individual, Case& instance, double base_cost, double threshold_ratio) {
    double threshold = base_cost * threshold_ratio;
    vector<std::unique_ptr<Individual>> intra_neighbors = one_point_intra_route_for_individual(individual, instance, threshold);
    vector<std::unique_ptr<Individual>> inter_neighbors = one_point_inter_route_for_individual(individual, instance, threshold);

    vector<std::unique_ptr<Individual>> neighbors;
    neighbors.reserve(intra_neighbors.size() + inter_neighbors.size()); // Pre-allocate memory for efficiency

    // Move elements from intra_neighbors to neighbors
    neighbors.insert(neighbors.end(),
                     std::make_move_iterator(intra_neighbors.begin()),
                     std::make_move_iterator(intra_neighbors.end()));

    // Move elements from inter_neighbors to neighbors
    neighbors.insert(neighbors.end(),
                     std::make_move_iterator(inter_neighbors.begin()),
                     std::make_move_iterator(inter_neighbors.end()));

    return neighbors;
}

vector<std::unique_ptr<Individual>> one_point_intra_route_for_individual(Individual& individual, Case& instance, double threshold) {
    vector<std::unique_ptr<Individual>> neighborhoods;

    for (int i = 0; i < individual.num_routes; i++) {
        int* route = individual.routes[i];
        int length = individual.num_nodes_per_route[i];
        double upper_cost = individual.upper_cost;

        if (length <= 4) continue;

        double old_cost, new_cost, change;
        for (int m = 1; m < length - 1; m++) {
            for (int n = 1; n < length - 1; n++) {
                if (m == n) continue;

                if (m < n) {
                    old_cost = instance.get_distance(route[m - 1], route[m]) + instance.get_distance(route[m], route[m + 1]) + instance.get_distance(route[n], route[n + 1]);
                    new_cost = instance.get_distance(route[m - 1], route[m + 1]) + instance.get_distance(route[n], route[m]) + instance.get_distance(route[m], route[n + 1]);
                } else {
                    old_cost = instance.get_distance(route[m - 1], route[m]) + instance.get_distance(route[m], route[m + 1]) + instance.get_distance(route[n - 1], route[n]);
                    new_cost = instance.get_distance(route[n - 1], route[m]) + instance.get_distance(route[m], route[n]) + instance.get_distance(route[m - 1], route[m + 1]);
                }

                change = old_cost - new_cost;
                if (upper_cost - change <= threshold) {
                    unique_ptr<Individual> new_ind = make_unique<Individual>(individual);
                    moveItoJ(new_ind->routes[i], m, n);
                    new_ind->upper_cost -= change;
                    neighborhoods.push_back(std::move(new_ind));
                }
            }
        }
    }

    return neighborhoods;
}

vector<std::unique_ptr<Individual>> one_point_inter_route_for_individual(Individual& individual, Case& instance, double threshold) {
    vector<std::unique_ptr<Individual>> neighborhoods;
    if (individual.num_routes == 1) {
        return neighborhoods;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);

    while (!route_pairs.empty())
    {
        int r1 = route_pairs.begin()->first;
        int r2 = route_pairs.begin()->second;
        route_pairs.erase(route_pairs.begin());

        int* route1 = individual.routes[r1];
        int* route2 = individual.routes[r2];
        int length1 = individual.num_nodes_per_route[r1];
        int length2 = individual.num_nodes_per_route[r2];
        int loading1 = individual.demand_sum_per_route[r1];
        int loading2 = individual.demand_sum_per_route[r2];
        double upper_cost = individual.upper_cost;

        if (length1 < 3 || length2 < 3) continue;

        for (int m = 1; m < length1 - 1; m++) {
            // vehicle capacity constraint check
            if (loading2 + instance.get_customer_demand_(route1[m]) > instance.max_vehicle_capa_) continue;

            for (int n = 0; n < length2 - 1; n++) {
                double old_cost = instance.get_distance(route1[m - 1], route1[m]) + instance.get_distance(route1[m], route1[m + 1]) + instance.get_distance(route2[n], route2[n + 1]);
                double new_cost = instance.get_distance(route1[m - 1], route1[m + 1]) + instance.get_distance(route2[n], route1[m]) + instance.get_distance(route1[m], route2[n + 1]);

                double change = old_cost - new_cost;
                if (upper_cost - change <= threshold) {
                    unique_ptr<Individual> new_ind = make_unique<Individual>(individual);

                    // update route1 and route2 in `new_ind`, and the corresponding node_num, demand_sum, and fit
                    int x = route1[m];
                    for (int p = m; p < length1 - 1; p++) {
                        new_ind->routes[r1][p] = new_ind->routes[r1][p + 1];
                    }
                    new_ind->num_nodes_per_route[r1]--;
                    new_ind->demand_sum_per_route[r1] -= instance.get_customer_demand_(x);
                    for (int q = length2; q > n + 1; q--) {
                        new_ind->routes[r2][q] = new_ind->routes[r2][q - 1];
                    }
                    new_ind->routes[r2][n + 1] = x;
                    new_ind->num_nodes_per_route[r2]++;
                    new_ind->demand_sum_per_route[r2] += instance.get_customer_demand_(x);
                    new_ind->upper_cost -= change;

                    // remove the potential empty route
                    // iterate the variable "demand_sum_per_route" to remove the empty route, if the demand_sum_per_route is 0, then remove the route
                    for (int p = 0; p < new_ind->num_routes; p++) {
                        if (new_ind->demand_sum_per_route[p] == 0) {
                            for (int q = p; q < new_ind->num_routes - 1; q++) {
                                int *temp = new_ind->routes[q];
                                new_ind->routes[q] = new_ind->routes[q + 1];
                                new_ind->routes[q + 1] = temp;
                                new_ind->num_nodes_per_route[q] = new_ind->num_nodes_per_route[q + 1];
                                new_ind->demand_sum_per_route[q] = new_ind->demand_sum_per_route[q + 1];
                            }
                            new_ind->num_routes--;
                            p--;
                        }
                    }
                    // update the variable "num_routes" and "route_cap" to remove the empty route
                    for (size_t k = new_ind->num_routes; k < new_ind->route_cap; ++k) {
                        new_ind->num_nodes_per_route[k] = 0;
                        new_ind->demand_sum_per_route[k] = 0;
                    }


                    neighborhoods.push_back(std::move(new_ind));
                }
            }
        }
    }

    return neighborhoods;
}

vector<std::unique_ptr<Individual>> two_point_move_neighbors(Individual& individual, Case& instance, double base_cost, double threshold_ratio) {
    double threshold = base_cost * threshold_ratio;
    vector<std::unique_ptr<Individual>> intra_neighbors = two_point_intra_route_for_individual(individual, instance, threshold);
    vector<std::unique_ptr<Individual>> inter_neighbors = two_point_inter_route_for_individual(individual, instance, threshold);

    vector<std::unique_ptr<Individual>> neighbors;
    neighbors.reserve(intra_neighbors.size() + inter_neighbors.size()); // Pre-allocate memory for efficiency

    // Move elements from intra_neighbors to neighbors
    neighbors.insert(neighbors.end(),
                     std::make_move_iterator(intra_neighbors.begin()),
                     std::make_move_iterator(intra_neighbors.end()));

    // Move elements from inter_neighbors to neighbors
    neighbors.insert(neighbors.end(),
                     std::make_move_iterator(inter_neighbors.begin()),
                     std::make_move_iterator(inter_neighbors.end()));

    return neighbors;
}

vector<std::unique_ptr<Individual>> two_point_intra_route_for_individual(Individual& individual, Case& instance, double threshold) {
    vector<std::unique_ptr<Individual>> neighborhoods;

    for (int i = 0; i < individual.num_routes; i++) {
        int* route = individual.routes[i];
        int length = individual.num_nodes_per_route[i];
        double upper_cost = individual.upper_cost;

        // boundary check
        if (length < 5) continue;

        // adjacent nodes do not swap
        for(int m = 1; m < length - 3; m++) {
            for(int n = m + 2; n < length - 1; n++) {
                double old_cost = instance.get_distance(route[m - 1], route[m]) + instance.get_distance(route[m], route[m + 1])
                                  + instance.get_distance(route[n - 1], route[n]) + instance.get_distance(route[n], route[n + 1]);
                double new_cost = instance.get_distance(route[m - 1], route[n]) + instance.get_distance(route[n], route[m + 1])
                                  + instance.get_distance(route[n - 1], route[m]) + instance.get_distance(route[m], route[n + 1]);
                double change = old_cost - new_cost;
                if (upper_cost - change <= threshold) {
                    unique_ptr<Individual> new_ind = make_unique<Individual>(individual);
                    swap(new_ind->routes[i][m], new_ind->routes[i][n]);
                    new_ind->upper_cost -= change;
                    neighborhoods.push_back(std::move(new_ind));
                }
            }
        }
    }

    return neighborhoods;
}

vector<std::unique_ptr<Individual>> two_point_inter_route_for_individual(Individual& individual, Case& instance, double threshold) {
    vector<std::unique_ptr<Individual>> neighborhoods;

    if (individual.num_routes == 1) {
        return neighborhoods;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.num_routes);

    while (!route_pairs.empty())
    {
        int r1 = route_pairs.begin()->first;
        int r2 = route_pairs.begin()->second;
        route_pairs.erase(route_pairs.begin());

        int* route1 = individual.routes[r1];
        int* route2 = individual.routes[r2];
        int length1 = individual.num_nodes_per_route[r1];
        int length2 = individual.num_nodes_per_route[r2];
        int loading1 = individual.demand_sum_per_route[r1];
        int loading2 = individual.demand_sum_per_route[r2];
        double upper_cost = individual.upper_cost;

        // boundary check
        if (length1 < 3 || length2 < 3) continue;

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
                    double change = old_cost - new_cost;
                    if (upper_cost - change <= threshold) {
                        unique_ptr<Individual> new_ind = make_unique<Individual>(individual);
                        swap(new_ind->routes[r1][i], new_ind->routes[r2][j]);
                        new_ind->demand_sum_per_route[r1] = loading1 - demand_I + demand_J;
                        new_ind->demand_sum_per_route[r2] = loading2 - demand_J + demand_I;
                        new_ind->upper_cost -= change;
                        neighborhoods.push_back(std::move(new_ind));
                    }
                }
            }
        }
    }

    return neighborhoods;
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

