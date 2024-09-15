//
// Created by Yinghao Qin on 14/09/2024.
//

#include "utils.hpp"



/****************************************************************/
/*                  Get Upper-level Solution                    */
/****************************************************************/

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
            load += instance.get_customer_demand(x[j]);
            if (i == j) {
                cost = instance.get_distance(instance.depot, x[j]) * 2;
            } else {
                cost -= instance.get_distance(x[j -1], instance.depot);
                cost += instance.get_distance(x[j -1], x[j]);
                cost += instance.get_distance(instance.depot, x[j]);
            }

            if (load <= instance.maxC) {
                if (vv[i - 1] + cost < vv[j]) {
                    vv[j] = vv[i - 1] + cost;
                    pp[j] = i - 1;
                }
                j++;
            }
        } while (!(j >= x.size() || load >instance.maxC));
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
    vector<int> customers(instance.customers);

    std::shuffle(customers.begin(), customers.end(), rng);

    vector<vector<int>> tours;

    vector<int> tour;
    while (!customers.empty()) {
        tour.clear();

        int anchor = customers.front();
        customers.erase(customers.begin());
        tour.push_back(anchor);
        int cap = instance.get_customer_demand(anchor);

        const vector<int> &nearby_customers = instance.customerClusterMap.at(anchor);
        for (int node: nearby_customers) {
            auto it = find(customers.begin(), customers.end(), node);
            if (it == customers.end()) {
                continue;
            }
            if (cap + instance.get_customer_demand(node) <= instance.maxC) {
                tour.push_back(node);
                cap += instance.get_customer_demand(node);
                customers.erase(it);
            } else {
                tours.push_back(tour);
                break;
            }
        }
    }

    tours.push_back(tour);

    return tours;
}

void hien_balancing(vector<vector<int>>& routes, const Case& instance, std::default_random_engine& rng) {
    vector<int>& lastRoute = routes.back();

    uniform_int_distribution<> distribution(0, static_cast<int >(lastRoute.size() -1) );
    int customer = lastRoute[distribution(rng)];  // Randomly choose a customer from the last route

    int cap1 = 0;
    for (int node : lastRoute) {
        cap1 += instance.get_customer_demand(node);
    }

    for (int x : instance.customerClusterMap.at(customer)) {
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
                cap2 += instance.get_customer_demand(node);
            }

            int demandX = instance.get_customer_demand(x);

            if (demandX + cap1 <= instance.maxC && abs((cap1 + demandX) - (cap2 - demandX)) < abs(cap1 - cap2)) {
                route2.erase(remove(route2.begin(), route2.end(), x), route2.end());
                lastRoute.push_back(x);
                cap1 += demandX;
            } else {
                break;
            }
        }
    }
}

vector<vector<int>> routes_constructor_with_split(Case& instance, std::default_random_engine& rng) {
    vector<int> a_giant_tour(instance.customers);

    shuffle(a_giant_tour.begin(), a_giant_tour.end(), rng);

    vector<vector<int>> all_routes = prins_split(a_giant_tour, instance);
    for (auto& route : all_routes) {
        route.insert(route.begin(), instance.depot);
        route.push_back(instance.depot);
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

    return routes;
}

// Jia Ya-Hui, et al., "Confidence-Based Ant Colony Optimization for Capacitated Electric Vehicle Routing Problem With Comparison of Different Encoding Schemes", 2022
vector<vector<int>> routes_constructor_with_direct_encoding(const Case& instance, std::default_random_engine& rng) {
    vector<int> customers(instance.customers);

    int vehicle_idx = 0; // vehicle index - starts from the vehicle 0
    int load_of_one_route = 0; // the load of the current vehicle
    vector<int> route = {instance.depot}; // the first route starts from depot 0

    vector<vector<int>> all_routes;
    while(!customers.empty()) {
        vector<int> all_temp;
        for(int i : customers) {
            if(instance.get_customer_demand(i) <= instance.maxC - load_of_one_route) {
                all_temp.push_back(i);
            }
        }

        int remain_total_demand = accumulate(customers.begin(), customers.end(), 0, [&](int total, int i) {
            return total + instance.get_customer_demand(i);
        });
        if(remain_total_demand <= instance.maxC * (instance.vehicleNumber - vehicle_idx - 1) || all_temp.empty()) {
            all_temp.push_back(instance.depot); // add depot node into the all_temp
        }

        int cur = route.back();
        uniform_int_distribution<> distribution(0, all_temp.size() - 1);
        int next = all_temp[distribution(rng)]; // int next = roulette_wheel_selection(all_temp, cur);
        route.push_back(next);

        if (next == instance.depot) {
            all_routes.push_back(route);
            vehicle_idx += 1;
            route = {0};
            load_of_one_route = 0;
        } else {
            load_of_one_route += instance.get_customer_demand(next);
            customers.erase(remove(customers.begin(), customers.end(), next), customers.end());
        }
    }

    route.push_back(instance.depot);
    all_routes.push_back(route);

    return all_routes;
}

/****************************************************************/
/*                    Local search Operators                    */
/****************************************************************/

void two_opt_for_single_route(int* route, int length, double& cost, Case& instance) {
    bool improved = true;

    while (improved) {
        improved = false;

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
                    improved = true;
                    cost += new_cost - old_cost;
                }
            }
        }
    }
}

// Croes, Georges A. "A method for solving traveling-salesman problems." Operations research 6, no. 6 (1958): 791-812.
void two_opt_for_individual(Individual& individual, Case& instance) {
    for (int i = 0; i < individual.route_num; ++i) {
        two_opt_for_single_route(individual.routes[i],  individual.node_num[i], individual.upper_cost, instance);
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
    if (individual.route_num == 1) {
        return false;
    }

    unordered_set<pair<int, int>, pair_hash> route_pairs = get_route_pairs(individual.route_num);
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
        for (int n1 = 0; n1 < individual.node_num[r1] - 1; n1++) {
            frdem += instance.get_customer_demand(individual.routes[r1][n1]);
            int srdem = 0;
            for (int n2 = 0; n2 < individual.node_num[r2] - 1; n2++) {
                srdem += instance.get_customer_demand(individual.routes[r2][n2]);
                if (frdem + individual.demand_sum[r2] - srdem <= instance.maxC && srdem + individual.demand_sum[r1] - frdem <= instance.maxC) {
                    double xx1 = instance.get_distance(individual.routes[r1][n1], individual.routes[r1][n1 + 1]) +
                                 instance.get_distance(individual.routes[r2][n2], individual.routes[r2][n2 + 1]);
                    double xx2 = instance.get_distance(individual.routes[r1][n1], individual.routes[r2][n2 + 1]) +
                                 instance.get_distance(individual.routes[r2][n2], individual.routes[r1][n1 + 1]);
                    double change = xx1 - xx2;
                    if (change > 0.00000001) {
                        individual.upper_cost -= change;
                        memcpy(tempr, individual.routes[r1], sizeof(int) * individual.node_cap);
                        int counter1 = n1 + 1;
                        for (int i = n2 + 1; i < individual.node_num[r2]; i++) {
                            individual.routes[r1][counter1] = individual.routes[r2][i];
                            counter1++;
                        }
                        int counter2 = n2 + 1;
                        for (int i = n1 + 1; i < individual.node_num[r1]; i++) {
                            individual.routes[r2][counter2] = tempr[i];
                            counter2++;
                        }
                        individual.node_num[r1] = counter1;
                        individual.node_num[r2] = counter2;
                        int newdemsum1 = frdem + individual.demand_sum[r2] - srdem;
                        int newdemsum2 = srdem + individual.demand_sum[r1] - frdem;
                        individual.demand_sum[r1] = newdemsum1;
                        individual.demand_sum[r2] = newdemsum2;
                        updated = true;
                        updated2 = true;
                        for (int i = 0; i < r1; i++) {
                            route_pairs.insert({i, r1});
                        }
                        for (int i = 0; i < r2; i++) {
                            route_pairs.insert({i, r2});
                        }
                        if (individual.demand_sum[r1] == 0) {
                            int* tempp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r1] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r1] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                route_pairs.erase({i, individual.route_num});
                            }
                        }
                        if (individual.demand_sum[r2] == 0) {
                            int* tempp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r2] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r2] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                route_pairs.erase({i, individual.route_num});
                            }
                        }
                        break;
                    }
                }
                else if (frdem + srdem <= instance.maxC && individual.demand_sum[r1] - frdem + individual.demand_sum[r2] - srdem <= instance.maxC) {
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
                        for (int i = individual.node_num[r1] - 1; i >= n1 + 1; i--) {
                            tempr2[counter2] = tempr[i];
                            counter2++;
                        }
                        for (int i = n2 + 1; i < individual.node_num[r2]; i++) {
                            tempr2[counter2] = individual.routes[r2][i];
                            counter2++;
                        }
                        memcpy(individual.routes[r2], tempr2, sizeof(int) * individual.node_cap);
                        individual.node_num[r1] = counter1;
                        individual.node_num[r2] = counter2;

                        int newdemsum1 = frdem + srdem;
                        int newdemsum2 = individual.demand_sum[r1] + individual.demand_sum[r2] - frdem - srdem;
                        individual.demand_sum[r1] = newdemsum1;
                        individual.demand_sum[r2] = newdemsum2;
                        updated = true;
                        updated2 = true;
                        for (int i = 0; i < r1; i++) {
                            route_pairs.insert({i, r1});
                        }
                        for (int i = 0; i < r2; i++) {
                            route_pairs.insert({i, r2});
                        }
                        if (individual.demand_sum[r1] == 0) {
                            int* tempp = individual.routes[r1];
                            individual.routes[r1] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r1] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r1] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                route_pairs.erase({i, individual.route_num});
                            }
                        }
                        if (individual.demand_sum[r2] == 0) {
                            int* tempp = individual.routes[r2];
                            individual.routes[r2] = individual.routes[individual.route_num - 1];
                            individual.routes[individual.route_num - 1] = tempp;
                            individual.demand_sum[r2] = individual.demand_sum[individual.route_num - 1];
                            individual.node_num[r2] = individual.node_num[individual.route_num - 1];
                            individual.route_num--;
                            for (int i = 0; i < individual.route_num; i++) {
                                route_pairs.erase({i, individual.route_num});
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
    return updated;
}