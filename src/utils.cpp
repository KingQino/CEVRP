//
// Created by Yinghao Qin on 14/09/2024.
//

#include "utils.hpp"



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