//
// Created by Yinghao Qin on 13/09/2024.
//

#ifndef CEVRP_CASE_HPP
#define CEVRP_CASE_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <cstring>
#include <string>
#include <map>
#include <unordered_map>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <numeric>

using namespace std;

const string kDataPath = "../data/";


class Case {
public:
    static const int MAX_EVALUATION_FACTOR;

    Case(int id, const string& file_name);
    ~Case();
    void read_problem(const string& file_path);					//reads .evrp file
    static double **generate_2D_matrix_double(int n, int m);
//    void init_customer_to_customers_maps(); // initialize customer_to_cluster_map_ and customer_to_restricted_candidate_list_map_
    void init_customer_nearest_station_map();
    double euclidean_distance(int i, int j);
    [[nodiscard]] int get_best_station(int from, int to) const;
    [[nodiscard]] int get_best_and_feasible_station(int from, int to, double max_dis) const; // the station within allowed max distance from "from", and min dis[from][s]+dis[to][s]
    [[nodiscard]] int get_customer_demand_(int customer) const;				//returns the customer demand
    [[nodiscard]] double get_distance(int from, int to);				//returns the distance
    [[nodiscard]] double get_evals() const;									//returns the number of evaluations
    [[nodiscard]] bool is_charging_station(int node) const;					//returns true if node is a charging station
    [[nodiscard]] double compute_total_distance(int** routes, int num_routes, const int* num_nodes_per_route);
    [[nodiscard]] double compute_total_distance(const int* route, int length) const;
    [[nodiscard]] double compute_total_distance(const vector<vector<int>>& routes); // customized fitness function
    [[nodiscard]] double compute_total_distance(const vector<int>& route) const;
    [[nodiscard]] vector<int> compute_demand_sum_per_route(const vector<vector<int>>& routes) const; // compute the demand_ sum of all customers_ for each route.
    void compute_demand_sum_per_route(int** routes, int num_routes, const int* num_nodes_per_route, int* demand_sum_per_route) const; // must be with "int* demand_sum_per_route assign 0"


    int id_;
    string file_name_;
    string instance_name_;

    int num_depot_{};
    int num_customer_{};
    int num_station_{};
    int num_vehicle_{};
    int problem_size_{}; //Total number of customers, charging stations and depot
    int max_vehicle_capa_{}; // maximum capacity of the vehicle
    double max_battery_capa_{}; // maximum energy capacity of the vehicle
    double energy_consumption_rate_{}; // energy consumption rate
    vector<pair<double, double>> positions_;
    vector<int> demand_;
    int depot_{};  //depot id (usually 0)
    double optimum_{};
    vector<int> customers_;
    vector<int> stations_;
    double max_distance_{};
    int total_demand_{};
    double** distances_{};
    int** best_station_{}; // "best_station_" is designed for two customers, bringing the minimum extra cost.
    int** sorted_nearby_customers{};  // For Hien's clustering usage only. For each customer, a list of customer nodes from near to far, e.g., {index 1: [5,3,2,6], index 2: [], ...}
    // key: customer id, value: a list of customer nodes from near to far (size = num_customer - 1)
//    unordered_map<int, vector<int>> customer_to_cluster_map_; // For Hien's clustering usage only. For each customer, a list of customer nodes from near to far, e.g., {1: [5,3,2,6], 2: [], ...}
    int restricted_candidate_list_size_{}; // min{num_customer_/2, 40}
    unordered_map<int, pair<int, double>> customer_to_nearest_station_map_; // for each customer, find the nearest station and store the corresponding distance
    double evals_{};
    double max_evals_{};
    int max_exec_time_{}; // seconds
};

#endif //CEVRP_CASE_HPP
