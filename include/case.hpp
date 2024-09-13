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

    Case(int id, const string& fileName);
    ~Case();
    void read_problem(const string& filePath);					//reads .evrp file
    static double **generate_2D_matrix_double(int n, int m);
    void init_customer_to_customers_maps(); // initialize customerClusterMap and customerRestrictedCandidateListMap
    void init_customer_nearest_station_map();
    double euclidean_distance(int i, int j);
    [[nodiscard]] int get_best_station(int from, int to) const;
    [[nodiscard]] int get_best_and_feasible_station(int from, int to, double max_dis) const; // the station within allowed max distance from "from", and min dis[from][s]+dis[to][s]
    [[nodiscard]] int get_customer_demand(int customer) const;				//returns the customer demand
    [[nodiscard]] double get_distance(int from, int to);				//returns the distance
    [[nodiscard]] double get_evals() const;									//returns the number of evaluations
    [[nodiscard]] double compute_total_distance(const vector<vector<int>>& routes); // customized fitness function
    [[nodiscard]] vector<int> compute_demand_sum(const vector<vector<int>>& routes) const; // compute the demand sum of all customers for each route.
    [[nodiscard]] bool is_charging_station(int node) const;					//returns true if node is a charging station


    int id;
    string fileName;

    int depotNumber{};
    int customerNumber{};
    int stationNumber{};
    int vehicleNumber{};
    int actualProblemSize{}; //Total number of customers, charging stations and depot
    int maxC{}; // maximum capacity of the vehicle
    double maxQ{}; // maximum energy capacity of the vehicle
    double conR{}; // energy consumption rate
    vector<pair<double, double>> positions;
    vector<int> demand;
    int depot{};  //depot id (usually 0)
    double optimum{};
    vector<int> customers;
    vector<int> stations;
    double maxDis{};
    int totalDem{};
    double** distances{};
    int** bestStation{}; // "bestStation" is designed for two customers, bringing the minimum extra cost.
    unordered_map<int, vector<int>> customerClusterMap; // For Hien's clustering usage only. For each customer, a list of customer nodes from near to far, e.g., {1: [5,3,2,6], 2: [], ...}
    int restrictedCandidateListSize{}; // min{customerNumber/2, 40}
    unordered_map<int, set<int>> customerRestrictedCandidateListMap; // search acceleration technique proposed in TAMLS
    unordered_map<int, pair<int, double>> customerNearestStationMap; // for each customer, find the nearest station and store the corresponding distance
    double evals{};
    double maxEvals{};
    int maxExecTime{}; // seconds
};

#endif //CEVRP_CASE_HPP
