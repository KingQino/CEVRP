//
// Created by Yinghao Qin on 13/09/2024.
//

#include "case.hpp"

const int Case::MAX_EVALUATION_FACTOR = 25'000;

Case::Case(int id, const string& fileName) {
    this->id = id;
    this->fileName = fileName;

    this->read_problem(kDataPath + fileName);
}

Case::~Case() {
    for (int i = 0; i < actualProblemSize; i++) {
        delete[] this->distances[i];
    }
    delete[] this->distances;
    for (int i = 0; i < depotNumber + customerNumber; i++) {
        delete[] this->bestStation[i];
    }
    delete[] this->bestStation;
}

void Case::read_problem(const std::string &filePath) {
    this->depotNumber = 1;

    ifstream infile(filePath);
    string line;

    // Helper lambda to extract values after a colon
    auto extract_value = [&](const string& line) {
        return line.substr(line.find(':') + 1);
    };

    // Read file line by line
    while (getline(infile, line)) {
        stringstream ss;
        if (line.find("DIMENSION:") != string::npos) {
            ss << extract_value(line);
            ss >> this->customerNumber;
            this->customerNumber--;  // Adjusting the customer number
        }
        else if (line.find("STATIONS:") != string::npos) {
            ss << extract_value(line);
            ss >> this->stationNumber;
        }
        else if (line.find("VEHICLES:") != string::npos) {
            ss << extract_value(line);
            ss >> this->vehicleNumber;
        }
        else if (line.find("CAPACITY:") != string::npos && line.find("ENERGY") == string::npos) {
            ss << extract_value(line);
            ss >> this->maxC;
        }
        else if (line.find("ENERGY_CAPACITY:") != string::npos) {
            ss << extract_value(line);
            ss >> this->maxQ;
        }
        else if (line.find("ENERGY_CONSUMPTION:") != string::npos) {
            ss << extract_value(line);
            ss >> this->conR;
        }
        else if (line.find("OPTIMAL_VALUE:") != string::npos) {
            ss << extract_value(line);
            ss >> this->optimum;
        }
        else if (line.find("NODE_COORD_SECTION") != string::npos) {
            this->actualProblemSize = depotNumber + customerNumber + stationNumber;
            positions.resize(actualProblemSize, {0, 0});

            // Reading coordinates
            for (int i = 0; i < actualProblemSize; ++i) {
                getline(infile, line);
                ss.str(line);
                int ind;
                double x, y;
                ss >> ind >> x >> y;
                positions[ind - 1] = {x, y};  // Set the position
            }
        }
        else if (line.find("DEMAND_SECTION") != string::npos) {
            int totalNumber = depotNumber + customerNumber;
            demand.resize(totalNumber, 0);

            // Reading demand values
            for (int i = 0; i < totalNumber; ++i) {
                getline(infile, line);
                ss.clear();
                ss.str(line);
                int ind, c;
                ss >> ind >> c;
                demand[ind - 1] = c;
                if (c == 0) depot = ind - 1;  // Identify depot
            }
        }
    }
    infile.close();

    // preprocess variables
    for (int i = 1; i < depotNumber + customerNumber; ++i) {
        customers.push_back(i);
    }
    for (int i = depotNumber + customerNumber; i < actualProblemSize; ++i) {
        stations.push_back(i);
    }
    this->maxDis = maxQ / conR;
    this->totalDem = std::accumulate(demand.begin(), demand.end(), 0);
    this->distances = generate_2D_matrix_double(actualProblemSize, actualProblemSize);
    for (int i = 0; i < actualProblemSize; i++) {
        for (int j = 0; j < actualProblemSize; j++) {
            distances[i][j] = euclidean_distance(i, j);
        }
    }
    this->bestStation = new int* [depotNumber + customerNumber];
    for (int i = 0; i < depotNumber + customerNumber; i++) {
        this->bestStation[i] = new int[depotNumber + customerNumber];
    }
    for (int i = 0; i < depotNumber + customerNumber - 1; i++) {
        for (int j = i + 1; j < depotNumber + customerNumber; j++) {
            this->bestStation[i][j] = this->bestStation[j][i] = get_best_station(i, j);
        }
    }

    init_customer_to_customers_maps();
    init_customer_nearest_station_map();

    this->evals = 0.0;
    this->maxEvals = actualProblemSize * MAX_EVALUATION_FACTOR;
    if (customerNumber <= 100) {
        maxExecTime = int (1 * (actualProblemSize / 100.0) * 60 * 60);
    } else if (customerNumber <= 915) {
        maxExecTime = int (2 * (actualProblemSize / 100.0) * 60 * 60);
    } else {
        maxExecTime = int (3 * (actualProblemSize / 100.0) * 60 * 60);
    }

}

double **Case::generate_2D_matrix_double(int n, int m) {
    auto **matrix = new double *[n];
    for (int i = 0; i < n; i++) {
        matrix[i] = new double[m];
    }
    //initialize the 2-d array
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            matrix[i][j] = 0.0;
        }
    }
    return matrix;
}

double Case::euclidean_distance(int i, int j) {
    return sqrt(pow(positions[i].first - positions[j].first, 2) +
                pow(positions[i].second - positions[j].second, 2));
}

int Case::get_best_station(int from, int to) const {
    int targetStation = -1;
    double minDis = std::numeric_limits<double>::max();

    for (int i = customerNumber + 1 ; i < actualProblemSize; ++i) {
        double dis = distances[from][i] + distances[to][i];

        if (minDis > dis && from != i && to != i) {
            targetStation = i;
            minDis = dis;
        }
    }

    return targetStation;
}

int Case::get_best_and_feasible_station(int from, int to, double max_dis) const {
    int targetStation = -1;
    double minDis = std::numeric_limits<double>::max();

    for (int i = customerNumber + 1; i < actualProblemSize; ++i) {
        if (distances[from][i] < max_dis &&
            minDis > distances[from][i]  + distances[to][i]  &&
            from != i && to != i &&
            distances[i][to] < maxDis) {

            targetStation = i;
            minDis = distances[from][i] + distances[to][i];
        }
    }

    return targetStation;
}

int Case::get_customer_demand(int customer) const {
    return demand[customer];
}

double Case::get_distance(int from, int to) {
    //adds partial evaluation to the overall fitness evaluation count
    //It can be used when local search is used and a whole evaluation is not necessary
    evals += (1.0 / actualProblemSize);

    return distances[from][to];
}

void Case::init_customer_to_customers_maps() {
    this->restrictedCandidateListSize = min(customerNumber / 2, 40);

    for (int node : customers) {
        // Create a sorted list of other customers based on the distance to the current node
        vector<int> other_customers = customers;
        other_customers.erase(remove(other_customers.begin(), other_customers.end(), node), other_customers.end());

        sort(other_customers.begin(), other_customers.end(), [&](int i, int j) {
            return distances[node][i] < distances[node][j];
        });

        customerClusterMap[node] = other_customers;

        // Create the restricted candidate list
        customerRestrictedCandidateListMap[node] = std::set<int>(
                other_customers.begin(), other_customers.begin() + restrictedCandidateListSize
        );
    }
}

void Case::init_customer_nearest_station_map() {
    for (int i = 1; i <= customerNumber; ++i) {
        int nearestStation = -1;
        double minDis = std::numeric_limits<double>::max();
        for (int j = customerNumber + 1; j < actualProblemSize; ++j) {
            double dis = distances[i][j];
            if (minDis > dis) {
                nearestStation = j;
                minDis = dis;
            }
        }
        customerNearestStationMap[i] = make_pair(nearestStation, minDis);
    }
}

double Case::get_evals() const {
    return evals;
}

double Case::compute_total_distance(int** routes, int num_routes, const int* num_nodes_per_route) {
    double tour_length = 0.0;
    for (int i = 0; i < num_routes; ++i) {
        for (int j = 0; j < num_nodes_per_route[i] - 1; ++j) {
            tour_length += distances[routes[i][j]][routes[i][j + 1]];
        }
    }

    evals++;

    return tour_length;
}

double Case::compute_total_distance(const vector<vector<int>> &routes) {
    double tour_length = 0.0;
    for (auto& route : routes) {
        for (int j = 0; j < route.size() - 1; ++j) {
            tour_length += distances[route[j]][route[j + 1]];
        }
    }

    evals++;

    return tour_length;
}

double Case::compute_total_distance(const vector<int> &route) const {
    double tour_length = 0.0;
    for (int j = 0; j < route.size() - 1; ++j) {
        tour_length += distances[route[j]][route[j + 1]];
    }

    return tour_length;
}

vector<int> Case::compute_demand_sum_per_route(const vector<vector<int>> &routes) const {
    vector<int> demand_sum_per_route;
    for (auto & route : routes) {
        int temp = 0;
        for (int node : route) {
            temp += get_customer_demand(node);
        }
        demand_sum_per_route.push_back(temp);
    }

    return demand_sum_per_route;
}

void Case::compute_demand_sum_per_route(int** routes, int num_routes, const int* num_nodes_per_route, int* demand_sum_per_route) const {
    for (int i = 0; i < num_routes; ++i) {
        int temp = 0;
        for (int j = 0; j < num_nodes_per_route[i]; ++j) {
            temp += get_customer_demand(routes[i][j]);
        }
        demand_sum_per_route[i] = temp;
    }
}

bool Case::is_charging_station(int node) const {

    bool flag;
    if (node == depot || ( node >= depotNumber + customerNumber && node < actualProblemSize))
        flag = true;
    else
        flag = false;
    return flag;
}