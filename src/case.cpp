//
// Created by Yinghao Qin on 13/09/2024.
//

#include "case.hpp"

const int Case::MAX_EVALUATION_FACTOR = 25'000;

Case::Case(int id, const string& file_name) {
    this->id_ = id;
    this->file_name_ = file_name;
    this->instance_name_ = file_name.substr(0, file_name.find('.'));

    this->read_problem(kDataPath + file_name);
}

Case::~Case() {
    for (int i = 0; i < problem_size_; i++) {
        delete[] this->distances_[i];
    }
    delete[] this->distances_;
    for (int i = 0; i < num_depot_ + num_customer_; i++) {
        delete[] this->best_station_[i];
    }
    delete[] this->best_station_;
    for (int i = 0; i < num_customer_ + 1; ++i) {
        delete[] sorted_nearby_customers[i];
    }
    delete[] sorted_nearby_customers;
}

void Case::read_problem(const std::string &filePath) {
    this->num_depot_ = 1;

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
            ss >> this->num_customer_;
            this->num_customer_--;  // Adjusting the customer number
        }
        else if (line.find("STATIONS:") != string::npos) {
            ss << extract_value(line);
            ss >> this->num_station_;
        }
        else if (line.find("VEHICLES:") != string::npos) {
            ss << extract_value(line);
            ss >> this->num_vehicle_;
        }
        else if (line.find("CAPACITY:") != string::npos && line.find("ENERGY") == string::npos) {
            ss << extract_value(line);
            ss >> this->max_vehicle_capa_;
        }
        else if (line.find("ENERGY_CAPACITY:") != string::npos) {
            ss << extract_value(line);
            ss >> this->max_battery_capa_;
        }
        else if (line.find("ENERGY_CONSUMPTION:") != string::npos) {
            ss << extract_value(line);
            ss >> this->energy_consumption_rate_;
        }
        else if (line.find("OPTIMAL_VALUE:") != string::npos) {
            ss << extract_value(line);
            ss >> this->optimum_;
        }
        else if (line.find("NODE_COORD_SECTION") != string::npos) {
            this->problem_size_ = num_depot_ + num_customer_ + num_station_;
            positions_.resize(problem_size_, {0, 0});

            // Reading coordinates
            for (int i = 0; i < problem_size_; ++i) {
                getline(infile, line);
                ss.str(line);
                int ind;
                double x, y;
                ss >> ind >> x >> y;
                positions_[ind - 1] = {x, y};  // Set the position
            }
        }
        else if (line.find("DEMAND_SECTION") != string::npos) {
            int totalNumber = num_depot_ + num_customer_;
            demand_.resize(totalNumber, 0);

            // Reading demand_ values
            for (int i = 0; i < totalNumber; ++i) {
                getline(infile, line);
                ss.clear();
                ss.str(line);
                int ind, c;
                ss >> ind >> c;
                demand_[ind - 1] = c;
                if (c == 0) depot_ = ind - 1;  // Identify depot_
            }
        }
    }
    infile.close();

    // preprocess variables
    for (int i = 1; i < num_depot_ + num_customer_; ++i) {
        customers_.push_back(i);
    }
    for (int i = num_depot_ + num_customer_; i < problem_size_; ++i) {
        stations_.push_back(i);
    }
    this->max_distance_ = max_battery_capa_ / energy_consumption_rate_;
    this->total_demand_ = std::accumulate(demand_.begin(), demand_.end(), 0);
    this->distances_ = generate_2D_matrix_double(problem_size_, problem_size_);
    for (int i = 0; i < problem_size_; i++) {
        for (int j = 0; j < problem_size_; j++) {
            distances_[i][j] = euclidean_distance(i, j);
        }
    }
    this->best_station_ = new int* [num_depot_ + num_customer_];
    for (int i = 0; i < num_depot_ + num_customer_; i++) {
        this->best_station_[i] = new int[num_depot_ + num_customer_];
    }
    for (int i = 0; i < num_depot_ + num_customer_ - 1; i++) {
        for (int j = i + 1; j < num_depot_ + num_customer_; j++) {
            this->best_station_[i][j] = this->best_station_[j][i] = get_best_station(i, j);
        }
    }

    sorted_nearby_customers = new int* [num_customer_ + 1];
    for (int i = 0; i < num_customer_ + 1; i++) {
        sorted_nearby_customers[i] = new int[num_customer_ - 1];
    }
    for (int i = 1; i <= num_customer_; i++) {
        int idx = 0;
        for(auto& node : customers_) {
            if (node == i) continue;
            sorted_nearby_customers[i][idx++] = node;
        }

        sort(sorted_nearby_customers[i], sorted_nearby_customers[i] + num_customer_ - 1, [&](int a, int b) {
            return distances_[i][a] < distances_[i][b];
        });
    }
    this->restricted_candidate_list_size_ = min(num_customer_ / 2, 40);

    init_customer_nearest_station_map();

    this->evals_ = 0.0;
    this->max_evals_ = problem_size_ * MAX_EVALUATION_FACTOR;
    if (num_customer_ <= 100) {
        max_exec_time_ = int (1 * (problem_size_ / 100.0) * 60 * 60);
    } else if (num_customer_ <= 915) {
        max_exec_time_ = int (2 * (problem_size_ / 100.0) * 60 * 60);
    } else {
        max_exec_time_ = int (3 * (problem_size_ / 100.0) * 60 * 60);
    }
    this->convergence_epsilon_ = 1e-4; // 0.0001
    this->max_no_change_count_ = 800; // adjust further

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
    return sqrt(pow(positions_[i].first - positions_[j].first, 2) +
                pow(positions_[i].second - positions_[j].second, 2));
}

int Case::get_best_station(int from, int to) const {
    int targetStation = -1;
    double minDis = std::numeric_limits<double>::max();

    for (int i = num_customer_ + 1 ; i < problem_size_; ++i) {
        double dis = distances_[from][i] + distances_[to][i];

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

    for (int i = num_customer_ + 1; i < problem_size_; ++i) {
        if (distances_[from][i] < max_dis &&
            minDis > distances_[from][i]  + distances_[to][i]  &&
            from != i && to != i &&
            distances_[i][to] < max_distance_) {

            targetStation = i;
            minDis = distances_[from][i] + distances_[to][i];
        }
    }

    return targetStation;
}

int Case::get_customer_demand_(int customer) const {
    return demand_[customer];
}

double Case::get_distance(int from, int to) {
    //adds partial evaluation to the overall fitness evaluation count
    //It can be used when local search is used and a whole evaluation is not necessary
    evals_ += (1.0 / problem_size_);

    return distances_[from][to];
}

//void Case::init_customer_to_customers_maps() {
//    for (int node : customers_) {
//        // Create a sorted list of other customers_ based on the distance to the current node
//        vector<int> other_customers = customers_;
//        other_customers.erase(remove(other_customers.begin(), other_customers.end(), node), other_customers.end());
//
//        sort(other_customers.begin(), other_customers.end(), [&](int i, int j) {
//            return distances_[node][i] < distances_[node][j];
//        });
//
//        customer_to_cluster_map_[node] = other_customers;
//    }
//}

void Case::init_customer_nearest_station_map() {
    for (int i = 1; i <= num_customer_; ++i) {
        int nearestStation = -1;
        double minDis = std::numeric_limits<double>::max();
        for (int j = num_customer_ + 1; j < problem_size_; ++j) {
            double dis = distances_[i][j];
            if (minDis > dis) {
                nearestStation = j;
                minDis = dis;
            }
        }
        customer_to_nearest_station_map_[i] = make_pair(nearestStation, minDis);
    }
}

double Case::get_evals() const {
    return evals_;
}

double Case::compute_total_distance(int** routes, int num_routes, const int* num_nodes_per_route) {
    double tour_length = 0.0;
    for (int i = 0; i < num_routes; ++i) {
        for (int j = 0; j < num_nodes_per_route[i] - 1; ++j) {
            tour_length += distances_[routes[i][j]][routes[i][j + 1]];
        }
    }

    evals_++;

    return tour_length;
}

double Case::compute_total_distance(const int* route, int length) const {
    double tour_length = 0.0;
    for (int j = 0; j < length - 1; ++j) {
        tour_length += distances_[route[j]][route[j + 1]];
    }

    return tour_length;
}

double Case::compute_total_distance(const vector<vector<int>> &routes) {
    double tour_length = 0.0;
    for (auto& route : routes) {
        for (int j = 0; j < route.size() - 1; ++j) {
            tour_length += distances_[route[j]][route[j + 1]];
        }
    }

    evals_++;

    return tour_length;
}

double Case::compute_total_distance(const vector<int> &route) const {
    double tour_length = 0.0;
    for (int j = 0; j < route.size() - 1; ++j) {
        tour_length += distances_[route[j]][route[j + 1]];
    }

    return tour_length;
}

vector<int> Case::compute_demand_sum_per_route(const vector<vector<int>> &routes) const {
    vector<int> demand_sum_per_route;
    for (auto & route : routes) {
        int temp = 0;
        for (int node : route) {
            temp += get_customer_demand_(node);
        }
        demand_sum_per_route.push_back(temp);
    }

    return demand_sum_per_route;
}

void Case::compute_demand_sum_per_route(int** routes, int num_routes, const int* num_nodes_per_route, int* demand_sum_per_route) const {
    for (int i = 0; i < num_routes; ++i) {
        int temp = 0;
        for (int j = 0; j < num_nodes_per_route[i]; ++j) {
            temp += get_customer_demand_(routes[i][j]);
        }
        demand_sum_per_route[i] = temp;
    }
}

bool Case::is_charging_station(int node) const {

    bool flag;
    if (node == depot_ || ( node >= num_depot_ + num_customer_ && node < problem_size_))
        flag = true;
    else
        flag = false;
    return flag;
}