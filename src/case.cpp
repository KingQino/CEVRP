//
// Created by Yinghao Qin on 13/09/2024.
//

#include "case.hpp"

const int Case::MAX_EVALUATION_FACTOR = 25'000;

Case::Case(const int id, const int num_granular, const string& file_name) {
    this->id_ = id;
    this->num_granular_ = num_granular;
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

void Case::read_problem(const std::string &file_path) {
    this->num_depot_ = 1;

    ifstream infile(file_path);
    string line;

    // Helper lambda to extract values after a colon
    auto extract_value = [&](const string& line_) {
        return line_.substr(line_.find(':') + 1);
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
    customers_ = vector<Customer>(num_customer_ + 1);
    customers_[0].coord_x = positions_[0].first;
    customers_[0].coord_y = positions_[0].second;
    for (int i = 1; i < num_depot_ + num_customer_; ++i) {
        customer_ids_.push_back(i);
        customers_[i].id = i;
        customers_[i].coord_x = positions_[i].first;
        customers_[i].coord_y = positions_[i].second;
        customers_[i].demand = demand_[i];
        customers_[i].polar_angle = CircleSector::positive_mod(static_cast<int>(32768.*atan2(customers_[i].coord_y - customers_[0].coord_y, customers_[i].coord_x - customers_[0].coord_x) / PI) );
    }
    for (int i = num_depot_ + num_customer_; i < problem_size_; ++i) {
        station_ids_.push_back(i);
    }
    this->max_distance_ = max_battery_capa_ / energy_consumption_rate_;
    this->max_demand_ = *std::max_element(demand_.begin(), demand_.end());
    this->total_demand_ = std::accumulate(demand_.begin(), demand_.end(), 0);
    this->distances_ = generate_2D_matrix_double(problem_size_, problem_size_);
    for (int i = 0; i < problem_size_; i++) {
        for (int j = 0; j < problem_size_; j++) {
            distances_[i][j] = euclidean_distance(i, j);
            if (distances_[i][j] > longest_arc_dis_) longest_arc_dis_ = distances_[i][j];
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
        for(auto& node : customer_ids_) {
            if (node == i) continue;
            sorted_nearby_customers[i][idx++] = node;
        }

        sort(sorted_nearby_customers[i], sorted_nearby_customers[i] + num_customer_ - 1, [&](const int a, const int b) {
            return distances_[i][a] < distances_[i][b];
        });
    }
    // Calculation of the correlated vertices for each customer (for the granular restriction)
    correlated_vertices_ = vector<vector<int>>(num_customer_ + 1);
    vector<set<int>> set_correlated_vertices = vector<set<int>>(num_customer_ + 1);
    vector<pair<double, int>> order_proximity;
    for (int i = 1; i <= num_customer_; i++) {
        order_proximity.clear();
        for (int j = 1; j <= num_customer_; j++) {
            if (i == j) continue;
            order_proximity.emplace_back(distances_[i][j], j);
        }
        std::sort(order_proximity.begin(), order_proximity.end());
        for (int j = 0; j < std::min<int>(num_granular_, num_customer_ - 1); ++j) {
            //  if i is correlated with j, then j should be correlated with i
            set_correlated_vertices[i].insert(order_proximity[j].second);
            set_correlated_vertices[order_proximity[j].second].insert(i);
        }
    }
    // Filling the vector with correlated vertices
    for (int i = 1; i <= num_customer_; i++) {
        for (int x : set_correlated_vertices[i]) {
            correlated_vertices_[i].push_back(x);
        }
    }

    // A reasonable scale for the initial values of the penalties
    this->penalty_duration_ = 1;
    this->penalty_capacity_ = std::max<double>(0.1, std::min<double>(1000., longest_arc_dis_ / max_demand_));

    this->is_duration_constraint_ = false;
    this->duration_limit_ = 1.e30;


    this->restricted_candidate_list_size_ = min(num_customer_ / 2, 40);

    // init_customer_nearest_station_map();

    this->evals_ = 0.0;
    this->max_evals_ = problem_size_ * MAX_EVALUATION_FACTOR;
    if (num_customer_ <= 100) {
        max_exec_time_ = static_cast<int>(1 * (problem_size_ / 100.0) * 60 * 60);
    } else if (num_customer_ <= 915) {
        max_exec_time_ = static_cast<int>(2 * (problem_size_ / 100.0) * 60 * 60);
    } else {
        max_exec_time_ = static_cast<int>(3 * (problem_size_ / 100.0) * 60 * 60);
    }
    this->convergence_epsilon_ = 1e-4; // 0.0001
    this->max_no_change_count_ = 800; // adjust further  default: 20,000

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

double Case::euclidean_distance(const int i, const int j) const {
    return sqrt(pow(positions_[i].first - positions_[j].first, 2) +
                pow(positions_[i].second - positions_[j].second, 2));
}

int Case::get_best_station(const int from, const int to) const {
    int targetStation = -1;
    double minDis = std::numeric_limits<double>::max();

    for (int i = num_customer_ + 1 ; i < problem_size_; ++i) {
        if (const double dis = distances_[from][i] + distances_[to][i]; minDis > dis && from != i && to != i) {
            targetStation = i;
            minDis = dis;
        }
    }

    return targetStation;
}

int Case::get_best_and_feasible_station(const int from, const int to, const double max_dis) const {
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

int Case::get_customer_demand_(const int customer) const {
    return demand_[customer];
}

double Case::get_distance(const int from, const int to) {
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

// void Case::init_customer_nearest_station_map() {
//     for (int i = 1; i <= num_customer_; ++i) {
//         int nearestStation = -1;
//         double minDis = std::numeric_limits<double>::max();
//         for (int j = num_customer_ + 1; j < problem_size_; ++j) {
//             if (const double dis = distances_[i][j]; minDis > dis) {
//                 nearestStation = j;
//                 minDis = dis;
//             }
//         }
//         customer_to_nearest_station_map_[i] = make_pair(nearestStation, minDis);
//     }
// }

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
        for (const int node : route) {
            temp += get_customer_demand_(node);
        }
        demand_sum_per_route.push_back(temp);
    }

    return demand_sum_per_route;
}

void Case::compute_demand_sum_per_route(int** routes, const int num_routes, const int* num_nodes_per_route, int* demand_sum_per_route) const {
    for (int i = 0; i < num_routes; ++i) {
        int temp = 0;
        for (int j = 0; j < num_nodes_per_route[i]; ++j) {
            temp += get_customer_demand_(routes[i][j]);
        }
        demand_sum_per_route[i] = temp;
    }
}

bool Case::is_charging_station(const int node) const {

    bool flag;
    if (node == depot_ || ( node >= num_depot_ + num_customer_ && node < problem_size_))
        flag = true;
    else
        flag = false;
    return flag;
}