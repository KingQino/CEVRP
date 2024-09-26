//
// Created by Yinghao Qin on 19/09/2024.
//

#include "ma.hpp"

Ma::Ma(Case *instance, int seed, int stop_criteria_option, bool enable_logging) : HeuristicInterface("CBMA", seed),
instance(instance), stop_criteria_option(stop_criteria_option), enable_logging(enable_logging) {
    this->uniform_int_dist = uniform_int_distribution<int>(0, this->instance->num_customer_ - 1);
    this->uniform_swap_dist = uniform_int_distribution<int>(0, this->instance->num_customer_ - 2);
    this->route_cap = this->instance->num_vehicle_ * 3;
    this->node_cap = this->instance->num_customer_ + 1;
    this->iter = 0;
    this->diversity = 0.0;

    this->pop_size = 100;
    this->elite_ratio = 0.01;
    this->immigrant_ratio = 0.05;
    this->crossover_prob = 1.0;
    this->mutation_prob = 0.5;
    this->mutation_ind_prob = 0.2;
    this->tournament_size = 2;
}

Ma::~Ma() {
    population.clear();
}

void Ma::run() {
    // Initialize time variables
    start = std::chrono::high_resolution_clock::now();

    if (enable_logging) {
        open_log_for_evolution();  // Open log if logging is enabled
    }

    initialize_heuristic();

    switch (this->stop_criteria_option) {
        case 0:
            while (!stop_criteria_max_evals()) {
                run_heuristic();

                if (enable_logging) {
                    flush_row_into_evol_log();
                }
            }
            break;

        case 1:
            while (!stop_criteria_max_exec_time(duration)) {
                run_heuristic();
                duration = std::chrono::high_resolution_clock::now() - start;

                if (enable_logging) {
                    flush_row_into_evol_log();
                }
            }
            break;

        default:
            std::cerr << "Invalid stop criteria option!" << std::endl;
            break;
    }

    if (enable_logging) {
        close_log_for_evolution();  // Close log if logging is enabled
        save_log_for_solution();    // Save the log if logging is enabled
    }
}

bool Ma::stop_criteria_max_evals() const {
    return instance->get_evals() >= instance->max_evals_;
}

bool Ma::stop_criteria_max_exec_time(const std::chrono::duration<double>& duration) const {
    return duration.count() >= instance->max_exec_time_;
}

void Ma::initialize_heuristic() {
    // using clustering approach to initialize the population
    for (int i = 0; i < this->pop_size; ++i) {
        vector<vector<int>> routes = routes_constructor_with_hien_method(*instance, random_engine);
        population.push_back(std::make_shared<Individual>(route_cap, node_cap, routes,
                                                          instance->compute_total_distance(routes),
                                                          instance->compute_demand_sum_per_route(routes)));
    }
    global_best = make_unique<Individual>(route_cap, node_cap);
    global_best->lower_cost = INFEASIBLE;
}

void Ma::run_heuristic() {
    iter++;

    for(auto& ind : population) {
        // upper-level optimisation
        two_opt_for_individual(*ind, *instance);
        two_opt_star_for_individual(*ind, *instance);
        one_point_move_intra_route_for_individual(*ind, *instance);

        // lower-level optimisation
        fix_one_solution(*ind, *instance);
    }

    // iteration-best individual goes to the next iteration
    shared_ptr<Individual> iter_best = select_best_individual(population); // create a new copy of the iter best
    if (global_best->lower_cost > iter_best->lower_cost) {
        global_best = make_unique<Individual>(*iter_best);
    }


    vector<vector<int>> local_optimised_pool;
    local_optimised_pool.reserve(pop_size);
    for(auto& ind : population) {
        local_optimised_pool.push_back(ind->get_chromosome());
    }

    vector<vector<int>> chromosomes;
    chromosomes.reserve(pop_size);

    // adaptive selection for crossover
    diversity = calculate_diversity_by_normalized_fitness_difference(extract_fitness_values(population));
    if (diversity > 0.5 ) {
        // a value close to 1 indicates high diversity
        int cx_count_between_elites = int(0.45 * pop_size);
        int cx_count_between_elite_and_immigrant = int(0.05 * pop_size);
        for (int i = 0; i < cx_count_between_elites; ++i) {
            vector<int> selected_indices = select_random(pop_size, 2, random_engine);
            vector<int> elite1(local_optimised_pool[selected_indices[0]]);
            vector<int> elite2(local_optimised_pool[selected_indices[1]]);

            cx_partially_matched(elite1, elite2, random_engine);

            chromosomes.push_back(std::move(elite1));
            chromosomes.push_back(std::move(elite2));
        }
        for (int i = 0; i < cx_count_between_elite_and_immigrant; ++i) {
            int selected_index = select_random(pop_size, 1, random_engine)[0];
            vector<int> elite(local_optimised_pool[selected_index]);
            vector<int> immigrant = get_immigrant_chromosome(random_engine);

            cx_partially_matched(elite, immigrant, random_engine);

            chromosomes.push_back(std::move(elite));
            chromosomes.push_back(std::move(immigrant));
        }
    } else {
        // a value near 0 indicates low diversity
        int cx_count = int(0.5 * pop_size);

        for (int i = 0; i < cx_count; ++i) {
            int selected_index = select_random(pop_size, 1, random_engine)[0];
            vector<int> elite(local_optimised_pool[selected_index]);
            vector<int> immigrant = get_immigrant_chromosome(random_engine);

            cx_partially_matched(elite, immigrant, random_engine);

            chromosomes.push_back(std::move(elite));
            chromosomes.push_back(std::move(immigrant));
        }
    }

    // mutation
    for (auto& chromosome : chromosomes) {
        if (uniform_real_dist(random_engine) < mutation_prob) {
            mut_shuffle_indexes(chromosome, mutation_ind_prob, random_engine);
        }
    }

    // free memory of population and refill it with new chromosomes
    population.clear();
    population.shrink_to_fit();
    population.reserve(pop_size);
    population.push_back(iter_best);
    for (int i = 0; i < pop_size - 1; ++i) {
        population.push_back(admit_one_individual(chromosomes[i]));
    }
}

void Ma::open_log_for_evolution() {
    string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
    create_directories_if_not_exists(directory);

    string file_name = "evols." + instance->instance_name_ + ".csv";
    log_evolution.open(directory + "/" + file_name);
    log_evolution << "iter,upper_cost,lower_cost,diversity,evals\n";
}

void Ma::flush_row_into_evol_log() {
    oss_row_evol << iter << "," << global_best->upper_cost << "," << global_best->lower_cost << ","
                 << diversity << "," << instance->get_evals() << "\n";

}

void Ma::close_log_for_evolution() {
    log_evolution << oss_row_evol.str();
    oss_row_evol.clear();
    log_evolution.close();
}

void Ma::save_log_for_solution() {
    string directory = kStatsPath + "/" + this->name + "/" + instance->instance_name_ + "/" + to_string(seed);
//    create_directories_if_not_exists(directory);

    string file_name = "solution." + instance->instance_name_ + ".txt";

    log_solution.open(directory + "/" + file_name);
    log_solution << fixed << setprecision(5) << global_best->lower_cost << endl;
    for (int i = 0; i < global_best->num_routes; ++i) {
        for (int j = 0; j < global_best->lower_num_nodes_per_route[i]; ++j) {
            log_solution << global_best->lower_routes[i][j] << ",";
        }
        log_solution << endl;
    }
    log_solution.close();
}

shared_ptr<Individual> Ma::admit_one_individual(const vector<int>& chromosome) {
    pair<vector<int>, double> result = classical_split(chromosome, *instance);
    vector<int> split_path = result.first;

    shared_ptr<Individual> ind_ptr = std::make_shared<Individual>(route_cap, node_cap);

    int route_index = 0;

    int j = static_cast<int>(chromosome.size());
    while (true) {
        int i = split_path[j];

        int customer_pos = 1;
        for (auto it = chromosome.begin() + i; it < chromosome.begin() + j; ++it) {
            ind_ptr->routes[route_index][customer_pos++] = *it;
        }
        ind_ptr->num_nodes_per_route[route_index] = customer_pos + 1;

        route_index++; // Move to the next route

        j = i;
        if (i == 0) {
            break;
        }
    }

    ind_ptr->num_routes = route_index;
    ind_ptr->upper_cost = result.second;

    instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, ind_ptr->demand_sum_per_route);

    return std::move(ind_ptr);
}

shared_ptr<Individual> Ma::select_best_individual(const vector<shared_ptr<Individual>> &individuals) {
    if (population.empty()) {
        return nullptr;
    }

    auto comparator = [](const shared_ptr<Individual>& ind1, const shared_ptr<Individual>& ind2) {
        return ind1->lower_cost < ind2->lower_cost;
    };

    auto best_individual = std::min_element(population.begin(), population.end(), comparator);

    return *best_individual;
}

double Ma::calculate_diversity_by_normalized_fitness_difference(const vector<double>& fitness_values) {
    int N = static_cast<int>(fitness_values.size());

    // Check if population size is sufficient
    if (N < 2) {
        std::cerr << "Population size must be at least 2 to calculate diversity." << std::endl;
        return 0.0;
    }

    double f_max = *std::max_element(fitness_values.begin(), fitness_values.end());
    double f_min = *std::min_element(fitness_values.begin(), fitness_values.end());

    // If all fitness values are the same, diversity is zero
    if (f_max == f_min) {
        return 0.0;
    }

    // Calculate the sum of normalized fitness differences between all pairs
    double diversity_sum = 0.0;
    for (int i = 0; i < N - 1; ++i) {
        for (int j = i + 1; j < N; ++j) {
            diversity_sum += std::abs(fitness_values[i] - fitness_values[j]) / (f_max - f_min);
        }
    }

    return (2.0 * diversity_sum) / (N * (N - 1));
}

vector<double> Ma::extract_fitness_values(const vector<shared_ptr<Individual>>& individuals) {
    vector<double> fitness_values;
    fitness_values.reserve(individuals.size());


    for (const auto& ind : individuals) {
        fitness_values.push_back(ind->lower_cost);
    }

    return std::move(fitness_values);
}

vector<int> Ma::select_random(int length, int k, std::default_random_engine &rng) {
    std::vector<int> indices(length);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    indices.resize(k);

    return std::move(indices);
}

void Ma::cx_partially_matched(vector<int>& parent1, vector<int>& parent2, std::default_random_engine &rng) {
    int point1 = uniform_int_dist(rng);
    int point2 = uniform_int_dist(rng);

    if (point1 > point2) {
        swap(point1, point2);
    }

    // Copy the middle segment from parents to children
    vector<int> child1(parent1.begin() + point1, parent1.begin() + point2);
    vector<int> child2(parent2.begin() + point1, parent2.begin() + point2);

    // Create a mapping of genes between parents
    unordered_map<int, int> mapping1, mapping2;

    // Initialize mapping with the middle segment
    for (int i = 0; i < point2 - point1; ++i) {
        mapping1[child2[i]] = child1[i];
        mapping2[child1[i]] = child2[i];
    }

    // Copy the rest of the genes, filling in the mapping
    for (int i = 0; i < parent1.size(); ++i) {
        if (i < point1 || i >= point2) {
            int gene1 = parent1[i];
            int gene2 = parent2[i];

            while (mapping1.find(gene1) != mapping1.end()) {
                gene1 = mapping1[gene1];
            }

            while (mapping2.find(gene2) != mapping2.end()) {
                gene2 = mapping2[gene2];
            }

            child1.push_back(gene2);
            child2.push_back(gene1);
        }
    }

    // Modify the input arguments directly
    parent1 = std::move(child1);
    parent2 = std::move(child2);
}

void Ma::mut_shuffle_indexes(vector<int>& chromosome, double independent_prob, std::default_random_engine& rng) {
    int size = static_cast<int>(chromosome.size());

    for (int i = 0; i < size; ++i) {
        if (uniform_real_dist(rng) < independent_prob) {
            int swap_index = uniform_swap_dist(rng);
            if (swap_index >= i) {
                swap_index += 1;
            }
            swap(chromosome[i], chromosome[swap_index]);
        }
    }
}

vector<int> Ma::get_immigrant_chromosome(std::default_random_engine& rng) const {
    vector<int> immigrant(instance->customers_);
    std::shuffle(immigrant.begin(), immigrant.end(), rng);

    return std::move(immigrant);
}