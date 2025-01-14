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
    this->num_closest = 5;
    this->num_elite = 4; // or 1
    this->tournament_size = 2;
    this->refine_threshold_ratio = 1.5;

    this->gammaL = 1.2;
    this->gammaR = 0.8;
    this->delta = 30;
    this->r = 0.0;
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

//    global_best = std::move(refine_limited_memory(*global_best, *instance, global_upper_best->upper_cost, refine_threshold_ratio));

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

bool Ma::stop_criteria_obj_convergence(double current_best_obj) const {
    static int no_change_count = 0; // consecutive no-change count
    static double prev_best_obj = std::numeric_limits<double>::max(); // previous best objective, initialized to infinity

    // calculate the change in objective value
    double obj_change = std::abs(current_best_obj - prev_best_obj);

    // If change is small, increment count; otherwise, reset
    if (obj_change < instance->convergence_epsilon_) {
        no_change_count++;
    } else {
        no_change_count = 0;
    }

    // update previous objective value
    prev_best_obj = current_best_obj;

    // check if max no-change count is reached
    return no_change_count >= instance->max_no_change_count_;
}

void Ma::initialize_heuristic() {
    // using clustering approach to initialize the population
    for (int i = 0; i < this->pop_size; ++i) {
        vector<vector<int>> routes = routes_constructor_with_hien_method(*instance, random_engine);
        population.push_back(std::make_unique<Individual>(route_cap, node_cap, routes,
                                                          instance->compute_total_distance(routes),
                                                          instance->compute_demand_sum_per_route(routes)));
    }
    global_best = make_unique<Individual>(route_cap, node_cap);
    global_best->lower_cost = INFEASIBLE;

//    global_upper_best = make_unique<Individual>(route_cap, node_cap);
//    global_upper_best->upper_cost = INFEASIBLE;
}

void Ma::run_heuristic() {
    iter++;

    vector<Individual*> S1;
    S1.reserve(population.size());
    for (const auto& individual : population) {
        S1.push_back(individual.get());
    }

    double v1 = 0;
    double v2;
    auto talented_ind = select_best_upper_individual_ptr(S1);
    if (iter > 30) {
        double old_cost = talented_ind->upper_cost;

        two_opt_intra_for_individual(*talented_ind, *instance);
        two_opt_inter_for_individual(*talented_ind, *instance);
        node_relocation_intra_for_individual(*talented_ind, *instance);

        double new_cost = talented_ind->upper_cost;
        v1 = old_cost - new_cost;
        v2 = *std::max_element(P.begin(), P.end());
        if (v2 < v1) v2 = v1 * gammaL;

        S1.clear();
        for (auto& ind:population) {
            if (ind->upper_cost - v2 <= new_cost) S1.push_back(ind.get());
        }

        auto it = std::find(S1.begin(), S1.end(), talented_ind);
        // If talented_ind is found in S1, remove it to avoid duplication for upper-level optimisation
        if (it != S1.end()) {
            S1.erase(it);
        }
    }

    // make local search on S1
    v2 = 0;
    for(auto ind : S1) {
        double old_cost = ind->upper_cost;
        two_opt_intra_for_individual(*ind, *instance);
        two_opt_inter_for_individual(*ind, *instance);
        node_relocation_intra_for_individual(*ind, *instance);
        if (v2 < old_cost - ind->upper_cost)
            v2 = old_cost - ind->upper_cost;
    }
    v2 = (v1 > v2) ? v1 : v2;
    P.push_back(v2);
    if (P.size() > delta)  P.pop_front();
    if (iter > delta) S1.push_back(talented_ind); //  *** switch off ***

    // Current S1 has been selected and local search.
    // Pick a portion of the upper sub-solutions to go for recharging process, by the difference between before and after charging of the best solution in S1
    vector<Individual*> S2 = S1;
    double v3;
    Individual* outstanding_upper = select_best_upper_individual_ptr(S1);
    if (iter > 0) { // switch - which is a virtual switch, always true, need to be implemented
        double upper_cost = outstanding_upper->upper_cost; // fitness without recharging f
        fix_one_solution(*outstanding_upper, *instance); // fitness with recharging F
        double lower_cost = outstanding_upper->lower_cost;
        v3 = lower_cost - upper_cost;
        if (r > v3) r = v3 * gammaR;

        S2.clear();
        for (auto& ind: S1) {
            if (ind->upper_cost + r <= lower_cost)
                S2.push_back(ind);
        }

        auto it = std::find(S2.begin(), S2.end(), outstanding_upper);
        // If outstanding_upper is found, remove it from S2
        if (it != S2.end()) {
            S2.erase(it);
        }
    }

    // Current S2 has been selected and ready for recharging, make recharging on S2
    vector<Individual*> S3;
    S3.push_back(outstanding_upper);
    for (auto& ind:S2) {
        double old_cost = ind->upper_cost;
        fix_one_solution(*ind, *instance);
        double new_cost = ind->lower_cost;
        S3.push_back(ind);
        if (v3 > new_cost - old_cost)
            v3 = new_cost - old_cost;
    }
    if (r == 0 || r > v3) {
        r = v3;
    }

    // statistics
    unique_ptr<Individual> iter_best = make_unique<Individual>(*select_best_individual_ptr(S3));
    if (global_best->lower_cost > iter_best->lower_cost) {
        global_best = make_unique<Individual>(*iter_best);
    }
//    update_proximate_individuals();
//    diversity = calculate_diversity_by_broken_paris_distance(population, num_closest);
//    auto iter_upper_best = select_best_upper_individual_ptr(S1);
//    if (global_upper_best->upper_cost > iter_upper_best->upper_cost) {
//        global_upper_best = make_unique<Individual>(*iter_upper_best); // create a new copy of the global upper best
//    }


    // adaptive selection for crossover
    vector<vector<int>> promising_seqs;
    promising_seqs.reserve(S3.size());
    for(auto& sol : S3) {
        promising_seqs.push_back(sol->get_chromosome()); // encoding
    }
    vector<vector<int>> average_seqs;
    for(auto& sol : population) {
        // judge whether sol in S3 or not
        auto it = std::find(S3.begin(), S3.end(), sol.get());
        if (it != S3.end()) continue;
        average_seqs.push_back(sol->get_chromosome()); // encoding
    }


    vector<vector<int>> chromosomes;
    chromosomes.reserve(pop_size);
    if (promising_seqs.size() == 1) {
        const vector<int>& father = promising_seqs[0];
        // 90% - elite x non-elites
        for (int i = 0; i < int (0.45 * pop_size); ++i) {
            vector<int> _father(father);
            vector<int> mother = sel_random(average_seqs, 1, random_engine)[0];

            cx_partially_matched(_father, mother, random_engine);

            chromosomes.push_back(std::move(_father));
            chromosomes.push_back(std::move(mother));
        }
        // 9%  - elite x immigrants
        for (int i = 0; i < int(0.05 * pop_size); ++i) {
            vector<int> _father(father);
            vector<int> mother(instance->customers_);

            shuffle(mother.begin(), mother.end(), random_engine);
            cx_partially_matched(_father, mother, random_engine);

            chromosomes.push_back(std::move(_father));
            chromosomes.push_back(std::move(mother));
        }
    } else {
        // boundary condition - average_seqs might be empty
        // if the size of the average population is quite small, then only the promising_seqs mate
        if (average_seqs.size() < int(0.1 * pop_size)) {
            for (int i = 0; i < int(pop_size/2); ++i) {
                vector<vector<int>> parents = sel_random(promising_seqs, 2, random_engine);

                cx_partially_matched(parents[0], parents[1], random_engine);

                chromosomes.push_back(std::move(parents[0]));
                chromosomes.push_back(std::move(parents[1]));
            }
        } else {
            // part of elites x elites
            int num_promising_seqs = static_cast<int>(promising_seqs.size());
            int loop_num = int(num_promising_seqs / 2.0) < (pop_size/2) ? int(num_promising_seqs / 2.0) : int(pop_size/4);
            for (int i = 0; i < loop_num; ++i) {
                vector<vector<int>> parents = sel_random(promising_seqs, 2, random_engine);

                cx_partially_matched(parents[0], parents[1], random_engine);

                chromosomes.push_back(std::move(parents[0]));
                chromosomes.push_back(std::move(parents[1]));
            }
            // portion of elites x non-elites
            int num_promising_x_average = pop_size - static_cast<int>(chromosomes.size());
            for (int i = 0; i < int(num_promising_x_average / 2.0); ++i) {
                vector<int> parent1 = sel_random(promising_seqs, 1, random_engine)[0];
                vector<int> parent2 = sel_random(average_seqs, 1, random_engine)[0];

                cx_partially_matched(parent1, parent2, random_engine);

                chromosomes.push_back(std::move(parent1));
                chromosomes.push_back(std::move(parent2));
            }
        }
    }

    // mutation
    for (auto& chromosome : chromosomes) {
        if (uniform_real_dist(random_engine) < mutation_prob) {
            mut_shuffle_indexes(chromosome, mutation_ind_prob, random_engine);
        }
    }

    // clean up the S1, S2, S3
    S3.clear();
    S2.clear();
    S1.clear();
    // update the population
    for (int i = 0; i < pop_size; ++i) {
        if (population[i].get() == iter_best.get()) continue; // Skip the best individual

        // reset individual & update through chromosome
        population[i]->reset();

        init_ind_by_chromosome(*population[i], chromosomes[i]);
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

void Ma::init_ind_by_chromosome(Individual &ind, const vector<int> &chromosome) const {
    pair<vector<int>, double> result = classical_split(chromosome, *instance);
    vector<int> split_path = result.first;

    int route_index = 0;

    int j = static_cast<int>(chromosome.size());
    while (true) {
        int i = split_path[j];

        int customer_pos = 1;
        for (auto it = chromosome.begin() + i; it < chromosome.begin() + j; ++it) {
            ind.routes[route_index][customer_pos] = *it;

            // Update predecessors and successors
            if (customer_pos == 1) {
                ind.predecessors[*it] = instance->depot_; // First customer of the route points to the depot
            } else {
                int prev_node = ind.routes[route_index][customer_pos - 1];
                ind.predecessors[*it] = prev_node;
                ind.successors[prev_node] = *it;
            }
            customer_pos++;
        }
        // Set depot as successor of the last node in the route
        int last_node = ind.routes[route_index][customer_pos - 1];
        ind.successors[last_node] = instance->depot_; // Last customer points back to the depot

        ind.num_nodes_per_route[route_index] = customer_pos + 1;

        route_index++; // Move to the next route

        j = i;
        if (i == 0) {
            break;
        }
    }

    ind.num_routes = route_index;
    ind.upper_cost = result.second;
    instance->compute_demand_sum_per_route(ind.routes, ind.num_routes, ind.num_nodes_per_route, ind.demand_sum_per_route);
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

Individual& Ma::select_best_individual_ref(const vector<unique_ptr<Individual>>& individuals) {
    assert(!individuals.empty());  // Ensure there's at least one individual

    auto comparator = [](const unique_ptr<Individual>& ind1, const unique_ptr<Individual>& ind2) {
        return ind1->lower_cost < ind2->lower_cost;
    };

    auto best_individual = std::min_element(individuals.begin(), individuals.end(), comparator);

    return **best_individual;
}

Individual* Ma::select_best_individual_ptr(const vector<Individual*>& individuals) {
    assert(!individuals.empty());  // Ensure there's at least one individual
    auto comparator = [](const Individual* ind1, const Individual* ind2) {
        return ind1->lower_cost < ind2->lower_cost;
    };

    auto best_individual = std::min_element(individuals.begin(), individuals.end(), comparator);
    return *best_individual;
}

Individual* Ma::select_best_upper_individual_ptr(const vector<Individual*>& individuals) {
    assert(!individuals.empty());  // Ensure there's at least one individual
    auto comparator = [](const Individual* ind1, const Individual* ind2) {
        return ind1->upper_cost < ind2->upper_cost;
    };
    auto best_individual = std::min_element(individuals.begin(), individuals.end(), comparator);
    return *best_individual;
}

double Ma::calculate_diversity_by_broken_paris_distance(const vector<unique_ptr<Individual>>& individuals, int num_closest) {
    double sum = 0.;
    int size = static_cast<int>(individuals.size());
    for (int i = 0; i < size; ++i) {
        sum += average_broken_pairs_distance_closest(*individuals[i], num_closest);
    }

    return sum/(double)size;
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

vector<double> Ma::extract_fitness_values(const vector<unique_ptr<Individual>>& individuals) {
    vector<double> fitness_values;
    fitness_values.reserve(individuals.size());


    for (const auto& ind : individuals) {
        fitness_values.push_back(ind->lower_cost);
    }

    return std::move(fitness_values);
}

vector<int> Ma::sel_random(int length, int k, std::default_random_engine &rng) {
    std::vector<int> selected_indices;
    std::uniform_int_distribution<int> dist(0, length - 1);

    while (static_cast<int>(selected_indices.size()) < k) {
        selected_indices.push_back(dist(rng));
    }

    return std::move(selected_indices);
}

vector<vector<int>> Ma::sel_random(const vector<vector<int>>& chromosomes, int k, std::default_random_engine& rng) {
    vector<vector<int>> selected_chromosomes;
    selected_chromosomes.reserve(k);

    std::uniform_int_distribution<std::size_t> dist(0, chromosomes.size() - 1);
    for (int i = 0; i < k; ++i) {
        std::size_t randomIndex = dist(rng);
        selected_chromosomes.push_back(chromosomes[randomIndex]);
    }

    return std::move(selected_chromosomes);
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

double Ma::broken_pairs_distance(const Individual& ind1, const Individual& ind2) const {
    int differences = 0;
    for (int j = 1; j <= instance->num_customer_; j++) {
        if (ind1.successors[j] != ind2.successors[j] && ind1.successors[j] != ind2.predecessors[j]) differences++;
        if (ind1.predecessors[j] == 0 && ind2.predecessors[j] != 0 && ind2.successors[j] != 0) differences++;
    }
    return (double)differences / (double)instance->num_customer_;
}

double Ma::average_broken_pairs_distance_closest(const Individual& ind, int num_closest = 5) {
    double result = 0.0;
    int max_size = std::min<int>(num_closest, static_cast<int>(ind.proximate_individuals.size()));
    auto it = ind.proximate_individuals.begin();
    for (int i = 0; i < max_size; i++) {
        result += it->first;
        ++it;
    }
    return result / (double)max_size;
}

void Ma::update_proximate_individuals() {
    for (size_t i = 0; i < population.size(); ++i) {
        auto& ind_i = *population[i];

        for (size_t j = i + 1; j < population.size(); ++j) {
            auto& ind_j = *population[j];

            // Calculate the distance only once for the pair
            double distance = broken_pairs_distance(ind_i, ind_j);

            // Update both individuals' proximity multisets
            ind_i.proximate_individuals.insert({distance, &ind_j});
            ind_j.proximate_individuals.insert({distance, &ind_i});
        }
    }
}

void Ma::update_biased_fitness() {
    for (size_t i = 0; i < population.size(); ++i) {
        auto& ind_i = *population[i];

        for (size_t j = i + 1; j < population.size(); ++j) {
            auto& ind_j = *population[j];

            // Calculate the distance only once for the pair
            double distance = broken_pairs_distance(ind_i, ind_j);

            // Update both individuals' proximity multisets
            ind_i.proximate_individuals.insert({distance, &ind_j});
            ind_j.proximate_individuals.insert({distance, &ind_i});
        }
    }

    std::sort(population.begin(), population.end(), [](const unique_ptr<Individual>& a, const unique_ptr<Individual>& b) {
        return a->upper_cost < b->upper_cost;
    });

    // Ranking the individuals based on their diversity contribution (decreasing order of distance)
    vector<pair<double, int>> div_ranking;
    div_ranking.reserve(pop_size);
    for (int i = 0 ; i < pop_size; i++) {
        div_ranking.emplace_back(-Ma::average_broken_pairs_distance_closest(*population[i], num_closest),i);
    }
    std::sort(div_ranking.begin(), div_ranking.end()); // the value is negative, so the smaller the value, the larger the distance (i.e., the higher the diversity)

    // Update the biased fitness values
    for (int i = 0; i < pop_size; ++i) {
        double normalized_div_ranking = (double)i / (double)(pop_size - 1); // ranking from 0 to 1 => 0 is the best
        double normalized_obj_ranking = (double)div_ranking[i].second / (double)(pop_size - 1); // calculate the corresponding ranking of the individual in terms of objective value => 0 is the best

        population[div_ranking[i].second]->biased_fitness = normalized_obj_ranking + (1.0 - (double)num_elite / (double)pop_size) * normalized_div_ranking;
    }
}