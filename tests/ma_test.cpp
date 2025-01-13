//
// Created by Yinghao Qin on 20/09/2024.
//

#include "gtest/gtest.h"
#include "ma.hpp"

using namespace ::testing;
using namespace std;

class MaTest : public Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(1, file_name_);
        ma = new Ma(instance, 1, 0, false);
    }

    void TearDown() override {
        delete ma;
        delete instance;
    }

    Case* instance{};
    Ma* ma{};
};


TEST_F(MaTest, AdmitOneIndividual) {
    shared_ptr<Individual> ind_ptr = ma->admit_one_individual(instance->customers_);

    EXPECT_EQ(ind_ptr->num_routes, 5);
    EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, 569.47817556079303);
    EXPECT_EQ(ind_ptr->num_nodes_per_route[0], 5);
}

TEST_F(MaTest, CalculateDiversityByNormalizedFitnessDifference) {
    std::vector<double> fitness_values = {45.2, 70, 57, 44, 62};

    double diversity = Ma::calculate_diversity_by_normalized_fitness_difference(fitness_values);
    EXPECT_NEAR(diversity, 0.529231, 0.0001);
}

TEST_F(MaTest, SelectRandom) {
    // Note: on different platforms or different-version compilers, the selected indices may vary (i.e., the generated random engine may be different)
    std::vector<int> chromosome = {1, 2, 3, 4, 5};
    std::vector<int> selected_indices = Ma::sel_random(5, 2, ma->random_engine);

    EXPECT_EQ(selected_indices.size(), 2);
}

TEST_F(MaTest, CxPartiallyMatched) {
    std::vector<int> parent1(instance->num_customer_);
    std::iota(parent1.begin(), parent1.end(), 1);
    vector<int> parent2(parent1);
    std::shuffle(parent2.begin(), parent2.end(), ma->random_engine);

    ma->cx_partially_matched(parent1, parent2, ma->random_engine);

    EXPECT_EQ(parent1.size(), instance->num_customer_);
    EXPECT_EQ(parent2.size(), instance->num_customer_);
}

TEST_F(MaTest, MutShuffleIndexes) {
    std::vector<int> chromosome(instance->num_customer_);
    std::iota(chromosome.begin(), chromosome.end(), 1);

    ma->mut_shuffle_indexes(chromosome, 0.5, ma->random_engine);

    EXPECT_EQ(chromosome.size(), instance->num_customer_);
    EXPECT_NE(chromosome[0], 1);
}

TEST_F(MaTest, Run) {
    ma->run();

    EXPECT_GE(instance->evals_, instance->max_evals_);
    EXPECT_EQ(ma->population.size(), 100);
    EXPECT_EQ(ma->global_best->num_routes, 4);
    EXPECT_NEAR(ma->global_best->lower_cost, 384.678, 0.0001);
}

TEST_F(MaTest, InitIndByChromosome) {
    vector<vector<int>> routes = {
            {0,17,20,18,15,12,0},
            {0,16,19,21,14,0},
            {0,10,1,2,5,7,9,0},
            {0,8,6,3,4,11,13,0}
    };
    double upper_cost = instance->compute_total_distance(routes);
    vector<int> demand_sum_per_route = instance->compute_demand_sum_per_route(routes);
    unique_ptr<Individual> ind_constructor = make_unique<Individual>(ma->route_cap, ma->node_cap, routes, upper_cost, demand_sum_per_route);


    vector<int> chromosome = {17,20,18,15,12,16,19,21,14,10,1,2,5,7,9,8,6,3,4,11,13};
    unique_ptr<Individual> ind_init_by_chromosome = make_unique<Individual>(ma->route_cap, ma->node_cap);
    ma->init_ind_by_chromosome(*ind_init_by_chromosome, chromosome);


    // check the diversity-related parameters
    EXPECT_EQ(ind_constructor->successors, ind_init_by_chromosome->successors);
    EXPECT_TRUE(std::equal(
            ind_constructor->predecessors.begin() + 1, ind_constructor->predecessors.end(),
            ind_init_by_chromosome->predecessors.begin() + 1, ind_init_by_chromosome->predecessors.end()
    ));
}

TEST_F(MaTest, UpdateProximateIndividuals) {
    ma->initialize_heuristic();

    ma->update_proximate_individuals();

    std::sort(ma->population.begin(), ma->population.end(), [](const unique_ptr<Individual>& a, const unique_ptr<Individual>& b) {
        return a->upper_cost < b->upper_cost;
    });

    // Ranking the individuals based on their diversity contribution (decreasing order of distance)
    vector<pair<double, int>> div_ranking;
    div_ranking.reserve((int)ma->population.size());
    for (int i = 0 ; i < (int)ma->population.size(); i++) {
        div_ranking.emplace_back(-Ma::average_broken_pairs_distance_closest(*ma->population[i], 5),i);
    }
    std::sort(div_ranking.begin(), div_ranking.end()); // the value is negative, so the smaller the value, the larger the distance (i.e., the higher the diversity)

    EXPECT_EQ(div_ranking.size(), 100);
    EXPECT_EQ(ma->population[div_ranking[99].second]->proximate_individuals.size(), 99);
    EXPECT_LT(div_ranking[0].first, div_ranking[99].first);

    // print div_ranking
//    cout << "Diversity ranking: \n";
//    for (int i = 0; i < (int)ma->population.size(); i++) {
//        cout << div_ranking[i].first << " ";
//        cout << div_ranking[i].second << " ";
//        cout << ma->population[div_ranking[i].second]->upper_cost << "\n";
//    }
//    cout << endl;

    // Update the biased fitness values
    for (int i = 0; i < ma->pop_size; ++i) {
        double normalized_div_ranking = (double)i / (double)(ma->pop_size - 1); // ranking from 0 to 1 => 0 is the best
        double normalized_obj_ranking = (double)div_ranking[i].second / (double)(ma->pop_size - 1); // calculate the corresponding ranking of the individual in terms of objective value => 0 is the best

        ma->population[div_ranking[i].second]->biased_fitness = normalized_obj_ranking + (1.0 - (double)ma->num_elite / (double)ma->pop_size) * normalized_div_ranking;
    }

    vector<pair<double, int>> biased_fitness_ranking;
    biased_fitness_ranking.reserve((int)ma->population.size());
    for (int i = 0 ; i < (int)ma->population.size(); i++) {
        biased_fitness_ranking.emplace_back(ma->population[i]->biased_fitness,i);
    }
    std::sort(biased_fitness_ranking.begin(), biased_fitness_ranking.end());

//    cout << "Biased fitness values: \n";
//    for (int i = 0; i < (int)ma->population.size(); i++) {
//        cout << biased_fitness_ranking[i].first << " ";
//        cout << biased_fitness_ranking[i].second << "\n";
//    }
//    cout << endl;

    EXPECT_EQ(biased_fitness_ranking.size(), 100);
    EXPECT_LT(biased_fitness_ranking[0].first, biased_fitness_ranking[99].first);

    // judge whether the duplicate individual is in the population
    int count = 0;
    for (int i = 0; i < ma->pop_size; ++i) {
        if (Ma::average_broken_pairs_distance_closest(*ma->population[i], 1) < 1e-8) {
            count += 1;
        }
    }

    EXPECT_GT(count, 1);
}