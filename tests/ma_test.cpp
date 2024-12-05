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
        random_engine = std::default_random_engine(0);
    }

    void TearDown() override {
        delete ma;
        delete instance;
    }

    Case* instance{};
    Ma* ma{};
    std::default_random_engine random_engine;
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
    std::vector<int> chromosome = {1, 2, 3, 4, 5};
    std::vector<int> selected_indices = Ma::select_random(5, 2, random_engine);

    EXPECT_EQ(selected_indices.size(), 2);
    EXPECT_EQ(chromosome.at(selected_indices[0]), 2);
}

TEST_F(MaTest, CxPartiallyMatched) {
    std::vector<int> parent1(instance->num_customer_);
    std::iota(parent1.begin(), parent1.end(), 1);
    vector<int> parent2(parent1);
    std::shuffle(parent2.begin(), parent2.end(), random_engine);

    ma->cx_partially_matched(parent1, parent2, random_engine);

    EXPECT_EQ(parent1.size(), instance->num_customer_);
    EXPECT_EQ(parent2.size(), instance->num_customer_);
    EXPECT_EQ(parent1[0], 15);
    EXPECT_EQ(parent2[0], 1);
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