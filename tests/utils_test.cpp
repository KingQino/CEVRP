//
// Created by Yinghao Qin on 14/09/2024.
//

#include "gtest/gtest.h"
#include "utils.hpp"

using namespace ::testing;
using namespace std;

class UtilsTest : public Test {
protected:
    void SetUp() override {
        string file_name_ = "E-n22-k4.evrp";
        instance = new Case(1, file_name_);
        rng = std::default_random_engine(0);;

        vector<vector<int>> routes = routes_constructor_with_hien_method(*instance, rng);
        individual = std::make_unique<Individual>(8, 22, routes,
                                           instance->compute_total_distance(routes),
                                           instance->compute_demand_sum_per_route(routes));
    }

    void TearDown() override {
        delete instance;
    }

    Case* instance{};
    std::default_random_engine rng;
    unique_ptr<Individual> individual;
};


TEST_F(UtilsTest, PrinsSplit) {
    vector<vector<int>> result = prins_split(    instance->customers_, *instance);

    EXPECT_EQ(result.size(), 5);
    EXPECT_EQ(result[0], vector<int>({19, 20, 21}));
}

TEST_F(UtilsTest, ClassicalSplit) {
    pair<vector<int>, double> cl_sp = classical_split(instance->customers_, *instance);
    vector<int> split_path = cl_sp.first;
//    cout << split_path.size() << endl;
//    for (const auto& node : split_path) {
//        cout << node << " ";
//    }
//    cout << endl << endl;

    vector<int> chromosome = instance->customers_;

    int route_count = 0;                    // Counter for the number of routes
    vector<int> customers_per_route;        // Vector to store the number of customers per route

    auto j = chromosome.size();
    while (true) {
        int i = split_path[j];

        int customer_count = 0;
        for (auto it = chromosome.begin() + i; it < chromosome.begin() + j; ++it) {
            customer_count++;
        }

        customers_per_route.push_back(customer_count);
        route_count++;

        j = i;
        if (i == 0) {
            break;
        }
    }

//    cout << "Total number of routes: " << route_count << endl;
//    for (int k = 0; k < customers_per_route.size(); ++k) {
//        cout << "Number of customers in Route " << k + 1 << ": " << customers_per_route[k] << endl;
//    }

    EXPECT_EQ(route_count, 5);
    EXPECT_EQ(customers_per_route[0], 3);
}

TEST_F(UtilsTest, Hien) {
    vector<vector<int>> routes = hien_clustering(*instance, rng);

    std::size_t size_1 = 0;
    for (const auto& route : routes) {
        size_1 += route.size();
//        cout << "Route: ";
//        for (const auto& customer : route) {
//            cout << customer << " ";
//        }
//        cout << endl;
    }
//    cout << endl;


    hien_balancing(routes, *instance, rng);
    std::size_t size_2 = 0;
    for (const auto& route : routes) {
        size_2 += route.size();
//        cout << "Route: ";
//        for (const auto& customer : route) {
//            cout << customer << " ";
//        }
//        cout << endl;
    }
//    cout << endl;

    // Assert
    EXPECT_EQ(size_1, instance->num_customer_);
    EXPECT_EQ(size_2, instance->num_customer_);
}

TEST_F(UtilsTest, RoutesConstructor) {
    vector<vector<int>> routes_x = routes_constructor_with_split(*instance, rng);
    vector<vector<int>> routes_y = routes_constructor_with_hien_method(*instance, rng);
    vector<vector<int>> routes_z = routes_constructor_with_direct_encoding(*instance, rng);

    EXPECT_EQ(routes_x[0][0], 0);
    EXPECT_NE(routes_x[0][1], 0);
    EXPECT_EQ(routes_y[0][0], 0);
    EXPECT_NE(routes_y[0][1], 0);
    EXPECT_EQ(routes_z[0][0], 0);
    EXPECT_NE(routes_z[0][1], 0);
}

TEST_F(UtilsTest, TwoOptForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 8, 0},
            {0, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {6000, 5500, 5900, 5100};
    shared_ptr<Individual> ind = std::make_shared<Individual>(8, 22, routes, 485.60155200568835, demand_sum_per_route);

    two_opt_for_individual(*ind, *instance);

    EXPECT_DOUBLE_EQ(ind->upper_cost, 392.23351570689545);
}

TEST_F(UtilsTest, TwoOptStarBetweenTwoRoutes) {
    int* route1 = new int[22]{0, 19, 21, 20, 17, 0};
    int* route2 = new int[22]{0, 12, 15, 18, 14, 16, 0};
    int length1 = 6;
    int length2 = 7;
    int loading1 = 6000;
    int loading2 = 5500;
    int node_cap = individual->node_cap;
    double cost = instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2);

    bool updated = two_opt_star_between_two_routes(route1, route2, length1, length2, loading1, loading2, cost, node_cap, *instance);

//    // print route1 and route2
//    for (int i = 0; i < length1; ++i) {
//        cout << route1[i] << " ";
//    }
//    cout << endl;
//    for (int i = 0; i < length2; ++i) {
//        cout << route2[i] << " ";
//    }
//    cout << endl;
//    // print length1 and length2
//    cout << "Length: " << length1 << " " << length2 << endl;
//    // check the loading
//    cout << "Loading: " << loading1 << " " << loading2 << endl;
//    // print the fitness value
//    cout << "Fitness value: " << cost << endl;
//    double cost_validation = instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2);
//    cout << "New Cost: " << cost_validation << endl;

    EXPECT_TRUE(updated);
    EXPECT_EQ(length1, 6);
    EXPECT_EQ(length2, 7);
    EXPECT_EQ(loading1, 5600);
    EXPECT_EQ(loading2, 5900);
    EXPECT_DOUBLE_EQ(cost, instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2));

    delete[] route1;
    delete[] route2;
}

TEST_F(UtilsTest, TwoOptStarForIndividual) {
//    two_opt_move_inter_route_for_individual
    vector<vector<int>> routes = {
            {0, 9, 7, 5, 2, 1, 6, 8, 0},
            {0, 10, 3, 4, 11, 13, 0},
            {0, 19, 21, 20, 17, 0},
            {0, 12, 15, 18, 14, 16, 0}
    };
    vector<int> demand_sum_per_route = {5700, 5300, 6000, 5500};
    shared_ptr<Individual> ind = std::make_shared<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);

    double fit_prev = ind->upper_cost;
    bool updated = two_opt_move_inter_route_for_individual(*ind, *instance);
    double fit_cur = ind->upper_cost;

    EXPECT_LT(fit_cur, fit_prev);
    EXPECT_TRUE(updated);
}

TEST_F(UtilsTest, OnePointMoveIntraRouteForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    shared_ptr<Individual> ind = std::make_shared<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);

    double fit_prev = ind->upper_cost;
    one_point_move_intra_route_for_individual(*ind, *instance);
    double fit_cur = ind->upper_cost;

    EXPECT_LT(fit_cur, fit_prev);
    EXPECT_DOUBLE_EQ(fit_cur, instance->compute_total_distance(ind->routes, ind->num_routes, ind->num_nodes_per_route));
}

TEST_F(UtilsTest, NodeShiftBetweenTwoRoutes) {
    // interesting node: 6, demand_ 400
    int* route1 = new int[22]{0,10,8,6,3, 4,11,13,0}; //5800
    int* route2 = new int[22]{0,1,2,5,7,9,0}; // 5200
    int length1 = 9;
    int length2 = 7;
    int loading1 = 5800;
    int loading2 = 5200;
    double cost = instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2);

    node_shift_between_two_routes(route1, route2, length1, length2, loading1, loading2, cost, *instance);

    // expect to obtain better routes, and they are {0,10,8,3,4,11,13,0} and {0,6,1,2,5,7,9,0}
    // print route1 and route2
//    for (int i = 0; i < length1; ++i) {
//        cout << route1[i] << " ";
//    }
//    cout << endl;
//    for (int i = 0; i < length2; ++i) {
//        cout << route2[i] << " ";
//    }
//    cout << endl;
//    // print length1 and length2
//    cout << "Length: " << length1 << " " << length2 << endl;
//    // check the loading
//    cout << "Loading: " << loading1 << " " << loading2 << endl;
//    // print the fitness value
//    cout << "Fitness value: " << cost << endl;
//    double cost_validation = instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2);
//    cout << "New Cost: " << cost_validation << endl;

    EXPECT_EQ(route2[1], 10);
    EXPECT_EQ(length1, 8);
    EXPECT_EQ(length2, 8);
    EXPECT_EQ(loading1, 5200);
    EXPECT_EQ(loading2, 5800);
    EXPECT_DOUBLE_EQ(cost, instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2));

    int* route3 = new int[22]{ 0, 10, 17, 11, 7, 1, 6, 0}; //5100
    int* route4 = new int[22]{ 0, 14, 12, 15, 20, 21, 0}; // 5000
    int length3 = 8;
    int length4 = 7;
    int loading3 = 5100;
    int loading4 = 5000;
    double cost_34 = instance->compute_total_distance(route3, length3) + instance->compute_total_distance(route4, length4);

    node_shift_between_two_routes(route3, route4, length3, length4, loading3, loading4, cost_34, *instance);

    EXPECT_EQ(route3[1], 17);
    EXPECT_EQ(length3, 7);
    EXPECT_EQ(length4, 8);
    EXPECT_EQ(loading3, 4500);
    EXPECT_EQ(loading4, 5600);
    EXPECT_DOUBLE_EQ(cost_34, instance->compute_total_distance(route3, length3) + instance->compute_total_distance(route4, length4));

    delete[] route1;
    delete[] route2;
    delete[] route3;
    delete[] route4;
}

TEST_F(UtilsTest, OnePointMoveInterRouteForIndividual) {
    // interesting node: 6, demand_ 400
    int* route1 = new int[22]{ 0, 10, 17, 11, 7, 1, 6, 0}; //5100
    int* route2 = new int[22]{ 0, 14, 12, 15, 20, 21, 0}; // 5000
    int length1 = 8;
    int length2 = 7;
    int loading1 = 5100;
    int loading2 = 5000;
    double cost = instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2);

    node_shift_between_two_routes(route1, route2, length1, length2, loading1, loading2, cost, *instance);

    // print route1 and route2
//    for (int i = 0; i < length1; ++i) {
//        cout << route1[i] << " ";
//    }
//    cout << endl;
//    for (int i = 0; i < length2; ++i) {
//        cout << route2[i] << " ";
//    }
//    cout << endl;
//    // print length1 and length2
//    cout << "Length: " << length1 << " " << length2 << endl;
//    // check the loading
//    cout << "Loading: " << loading1 << " " << loading2 << endl;
//    // print the fitness value
//    cout << "Fitness value: " << cost << endl;
//    double cost_validation = instance->compute_total_distance(route1, length1) + instance->compute_total_distance(route2, length2);
//    cout << "New Cost: " << cost_validation << endl;
//
//    delete[] route1;
//    delete[] route2;

    vector<vector<int>> routes = routes_constructor_with_split(*instance, rng);
    shared_ptr<Individual> ind = std::make_shared<Individual>(8, 22, routes,
                                                               instance->compute_total_distance(routes),
                                                               instance->compute_demand_sum_per_route(routes));
    double cost_prev = ind->upper_cost;

    one_point_move_inter_route_for_individual(*ind, *instance);

    double cost_cur = ind->upper_cost;

    EXPECT_LT(cost_cur, cost_prev);
    EXPECT_DOUBLE_EQ(cost_cur, instance->compute_total_distance(ind->routes, ind->num_routes, ind->num_nodes_per_route));
}

TEST_F(UtilsTest, TwoPointMoveIntraRouteForIndividual) {
    // expected to obtain a better route, and it is {0, 10, 8, 3, 4, 11, 13, 0}
    int* route = new int[22]{0,3,8,13,4,11,10,0};
    int length = 8;
    double cost = instance->compute_total_distance({0,3,8,13,4,11,10,0});
    double cost_prev = cost;

    two_nodes_swap_for_single_route(route, length, cost, *instance);

    EXPECT_LT(cost, cost_prev);

    delete[] route;


    double fit_prev = individual->upper_cost;
    two_point_move_intra_route_for_individual(*individual, *instance);
    double fit_cur = individual->upper_cost;

    double fit_eval = instance->compute_total_distance(individual->routes, individual->num_routes, individual->num_nodes_per_route);

    EXPECT_LT(fit_cur, fit_prev);
    EXPECT_NEAR(fit_cur, fit_eval, 0.000'000'001);
}

TEST_F(UtilsTest, TwoNodesSwapBetweenTwoRoutes) {
    int* route1 = new int[22]{0,8,3,4,11,13,6, 0};
    int* route2 = new int[22]{0,10, 1,2,5,7,9,0};
    int length1 = 8;
    int length2 = 8;
    int loading1 = 5400;
    int loading2 = 5600;
    double fitv = instance->compute_total_distance({0,8,3,4,11,13,6,0}) + instance->compute_total_distance({0,10,1,2,5,7,9,0});
    double fit_prev = fitv;
    int loading1_prev = loading1;
    int loading2_prev = loading2;

    bool updated = two_nodes_swap_between_two_routes(route1, route2, length1, length2, loading1, loading2, fitv, *instance);

    EXPECT_TRUE(updated);
    EXPECT_EQ(route2[1], 6);
    EXPECT_LT(fitv, fit_prev);
    EXPECT_NE(loading1, loading1_prev);
    EXPECT_EQ(loading1_prev + loading2_prev, loading1 + loading2);

    delete[] route1;
    delete[] route2;
}

TEST_F(UtilsTest, TwoPointMoveInterRouteForIndividual) {
    double fit_prev = individual->upper_cost;
    two_point_move_inter_route_for_individual(*individual, *instance);
    double fit_cur = individual->upper_cost;

    double fit_eval = instance->compute_total_distance(individual->routes, individual->num_routes, individual->num_nodes_per_route);
    int loadingSum = 0;
    for (int i = 0; i < individual->num_routes; ++i) {
        loadingSum += individual->demand_sum_per_route[i];
    }

    EXPECT_LT(fit_cur, fit_prev);
    EXPECT_NEAR(fit_cur, fit_eval, 0.000'000'001);
    EXPECT_EQ(loadingSum, 22'500);
}

TEST_F(UtilsTest, FixOneSolution) {
    vector<vector<int>> routes = {
            {0, 8, 15, 20, 21, 16, 0},
            {0, 14, 18, 9, 5, 7, 10, 0},
            {0, 4, 13, 0},
            {0, 17, 12, 6, 1, 3, 11, 0},
            {0, 19, 2, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5200, 2700, 5800, 3200};
    unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, 678.8177686900328, demand_sum_per_route);

    fix_one_solution(*ind, *instance);
    double fixed_fit = ind->lower_cost;
    int num_nodes = std::accumulate(ind->lower_num_nodes_per_route, ind->lower_num_nodes_per_route + ind->num_routes, 0);

    EXPECT_EQ(fixed_fit, 685.4783939004661);
    EXPECT_EQ(num_nodes,  39);
}

TEST_F(UtilsTest, InsertStationBySimpleEnumerationArray) {
    //  0 9 7 5 2 1 29 6 8 0 10 25 3 4 11 13 0 22 19 21 20 17 0 12 15 18 14 16 0
    // {0,9,7,5,2,1,29,6,8,0}
    // {0,10,25,3,4,11,13,0}
    // {0,22,19,21,20,17,0}
    // {0,12,15,18,14,16,0}
    vector<vector<int>> routes = {
            {0, 9, 7, 5, 2, 1, 6, 8, 0},
            {0, 10, 3, 4, 11, 13, 0},
            {0, 19, 21, 20, 17, 0},
            {0, 12, 15, 18, 14, 16, 0}
    };

    for (auto& r : routes) {
        int* route = new int [instance->num_customer_ + instance->num_depot_];
        memcpy(route, r.data(), r.size() * sizeof(int));
        int length = static_cast<int>(r.size());
        double original_cost = instance->compute_total_distance(route, length);

        // print the original route
//        cout << "Original Route: ";
//        for (int i = 0; i < length; ++i) {
//            cout << route[i] << " ";
//        }
//        cout << endl;
//        cout << "Original Length: " << length << endl;
//        cout << "Original Cost: " << original_cost << endl;

        int* repaired_route = new int[instance->num_customer_ + instance->num_depot_];
        memcpy(repaired_route, route, length * sizeof(int));
        int repaired_length = length;
        double repaired_cost = insert_station_by_simple_enumeration_array(route, length, repaired_route, repaired_length, *instance);

        // print the repaired route
//        cout << "Repaired Route: ";
//        for (int i = 0; i < repaired_length; ++i) {
//            cout << repaired_route[i] << " ";
//        }
//        cout << endl;
//        cout << "Repaired Length: " << repaired_length << endl;
//        cout << "Repaired Cost: " << repaired_cost << endl;
//        cout << endl;

        EXPECT_GE(repaired_cost, original_cost);
        EXPECT_GE(repaired_length, length);
        delete[] route;
        delete[] repaired_route;
    }
}


//insert_station_by_remove_array
TEST_F(UtilsTest, InsertStationByRemoveArray) {
    //  0 9 7 5 2 1 29 6 8 0 10 25 3 4 11 13 0 22 19 21 20 17 0 12 15 18 14 16 0
    // {0,9,7,5,2,1,29,6,8,0}
    // {0,10,25,3,4,11,13,0}
    // {0,22,19,21,20,17,0}
    // {0,12,15,18,14,16,0}
    vector<vector<int>> routes = {
            {0, 9, 7, 5, 2, 1, 6, 8, 0},
            {0, 10, 3, 4, 11, 13, 0},
            {0, 19, 21, 20, 17, 0},
            {0, 12, 15, 18, 14, 16, 0}
    };

    for (auto& r : routes) {
        int *route = new int[instance->num_customer_ + instance->num_depot_];
        memcpy(route, r.data(), r.size() * sizeof(int));
        int length = static_cast<int>(r.size());
        double original_cost = instance->compute_total_distance(route, length);

        // print the original route
//        cout << "Original Route: ";
//        for (int i = 0; i < length; ++i) {
//            cout << route[i] << " ";
//        }
//        cout << endl;
//        cout << "Original Length: " << length << endl;
//        cout << "Original Cost: " << original_cost << endl;

        int *repaired_route = new int[instance->num_customer_ + instance->num_depot_];
        memcpy(repaired_route, route, length * sizeof(int));
        int repaired_length = length;
        double repaired_cost = insert_station_by_remove_array(route, length, repaired_route, repaired_length, *instance);

        // print the repaired route
//        cout << "Repaired Route: ";
//        for (int i = 0; i < repaired_length; ++i) {
//            cout << repaired_route[i] << " ";
//        }
//        cout << endl;
//        cout << "Repaired Length: " << repaired_length << endl;
//        cout << "Repaired Cost: " << repaired_cost << endl;
//        cout << endl;

        EXPECT_GE(repaired_cost, original_cost);
        EXPECT_GE(repaired_length, length);
        delete[] route;
        delete[] repaired_route;
    }
}

TEST_F(UtilsTest, ComparisonOperatorsWithOrWithoutSearchAccelerationForIntraOperators) {
//    vector<string> file_names = {"E-n22-k4.evrp", "X-n214-k11.evrp", "X-n1001-k43.evrp"};
//
//    std::chrono::high_resolution_clock::time_point start{};
//    std::chrono::duration<double> duration{};
//
//    int loop_base = 1'000;
//    for (const auto& file_name : file_names) {
//        cout << "File name: " << file_name << endl;
//        Case* exp_instance = new Case(1, file_name);
//
//        vector<vector<int>> total_routes;
//        for (int i = 0; i < loop_base; ++i) {
//            vector<vector<int>> routes = routes_constructor_with_split(*exp_instance, rng);
//            total_routes.insert(total_routes.end(), routes.begin(), routes.end());
//        }
//
//        vector<vector<int>> backup_total_routes_1 = total_routes;
//        vector<vector<int>> backup_total_routes_2 = total_routes;
//
//        // compare the efficiency of two operators - time used
//        start = std::chrono::high_resolution_clock::now();
//        for (auto& route: backup_total_routes_1) {
//            double cost = 0;
////            two_opt_for_single_route(route.data(), static_cast<int>(route.size()), cost, *exp_instance);
////            node_shift(route.data(), static_cast<int>(route.size()), cost, *exp_instance);
//            two_nodes_swap_for_single_route(route.data(), static_cast<int>(route.size()), cost, *exp_instance);
//        }
//        duration = std::chrono::high_resolution_clock::now() - start;
//        cout << "Time used for the original operator: " << duration.count() << " seconds" << endl;
//        start = std::chrono::high_resolution_clock::now();
//        for (auto& route: backup_total_routes_2) {
//            double cost = 0;
////            two_opt_for_single_route_acceleration(route.data(), static_cast<int>(route.size()), cost, *exp_instance);
////            node_shift_acceleration(route.data(), static_cast<int>(route.size()), cost, *exp_instance);
//            two_nodes_swap_for_single_route_acceleration(route.data(), static_cast<int>(route.size()), cost, *exp_instance);
//        }
//        duration = std::chrono::high_resolution_clock::now() - start;
//        cout << "Time used for the accelerated operator: " << duration.count() << " seconds" << endl;
//
//        // compare the effectiveness of two operators - fitness value
//        int num_cost_accele_smaller_than_original = 0;
//        int num_total_routes = static_cast<int>(total_routes.size());
//        for(auto& route : total_routes) {
//            vector<int> route_copy = route;
//
//            double cost_original = exp_instance->compute_total_distance(route);
////            two_opt_for_single_route(route.data(), static_cast<int>(route.size()), cost_original, *exp_instance);
////            node_shift(route.data(), static_cast<int>(route.size()), cost_original, *exp_instance);
//            two_nodes_swap_for_single_route(route.data(), static_cast<int>(route.size()), cost_original, *exp_instance);
//
//            double cost_accelerated = exp_instance->compute_total_distance(route_copy);
////            two_opt_for_single_route_acceleration(route_copy.data(), static_cast<int>(route_copy.size()), cost_accelerated, *exp_instance);
////            node_shift_acceleration(route_copy.data(), static_cast<int>(route_copy.size()), cost_accelerated, *exp_instance);
//            two_nodes_swap_for_single_route_acceleration(route_copy.data(), static_cast<int>(route_copy.size()), cost_accelerated, *exp_instance);
//
//            if (cost_accelerated < cost_original || cost_accelerated == cost_original) num_cost_accele_smaller_than_original++;
//
//            if (cost_accelerated - exp_instance->compute_total_distance(route_copy) > 0.000'000'001) {
//                std::cerr << "Error!" << std::endl;
//            }
//        }
//        cout << "Total number of routes: " << num_total_routes << endl;
//        cout << "Number of routes that the accelerated operator performs better than original one: " << num_cost_accele_smaller_than_original << endl;
//        cout << "Ratio of accelerated operator performs better than or equal to original one: " << (double)num_cost_accele_smaller_than_original / num_total_routes << endl;
//        cout << endl;
//
//        delete exp_instance;
//    }

    EXPECT_TRUE(true);
}

TEST_F(UtilsTest, ComparisonOperatorsWithOrWithoutSearchAccelerationForInterOperators) {
//    vector<string> file_names = {"E-n22-k4.evrp", "X-n214-k11.evrp", "X-n1001-k43.evrp"};
//
//    std::chrono::high_resolution_clock::time_point start{};
//    std::chrono::duration<double> duration{};
//
//    for (const auto& file_name : file_names) {
//        cout << "File name: " << file_name << endl;
//        Case* exp_instance = new Case(1, file_name);
//
//        int route_cap = 3 * exp_instance->num_vehicle_;
//        int node_cap = exp_instance->num_customer_ + exp_instance->num_depot_;
//        vector<shared_ptr<Individual>> population;
//        for (int i = 0; i < 100; ++i) {
//            vector<vector<int>> routes = routes_constructor_with_split(*exp_instance, rng);
//            population.push_back(std::make_shared<Individual>(route_cap, node_cap, routes,
//                                                              exp_instance->compute_total_distance(routes),
//                                                              exp_instance->compute_demand_sum_per_route(routes)));
//        }
//
//        vector<shared_ptr<Individual>> backup_population_1 = population;
//        vector<shared_ptr<Individual>> backup_population_2 = population;
//
//        // compare the efficiency of two operators - time used
//        start = std::chrono::high_resolution_clock::now();
//        for (auto& ind : backup_population_1) {
////            two_opt_move_inter_route_for_individual(*ind, *exp_instance);
////            one_point_move_inter_route_for_individual(*ind, *exp_instance);
//            two_point_move_inter_route_for_individual(*ind, *exp_instance);
//        }
//        duration = std::chrono::high_resolution_clock::now() - start;
//        cout << "Time used for the original operator: " << duration.count() << " seconds" << endl;
//        start = std::chrono::high_resolution_clock::now();
//        for (auto& ind: backup_population_2) {
////            two_opt_move_inter_route_for_individual_acceleration(*ind, *exp_instance);
////            one_point_move_inter_route_for_individual_acceleration(*ind, *exp_instance);
//            two_point_move_inter_route_for_individual_acceleration(*ind, *exp_instance);
//        }
//        duration = std::chrono::high_resolution_clock::now() - start;
//        cout << "Time used for the accelerated operator: " << duration.count() << " seconds" << endl;
//
//
//        // compare the effectiveness of two operators - fitness value
//        int num_cost_accele_smaller_than_original = 0;
//        for(auto& ind : population) {
//            shared_ptr<Individual> ind_copy = make_shared<Individual>(*ind);
//
////            two_opt_move_inter_route_for_individual(*ind, *exp_instance);
////            one_point_move_inter_route_for_individual(*ind, *exp_instance);
//            two_point_move_inter_route_for_individual(*ind, *exp_instance);
//
////            two_opt_move_inter_route_for_individual_acceleration(*ind_copy, *exp_instance);
////            one_point_move_inter_route_for_individual_acceleration(*ind_copy, *exp_instance);
//            two_point_move_inter_route_for_individual_acceleration(*ind_copy, *exp_instance);
//
//
//            if (ind_copy->upper_cost < ind->upper_cost || ind_copy->upper_cost == ind->upper_cost) num_cost_accele_smaller_than_original++;
//
//            double cost_validation = exp_instance->compute_total_distance(ind_copy->routes, ind_copy->num_routes, ind_copy->num_nodes_per_route);
//            if (ind_copy->upper_cost - cost_validation > 0.000'001) {
//                std::cerr << "Error!" << std::endl;
//            }
//        }
//        cout << "Total number of routes: " << 100 << endl;
//        cout << "Number of routes that the accelerated operator performs better than original one: " << num_cost_accele_smaller_than_original << endl;
//        cout << "Ratio of accelerated operator performs better than or equal to original one: " << (double)num_cost_accele_smaller_than_original / 100 << endl;
//
//        cout << endl;
//
//        delete exp_instance;
//    }

    EXPECT_TRUE(true);
}

TEST_F(UtilsTest, NeighborExpanding_OnePointIntraRouteForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);


    vector<std::unique_ptr<Individual>> neighbors = one_point_intra_route_for_individual(*ind, *instance, ind->upper_cost * 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 0);
}

TEST_F(UtilsTest, NeighborExpanding_OnePointInterRouteForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);


    vector<std::unique_ptr<Individual>> neighbors = one_point_inter_route_for_individual(*ind, *instance, ind->upper_cost * 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 0);
}

TEST_F(UtilsTest, NeighborExpanding_OnePointMoveNeighbors) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);

    vector<std::unique_ptr<Individual>> neighbors = one_point_move_neighbors(*ind, *instance, ind->upper_cost, 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 0);
}

TEST_F(UtilsTest, NeighborExpanding_TwoPointIntraRouteForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);


    vector<std::unique_ptr<Individual>> neighbors = two_point_intra_route_for_individual(*ind, *instance, ind->upper_cost * 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 0);
}

TEST_F(UtilsTest, NeighborExpanding_TwoPointInterRouteForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);


    vector<std::unique_ptr<Individual>> neighbors = two_point_inter_route_for_individual(*ind, *instance, ind->upper_cost * 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 0);
}

TEST_F(UtilsTest, NeighborExpanding_TwoPointMoveNeighbors) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);

    vector<std::unique_ptr<Individual>> neighbors = two_point_move_neighbors(*ind, *instance, ind->upper_cost, 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 0);
}

TEST_F(UtilsTest, NeighborExpanding_TwoOptIntraRouteForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);

    vector<std::unique_ptr<Individual>> neighbors = two_opt_intra_route_for_individual(*ind, *instance, ind->upper_cost * 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 0);
}

TEST_F(UtilsTest, NeighborExpanding_TwoOptInterRouteForIndividual) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);

    vector<std::unique_ptr<Individual>> neighbors = two_opt_inter_route_for_individual(*ind, *instance, ind->upper_cost * 1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 1);
}

TEST_F(UtilsTest, NeighborExpanding_TwoOptMoveNeighbors) {
    vector<vector<int>> routes = {
            {0, 2, 1, 5, 7, 6, 9, 0},
            {0, 8, 3, 4, 11, 10, 13, 0},
            {0, 21, 19, 17, 20, 0},
            {0, 14, 16, 12, 15, 18, 0}
    };
    vector<int> demand_sum_per_route = {5600, 5400, 6000, 5500};
    std::unique_ptr<Individual> ind = std::make_unique<Individual>(8, 22, routes, instance->compute_total_distance(routes), demand_sum_per_route);

    vector<std::unique_ptr<Individual>> neighbors = two_opt_move_neighbors(*ind, *instance, ind->upper_cost,  1.1);

    for (auto& ind_ptr:neighbors) {
        double cost_calculated = instance->compute_total_distance(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route);
        int* total_demand_pre_route = new int[ind_ptr->num_routes];
        instance->compute_demand_sum_per_route(ind_ptr->routes, ind_ptr->num_routes, ind_ptr->num_nodes_per_route, total_demand_pre_route);
        for (int i = 0; i < ind_ptr->num_routes; ++i) {
            EXPECT_EQ(total_demand_pre_route[i], ind_ptr->demand_sum_per_route[i]);
        }

        EXPECT_DOUBLE_EQ(ind_ptr->upper_cost, cost_calculated);

        delete[] total_demand_pre_route;
    }

    EXPECT_GT(neighbors.size(), 1);
}


