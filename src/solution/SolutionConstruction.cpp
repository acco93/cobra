/**
 * Simple savings implementation.
 */

#include <cassert>
#include <algorithm>

#include <cobra/Instance.hpp>
#include <cobra/Solution.hpp>

void cobra::Solution::clarke_and_wright(const cobra::Instance& instance, cobra::Solution& solution, float lambda, int neighbors_num) {

    solution.reset();

    for (auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {
        solution.build_one_customer_route(i);
    }

    neighbors_num = std::min(instance.get_customers_num() - 1, neighbors_num);

    const auto savings_num = instance.get_customers_num() * neighbors_num;

    struct Saving {
        int i;
        int j;
        float value;
    };

    auto savings = std::vector<Saving>();
    savings.reserve(static_cast<unsigned long>(savings_num));

    for(auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {

        for(auto n = 1u, added=0u; added < static_cast<unsigned int>(neighbors_num) && n < instance.get_neighbors_of(i).size(); n++) {

            const auto j = instance.get_neighbors_of(i)[n];

            if (i < j) {

                const auto value = +instance.get_cost(i, instance.get_depot()) + instance.get_cost(instance.get_depot(), j) - lambda*instance.get_cost(i, j);

                savings.push_back({i, j, value});

                added++;

            }
        }
    }


    std::sort(savings.begin(), savings.end(), [](const Saving &a, const Saving &b) { return a.value > b.value; });

    for (auto &saving : savings) {

        const auto i = saving.i;
        const auto j = saving.j;

        const auto iRoute = solution.get_route_index(i);
        const auto jRoute = solution.get_route_index(j);

        if (iRoute == jRoute) { continue; }

        if (solution.get_last_customer(iRoute) == i && solution.get_first_customer(jRoute) == j &&
            solution.get_route_load(iRoute) + solution.get_route_load(jRoute) <= instance.get_vehicle_capacity()) {

            solution.append_route(iRoute, jRoute);


        } else if (solution.get_last_customer(jRoute) == j && solution.get_first_customer(iRoute) == i &&
                   solution.get_route_load(iRoute) + solution.get_route_load(jRoute) <= instance.get_vehicle_capacity()) {

            solution.append_route(jRoute, iRoute);

        }

    }

    assert(solution.is_feasible());

}

