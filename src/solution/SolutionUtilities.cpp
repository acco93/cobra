/**
 * Solution utilities.
 */

#include <cobra/Solution.hpp>

namespace cobra {

    void Solution::store_to_file(const cobra::Instance& instance, const cobra::Solution& solution, const std::string& path) {

        auto out_stream = std::ofstream(path);

        for(auto route = solution.get_first_route(), idx = 1; route != cobra::Solution::dummy_route; route = solution.get_next_route(route), idx++) {
            out_stream << "Route #" << idx << ":";
            for(auto customer = solution.get_first_customer(route); customer != instance.get_depot(); customer = solution.get_next_vertex(customer)) {
                out_stream << " " << std::to_string(customer);
            }
            out_stream << "\n";
        }

        out_stream << "Cost " << std::to_string(solution.get_cost());

    }

}