/**
 * Move generator views implementation.
 */

#include <cobra/MoveGenerators.hpp>
#include <utility>
#include <cmath>

namespace cobra {

    AbstractMoveGeneratorsView::AbstractMoveGeneratorsView(const cobra::Instance &instance_, std::function<std::vector<int>(int)> generator_) :
                                                           instance(instance_),
                                                           generator(std::move(generator_)) {

        move_generator_indices_involving.resize(instance.get_vertices_num());
        all_move_generator_indices_involving.resize(instance.get_vertices_num());

    }

    auto AbstractMoveGeneratorsView::get_generator() -> std::function<std::vector<int>(int)> & {
        return generator;
    }

    auto AbstractMoveGeneratorsView::get_move_generator_indices_involving(int vertex) -> std::vector<int> & {
        return move_generator_indices_involving[vertex];
    }

    auto AbstractMoveGeneratorsView::setup_active_trackers() -> void {

        // Each view must keep track of the vertices in which each move generator is active
        for(auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {
            for(const auto idx : all_move_generator_indices_involving[i]) {
                if(move_map.count(idx)) { continue; }
                move_map.insert({idx, active_in.size()});
                active_in.emplace_back(false, false); // at the beginning the move gen is not active in any vertex
            }
        }

    }

    KNeighborsMoveGeneratorsView::KNeighborsMoveGeneratorsView(const cobra::Instance& instance, int k):
    AbstractMoveGeneratorsView(instance, [&instance, k](auto i)-> std::vector<int> {

        auto endpoints = std::vector<int>();

        const auto max_neighbors = std::min(k, instance.get_vertices_num() - 1);

        for(auto n = 1, added = 0; added < max_neighbors; n++) {

            auto j = instance.get_neighbors_of(i)[n];

            endpoints.emplace_back(j);
            added++;
        }

        return endpoints;

    }), max_neighbors_num(std::min(k, instance.get_vertices_num() - 1)) // skip self-move
    {

        curr_neighbors_num.resize(instance.get_vertices_num(), 0);

    }

    void KNeighborsMoveGeneratorsView::callback_notify_move_indices([[maybe_unused]]int vertex, std::vector<int>& indices, MoveGenerators &moves) {

        for(const auto index : indices) {

            const auto& move = moves.get(index);

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            all_move_generator_indices_involving[i].emplace_back(index);
            all_move_generator_indices_involving[j].emplace_back(index);

        }

    }

    void KNeighborsMoveGeneratorsView::callback_notify_build_complete(MoveGenerators &moves) {

        for(auto i = instance.get_customers_begin(); i < instance.get_customers_end(); i++) {

            auto& indices =  all_move_generator_indices_involving[i];

            auto set = std::unordered_set<int>();

            set.insert(indices.begin(), indices.end());

            indices.clear();
            indices.insert(indices.begin(), set.begin(), set.end());
            indices.shrink_to_fit();

            std::sort(indices.begin(), indices.end(),
                      [&moves, this](auto i, auto j){
                          const auto& i_move = moves.get(i);
                          const auto i_cost = this->instance.get_cost(i_move.get_first_vertex(), i_move.get_second_vertex());

                          const auto& j_move = moves.get(j);
                          const auto j_cost = this->instance.get_cost(j_move.get_first_vertex(), j_move.get_second_vertex());

                          return i_cost < j_cost;}
                );


        }

        setup_active_trackers();


    }

    void KNeighborsMoveGeneratorsView::set_active_percentage(std::vector<float>& percentage, std::vector<int>& vertices, MoveGenerators& moves, cobra::VertexSet& vertices_in_updated_moves) {

        auto vertices_to_update = std::vector<int>();

        // First, for each vertex identify what are the move generators to set as active or not active
        for(const auto vertex : vertices) {

            const auto new_num = static_cast<int>(std::round(percentage[vertex] * static_cast<float>(max_neighbors_num)));

            if(new_num == curr_neighbors_num[vertex]) { continue; }

            vertices_to_update.push_back(vertex);

            // Set as active or not active the move generators associated to vertex according to 'new_num'
            // Keep track of the moves involved in the update by storing the vertices in 'vertices_in_updated_moves'
            // This set will be later used by the 'MoveGenerators' class to perform a selective update of the necessary
            // data structures

            if(new_num < curr_neighbors_num[vertex]) {

                // removal
                for(auto n = new_num; n < curr_neighbors_num[vertex]; n++) {
                    const auto move_idx = all_move_generator_indices_involving[vertex][n];
                    const auto& move = moves.get(move_idx);
                    this->set_not_active_in(move, move_idx, vertex);
                    vertices_in_updated_moves.insert(move.get_first_vertex());
                    vertices_in_updated_moves.insert(move.get_second_vertex());
                }

            } else {

                // addition
                for(auto n = curr_neighbors_num[vertex]; n < new_num; n++) {
                    const auto move_idx = all_move_generator_indices_involving[vertex][n];
                    const auto& move = moves.get(move_idx);
                    this->set_active_in(move, move_idx, vertex);
                    vertices_in_updated_moves.insert(move.get_first_vertex());
                    vertices_in_updated_moves.insert(move.get_second_vertex());
                }

            }

            curr_neighbors_num[vertex] = new_num;

        }


        for(const auto vertex : vertices_to_update) {

            move_generator_indices_involving[vertex].clear();

            auto n = 0;

            for (; n < curr_neighbors_num[vertex]; n++) {

                const auto idx = all_move_generator_indices_involving[vertex][n];
                const auto &move = moves.get(idx);

                const auto i = move.get_first_vertex();
                const auto j = move.get_second_vertex();

                if (vertex==i) {

                    move_generator_indices_involving[i].emplace_back(idx);

                    if (!is_active_in_other(move, idx, vertex)) { // if it were not already active in j => move_generator_indices_involving[j] does not have it
                        move_generator_indices_involving[j].emplace_back(idx);
                    }

                } else {

                    move_generator_indices_involving[j].emplace_back(idx);

                    if (!is_active_in_other(move,idx, vertex)) { // if it were not already active in i => move_generator_indices_involving[i] does not have it
                        move_generator_indices_involving[i].emplace_back(idx);
                    }

                }

            }

            for (; n < static_cast<int>(all_move_generator_indices_involving[vertex].size()); n++) {

                const auto idx = all_move_generator_indices_involving[vertex][n];
                const auto &move = moves.get(idx);

                // add all move generators that are active in the other vertex
                if (is_active_in_other(move, idx, vertex)) {
                    move_generator_indices_involving[vertex].emplace_back(idx);
                }

            }

        }

    }

    CostBasedMoveGeneratorsView::CostBasedMoveGeneratorsView(const cobra::Instance &instance, std::function<std::vector<int>(int)> generator):
    AbstractMoveGeneratorsView(instance, std::move(generator)) {

        curr_percentage.resize(instance.get_vertices_num(), 0.0f);
        curr_num.resize(instance.get_vertices_num(), 0);
        inclusion_percentage.resize(instance.get_vertices_num());

    }

    void CostBasedMoveGeneratorsView::callback_notify_move_indices([[maybe_unused]] int vertex, std::vector<int>& indices, [[maybe_unused]]MoveGenerators &moves) {
        // store all move generators into a flat array
        flat_indices.insert(flat_indices.end(), indices.begin(), indices.end());
    }

    void CostBasedMoveGeneratorsView::callback_notify_build_complete(MoveGenerators &moves) {

        std::sort(flat_indices.begin(), flat_indices.end(), [&moves, this](auto i, auto j){

                const auto& i_move = moves.get(i);
                const auto i_cost = this->instance.get_cost(i_move.get_first_vertex(), i_move.get_second_vertex());

                const auto& j_move = moves.get(j);
                const auto j_cost = this->instance.get_cost(j_move.get_first_vertex(), j_move.get_second_vertex());

                return i_cost < j_cost;
        });

        for(auto n = 0u; n < flat_indices.size(); n++) {

            const auto move_index = flat_indices[n];

            const auto& move = moves.get(move_index);

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto pi = static_cast<float>(n+1) / static_cast<float>(flat_indices.size());

            all_move_generator_indices_involving[i].emplace_back(move_index);
            all_move_generator_indices_involving[j].emplace_back(move_index);

            inclusion_percentage[i].emplace_back(pi);   // note that pi is already increasing since we ordered the moves before
            inclusion_percentage[j].emplace_back(pi);   // note that pi is already increasing since we ordered the moves before

        }

        // we can now get rid of move indices...
        flat_indices.clear();
        flat_indices.shrink_to_fit();

        setup_active_trackers();

    }

    void CostBasedMoveGeneratorsView::set_active_percentage(std::vector<float>& percentage, std::vector<int>& vertices, MoveGenerators& moves, cobra::VertexSet& vertices_in_updated_moves) {

        auto vertices_to_update = std::vector<int>();

        // First, for each vertex identify what are the move generators to set as active or not active
        for(const auto vertex : vertices) {

            if(std::fabs(curr_percentage[vertex] - percentage[vertex]) < 0.01f) { continue; }

            vertices_to_update.push_back(vertex);

            auto n = curr_num[vertex]; // current number of move generators in use from vertex: 0 -> n - 1

            if(percentage[vertex] < curr_percentage[vertex]) {

                // removal
                for(n = n-1; n >= 0 && inclusion_percentage[vertex][n] > percentage[vertex]; n--) {
                    const auto move_idx = all_move_generator_indices_involving[vertex][n];
                    const auto& move = moves.get(move_idx);
                    this->set_not_active_in(move, move_idx, vertex);
                    vertices_in_updated_moves.insert(move.get_first_vertex());
                    vertices_in_updated_moves.insert(move.get_second_vertex());
                }
                n = n+1; // we were using n as an index thus the number of elements is n+1

            } else {

                // addition
                for(; n < static_cast<int>(all_move_generator_indices_involving[vertex].size()) && inclusion_percentage[vertex][n] <= percentage[vertex]; n++) {
                    const auto move_idx = all_move_generator_indices_involving[vertex][n];
                    const auto& move = moves.get(move_idx);
                    this->set_active_in(move, move_idx, vertex);
                    vertices_in_updated_moves.insert(move.get_first_vertex());
                    vertices_in_updated_moves.insert(move.get_second_vertex());
                }

            }

            curr_percentage[vertex] = percentage[vertex];
            curr_num[vertex] = n;

        }

        for(const auto vertex : vertices_to_update) {

            move_generator_indices_involving[vertex].clear();

            auto n = 0;
            for (; n < curr_num[vertex]; n++) {

                const auto idx = all_move_generator_indices_involving[vertex][n];
                const auto &move = moves.get(idx);

                const auto i = move.get_first_vertex();
                const auto j = move.get_second_vertex();

                if (vertex==i) {

                    move_generator_indices_involving[i].emplace_back(idx);

                    if (!is_active_in_other(move, idx, vertex)) { // if it were not already active in j => move_generator_indices_involving[j] does not have it
                        move_generator_indices_involving[j].emplace_back(idx);
                    }

                } else {

                    move_generator_indices_involving[j].emplace_back(idx);

                    if (!is_active_in_other(move,idx, vertex)) { // if it were not already active in i => move_generator_indices_involving[i] does not have it
                        move_generator_indices_involving[i].emplace_back(idx);
                    }

                }

            }

            for (; n < static_cast<int>(all_move_generator_indices_involving[vertex].size()); n++) {

                const auto idx = all_move_generator_indices_involving[vertex][n];
                const auto &move = moves.get(idx);

                // add all move generators that are active in the other vertex
                if (is_active_in_other(move, idx, vertex)) {
                    move_generator_indices_involving[vertex].emplace_back(idx);
                }

            }

        }

    }

}


