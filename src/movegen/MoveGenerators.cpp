/**
 * Move generators container implementation.
 */

#include <cobra/MoveGenerators.hpp>
#include <map>
#include <cobra/VertexSet.hpp>

namespace cobra {

    MoveGenerators::MoveGenerators(const cobra::Instance& instance_, std::vector<AbstractMoveGeneratorsView*>& views_) :
    instance(instance_), views(views_) {

        update_bits.resize(instance.get_vertices_num(), 2); // this data structure will be used during the SMD update stage for asymmetric neighborhoods.
        // In particular, for each affected vertex i, we will update move generators
        // - (i, j) when update_bits[i][0] == true
        // - (j, i) when update_bits[i][1] == true

        struct pair_hash {
            auto operator()(const std::pair<int, int> &p) const -> size_t {
                const auto prime = 31;
                auto result = 1;
                result = prime * result + p.first;
                result = prime * result + p.second;
                return std::hash<int>()(result);
            }
        };

        // map move generator -> unique index
        auto unique_moves = std::unordered_map<std::pair<int, int>, int, pair_hash>();

        // identify the set of unique move generators across all views
        for (auto &view : views) {

            const auto &generator = view->get_generator();

            for (auto i = instance.get_vertices_begin(); i < instance.get_vertices_end(); i++) {

                const auto &endpoints = generator(i);

                auto move_indices_in_moves = std::vector<int>();
                move_indices_in_moves.reserve(endpoints.size());

                for (auto j : endpoints) {

                    auto a = i;
                    auto b = j;
                    if(b < a) { std::swap(a, b); }

                    if (unique_moves.count({a, b})) {

                        const auto move_index = unique_moves[{a, b}];

                        move_indices_in_moves.emplace_back(move_index);

                    } else {

                        const auto move_index = moves.size();
                        move_indices_in_moves.emplace_back(move_index);

                        moves.emplace_back(a, b);
                        moves.emplace_back(b, a);

                        unique_moves[{a, b}] = move_index;

                    }


                }

                view->callback_notify_move_indices(i, move_indices_in_moves, *this);

            }

            view->callback_notify_build_complete(*this);


        }

        // note that the heap MUST be initialized once all the move generators have been placed into the `moves` vector
        this->heap = new MoveGeneratorsHeap(*this);

        this->move_generator_indices_involving.resize(instance.get_vertices_num());

        this->prev_percentage.resize(instance.get_vertices_num(), 0.0f);

    }

    MoveGenerators::~MoveGenerators() {
        delete this->heap;
    }

    void MoveGenerators::set_active_percentage(std::vector<float>& percentage, std::vector<int>& vertices) {

        // Update active move generators of each view associated with the list of vertices in input

        // First identify all vertices that needs an update
        for(auto n = 0u; n < vertices.size();) {
            const auto vertex = vertices[n];
            if (std::fabs(percentage[vertex] - prev_percentage[vertex]) < 0.01f) {
                // Remove a vertex if it does not need to be updated
                std::swap(vertices[n], vertices[vertices.size()-1]);
                vertices.pop_back();
            } else {
                n++;
            }
        }

        if(vertices.empty()) { return; }

        // For each view, set or unset the move generators involving each 'vertex' according to percentage[vertex]
        // Collect all vertices involving moves that are set or unset
        auto vertices_in_updated_moves = cobra::VertexSet(instance.get_vertices_num());
        for(auto& view : views) {
            view->set_active_percentage(percentage, vertices, *this, vertices_in_updated_moves);
        }

        // The previous view->set_active_percentage(...) for vertex 'i' may cause
        // - direct updates: we are adding or removing move generators (i, j) to the list of move generators involving 'i'
        // - indirect updates: we are adding move generators (i, j) to the list of move generators involving 'j' because
        //   of the direct updates (addition) or we are setting as not active move generators (i, j) that were in the list
        //   of move generators involving 'j' (removal). For the latter, when (i, j) was only active due to 'i' and should now
        //   be removed from the list of move generators involving 'j' because no longer active in 'j', we do not have an efficient
        //   way to remove it from there. A possibility would be to store in each view a matrix of move generator positions.
        //   A simpler way is to check and not add move generators that are not active in both vertices.
        //   Note that this latter approach causes the views to always stay in possibly inconsistent states. One can
        //   access the correct list of move generators associated to a given vertex 'i' by accessing the filtered list in
        //   the 'MoveGenerators' class.
        // Here in the following we update those lists.

        auto unique_move_generators = std::vector<int>();
        auto unique_endpoints = cobra::VertexSet(instance.get_vertices_num());
        for(const auto vertex : vertices_in_updated_moves.get_vertices()) {
            //unique_move_generator_set.clear();
            unique_move_generators.clear();
            unique_endpoints.clear();
            for(auto& view : views) {
                for(auto move_idx : view->get_move_generator_indices_involving(vertex)) {
                    if(view->is_active(move_idx)) {
                        const auto& move = moves[move_idx];
                        int other_vertex;
                        if(vertex == move.get_first_vertex()) {
                            other_vertex = move.get_second_vertex();
                        } else {
                            other_vertex = move.get_first_vertex();
                        }
                        if(!unique_endpoints.contains(other_vertex)) {
                            unique_endpoints.insert_without_checking_existance(other_vertex);
                            unique_move_generators.push_back(move_idx);
                        }
                        //unique_move_generator_set.insert(move_idx);
                    }
                }
            }

            move_generator_indices_involving[vertex] = unique_move_generators;


        }

        // Finally, update the stored percentages
        for(const auto vertex : vertices) {
            prev_percentage[vertex] = percentage[vertex];
        }

    }


}