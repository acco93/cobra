/**
 * Static Move Descriptor-based Local search operators. AbstractOperator is the base abstract class whose procedures
 * are implemented in the specific operator sub-classes.
 */

#ifndef COBRA_LOCALSEARCH_HPP
#define COBRA_LOCALSEARCH_HPP

#include "Instance.hpp"
#include "MoveGenerators.hpp"
#include "Solution.hpp"
#include "BitMatrix.hpp"
#include "VertexSet.hpp"
#include <set>
#include <map>
#include <cfloat>

                                // For the (restricted) update stage of asymmetric moves
#define UPDATE_BITS_FIRST (0)   // when i is an affected vertex, update all move generators (a, b) where a == i; i.e., i is the first vertex
#define UPDATE_BITS_SECOND (1)  // when i is an affected vertex, update all move generators (a, b) where b == i; i.e., i is the second vertex

namespace cobra {

    template <bool handle_partial_solutions=false>
    class AbstractOperator : private NonCopyable<AbstractOperator<handle_partial_solutions>>{

    public:

        AbstractOperator(const cobra::Instance& instance_, MoveGenerators &moves_, float tolerance_, bool is_symmetric_) :
                instance(instance_), moves(moves_), heap(moves.get_heap()), tolerance(tolerance_),
                timegen(moves_.get_timestamp_generator()), update_bits(moves_.get_update_bits()), affected_vertices(instance_.get_vertices_num()) {

            if (is_symmetric_) {
                descriptors_initialization = &AbstractOperator::initialize_symmetric_descriptors;
                descriptors_update = &AbstractOperator::symmetric_update;
            } else {
                descriptors_initialization = &AbstractOperator::initialize_asymmetric_descriptors;
                descriptors_update = &AbstractOperator::asymmetric_update;
            }

        }

        virtual ~AbstractOperator() = default;

        bool apply_rough_best_improvement(cobra::Solution &solution) {

            auto& moves_vector = moves.get_raw_vector();

            heap.reset();

            pre_processing(solution);

            (this->*descriptors_initialization)(solution);

            auto improved = false;

            auto index = 0;

            while(index < heap.size()){

                auto& move = moves_vector[heap.spy(index)];

                index++;

                if constexpr(handle_partial_solutions) {
                    if(!solution.is_vertex_in_solution(move.get_first_vertex()) || !solution.is_vertex_in_solution(move.get_second_vertex())) {
                        continue;
                    }
                }

                if(!is_feasible(solution, move)) { continue; }

                execute(solution, move, affected_vertices);

                improved = true;

                index = 0;

                (this->*descriptors_update)(solution);

                affected_vertices.clear();

            }

            return improved;

        }

     protected:

        const cobra::Instance& instance;
        MoveGenerators& moves;
        MoveGeneratorsHeap& heap;

        virtual void pre_processing(cobra::Solution& solution) = 0;
        virtual float compute_cost(const cobra::Solution& solution, const MoveGenerator& move) = 0;
        virtual bool is_feasible(const cobra::Solution& solution, const MoveGenerator& move) = 0;
        virtual void execute(cobra::Solution& solution, const MoveGenerator& move, cobra::VertexSet& storage) = 0;

        const float tolerance;

        Flat2DVector<bool>& update_bits; // used to reduce move generators updates for asymmetric neighborhoods

     private:

        cobra::VertexSet affected_vertices;
        TimestampGenerator& timegen;

        typedef void (AbstractOperator::*initialization_function)(const cobra::Solution& solution);
        initialization_function descriptors_initialization = nullptr;

        void initialize_symmetric_descriptors(const cobra::Solution &solution) {
            initialize_descriptors(solution, 2);
        }

        void initialize_asymmetric_descriptors(const cobra::Solution &solution) {
            initialize_descriptors(solution, 1);
        }

        typedef void (AbstractOperator::*update_function)(const cobra::Solution& solution);
        update_function descriptors_update = nullptr;

        void initialize_descriptors(const cobra::Solution &solution, const int increment) {

            const auto& vertices = solution.get_cache();

            const auto timestamp = timegen.get() + 1;

            for(auto i = vertices.begin(); i != cobra::LRUCache::Entry::dummy_vertex; i = vertices.get_next(i)) {

                const auto& i_moves = moves.get_move_generator_indices_involving(i);

                for (const auto move_index : i_moves) {

                    auto &move = moves.get(move_index);

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(move.get_first_vertex()) ||
                            !solution.is_vertex_in_solution(move.get_second_vertex())) {
                            continue;
                        }
                    }

                    if (move.get_timestamp() == timestamp) {
                        continue;
                    }

                    move.set_delta(compute_cost(solution, move));
                    move.set_heap_index(MoveGeneratorsHeap::unheaped);
                    if (move.get_delta() < -tolerance) {
                        heap.insert(move_index);
                    }

                    move.set_timestamp(timestamp);

                }

                if (increment == 1) {

                    for (const auto upper_move_index : i_moves) {
                        const auto move_index = upper_move_index + 1;

                        auto &move = moves.get(move_index);

                        if constexpr (handle_partial_solutions) {
                            if (!solution.is_vertex_in_solution(move.get_first_vertex()) ||
                                !solution.is_vertex_in_solution(move.get_second_vertex())) {
                                continue;
                            }
                        }

                        if (move.get_timestamp() == timestamp) {
                            continue;
                        }

                        move.set_delta(compute_cost(solution, move));
                        move.set_heap_index(MoveGeneratorsHeap::unheaped);
                        if (move.get_delta() < -tolerance) {
                            heap.insert(move_index);
                        }

                        move.set_timestamp(timestamp);

                    }
                }

            }

            timegen.increment();


        }

        void symmetric_update(const cobra::Solution &solution) {

            const auto timestamp = timegen.get() + 1;

            auto &moves_vector = moves.get_raw_vector();

            for (const auto i : affected_vertices.get_vertices()) {

                for (const auto move_index : moves.get_move_generator_indices_involving(i)) {

                    auto &move = moves_vector[move_index];

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(move.get_first_vertex()) ||
                            !solution.is_vertex_in_solution(move.get_second_vertex())) {
                            continue;
                        }
                    }

                    if (move.get_timestamp() == timestamp) {
                        continue;
                    }

                    const auto delta = compute_cost(solution, move);

                    if (delta > -tolerance) {

                        if (move.get_heap_index() != MoveGeneratorsHeap::unheaped) {
                            heap.remove(move_index);
                        }

                        move.set_delta(delta);

                    } else {

                        if (move.get_heap_index() == MoveGeneratorsHeap::unheaped) {
                            move.set_delta(delta);
                            heap.insert(move_index);
                        } else {
                            heap.change_value(move_index, delta);
                        }

                    }

                    move.set_timestamp(timestamp);

                }



            }
            timegen.increment();

        }

        void asymmetric_update(const cobra::Solution &solution) {

            const auto timestamp = timegen.get() + 1;


            auto &moves_vector = moves.get_raw_vector();

            for (const auto i : affected_vertices.get_vertices()) {

                for (const auto move_index : moves.get_move_generator_indices_involving(i)) {

                    auto &move = moves_vector[move_index];

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(move.get_first_vertex()) ||
                            !solution.is_vertex_in_solution(move.get_second_vertex())) {
                            continue;
                        }
                    }

                    if (move.get_timestamp() == timestamp) {
                        continue;
                    }

                    if(!update_bits.at(i, UPDATE_BITS_FIRST) && move.get_first_vertex() == i) { continue; }
                    if(!update_bits.at(i, UPDATE_BITS_SECOND) && move.get_second_vertex() == i) { continue; }

                    const auto delta = compute_cost(solution, move);

                    if (delta > -tolerance) {

                        if (move.get_heap_index() != MoveGeneratorsHeap::unheaped) {
                            heap.remove(move_index);
                        }

                        move.set_delta(delta);

                    } else {

                        if (move.get_heap_index() == MoveGeneratorsHeap::unheaped) {
                            move.set_delta(delta);
                            heap.insert(move_index);
                        } else {
                            heap.change_value(move_index, delta);
                        }

                    }

                    move.set_timestamp(timestamp);

                }

                for (const auto upper_move_index : moves.get_move_generator_indices_involving(i)) {

                    const auto move_index = upper_move_index + 1;

                    auto &move = moves_vector[move_index];

                    if constexpr (handle_partial_solutions) {
                        if (!solution.is_vertex_in_solution(move.get_first_vertex()) ||
                            !solution.is_vertex_in_solution(move.get_second_vertex())) {
                            continue;
                        }
                    }

                    if (move.get_timestamp() == timestamp) {
                        continue;
                    }

                    if(!update_bits.at(i, UPDATE_BITS_FIRST) && move.get_first_vertex() == i) { continue; }
                    if(!update_bits.at(i, UPDATE_BITS_SECOND) && move.get_second_vertex() == i) { continue; }

                    const auto delta = compute_cost(solution, move);

                    if (delta > -tolerance) {

                        if (move.get_heap_index() != MoveGeneratorsHeap::unheaped) {
                            heap.remove(move_index);
                        }

                        move.set_delta(delta);

                    } else {

                        if (move.get_heap_index() == MoveGeneratorsHeap::unheaped) {
                            move.set_delta(delta);
                            heap.insert(move_index);
                        } else {
                            heap.change_value(move_index, delta);
                        }

                    }

                    move.set_timestamp(timestamp);

                }

                update_bits.at(i, UPDATE_BITS_FIRST, false);
                update_bits.at(i, UPDATE_BITS_SECOND, false);

            }

            timegen.increment();

        }

    };

    //Supported local search operators
    enum Operator {
        E10, E11, E20, E21, E22, E30, E31, E32, E33, SPLIT, TAILS, TWOPT, EJCH, RE20, RE21, RE22B, RE22S, RE30, RE31,
        RE32B, RE32S, RE33B, RE33S
    };

    template<bool handle_partial_solution=false>
    class OneZeroExchange : public AbstractOperator<handle_partial_solution> {

     public:
        OneZeroExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance) :
                    AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) {

        }

     protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            return -this->instance.get_cost(iPrev, i) - this->instance.get_cost(i, iNext) +
            this->instance.get_cost(iPrev, iNext) - this->instance.get_cost(jPrev, j) +
            this->instance.get_cost(jPrev, i) + this->instance.get_cost(i, j);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            return (iRoute != jRoute &&
                    solution.get_route_load(jRoute) + this->instance.get_demand(i) <= this->instance.get_vehicle_capacity())
                    ||
                    (iRoute == jRoute && j != solution.get_next_vertex(iRoute, i));

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(jPrev);
            storage.insert(j);

            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);

            solution.remove_vertex(iRoute, i);
            solution.insert_vertex_before(jRoute, j, i);

            if (solution.is_route_empty(iRoute)) {
                solution.remove_route(iRoute);
            }

        }

    };

    template<bool handle_partial_solution=false>
    class OneOneExchange : public AbstractOperator<handle_partial_solution> {

    public:

        OneOneExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance) :
                AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) {}

    private:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        inline float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto i_route = solution.get_route_index(i, j);
            const auto j_route = solution.get_route_index(j, i);

            const auto i_prev = solution.get_prev_vertex(i_route, i);
            const auto i_next = solution.get_next_vertex(i_route, i);

            const auto j_prev = solution.get_prev_vertex(j_route, j);

            const auto j_prev_prev = solution.get_prev_vertex(j_route, j_prev);

            const auto i_rem = -this->instance.get_cost(i_prev, i) - this->instance.get_cost(i, i_next);
            const auto j_prev_rem = -this->instance.get_cost(j_prev_prev, j_prev) - this->instance.get_cost(j_prev, j);
            const auto i_add = +this->instance.get_cost(j_prev_prev, i) + this->instance.get_cost(i, j);
            const auto j_prev_add = +this->instance.get_cost(i_prev, j_prev) + this->instance.get_cost(j_prev, i_next);

            return i_add + j_prev_add + i_rem + j_prev_rem;

        }

        inline bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto i_route = solution.get_route_index(i, j);
            const auto j_route = solution.get_route_index(j, i);

            const auto j_prev = solution.get_prev_vertex(j_route, j);

            return (i_route != j_route &&
                    j_prev != this->instance.get_depot() &&
                    solution.get_route_load(i_route) - this->instance.get_demand(i) + this->instance.get_demand(j_prev) <= this->instance.get_vehicle_capacity() &&
                    solution.get_route_load(j_route) - this->instance.get_demand(j_prev) + this->instance.get_demand(i) <= this->instance.get_vehicle_capacity())
                    ||
                    (i_route == j_route && i != j_prev && j_prev != solution.get_next_vertex(i_route, i));

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jNext = solution.get_next_vertex(jRoute, j);

            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);

            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.insert_vertex_before(jRoute, j, i);

            solution.remove_vertex(jRoute, jPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrev);

        }



    };

    template<bool handle_partial_solution=false>
    class TwoZeroExchange : public AbstractOperator<handle_partial_solution> {
    public:
        TwoZeroExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) {

        }

    protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            return -this->instance.get_cost(iPrevPrev, iPrev) - this->instance.get_cost(i, iNext)
            + this->instance.get_cost(iPrevPrev, iNext)
            - this->instance.get_cost(jPrev, j)
            + this->instance.get_cost(jPrev, iPrev) + this->instance.get_cost(i, j);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);

            return (iRoute != jRoute &&
                    iPrev != this->instance.get_depot() &&
                    solution.get_route_load(jRoute) + this->instance.get_demand(i) + this->instance.get_demand(iPrev) <= this->instance.get_vehicle_capacity())
                    ||
                    (iRoute == jRoute && j != solution.get_next_vertex(iRoute, i) && iPrev != j);

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            const auto jRoute = solution.get_route_index(j, i);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jNext = solution.get_next_vertex(jRoute, j);

            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);

            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);

            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, i);
            solution.insert_vertex_before(jRoute, j, iPrev);
            solution.insert_vertex_before(jRoute, j, i);

            if (solution.is_route_empty(iRoute)) {
                solution.remove_route(iRoute);
            }

        }

    };

    template<bool handle_partial_solution=false>
    class RevTwoZeroExchange : public AbstractOperator<handle_partial_solution> {
     public:
        RevTwoZeroExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
            : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

     protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);

            return -this->instance.get_cost(iPrevPrev, iPrev)
                - this->instance.get_cost(i, iNext)
                + this->instance.get_cost(iPrevPrev, iNext)
                - this->instance.get_cost(j, jNext)
                + this->instance.get_cost(i, j) + this->instance.get_cost(iPrev, jNext);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);

            return (iRoute != jRoute &&
                iPrev != this->instance.get_depot() &&
                solution.get_route_load(jRoute) + this->instance.get_demand(i) + this->instance.get_demand(iPrev) <= this->instance.get_vehicle_capacity())
                ||
                    (iRoute == jRoute && iPrev != j && j != solution.get_prev_vertex(iRoute, iPrev));

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();


            const auto iRoute = solution.get_route_index(i, j);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            const auto jRoute = solution.get_route_index(j, i);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);

            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true); // predecessor of iPrev changes due to the reversal
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);

            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, i);
            solution.insert_vertex_before(jRoute, jNext, i);
            solution.insert_vertex_before(jRoute, jNext, iPrev);

            if (solution.is_route_empty(iRoute)) {
                solution.remove_route(iRoute);
            }

        }

    };

    template<bool handle_partial_solution=false>
    class TwoOneExchange : public AbstractOperator<handle_partial_solution> {
    public:
        TwoOneExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

    protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrev, iPrev) - this->instance.get_cost(i, iNext);
            const auto jPrevRem = -this->instance.get_cost(jPrevPrev, jPrev) - this->instance.get_cost(jPrev, j);

            const auto iSequenceAdd = +this->instance.get_cost(jPrevPrev, iPrev) + this->instance.get_cost(i, j);
            const auto jPrevAdd = +this->instance.get_cost(iPrevPrev, jPrev) + this->instance.get_cost(jPrev, iNext);

            const auto delta = iSequenceAdd + jPrevAdd + iSequenceRem + jPrevRem;

            return delta;
        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto jPrev = solution.get_prev_vertex(jRoute, j);


            return (iRoute != jRoute && iPrev != this->instance.get_depot() && jPrev != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jPrev) +
                    this->instance.get_demand(iPrev) +
                    this->instance.get_demand(i) <= this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jPrev) -
                    this->instance.get_demand(iPrev) -
                    this->instance.get_demand(i) <= this->instance.get_vehicle_capacity()) ||
                    (iRoute == jRoute && i != jPrev && solution.get_next_vertex(iRoute, i) != jPrev && iPrev != j);

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jNext = solution.get_next_vertex(jRoute, j);

            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);

            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);

            solution.insert_vertex_before(jRoute, j, iPrev);
            solution.insert_vertex_before(jRoute, j, i);

            solution.remove_vertex(jRoute, jPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrev);

        }

    };

    template<bool handle_partial_solution=false>
    class RevTwoOneExchange : public AbstractOperator<handle_partial_solution> {
     public:
        RevTwoOneExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
            : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

     protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrev, iPrev) - this->instance.get_cost(i, iNext);
            const auto jPrevRem = -this->instance.get_cost(j, jNext) - this->instance.get_cost(jNext, jNextNext);

            const auto iSequenceAdd = +this->instance.get_cost(jNextNext, iPrev) + this->instance.get_cost(i, j);
            const auto jNextAdd = +this->instance.get_cost(iPrevPrev, jNext) + this->instance.get_cost(jNext, iNext);

            const auto delta = iSequenceAdd + jNextAdd + iSequenceRem + jPrevRem;

            return delta;
        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto jNext = solution.get_next_vertex(jRoute, j);


            return (iRoute != jRoute && iPrev != this->instance.get_depot() && jNext != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jNext) +
                    this->instance.get_demand(iPrev) +
                    this->instance.get_demand(i) <= this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jNext) -
                    this->instance.get_demand(iPrev) -
                    this->instance.get_demand(i) <= this->instance.get_vehicle_capacity()) ||
                (iRoute == jRoute && j != iPrev && j != iPrevPrev && jNext != iPrevPrev);

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);

            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);
            storage.insert(jNextNextNext);

            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(jRoute, jNext);
            solution.insert_vertex_before(iRoute, iNext, jNext);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);

            solution.insert_vertex_before(jRoute, jNextNext, i);
            solution.insert_vertex_before(jRoute, jNextNext, iPrev);

        }

    };

    template<bool handle_partial_solution=false>
    class TwoTwoExchange : public AbstractOperator<handle_partial_solution> {
    public:
        TwoTwoExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

    protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrev, iPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(jPrevPrevPrev, jPrevPrev) - this->instance.get_cost(jPrev, j);

            const auto iSequenceAdd = +this->instance.get_cost(jPrevPrevPrev, iPrev) + this->instance.get_cost(i, j);
            const auto jSequenceAdd = +this->instance.get_cost(iPrevPrev, jPrevPrev) + this->instance.get_cost(jPrev, iNext);

            const auto delta = iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem;

            return delta;
        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && jPrev != this->instance.get_depot() &&
                jPrevPrev != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jPrev) -
                    this->instance.get_demand(jPrevPrev)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) <=
                    this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jPrev) +
                    this->instance.get_demand(jPrevPrev)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) <=
                    this->instance.get_vehicle_capacity()) ||
                   (iRoute == jRoute && i != jPrev && i != jPrevPrev && solution.get_next_vertex(iRoute, i) != jPrevPrev && j != iPrev);
        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(jPrevPrevPrev);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);

            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);

            solution.insert_vertex_before(jRoute, j, iPrev);
            solution.insert_vertex_before(jRoute, j, i);

            solution.remove_vertex(jRoute, jPrev);
            solution.remove_vertex(jRoute, jPrevPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrevPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrev);

        }

    };

    template< bool reverse_both_strings, bool handle_partial_solution=false>
    class RevTwoTwoExchange : public AbstractOperator<handle_partial_solution> {
     public:
        RevTwoTwoExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
            : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

     protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrev, iPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(j, jNext) - this->instance.get_cost(jNextNext, jNextNextNext);

            const auto iSequenceAdd = +this->instance.get_cost(jNextNextNext, iPrev) + this->instance.get_cost(i, j);

            float jSequenceAdd;
            if constexpr (reverse_both_strings) {
                jSequenceAdd = +this->instance.get_cost(iPrevPrev, jNextNext) + this->instance.get_cost(jNext, iNext);
            } else {
                jSequenceAdd = +this->instance.get_cost(iPrevPrev, jNext) + this->instance.get_cost(jNextNext, iNext);
            }

            const auto delta = iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem;

            return delta;
        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && jNext != this->instance.get_depot() &&
                jNextNext != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jNext) -
                    this->instance.get_demand(jNextNext)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) <=
                    this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jNext) +
                    this->instance.get_demand(jNextNext)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) <=
                    this->instance.get_vehicle_capacity()) ||
                (iRoute == jRoute && j != iPrev && jNext != iPrev && jNextNext != iPrev && jNextNext != solution.get_prev_vertex(iRoute, iPrev));

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);
            const auto iPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrev);

            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);
            const auto jNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNext);

            storage.insert(iPrevPrevPrevPrev);
            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);
            storage.insert(jNextNextNext);
            storage.insert(jNextNextNextNext);

            this->update_bits.at(iPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);

            solution.insert_vertex_before(jRoute, jNextNextNext, i);
            solution.insert_vertex_before(jRoute, jNextNextNext, iPrev);

            solution.remove_vertex(jRoute, jNext);
            solution.remove_vertex(jRoute, jNextNext);

            if constexpr (reverse_both_strings) {
                solution.insert_vertex_before(iRoute, iNext, jNextNext);
                solution.insert_vertex_before(iRoute, iNext, jNext);
            } else {
                solution.insert_vertex_before(iRoute, iNext, jNext);
                solution.insert_vertex_before(iRoute, iNext, jNextNext);
            }


        }


    };

    template<bool handle_partial_solution=false>
    class ThreeZeroExchange : public AbstractOperator<handle_partial_solution> {
    public:
        ThreeZeroExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

    protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(j, jPrev);
            const auto iSequenceAdd = +this->instance.get_cost(jPrev, iPrevPrev) + this->instance.get_cost(i, j);
            const auto iFilling = +this->instance.get_cost(iPrevPrevPrev, iNext);
            return (iSequenceAdd + iFilling + iSequenceRem + jSequenceRem);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                    solution.get_route_load(jRoute) + this->instance.get_demand(i) + this->instance.get_demand(iPrev) + this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                    ||
                    (iRoute == jRoute && j != iPrev && j != iPrevPrev && j!= solution.get_next_vertex(iRoute, i));

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);

            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, j, iPrevPrev);
            solution.insert_vertex_before(jRoute, j, iPrev);
            solution.insert_vertex_before(jRoute, j, i);

            if (solution.is_route_empty(iRoute)) {
                solution.remove_route(iRoute);
            }
        }

    };

    template<bool handle_partial_solution=false>
    class RevThreeZeroExchange : public AbstractOperator<handle_partial_solution> {
     public:
        RevThreeZeroExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
            : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

     protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(j, jNext);
            const auto iSequenceAdd = +this->instance.get_cost(jNext, iPrevPrev) + this->instance.get_cost(i, j);
            const auto iFilling = +this->instance.get_cost(iPrevPrevPrev, iNext);
            return (iSequenceAdd + iFilling + iSequenceRem + jSequenceRem);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);


            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                solution.get_route_load(jRoute) + this->instance.get_demand(i) + this->instance.get_demand(iPrev) + this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                ||
                (iRoute == jRoute && j != iPrev && j != iPrevPrev && j!= solution.get_prev_vertex(iRoute, iPrevPrev));
        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);

            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);
            storage.insert(jNextNextNext);

            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true); // because of the path reversal
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true); // because of the path reversal
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, jNext, i);
            solution.insert_vertex_before(jRoute, jNext, iPrev);
            solution.insert_vertex_before(jRoute, jNext, iPrevPrev);

            if (solution.is_route_empty(iRoute)) {
                solution.remove_route(iRoute);
            }
        }
    };

    template<bool handle_partial_solution=false>
    class ThreeOneExchange : public AbstractOperator<handle_partial_solution> {
    public:
        ThreeOneExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

    protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(jPrevPrev, jPrev) - this->instance.get_cost(jPrev, j);

            const auto iSequenceAdd = +this->instance.get_cost(jPrevPrev, iPrevPrev) + this->instance.get_cost(i, j);
            const auto jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jPrev) + this->instance.get_cost(jPrev, iNext);

            return iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem;

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                jPrev != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jPrev)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) +
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jPrev)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) -
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                    ||
                    (iRoute == jRoute && i != jPrev && i != solution.get_prev_vertex(jRoute, jPrev) && j != iPrev && j != iPrevPrev);

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);

            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, j, iPrevPrev);
            solution.insert_vertex_before(jRoute, j, iPrev);
            solution.insert_vertex_before(jRoute, j, i);

            solution.remove_vertex(jRoute, jPrev);

            solution.insert_vertex_before(iRoute, iNext, jPrev);

        }

    };

    template<bool handle_partial_solution=false>
    class RevThreeOneExchange : public AbstractOperator<handle_partial_solution> {
     public:
        RevThreeOneExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
            : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

     protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(j, jNext) - this->instance.get_cost(jNext, jNextNext);

            const auto iSequenceAdd = +this->instance.get_cost(jNextNext, iPrevPrev) + this->instance.get_cost(i, j);
            const auto jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jNext) + this->instance.get_cost(jNext, iNext);

            return iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem;

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                jNext != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jNext)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) +
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jNext)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) -
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                ||
                    (iRoute == jRoute &&
                    j != iPrev &&
                    j != iPrevPrev &&
                    jNext != iPrevPrev &&
                    jNext != solution.get_prev_vertex(iRoute, iPrevPrev));

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);
            const auto iPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);
            const auto jNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNext);

            storage.insert(iPrevPrevPrevPrev);
            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);
            storage.insert(jNextNextNext);
            storage.insert(jNextNextNextNext);

            this->update_bits.at(iPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, jNextNext, i);
            solution.insert_vertex_before(jRoute, jNextNext, iPrev);
            solution.insert_vertex_before(jRoute, jNextNext, iPrevPrev);

            solution.remove_vertex(jRoute, jNext);

            solution.insert_vertex_before(iRoute, iNext, jNext);

        }


    };

    template<bool handle_partial_solution=false>
    class ThreeTwoExchange : public AbstractOperator<handle_partial_solution> {
    public:
        ThreeTwoExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

    protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(jPrevPrevPrev, jPrevPrev) - this->instance.get_cost(jPrev, j);

            const auto iSequenceAdd = +this->instance.get_cost(jPrevPrevPrev, iPrevPrev) + this->instance.get_cost(i, j);
            const auto jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jPrevPrev) + this->instance.get_cost(jPrev, iNext);

            return (iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                    jPrev != this->instance.get_depot() && jPrevPrev != this->instance.get_depot() &&
                    solution.get_route_load(jRoute) - this->instance.get_demand(jPrev) -
                    this->instance.get_demand(jPrevPrev)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) +
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity() &&
                    solution.get_route_load(iRoute) + this->instance.get_demand(jPrev) +
                    this->instance.get_demand(jPrevPrev)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) -
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                    ||
                    (iRoute == jRoute &&
                        i != jPrev &&
                        i != jPrevPrev &&
                        i != jPrevPrevPrev &&
                        j != iPrev &&
                        j != iPrevPrev);

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(jPrevPrevPrev);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);

            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, j, iPrevPrev);
            solution.insert_vertex_before(jRoute, j, iPrev);
            solution.insert_vertex_before(jRoute, j, i);

            solution.remove_vertex(jRoute, jPrev);
            solution.remove_vertex(jRoute, jPrevPrev);

            solution.insert_vertex_before(iRoute, iNext, jPrevPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrev);

        }

    };

    template<bool reverse_both_strings, bool handle_partial_solution=false>
    class RevThreeTwoExchange : public AbstractOperator<handle_partial_solution> {
     public:
        RevThreeTwoExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
            : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

     protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(j, jNext) - this->instance.get_cost(jNextNext, jNextNextNext);

            const auto iSequenceAdd = +this->instance.get_cost(jNextNextNext, iPrevPrev) + this->instance.get_cost(i, j);

            float jSequenceAdd;
            if constexpr (reverse_both_strings) {
                jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jNextNext) + this->instance.get_cost(jNext, iNext);
            } else {
                jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jNext) + this->instance.get_cost(jNextNext, iNext);
            }

            return (iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                jNext != this->instance.get_depot() && jNextNext != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jNext) -
                    this->instance.get_demand(jNextNext)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) +
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jNext) +
                    this->instance.get_demand(jNextNext)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) -
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                ||
                    (iRoute == jRoute &&
                        j != iPrev &&
                        j != iPrevPrev &&
                        jNext != iPrevPrev &&
                        jNextNext != iPrevPrev &&
                        jNextNext != solution.get_prev_vertex(iRoute, iPrevPrev));


        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);
            const auto iPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrev);
            const auto iPrevPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);
            const auto jNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNext);
            const auto jNextNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNextNext);

            storage.insert(iPrevPrevPrevPrevPrev);
            storage.insert(iPrevPrevPrevPrev);
            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);
            storage.insert(jNextNextNext);
            storage.insert(jNextNextNextNext);
            storage.insert(jNextNextNextNextNext);

            this->update_bits.at(iPrevPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, jNextNextNext, i);
            solution.insert_vertex_before(jRoute, jNextNextNext, iPrev);
            solution.insert_vertex_before(jRoute, jNextNextNext, iPrevPrev);

            solution.remove_vertex(jRoute, jNext);
            solution.remove_vertex(jRoute, jNextNext);


            if constexpr (reverse_both_strings) {
                solution.insert_vertex_before(iRoute, iNext, jNextNext);
                solution.insert_vertex_before(iRoute, iNext, jNext);
            } else {
                solution.insert_vertex_before(iRoute, iNext, jNext);
                solution.insert_vertex_before(iRoute, iNext, jNextNext);
            }

        }

    };

    template<bool handle_partial_solution=false>
    class ThreeThreeExchange : public AbstractOperator<handle_partial_solution> {
    public:
        ThreeThreeExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance) :
                AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

    protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);
            const auto jPrevPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrevPrev);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(jPrevPrevPrevPrev, jPrevPrevPrev) - this->instance.get_cost(jPrev, j);

            const auto iSequenceAdd = +this->instance.get_cost(jPrevPrevPrevPrev, iPrevPrev) + this->instance.get_cost(i, j);
            const auto jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jPrevPrevPrev) + this->instance.get_cost(jPrev, iNext);

            const auto delta = iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem;

            return delta;
        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                jPrev != this->instance.get_depot() && jPrevPrev != this->instance.get_depot() &&
                jPrevPrevPrev != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jPrev) -
                    this->instance.get_demand(jPrevPrev) -
                    this->instance.get_demand(jPrevPrevPrev)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) +
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jPrev) +
                    this->instance.get_demand(jPrevPrev) +
                    this->instance.get_demand(jPrevPrevPrev)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) -
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                    ||
                    (iRoute == jRoute &&
                        i != jPrev && i != jPrevPrev && i != jPrevPrevPrev && solution.get_next_vertex(iRoute, i) != jPrevPrevPrev &&
                        j != iPrev &&
                        j != iPrevPrev);

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);
            const auto iNextNextNextNext = solution.get_next_vertex(iRoute, iNextNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);
            const auto jPrevPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);

            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(iNextNextNextNext);
            storage.insert(jPrevPrevPrevPrev);
            storage.insert(jPrevPrevPrev);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);
            storage.insert(jNextNextNext);

            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNextNextNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, j, iPrevPrev);
            solution.insert_vertex_before(jRoute, j, iPrev);
            solution.insert_vertex_before(jRoute, j, i);

            solution.remove_vertex(jRoute, jPrev);
            solution.remove_vertex(jRoute, jPrevPrev);
            solution.remove_vertex(jRoute, jPrevPrevPrev);

            solution.insert_vertex_before(iRoute, iNext, jPrevPrevPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrevPrev);
            solution.insert_vertex_before(iRoute, iNext, jPrev);

        }


    };

    template<bool reverse_both_strings, bool handle_partial_solution=false>
    class RevThreeThreeExchange : public AbstractOperator<handle_partial_solution> {
     public:
        RevThreeThreeExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
            : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

     protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);
            const auto jNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNext);

            const auto iSequenceRem = -this->instance.get_cost(iPrevPrevPrev, iPrevPrev) - this->instance.get_cost(i, iNext);
            const auto jSequenceRem = -this->instance.get_cost(j, jNext) - this->instance.get_cost(jNextNextNext, jNextNextNextNext);

            const auto iSequenceAdd = +this->instance.get_cost(jNextNextNextNext, iPrevPrev) + this->instance.get_cost(i, j);

            float jSequenceAdd;
            if constexpr (reverse_both_strings) {
                jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jNextNextNext) + this->instance.get_cost(jNext, iNext);
            } else {
                jSequenceAdd = +this->instance.get_cost(iPrevPrevPrev, jNext) + this->instance.get_cost(jNextNextNext, iNext);
            }

            return (iSequenceAdd + jSequenceAdd + iSequenceRem + jSequenceRem);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);

            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);

            return (iRoute != jRoute && iPrev != this->instance.get_depot() && iPrevPrev != this->instance.get_depot() &&
                jNext != this->instance.get_depot() && jNextNext != this->instance.get_depot() && jNextNextNext != this->instance.get_depot() &&
                solution.get_route_load(jRoute) - this->instance.get_demand(jNext) -
                    this->instance.get_demand(jNextNext) - this->instance.get_demand(jNextNextNext)
                    + this->instance.get_demand(i) + this->instance.get_demand(iPrev) +
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity() &&
                solution.get_route_load(iRoute) + this->instance.get_demand(jNext) +
                    this->instance.get_demand(jNextNext) + this->instance.get_demand(jNextNextNext)
                    - this->instance.get_demand(i) - this->instance.get_demand(iPrev) -
                    this->instance.get_demand(iPrevPrev) <= this->instance.get_vehicle_capacity())
                ||
                    (iRoute == jRoute &&
                        j != iPrev &&
                        j != iPrevPrev &&
                        jNext != iPrevPrev &&
                        jNextNext != iPrevPrev &&
                        jNextNextNext != iPrevPrev &&
                        jNextNextNext != solution.get_prev_vertex(iRoute, iPrevPrev));

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iPrevPrev = solution.get_prev_vertex(iRoute, iPrev);
            const auto iPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrev);
            const auto iPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrev);
            const auto iPrevPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrevPrev);
            const auto iPrevPrevPrevPrevPrevPrev = solution.get_prev_vertex(iRoute, iPrevPrevPrevPrevPrev);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto iNextNext = solution.get_next_vertex(iRoute, iNext);
            const auto iNextNextNext = solution.get_next_vertex(iRoute, iNextNext);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);
            const auto jPrevPrev = solution.get_prev_vertex(jRoute, jPrev);
            const auto jPrevPrevPrev = solution.get_prev_vertex(jRoute, jPrevPrev);


            const auto jNext = solution.get_next_vertex(jRoute, j);
            const auto jNextNext = solution.get_next_vertex(jRoute, jNext);
            const auto jNextNextNext = solution.get_next_vertex(jRoute, jNextNext);
            const auto jNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNext);
            const auto jNextNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNextNext);
            const auto jNextNextNextNextNextNext = solution.get_next_vertex(jRoute, jNextNextNextNextNext);

            storage.insert(iPrevPrevPrevPrevPrevPrev);
            storage.insert(iPrevPrevPrevPrevPrev);
            storage.insert(iPrevPrevPrevPrev);
            storage.insert(iPrevPrevPrev);
            storage.insert(iPrevPrev);
            storage.insert(iPrev);
            storage.insert(i);
            storage.insert(iNext);
            storage.insert(iNextNext);
            storage.insert(iNextNextNext);
            storage.insert(jPrevPrevPrev);
            storage.insert(jPrevPrev);
            storage.insert(jPrev);
            storage.insert(j);
            storage.insert(jNext);
            storage.insert(jNextNext);
            storage.insert(jNextNextNext);
            storage.insert(jNextNextNextNext);
            storage.insert(jNextNextNextNextNext);
            storage.insert(jNextNextNextNextNextNext);

            this->update_bits.at(iPrevPrevPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iPrev, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(i, UPDATE_BITS_SECOND, true);
            this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNextNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jNext, UPDATE_BITS_FIRST, true);
            this->update_bits.at(jNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_FIRST, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrev, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrevPrevPrev, UPDATE_BITS_SECOND, true);

            solution.remove_vertex(iRoute, i);
            solution.remove_vertex(iRoute, iPrev);
            solution.remove_vertex(iRoute, iPrevPrev);

            solution.insert_vertex_before(jRoute, jNextNextNextNext, i);
            solution.insert_vertex_before(jRoute, jNextNextNextNext, iPrev);
            solution.insert_vertex_before(jRoute, jNextNextNextNext, iPrevPrev);

            solution.remove_vertex(jRoute, jNext);
            solution.remove_vertex(jRoute, jNextNext);
            solution.remove_vertex(jRoute, jNextNextNext);


            if constexpr (reverse_both_strings) {
                solution.insert_vertex_before(iRoute, iNext, jNextNextNext);
                solution.insert_vertex_before(iRoute, iNext, jNextNext);
                solution.insert_vertex_before(iRoute, iNext, jNext);
            } else {
                solution.insert_vertex_before(iRoute, iNext, jNext);
                solution.insert_vertex_before(iRoute, iNext, jNextNext);
                solution.insert_vertex_before(iRoute, iNext, jNextNextNext);
            }

        }

    };

    template<bool handle_partial_solution=false>
    class SplitExchange : public AbstractOperator<handle_partial_solution> {
    public:
        SplitExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, true) { }

    protected:

        void pre_processing(cobra::Solution &solution) override {

            for (int route = solution.get_first_route(); route != cobra::Solution::dummy_route; route = solution.get_next_route(route)) {
                solution.update_cumulative_route_loads(route);
            }

        }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto jNext = solution.get_next_vertex(jRoute, j);

            return -this->instance.get_cost(i, iNext) + this->instance.get_cost(i, j) -
                   this->instance.get_cost(j, jNext) + this->instance.get_cost(jNext, iNext);

        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            return iRoute != jRoute &&
                   (solution.get_route_load_before_included(i) + solution.get_route_load_before_included(j) <=
                    this->instance.get_vehicle_capacity() &&
                    solution.get_route_load_after_included(j) - this->instance.get_demand(j) +
                    solution.get_route_load_after_included(i) -
                    this->instance.get_demand(i) <= this->instance.get_vehicle_capacity());


        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            assert(solution.get_first_customer(iRoute) != this->instance.get_depot());
            assert(solution.get_first_customer(jRoute) != this->instance.get_depot());

            storage.insert(this->instance.get_depot());
            for(auto curr = i; curr != this->instance.get_depot(); curr = solution.get_next_vertex(curr)) {
                storage.insert(curr);
            }

            const auto jNextNext = solution.get_next_vertex(jRoute, solution.get_next_vertex(j));
            const auto jStop = jNextNext == solution.get_first_customer(jRoute) ? this->instance.get_depot() : jNextNext; // handle edge case
            for(auto curr = solution.get_first_customer(jRoute); curr != jStop; curr = solution.get_next_vertex(curr)) {
                storage.insert(curr);
            }

            solution.split(i, iRoute, j, jRoute);

            if (solution.is_route_empty(iRoute)) {
                solution.remove_route(iRoute);
            } else {
                solution.update_cumulative_route_loads(iRoute);
            }

            if (solution.is_route_empty(jRoute)) {
                solution.remove_route(jRoute);
            } else {
                solution.update_cumulative_route_loads(jRoute);
            }


        }


    };

    template<bool handle_partial_solution=false>
    class TailsExchange : public AbstractOperator<handle_partial_solution> {
    public:
        TailsExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false) { }

    protected:
        void pre_processing(cobra::Solution &solution) override {

            for (int route = solution.get_first_route();
                 route != cobra::Solution::dummy_route; route = solution.get_next_route(route)) {
                solution.update_cumulative_route_loads(route);
            }

        }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            const auto delta = -this->instance.get_cost(i, iNext) + this->instance.get_cost(i, j) -
                               this->instance.get_cost(jPrev, j) + this->instance.get_cost(jPrev, iNext);

            return delta;
        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            return iRoute != jRoute &&
                   //iRoute != jRoute if both i and j are different from the depot
                   solution.get_route_load_before_included(i) + solution.get_route_load_after_included(j) <=
                   this->instance.get_vehicle_capacity() &&
                   solution.get_route_load_before_included(j) - this->instance.get_demand(j) +
                   solution.get_route_load_after_included(i) -
                   this->instance.get_demand(i) <= this->instance.get_vehicle_capacity();

        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iNext = solution.get_next_vertex(i);
            const auto jPrev = solution.get_prev_vertex(j);

            storage.insert(i);
            storage.insert(iNext);
            storage.insert(jPrev);
            storage.insert(j);

            this->update_bits.at(i, UPDATE_BITS_FIRST, true);
            this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
            this->update_bits.at(j, UPDATE_BITS_SECOND, true);
            this->update_bits.at(jPrev, UPDATE_BITS_FIRST, true);

            const auto i_route = solution.get_route_index(i);
            const auto j_route = solution.get_route_index(j);

            solution.swap_tails(i, i_route, j, j_route);

            if (solution.is_route_empty(i_route)) {
                solution.remove_route(i_route);
            } else {
                solution.update_cumulative_route_loads(i_route);
            }

            if (solution.is_route_empty(j_route)) {
                solution.remove_route(j_route);
            } else {
                solution.update_cumulative_route_loads(j_route);
            }


        }


    };

    template<bool handle_partial_solution=false>
    class TwoOptExchange : public AbstractOperator<handle_partial_solution> {
    public:
        TwoOptExchange(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, true) { }

    protected:
        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iNext = solution.get_next_vertex(iRoute, i);
            const auto jNext = solution.get_next_vertex(jRoute, j);

            return -this->instance.get_cost(i, iNext) + this->instance.get_cost(i, j) -
                   this->instance.get_cost(j, jNext) + this->instance.get_cost(jNext, iNext);
        }

        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &move) override {
            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            return iRoute == jRoute;
        }

        void execute(cobra::Solution &solution, const MoveGenerator &move, cobra::VertexSet& storage) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);

            assert(solution.get_first_customer(iRoute) != this->instance.get_depot());

            const auto jNextNext = solution.get_next_vertex(iRoute, solution.get_next_vertex(iRoute, j));
            // selective update of i, iNext, j, jNext that are directly involved and of the reversed part only in which prev & next pointer where changed
            // use the do-while for very short tour (4 vertices) in which jNextNext equal i
            auto curr = i;
            do {
                storage.insert(curr);
                curr = solution.get_next_vertex(iRoute, curr);
            } while(curr != jNextNext);

            const auto iNext = solution.get_next_vertex(iRoute, i);

            solution.reverse_route_path(iRoute, iNext, j);

        }

    };

    template<bool handle_partial_solution=false, int max_relocation_nodes=25>
    class EjectionChain : public AbstractOperator<handle_partial_solution> {

    public:
        EjectionChain(const cobra::Instance& instance, MoveGenerators &moves, float tolerance)
                : AbstractOperator<handle_partial_solution>(instance, moves, tolerance, false),
                    forbidden_i(max_relocation_nodes, instance.get_vertices_num()),
                    forbidden_j(max_relocation_nodes, instance.get_vertices_num()){

            relocation_nodes.resize(max_relocation_nodes);
            feasible_chains.reserve(max_relocation_nodes);
            heap_array.resize(max_relocation_nodes);

        }

    protected:

        void pre_processing(__attribute__((unused)) cobra::Solution &solution) override { }

        float compute_cost(const cobra::Solution &solution, const MoveGenerator &move) override {

            const auto i = move.get_first_vertex();
            const auto j = move.get_second_vertex();

            const auto iRoute = solution.get_route_index(i, j);
            const auto jRoute = solution.get_route_index(j, i);

            const auto iPrev = solution.get_prev_vertex(iRoute, i);
            const auto iNext = solution.get_next_vertex(iRoute, i);

            const auto jPrev = solution.get_prev_vertex(jRoute, j);

            auto delta = 0.0f;

            if (j != iNext) {
                delta = -this->instance.get_cost(iPrev, i) - this->instance.get_cost(i, iNext) +
                        this->instance.get_cost(iPrev, iNext)
                        - this->instance.get_cost(jPrev, j) + this->instance.get_cost(jPrev, i) +
                        this->instance.get_cost(i, j);
            }

            return delta;
        }

        // The feasibility step for the ejection chain is more complex than
        // that of other operators. In this context, we use the move generator
        // as the starting point from which a tree of relocations is generated.
        bool is_feasible(const cobra::Solution &solution, const MoveGenerator &generating_move) override {

            auto rni = 0; // relocate node index, no. of generated nodes

            feasible_chains.clear();

            // First check whether the current `generating_move` is itself a
            // feasible relocate move. If this is the case, apply it without
            // further searching for feasible ejection chains.

            // The following code is into a block to avoid hiding subsequent variables!
            {

                auto i = generating_move.get_first_vertex();
                auto j = generating_move.get_second_vertex();

                auto iRoute = solution.get_route_index(i, j);
                auto jRoute = solution.get_route_index(j, i);

                auto iPrev = solution.get_prev_vertex(iRoute, i);
                auto iNext = solution.get_next_vertex(iRoute, i);
                auto jPrev = solution.get_prev_vertex(jRoute, j);

                relocation_nodes[rni].move = &generating_move;

                if (iRoute == jRoute ||(solution.get_route_load(jRoute) + this->instance.get_demand(i) <= this->instance.get_vehicle_capacity())) {
                    feasible_chains.push_back(0);
                    relocation_nodes[0].predecessor = -1;
                    forbidden_i.reset(0);
                    forbidden_i.set(0, iPrev);
                    forbidden_i.set(0, i);
                    forbidden_i.set(0, iNext);
                    forbidden_i.set(0, jPrev);
                    forbidden_i.set(0, j);
                    return true;
                }

                // If the generating move is not feasible by itself, we start a relocation chain

                // set up state variables
                relocation_nodes[rni].delta_sum = generating_move.get_delta();

                forbidden_i.reset(rni);
                forbidden_i.set(rni, iPrev);
                forbidden_i.set(rni, i);
                forbidden_i.set(rni, iNext);
                forbidden_i.set(rni, jPrev);
                forbidden_i.set(rni, j);

                forbidden_j.reset(rni);
                forbidden_j.set(rni, i);
                forbidden_j.set(rni, iNext);
                forbidden_j.set(rni, j);

                relocation_nodes[rni].modified_routes_loads.clear();
                relocation_nodes[rni].modified_routes_loads[iRoute] = solution.get_route_load(iRoute) - this->instance.get_demand(i);
                relocation_nodes[rni].modified_routes_loads[jRoute] = solution.get_route_load(jRoute) + this->instance.get_demand(i);
                relocation_nodes[rni].predecessor = -1;
                rni++;

                heap_reset();
                heap_insert(0);

            }

            while (heap_len) {

                const auto curr_index = heap_get();

                auto &curr = relocation_nodes[curr_index];

                // retrieve the route from which we would like to remove some vertex
                const auto iRoute = solution.get_route_index(curr.move->get_second_vertex());

                // retrieve the updated 'iRoute' load (iRoute will always be in the map)
                const auto iRoute_load = curr.modified_routes_loads[iRoute];

                // scan 'iRoute' searching for customers to remove, which will make the route feasible
                for (auto i = solution.get_first_customer(iRoute); i != this->instance.get_depot(); i = solution.get_next_vertex(i)) {

                    // check whether removing 'i' is sufficient to restore the 'iRoute' feasibility
                    if (iRoute_load - this->instance.get_demand(i) > this->instance.get_vehicle_capacity()) { continue; }

                    // check whether this vertex can be used as 'i'
                    if (forbidden_i.is_set(curr_index, i)) { continue; }

                    // retrieve the available generating_move generators involving 'i'. They may be {i j} and {j i}
                    // scan them ...
                    for (const auto move_index : this->moves.get_move_generator_indices_involving(i)) {

                        auto &move = this->moves.get(move_index);

                        if constexpr (handle_partial_solution) {
                            if (!solution.is_vertex_in_solution(move.get_first_vertex()) || !solution.is_vertex_in_solution(move.get_second_vertex())) {
                                continue;
                            }
                        }

                        // ... searching for {i j} generators ...
                        if (move.get_first_vertex() != i) { continue; }

                        const auto j = move.get_second_vertex();

                        // check whether this vertex can be used as 'j', we cannot use the depot because we want to relocate in another route
                        if (j==this->instance.get_depot() || forbidden_j.is_set(curr_index, j)) { continue; }

                        const auto jRoute = solution.get_route_index(j);

                        // relocate in another route! Note we are removing 'i' to restore 'iRoute' feasibility
                        if (jRoute==iRoute) { continue; }

                        // For move generators not in the 'MovesHeap' we don't know whether
                        // their delta was updated during the initialization stage or not. To this end,
                        // we just recompute the value of 'delta'.
                        const auto iPrev = solution.get_prev_vertex(iRoute, i);
                        const auto iNext = solution.get_next_vertex(iRoute, i);
                        const auto jPrev = solution.get_prev_vertex(jRoute, j);
                        const auto correct_delta = -this->instance.get_cost(iPrev, i) - this->instance.get_cost(i, iNext)
                            + this->instance.get_cost(iPrev, iNext)
                            - this->instance.get_cost(jPrev, j) + this->instance.get_cost(jPrev, i) +
                            this->instance.get_cost(i, j);
                        move.set_delta(correct_delta);

                        // ... that keeps the chain improving
                        if (move.get_delta() + curr.delta_sum > -this->tolerance) { continue; }

                        // check whether we have already worked with 'jRoute'
                        auto jRoute_load = 0;
                        if (curr.modified_routes_loads.count(jRoute)) {
                            jRoute_load = curr.modified_routes_loads[jRoute];
                        } else {
                            jRoute_load = solution.get_route_load(jRoute);
                        }

                        // at this point, the move {i j} is able to restore 'iRoute' feasibility, store it in 'rni' position
                        relocation_nodes[rni].move = &move;
                        relocation_nodes[rni].delta_sum = curr.delta_sum + move.get_delta();

                        forbidden_i.overwrite(curr_index, rni);
                        forbidden_i.set(rni, iPrev);
                        forbidden_i.set(rni, i);
                        forbidden_i.set(rni, iNext);
                        forbidden_i.set(rni, jPrev);
                        forbidden_i.set(rni, j);

                        forbidden_j.overwrite(curr_index, rni);
                        forbidden_j.set(rni, i);
                        forbidden_j.set(rni, iNext);
                        forbidden_j.set(rni, j);

                        relocation_nodes[rni].modified_routes_loads = curr.modified_routes_loads;
                        relocation_nodes[rni].modified_routes_loads[iRoute] = iRoute_load - this->instance.get_demand(i);
                        relocation_nodes[rni].modified_routes_loads[jRoute] = jRoute_load + this->instance.get_demand(i);
                        relocation_nodes[rni].predecessor = curr_index;
                        heap_insert(rni);

                        // if also 'jRoute' is feasible we have found a feasible chain!
                        if (jRoute_load + this->instance.get_demand(i) <= this->instance.get_vehicle_capacity()) {
                            feasible_chains.push_back(rni);
                            goto end;
                        }

                        rni++;

                        if (rni==max_relocation_nodes) { goto end; }

                    }

                }

            }

            end:

            return !feasible_chains.empty();

        }

        void execute(cobra::Solution &solution, __attribute__((unused)) const MoveGenerator &p_move, cobra::VertexSet& storage) override {

            // Find the most improving feasible ejection chain
            std::sort(feasible_chains.begin(), feasible_chains.end(), [this](int a, int b) -> bool {
              return relocation_nodes[a].delta_sum < relocation_nodes[b].delta_sum;
            });

            const auto best_chain_index = feasible_chains[0];
            auto &best_chain = relocation_nodes[best_chain_index];

            for(auto i : forbidden_i.get_set_entries_possibly_with_duplicates(best_chain_index)) {
                storage.insert(i);
            }

            for (auto ptr = best_chain_index; ptr != -1; ptr = relocation_nodes[ptr].predecessor) {
                const auto &move = relocation_nodes[ptr].move;

                const auto i = move->get_first_vertex();
                const auto j = move->get_second_vertex();

                const auto iRoute = solution.get_route_index(i, j);
                const auto jRoute = solution.get_route_index(j, i);

                this->update_bits.at(solution.get_prev_vertex(iRoute, i), UPDATE_BITS_FIRST, true);
                this->update_bits.at(i, UPDATE_BITS_FIRST, true);
                this->update_bits.at(i, UPDATE_BITS_SECOND, true);
                const auto iNext = solution.get_next_vertex(iRoute, i);
                this->update_bits.at(iNext, UPDATE_BITS_FIRST, true);
                this->update_bits.at(iNext, UPDATE_BITS_SECOND, true);
                this->update_bits.at(j, UPDATE_BITS_FIRST, true);
                this->update_bits.at(j, UPDATE_BITS_SECOND, true);
                this->update_bits.at(solution.get_prev_vertex(jRoute, j), UPDATE_BITS_FIRST, true);

                solution.remove_vertex(iRoute, i);
                solution.insert_vertex_before(jRoute, j, i);

                if (solution.is_route_empty(iRoute)) {
                    solution.remove_route(iRoute);
                }

            }

        }

     private:

        static const int heap_unheaped = -1;

        struct Relocation {
            int heap_index = heap_unheaped;
            float delta_sum = 0.0f;
            std::unordered_map<int, int> modified_routes_loads;
            const MoveGenerator *move = nullptr;
            int predecessor = 0;
        };

        cobra::BitMatrix forbidden_i;
        cobra::BitMatrix forbidden_j;

        std::vector <Relocation> relocation_nodes;
        std::vector<int> feasible_chains;

        /* ===== HEAP SECTION ====== */

        #define LEFT(xxx) (2 * (xxx) + 1)
        #define RIGHT(xxx) (2 * (xxx) + 2)
        #define PARENT(xxx) (((xxx) - 1) / 2)

        std::vector<int> heap_array;
        int heap_len = 0;

        bool is_heap() {

            for (auto n = 0; n < heap_len; n++) {
                if (relocation_nodes[heap_array[n]].heap_index != n) {
                    return false;
                }
            }

            for (auto n = 0; n < heap_len; n++) {
                const auto left_index = LEFT(n);
                const auto right_index = RIGHT(n);
                if (left_index < heap_len) {
                    if (relocation_nodes[heap_array[n]].delta_sum >
                        relocation_nodes[heap_array[left_index]].delta_sum) {
                        return false;
                    }
                }
                if (right_index < heap_len) {
                    if (relocation_nodes[heap_array[n]].delta_sum >
                        relocation_nodes[heap_array[right_index]].delta_sum) {
                        return false;
                    }
                }
            }

            return true;
        }

        void heap_reset() { heap_len = 0; }

        void heap_insert(int relocate_index) {
            assert(heap_len < max_relocation_nodes);
            int heap_index = heap_len;

            heap_len++;

            while (heap_index && relocation_nodes[relocate_index].delta_sum <
                relocation_nodes[heap_array[PARENT(heap_index)]].delta_sum) {

                const auto parent_index = PARENT(heap_index);
                heap_array[heap_index] = heap_array[parent_index];
                relocation_nodes[heap_array[heap_index]].heap_index = heap_index;
                heap_index = parent_index;

            }

            heap_array[heap_index] = relocate_index;
            relocation_nodes[relocate_index].heap_index = heap_index;

            assert(is_heap());

        }

        int heap_get() {

            assert(heap_len > 0);

            const auto move_index = heap_array[0];

            heap_array[0] = heap_array[heap_len - 1];
            relocation_nodes[heap_array[0]].heap_index = 0;
            heap_len--;

            heap_heapify(0);

            relocation_nodes[move_index].heap_index = -1;

            assert(is_heap());

            return move_index;

        }

        void heap_heapify(int heap_index) {
            auto smallest = -1;
            auto index = heap_index;

            while (index <= heap_len) {

                auto left_index = LEFT(index);
                auto right_index = RIGHT(index);

                if (left_index < heap_len && relocation_nodes[heap_array[left_index]].delta_sum <
                    relocation_nodes[heap_array[index]].delta_sum) {
                    smallest = left_index;
                } else {
                    smallest = index;
                }

                if (right_index < heap_len && relocation_nodes[heap_array[right_index]].delta_sum <
                    relocation_nodes[heap_array[smallest]].delta_sum) {
                    smallest = right_index;
                }

                if (smallest != index) {

                    const auto tmp = heap_array[index];
                    heap_array[index] = heap_array[smallest];
                    heap_array[smallest] = tmp;

                    relocation_nodes[heap_array[index]].heap_index = index;
                    relocation_nodes[heap_array[smallest]].heap_index = smallest;

                    index = smallest;

                } else {
                    break;
                }

            }

        }

        /* ===== END HEAP SECTION ====== */

    };

    class VariableNeighborhoodDescentInterface {

     public:
        virtual void apply(Solution& solution) = 0;

    };

    template <bool handle_partial_solutions = false>
    class RandomizedVariableNeighborhoodDescent : private NonCopyable<RandomizedVariableNeighborhoodDescent<handle_partial_solutions>>, public VariableNeighborhoodDescentInterface {

    public:

        RandomizedVariableNeighborhoodDescent(const cobra::Instance& instance_, MoveGenerators& moves_, std::initializer_list<Operator> operators, std::mt19937& rand_engine_, float tolerance = 0.01f) :
        instance(instance_), moves(moves_), rand_engine(rand_engine_){

            OperatorInitTable[E10] = [this, tolerance](){return new OneZeroExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E11] = [this, tolerance](){return new OneOneExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E20] = [this, tolerance](){return new TwoZeroExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E21] = [this, tolerance](){return new TwoOneExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E22] = [this, tolerance](){return new TwoTwoExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E30] = [this, tolerance](){return new ThreeZeroExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E31] = [this, tolerance](){return new ThreeOneExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E32] = [this, tolerance](){return new ThreeTwoExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[E33] = [this, tolerance](){return new ThreeThreeExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[SPLIT] = [this, tolerance](){return new SplitExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[TAILS] = [this, tolerance](){return new TailsExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[TWOPT] = [this, tolerance](){return new TwoOptExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[EJCH] = [this, tolerance](){return new EjectionChain<handle_partial_solutions, 25>(instance, moves, tolerance);};
            OperatorInitTable[RE20] = [this, tolerance](){return new RevTwoZeroExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE21] = [this, tolerance](){return new RevTwoOneExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE22B] = [this, tolerance](){return new RevTwoTwoExchange<true, handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE22S] = [this, tolerance](){return new RevTwoTwoExchange<false, handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE30] = [this, tolerance](){return new RevThreeZeroExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE31] = [this, tolerance](){return new RevThreeOneExchange<handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE32B] = [this, tolerance](){return new RevThreeTwoExchange<true, handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE32S] = [this, tolerance](){return new RevThreeTwoExchange<false, handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE33B] = [this, tolerance](){return new RevThreeThreeExchange<true, handle_partial_solutions>(instance, moves, tolerance);};
            OperatorInitTable[RE33S] = [this, tolerance](){return new RevThreeThreeExchange<false, handle_partial_solutions>(instance, moves, tolerance);};

            for(auto op : operators) {
                auto ptr = OperatorInitTable[op]();
                fixed_list.push_back(ptr);
                sortable_list.push_back(ptr);
            }

        }

        virtual ~RandomizedVariableNeighborhoodDescent() {
            for(auto move : fixed_list) { delete move; }
        }

        void apply(cobra::Solution& solution) override {

            std::shuffle(sortable_list.begin(), sortable_list.end(), rand_engine);

            auto end = 0u;

            auto curr = end;

            do {

                auto ptr = sortable_list[curr];

                const auto improved = ptr->apply_rough_best_improvement(solution);

                if(improved) { end = curr; }

                curr = (curr + 1) % sortable_list.size();

            } while (curr != end);

        }

     private:
        const cobra::Instance& instance;
        MoveGenerators& moves;
        std::mt19937& rand_engine;

        std::unordered_map<Operator, std::function<AbstractOperator<handle_partial_solutions>*()>> OperatorInitTable;

        std::vector<AbstractOperator<handle_partial_solutions>*> fixed_list;
        std::vector<AbstractOperator<handle_partial_solutions>*> sortable_list;

    };

    class HierarchicalVariableNeighborhoodDescent {

     public:

        explicit HierarchicalVariableNeighborhoodDescent(float tolerance_) : tolerance(tolerance_){};

        void append(VariableNeighborhoodDescentInterface* vnd) {
            tiers.push_back(vnd);
        }

        void apply(Solution& solution) {

            __again__:
            for(auto n = 0u; n < tiers.size(); n++) {
                const auto curr_cost = solution.get_cost();
                tiers[n]->apply(solution);
                if(n > 0 && solution.get_cost() + tolerance < curr_cost) {
                    goto __again__;
                }
            }

        }

     private:

        const float tolerance;
        std::vector<VariableNeighborhoodDescentInterface*> tiers;

    };

}



#endif //COBRA_LOCALSEARCH_HPP
