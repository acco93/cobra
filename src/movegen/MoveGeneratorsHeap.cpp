/**
 * Move generators heap implementation.
 */

#include <cobra/MoveGenerators.hpp>
#include <cassert>
#include <iostream>
#include "cobra/mm.hpp"

namespace cobra {

#define LEFT(xxx) (2 * (xxx) + 1)
#define RIGHT(xxx) (2 * (xxx) + 2)
#define PARENT(xxx) (((xxx) - 1) / 2)


    const int MoveGeneratorsHeap::unheaped = -1;

    MoveGeneratorsHeap::MoveGeneratorsHeap(MoveGenerators &move_generators) : moves(move_generators.get_raw_vector()) {

        heap = cobra::request_raw_contiguous_memory<int>(moves.size());
        heap_len = 0;

    }

    MoveGeneratorsHeap::~MoveGeneratorsHeap() {

        cobra::release_raw_contiguous_memory(heap);

    }

    void MoveGeneratorsHeap::reset() {

        for(auto heap_index = 0; heap_index < heap_len; heap_index++) { moves[heap[heap_index]].set_heap_index(unheaped); }
        heap_len = 0;

    }

    bool MoveGeneratorsHeap::is_empty() const {
        return !heap_len;
    }

    void MoveGeneratorsHeap::insert(int move_index) {

        assert(heap_len < static_cast<int>(moves.size()));

        int heap_index = heap_len;

        heap_len++;

        while(heap_index && moves[move_index].get_delta() < moves[heap[PARENT(heap_index)]].get_delta()) {

            const auto parent_index = PARENT(heap_index);
            heap[heap_index] = heap[parent_index];
            moves[heap[heap_index]].set_heap_index(heap_index);
            heap_index = parent_index;

        }

        heap[heap_index] = move_index;
        moves[move_index].set_heap_index(heap_index);

        assert(is_heap());

    }

    int MoveGeneratorsHeap::get() {

        assert(!is_empty());

        const auto move_index = heap[0];

        heap[0] = heap[heap_len - 1];
        moves[heap[0]].set_heap_index(0);
        heap_len--;

        heapify(0);

        moves[move_index].set_heap_index(unheaped);

        assert(is_heap());

        return move_index;
    }

    void MoveGeneratorsHeap::remove(int move_index) {

        const auto heap_index = moves[move_index].get_heap_index();

        const auto delta = moves[move_index].get_delta();

        heap[heap_index] = heap[heap_len - 1];
        moves[heap[heap_index]].set_heap_index(heap_index);
        heap_len--;

        if(moves[heap[heap_index]].get_delta() < delta) {
            upsift(heap_index);
        } else if(moves[heap[heap_index]].get_delta() > delta) {
            heapify(heap_index);
        }

        moves[move_index].set_heap_index(unheaped);

        assert(is_heap());

    }

    void MoveGeneratorsHeap::change_value(int move_index, float value) {

        const auto delta = moves[move_index].get_delta();
        const auto heap_index = moves[move_index].get_heap_index();

        moves[move_index].set_delta(value);

        if(value < delta) {
            upsift(heap_index);
        } else if(value > delta) {
            heapify(heap_index);
        }

        assert(is_heap());

    }

    bool MoveGeneratorsHeap::is_heap() {

        for(auto n = 0; n < heap_len; n++) {
            if(moves[heap[n]].get_heap_index() != n) {
                std::cout << "moves[heap[n]].get_heap_index() != n\n";
                std::cout << moves[heap[n]].get_heap_index() << " " << n << "\n";
                dump();
                return false;
            }
        }

        for(auto n = 0; n < heap_len; n++) {
            const auto left_index = LEFT(n);
            const auto right_index = RIGHT(n);
            if(left_index < heap_len) {
                if(moves[heap[n]].get_delta() > moves[heap[left_index]].get_delta()) {
                    std::cout << "left: " << moves[heap[n]].get_delta()  << " > "  << moves[heap[left_index]].get_delta() << "\n";
                    dump();
                    return false;
                }
            }
            if(right_index < heap_len) {
                if(moves[heap[n]].get_delta() > moves[heap[right_index]].get_delta()) {
                    std::cout << "right: " << moves[heap[n]].get_delta()  << " > "  << moves[heap[right_index]].get_delta() << "\n";
                    dump();
                    return false;
                }
            }
        }

        return true;
    }

    void MoveGeneratorsHeap::dump() {
        for(auto n = 0; n < heap_len; n++) {
            const auto move = moves[heap[n]];
            std::cout << "[" << n << "] (" << move.get_first_vertex() << ", " << move.get_second_vertex() << ") delta = " << move.get_delta() << " heap index = " << move.get_heap_index() << "move index = " << heap[n] << "\n";
        }
    }

    void MoveGeneratorsHeap::upsift(int heap_index) {

        const auto move_index = heap[heap_index];

        while(heap_index && moves[move_index].get_delta() < moves[heap[PARENT(heap_index)]].get_delta()) {

            const auto parent_index = PARENT(heap_index);
            heap[heap_index] = heap[parent_index];
            moves[heap[heap_index]].set_heap_index(heap_index);
            heap_index = parent_index;

        }

        heap[heap_index] = move_index;
        moves[move_index].set_heap_index(heap_index);

        assert(is_heap());

    }

    void MoveGeneratorsHeap::heapify(int heap_index) {

        auto smallest = unheaped;
        auto index = heap_index;

        while(index <= heap_len) {

            auto left_index = LEFT(index);
            auto right_index = RIGHT(index);

            if(left_index < heap_len && moves[heap[left_index]].get_delta() < moves[heap[index]].get_delta()) {
                smallest = left_index;
            } else {
                smallest = index;
            }

            if(right_index < heap_len && moves[heap[right_index]].get_delta() < moves[heap[smallest]].get_delta()) {
                smallest = right_index;
            }

            if(smallest != index) {

                const auto tmp = heap[index];
                heap[index] = heap[smallest];
                heap[smallest] = tmp;

                moves[heap[index]].set_heap_index(index);
                moves[heap[smallest]].set_heap_index(smallest);

                index = smallest;

            } else {
                break;
            }

        }

        assert(is_heap());

    }

    int MoveGeneratorsHeap::size() const {
        return heap_len;
    }

    int MoveGeneratorsHeap::spy(int heap_index) const {
        return heap[heap_index];
    }



}