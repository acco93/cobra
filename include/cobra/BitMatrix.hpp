/**
 * Space-efficient matrix of bits.
 */

#ifndef COBRA_BITMATRIX_HPP_
#define COBRA_BITMATRIX_HPP_

#include <vector>
#include <cmath>
#include "NonCopyable.hpp"

namespace cobra {

class BitMatrix : NonCopyable<BitMatrix> {

 public:

    BitMatrix(int rows, int entries_num) {

        data.resize(rows);
        for(auto & n : data) {
            n.resize(entries_num);
        }

        fast_access_with_duplicates.resize(rows);

    }

    inline auto reset(int row) -> void {
        for(auto vertex : fast_access_with_duplicates[row]) {
            data[row][vertex] = false;
        }
        fast_access_with_duplicates[row].clear();
    }

    inline auto set(int row, int entry) -> void {
        data[row][entry] = true;
        fast_access_with_duplicates[row].emplace_back(entry);
    }

    inline auto is_set(int row, int entry) -> bool {
        return data[row][entry];
    }

    inline auto overwrite(int source_row, int destination_row) {
        reset(destination_row);
        for(auto vertex : fast_access_with_duplicates[source_row]) {
            set(destination_row, vertex);
        }
    }

    inline auto get_set_entries_possibly_with_duplicates(int node) -> const std::vector<int>& {
        return fast_access_with_duplicates[node];
    }

 private:

    std::vector<std::vector<bool>> data;
    std::vector<std::vector<int>> fast_access_with_duplicates; // store entries set to 1, without caring for duplicates


};

}

#endif //COBRA_BITMATRIX_HPP_
