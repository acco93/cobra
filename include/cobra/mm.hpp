/*
* Raw memory management.
*/

#ifndef COBRA_MM_HPP
#define COBRA_MM_HPP

#include <cstdlib>
#include <cassert>
#include <cstring>
#include <iostream>

namespace cobra {

    /*
    * Declare an efficient 2d matrix of contiguous memory of primitive data types.
    * Cannot be used for objects that requires allocation with new.
    * It should be used for up to average sized matrices since memory is contiguous.
    * Matrix could be accessed in the classical way matrix[i][j]
    */
    template<class T>
    static inline T **request_raw_contiguous_memory(const size_t rows, const size_t columns) {

        // allocate space for:
        // the data: columns * rows * sizeof(T)
        // the row pointers: rows * sizeof(T*)
        // [ROW POINTERS] [---------------- DATA -------------]
        const size_t size = columns * rows * sizeof(T) + rows * sizeof(T *);
        auto **matrix = static_cast<T **>(malloc(size));

        if(!matrix) {
            std::cerr << "mm::request_raw_contiguous_memory error: malloc returned NULL\n";
            abort();
        }

        memset(matrix, 0, size);

        // the initial space contains the row pointers. Let make them point to the right data
        // base_address contains the address of the memory after the row pointers
        // [ROW POINTERS] [---------------- DATA -------------]
        // base_address-->|
        // then each chunk of data contains "columns" columns
        auto *base_address = reinterpret_cast<T *>(&matrix[rows]);
        for (size_t i = 0; i < rows; i++) {
            matrix[i] = base_address + i * columns;
        }

        return matrix;
    }

    template<class T>
    static inline T *request_raw_contiguous_memory(const size_t elements) {
        auto *ptr = static_cast<T *>(calloc(elements, sizeof(T)));
        assert(ptr);
        return ptr;
    }

    template<class T>
    static inline void release_raw_contiguous_memory(T *&ptr) {
        free(ptr);
        ptr = nullptr;
    }

}

#endif // COBRA_MM_HPP
