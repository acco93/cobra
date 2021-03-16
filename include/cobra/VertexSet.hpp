/**
 * Simple set of vertices with fixed max size.
 */

#ifndef COBRA_INCLUDE_COBRA_VERTEXSET_HPP_
#define COBRA_INCLUDE_COBRA_VERTEXSET_HPP_

#include <vector>
#include <cmath>
namespace cobra {

class VertexSet {

 public:

    VertexSet(unsigned int entries_num) {

        data.resize(entries_num);

    }

    VertexSet(const VertexSet &other) {
        data = other.data;
    }

    VertexSet &operator=(const VertexSet &other) {
        data = other.data;
        return *this;
    }

    inline void insert(int vertex) {
        const auto already_here = contains(vertex);
        if(!already_here) {
            insert_without_checking_existance(vertex);
        }
    }

    inline void insert_without_checking_existance(int vertex) {
        data[vertex] = true;
        vertices.push_back(vertex);
    }

    inline bool contains(int vertex) {
        return data[vertex];
    }

    void clear() {
        for(auto vertex : vertices) {
            data[vertex] = false;
        }
        vertices.clear();
    }

    const std::vector<int>& get_vertices() const {
        return vertices;
    }

    unsigned int size() const {
        return vertices.size();
    }

 private:

    std::vector<bool> data;
    std::vector<int> vertices;

};

}

#endif //COBRA_INCLUDE_COBRA_VERTEXSET_HPP_