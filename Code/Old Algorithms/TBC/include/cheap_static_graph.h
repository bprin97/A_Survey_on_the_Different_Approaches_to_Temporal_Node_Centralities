#ifndef STATIC_GRAPH_H
#define STATIC_GRAPH_H

// Libraries:
// standard C++
#include <algorithm>
#include <assert.h>
#include <map>
#include <set>
#include <vector>

// TGLib
#include "tglib_cpp/src/core/TRSGraph.h"

class StaticGraph{
    public:

        StaticGraph(){
            size = 0;
            nodes = nullptr;
            adj_matrix = nullptr;
        }

        StaticGraph(int number_nodes) : size(number_nodes){
            assert(number_nodes > 0);
            createVoidGraph();            
        }

        void addEdge(int tail, int head){
            assert(size > 0);
            assert(tail < size);
            assert(head < size);
            // Insert the nodes into the node set
            nodes[tail] = true;
            nodes[head] = true;
            // Add the edge to the graph
            adj_matrix[tail][head] = true;
        }

        void removeNode(int node){
            assert(size > 0);
            assert(node < size);
            nodes[node] = false;
            removeEdges(node);
            removePredecessors(node);
        }

        void removeEdges(int node){
            assert(node < size);
            std::fill_n(adj_matrix[node], size, false);
        }

        void removePredecessors(int node){
            assert(node < size);
            for(int i = 0; i < size; i++){
                adj_matrix[i][node] = false;
            }
        }

        void clearGraph(){
            if(size < 1)    
                return;
            std::fill_n(nodes, size, false);
            for(int i = 0 ; i < size; i++){
                removeEdges(i);
            }
        }

        void setNewVoidGraph(int number_nodes){
            assert(number_nodes > 0);
            if(size != 0){
                delete[] nodes;
                for(int i = 0 ; i < size; i++){
                    delete[] adj_matrix[i];
                }
                delete[] adj_matrix;
            }

            size = number_nodes;

            createVoidGraph();
        }

        int getMaxN(){
            return size;
        }

        char* getNodes(){
            return nodes;
        }

        const char* getEdges(int node){
            assert(node < size);
            return adj_matrix[node];
        }

        std::vector<char> getTranspose(int node){
            assert(node < size);
            std::vector<char> transpose(size);
            for(int i = 0; i < size; i++){
                transpose[i] = adj_matrix[i][node];
            }
            return transpose;
        }

    private:

        void createVoidGraph(){
            nodes = new char[size];
            std::fill_n(nodes, size, false);
            adj_matrix = new char*[size];
            for(int i = 0 ; i < size; i++){
                adj_matrix[i] = new char[size];
                std::fill_n(adj_matrix[i], size, false);
            }
        }

        int size;

        char* nodes;
        char** adj_matrix;
};

#endif // STATIC_GRAPH_H