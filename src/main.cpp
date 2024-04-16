/* Running From The Night:
Calculating The Lunar Magellan Route in Parallel
Authors: Kevin Fang (kevinfan) and Nikolai Stefanov (nstefano) */

#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>
#include <random>

#include <mpi.h>
#include <unordered_map>

// Are we going to use Nodes? Squares? At which fidelity?
struct Node {
    int x, y; // Node coordinates
    float cost; // Cost to reach this node
    float heuristic; // Heuristic estimate to the goal
    Node* parent; // Pointer to parent node for path reconstruction

    // Comparator for priority queue
    bool operator>(const Node& other) const {
        return cost + heuristic > other.cost + other.heuristic;
    }
};

bool validDirection(int x, int y, int dx, int dy, int image_width, int image_height) {
    if (x + dx < 0) {
        return false;
    } else if (x + dx >= image_height) {
        return false;
    }
    if (y + dy < 0) {
        return false;
    } else if (y + dy >= image_width) {
        return false;
    }
    return true;
}



void doAStar(std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier,  
            std::vector<std::vector<Node>> routeMap, std::unordered_map<int, Node> came_from, 
            std::unordered_map<int, float> cost_so_far, int image_width, int image_height, int goal_x, int goal_y) {
                return;
            }

int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);

    int world_size;
    int world_rank;
    int image_width, image_height;
    int starting_x;

    MPI_Comm_size(MPI_COMM_WORLD, &world_size);
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);
    if (world_size != 0) {
        if (world_rank == 0) {
            // For simplicity, assume we're sending out initial nodes to each worker
            for (int i = 1; i < world_size; i++) {
                // Send initial nodes or sectors to each worker
                // MPI_Send(...);
            }
        } else {
            // Worker processes
            std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier;
            std::unordered_map<int, Node> came_from;
            std::unordered_map<int, float> cost_so_far;
            std::vector<std::vector<Node>> routeMap(image_height, std::vector<Node>(image_width));
            
            int goal_x, goal_y;

            // Receive initial node or sector
            // MPI_Recv(...);

            int x, y; 
            // Implement the A* search loop here
            while (!frontier.empty()) {
                Node current = frontier.top();
                frontier.pop();
                
                // Check for goal

                // Expand to neighbors
                // Update frontier with new nodes
            }

            // Send results back to the master
            // MPI_Send(...);
        }
    } else {
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> frontier;
        std::unordered_map<int, Node> came_from;
        std::unordered_map<int, float> cost_so_far;
        Node routeMap[image_height][image_width];

        for (int i = 0; i < image_height; i++) {
            for (int j = 0; j < image_width; j++) {
                routeMap[i][j].cost = -1;
                routeMap[i][j].heuristic = -1;
                routeMap[i][j].parent_i = -1;
                routeMap[i][j].parent_j = -1;
            }
        }
        


        int goal_x = starting_x;
        int goal_y = image_width;
        int sx = starting_x; 
        int sy = 0;


        routeMap[sx][sy].x = sx;
        routeMap[sx][sy].x = sy;
        routeMap[sx][sy].cost = 0;
        routeMap[sx][sy].heuristic = 0;
        routeMap[sx][sy].parent = routeMap[sx][sy];
        frontier.push(routeMap[sx][sy])

        doAStar(frontier, routeMap, came_from, cost_so_far, image_width, image_height, goal_x, goal_y);
        // Implement the A* search loop here
        while (!frontier.empty()) {
            Node current = frontier.top();
            frontier.pop();
            
            // Check for goal
            int x = current.x;
            int y = current.y;

            int newHeur, newCost;
            // Expand to neighbors
            for (int dx = -1; dx < 2; dx++) {
                for (int dy = 0; dy < 2; dy++) {
                    if (dx == 0 || dy == 0) { //Don't allow diagonals now
                        if (validDirection(x, y, dx, dy, image_width, image_height)) {
                            if (x + dx == goal_x && y + dy == goal_y) {
                                printf("Found goal \n");
                                routeMap[x + dx][y + dy].parent = routeMap[x][y];
                                return 1;
                            }
                            else if ()
                        }
                    }
                }
            }

            // Update frontier with new nodes
    }
    // Your code here

    MPI_Finalize();
    return 0;
}