/* Running From The Night:
Calculating The Lunar Magellan Route in Parallel
Authors: Kevin Fang (kevinfan) and Nikolai Stefanov (nstefano) */
#include <mpi.h>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>
#include <random>
#include <stack>
#include <queue>
using namespace std;


#include <unordered_map>


// Creating a shortcut for int, int pair type
typedef std::pair<int, int> Pair;
 

// Are we going to use Nodes? Squares? At which fidelity?
struct Node {
    int x, y; // Node coordinates
    float cost; // Cost to reach this node
    float heuristic; // Heuristic estimate to the goal
    Node* parent; // Pointer to parent node for path reconstruction
    bool blocked;
    bool shadowed;

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


bool goodForAntenna(std::vector<std::vector<Node> > routeMap, int image_width, int image_height, int x, int y, Node *Unacceptable, int unaccLen) {
    if (routeMap[x][y].shadowed) {
        if (x + 1 < image_height && x - 1 >= 0) {
            if (routeMap[x+1][y].shadowed || routeMap[x-1][y].shadowed) {//Either top or bottom are unshadowed
                return true;
            }
        }
    }
    return false;
}

Node findAntenna(std::vector<std::vector<Node> > routeMap, int image_width, int image_height, Node *unaccept, int unaccLen) {
    Node possible[image_width];
    for (int j = 0; j < image_width; j++) { 
        int count = 0;
        for (int i = 0; i < image_height; i++) {
            if (goodForAntenna(routeMap, image_width, image_height, i, j, unaccept, unaccLen)) {
                possible[count] = routeMap[i][j];
                count++;
            }
        }

        if (count > 0) {
            return routeMap[count/2][j]; //Return midmost array
        }
    }
    return routeMap[0][1];
}


int calcHeur(int x, int y, int goal_x, int goal_y) {
    return std::abs(goal_x - x) + std::abs(goal_y - y);
}


void printPath(std::vector<std::vector<Node> > routeMap, Node dest)
{
    printf("\nThe Path is ");
    int row = dest.x;
    int col = dest.y;
 
    stack<Pair> Path;
    int i = 0;
    while (!((*routeMap[row][col].parent).x == row
             && (*routeMap[row][col].parent).y == col)) { //The source parent points to itself
        Pair p = make_pair(row,col);
        Path.push(p);
        int temp_row = (*routeMap[row][col].parent).x;
        int temp_col = (*routeMap[row][col].parent).y;
        row = temp_row;
        col = temp_col;
        i++;
    }
 
    Path.push(std::make_pair(row, col));
    while (!Path.empty()) {
        std::pair<int, int> p = Path.top();
        Path.pop();
        printf("->(%d,%d)", p.first, p.second);
    }
 
    return;
}


bool notBlocked(std::vector<std::vector<Node> > routeMap, int x, int y) {
    return (routeMap[x][y].blocked == 0);
}

void doAStar(std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier,  
            std::vector<std::vector<Node> > routeMap, std::vector<std::vector<bool> > possiblePath,
            std::unordered_map<int, Node> came_from, std::unordered_map<int, float> cost_so_far, int image_width, int image_height, int goal_x, int goal_y) {
    // printf("96 \n");
    Node startNode = frontier.top();
    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();
        
        // Check for goal
        int x = current.x;
        int y = current.y;
        // printf("100 %d %d\n", x, y);
        // preventedPaths[x][y] = true;

        int newHeur, newCost;
        // Expand to neighbors
        for (int dx = -1; dx < 2; dx++) { //Rows
            for (int dy = -1; dy < 2; dy++) { //Cols can only move forward
                if (dx == 0 || dy == 0) { //Don't allow diagonals now
                    if (validDirection(x, y, dx, dy, image_width, image_height)) {
                        // printf("112 \n");
                        if (x + dx == goal_x && y + dy == goal_y) {
                            // printf("Found goal %d %d \n", x + dx, x + dy);
                            routeMap[x + dx][y + dy].parent = &(routeMap[x][y]);
                            printPath(routeMap, routeMap[x + dx][y + dy]);
                            return;
                        }
                        else if (notBlocked(routeMap, x+dx, y+dy)) {
                            // printf("120 \n");
                            newCost = routeMap[x][y].cost + 1;
                            newHeur = calcHeur(x + dx, y + dy, goal_x, goal_y);

                            if (routeMap[x + dx][y + dy].cost == -1 || routeMap[x + dx][y + dy].cost + routeMap[x + dx][y + dy].heuristic > newCost + newHeur) {
                                // printf("125 \n");
                                Node n = routeMap[x+dx][y+dy];
                                n.cost = newCost;
                                n.heuristic = newHeur;
                                n.x = x + dx;
                                n.y = y + dy;
                                n.parent = &(routeMap[x][y]);
                                routeMap[x + dx][y + dy] = n;
                                frontier.push(n);
                            }
                        }
                        // printf("136 \n");
                    }
                    // printf("138 \n");
                }
            }
        }
    }
    printf("Can't get a path under those conditions \n");
    return;
}

int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);
    // printf("143 \n");
    int world_size;
    int world_rank;
    int image_width, image_height;
    // std::vector<std::vector<int> > moonMap(image_height, std::vector<Node>(image_width));
    int starting_x = 0;

    image_height = 9;
    image_width = 10;



    int widthPerProc = image_width/world_size;

    if (widthPerProc * world_size < image_width) { //Probably want to change in future.
        widthPerProc += 1;
    }
    

    MPI_Comm_size(MPI_COMM_WORLD, &world_size);
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);


    int startingWidth = widthPerProc * world_rank;
    

    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
    std::unordered_map<int, Node> came_from;
    std::unordered_map<int, float> cost_so_far;
    std::vector<std::vector<Node> > routeMap(image_height, std::vector<Node>(image_width));
    std::vector<std::vector<bool> > preventedPaths;


    int grid[9][10]  
        = { { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
            { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
            { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 } };
    
    if (false) {//world_size != 0) {
        if (world_rank == 0) {
            // For simplicity, assume we're sending out initial nodes to each worker
            for (int i = 1; i < world_size; i++) {
                // Send initial nodes or sectors to each worker
                // MPI_Send(...);
            }
        } else {
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

        for (int i = 0; i < image_height; i++) {
            for (int j = 0; j < image_width; j++) {
                routeMap[i][j].cost = -1;
                routeMap[i][j].heuristic = -1;
                routeMap[i][j].parent = NULL;
                if (grid[i][j] == 0) {
                    routeMap[i][j].blocked = true;
                }
                //routeMap[i][j].parent_j = -1;
            }
        }
        

        // printf("217 \n");
        int sx = 8;
        //int goal_x = starting_x;
        //int goal_y = image_width;
        int goal_x = 0;
        int goal_y = 0;
        // int sx = starting_x; 
        int sy = 0;


        routeMap[sx][sy].x = sx;
        routeMap[sx][sy].y = sy;
        //printf("%d %d \n", routeMap[sx][sy].x, frontier.top().y);
        routeMap[sx][sy].cost = 0;
        routeMap[sx][sy].heuristic = 0;
        routeMap[sx][sy].parent = &(routeMap[sx][sy]);
        frontier.push(routeMap[sx][sy]);
        // void doAStar(frontier, std::vector<std::vector<bool> > possiblePath,
        //     std::unordered_map<int, Node> came_from, std::unordered_map<int, float> cost_so_far, int image_width, int image_height, int goal_x, int goal_y)
        // printf("%d %d \n", frontier.top().x, frontier.top().y);
        doAStar(frontier, routeMap, preventedPaths, came_from, cost_so_far, image_width, image_height, goal_x, goal_y);
        // Implement the A* search loop here
        // printf("234 \n");
            // Update frontier with new nodes
    }
    // Your code here

    MPI_Finalize();
    return 0;
}