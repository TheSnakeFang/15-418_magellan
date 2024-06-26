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
#include <ctime>
#include <chrono>
#include "shadow_map.h"
using namespace std;

#include <unordered_map>

// Shortcut
typedef std::pair<int, int> Pair;

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

//Returns if it is a valid direction for the A* algorithm
bool validDirection(int x, int y, int dx, int dy, int image_width, int image_height, int startingHeight, int endingHeight) {
    if (x + dx < startingHeight) {
        return false;
    } else if (x + dx >= endingHeight) {
        return false;
    }
    if (y + dy < 0) {
        return false;
    } else if (y + dy >= image_width) {
        return false;
    }
    return true;
}


//Returns if a location on the grid is good for an antenna
bool goodForAntenna(int grid[5058][5058], std::vector<std::vector<Node> > routeMap, int image_width, int image_height, int x, int y) {
    if (routeMap[x][y].shadowed == false) {
        if (x + 1 < image_height && x - 1 >= 0) {
            if (routeMap[x+1][y].shadowed == true || routeMap[x-1][y].shadowed == true) {//Either top or bottom are shadowed
                return true;
            }
        }
    }
    return false;
}

//Old code for finding antenna heights, left here for testing purposes
std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > findAntennasHeight(int grid[5058][5058], std::vector<std::vector<Node> > routeMap, int startingHeight, int endingHeight, int startingWidth, int image_width, int image_height, int numAntennas) {
    std::vector<std::vector<std::pair<int,int> > > antennaList(numAntennas, std::vector<std::pair<int, int> >(endingHeight - startingHeight));
    std::vector<int> counts(numAntennas);
    int sepBetAnt = image_width/numAntennas;
    int count = 0;
    int currRow = 0;
    int sCol = startingWidth;
    int subCurr = 0;
    while (count < numAntennas) {
        int currCounts = 0;
        if (sCol > image_width || (sCol < startingWidth || subCurr >= sepBetAnt)) {
            std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
            return r;
        }
        for (int i = startingHeight; i < endingHeight; i++) {
            if (goodForAntenna(grid, routeMap, image_width, image_height, i, sCol)) {
                antennaList[count][currCounts].first = i;
                antennaList[count][currCounts].second = sCol;
                currCounts++;
            }
        }
        if (currCounts > 0) {
            counts[count] = currCounts;
            count++;
            currRow++;
            sCol += sepBetAnt;
            sCol = std::min(sCol, image_width);
        } else {
            subCurr++;
            sCol++;
        }
    }
    std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
    return r;
}

//Finds the antenna heights to better balance workload, ensures that every thread looks through at most 30 columns
std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > findAntennasHeightNew(int grid[5058][5058], std::vector<std::vector<Node> > routeMap, int startingHeight, int endingHeight, int startingWidth, int image_width, int image_height, int numAntennas) {
    int maxColCount = 30;
    std::vector<std::vector<std::pair<int,int> > > antennaList(numAntennas, std::vector<std::pair<int, int> >(maxColCount *(endingHeight - startingHeight)));
    std::vector<int> counts(numAntennas);
    int sepBetAnt = image_width/numAntennas;
    int count = 0;
    int currRow = 0;
    int sCol = startingWidth;
    int subCurr = 0;
    int colCount = 0;
    
    int currCounts = 0;
    int countCounts = 0;
    while (count < numAntennas && sCol < image_width) {
        if (sCol > image_width || (sCol < startingWidth)) {
            std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
            return r;
        }
        for (int i = startingHeight; i < endingHeight; i++) {
            if (goodForAntenna(grid, routeMap, image_width, image_height, i, sCol)) {
                antennaList[count][countCounts + currCounts].first = i;
                antennaList[count][countCounts + currCounts].second = sCol;
                currCounts++;
            }
        }

        if (colCount >= maxColCount) {
            startingWidth += sepBetAnt;
            sCol = startingWidth;
            sCol = std::min(sCol, image_width);
            counts[count] = countCounts;
            
            countCounts = 0;
            colCount = 0;
            currCounts = 0;
            count++;
        }
        else if (currCounts > 0) {
            countCounts += currCounts;
            sCol += 1;
            colCount += 1;
            currCounts = 0;
        } else {
            colCount++;
            subCurr++;
            sCol++;
        }
    }
    std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
    return r;
}

//Finds the antenna heights for the across columns approach
std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > findAntennasHeightAcrossWidth(int grid[5058][5058], std::vector<std::vector<Node> > routeMap, int startingWidth, int endingWidth, int image_width, int image_height, int numAntennasPerProc) {
    int maxColCount = 30;
    int maxNumAntennas = 20;
    int placedAntennas = 0;
    std::vector<std::vector<std::pair<int,int> > > antennaList(numAntennasPerProc, std::vector<std::pair<int, int> >(maxColCount*image_height));
    std::vector<int> counts(numAntennasPerProc);
    int sepBetAnt = (endingWidth - startingWidth)/numAntennasPerProc;
    int count = 0;
    int currRow = 0;
    int sCol = startingWidth;
    int subCurr = 0;
    int colCount = 0;
    
    int currCounts = 0;
    int countCounts = 0;
    while (count < numAntennasPerProc && sCol < image_width) {
        if (sCol > image_width || (sCol < startingWidth)) {
            std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
            return r;
        }
        for (int i = 0; i < image_height; i++) {         
            if (goodForAntenna(grid, routeMap, image_width, image_height, i, sCol)) {
                antennaList[count][countCounts + currCounts].first = i;
                antennaList[count][countCounts + currCounts].second = sCol;
                placedAntennas++;
                currCounts++;
            }
        }

        if (colCount >= maxColCount) {
            startingWidth += sepBetAnt;
            sCol = startingWidth;
            sCol = std::min(sCol, image_width);
            counts[count] = countCounts;
            placedAntennas = 0;
            countCounts = 0;
            colCount = 0;
            currCounts = 0;
            count++;
        }
        else if (currCounts > 0) {

            countCounts += currCounts;
            sCol += 1;
            colCount += 1;
            currCounts = 0;
        } else {

            colCount++;
            subCurr++;
            sCol++;
        }

    }

    std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
    return r;
}


//Calculates the heuristic according to manhattan distance
int calcHeur(int x, int y, int goal_x, int goal_y) {
    return std::abs(goal_x - x) + std::abs(goal_y - y);
}


//Prints the path 
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
    printf("\n");
    return;
}


//Checks if a path is blocked
bool notBlocked(std::vector<std::vector<Node> > routeMap, int x, int y) {
    return (routeMap[x][y].blocked == false);
}

//Counts the path from the start node to the destination node
int countPath(std::vector<std::vector<Node> > routeMap, Node dest, int start_x, int start_y)
{
    int row = dest.x;
    int col = dest.y;

    int i = 0;
    while (!(row == start_x && col == start_y)) {    
        if (routeMap[row][col].parent == NULL) {
            return i;
        }
        int temp_row = (*routeMap[row][col].parent).x;
        int temp_col = (*routeMap[row][col].parent).y;
        row = temp_row;
        col = temp_col;
        i++;
    }
    return i;
}

//Runs the typical A* algorithm, returning the cost of the path and the updated routeMap
std::pair<int, std::vector<std::vector<Node> > > doAStar(std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier,  
            std::vector<std::vector<Node> > routeMap, std::vector<std::vector<bool> > possiblePath,
            std::unordered_map<int, Node> came_from, std::unordered_map<int, float> cost_so_far, int image_width, int image_height, int startingHeight, int endingHeight, int start_x, int start_y, int goal_x, int goal_y) {
    Node startNode = frontier.top();
    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();
        
        // Check for goal
        int x = current.x;
        int y = current.y;


        int newHeur, newCost;
        // Expand to neighbors
        for (int dx = -1; dx < 2; dx++) { //Rows
            for (int dy = -1; dy < 2; dy++) { //Cols can only move forward
                if (dx == 0 || dy == 0) { //Don't allow diagonals now
                    if (validDirection(x, y, dx, dy, image_width, image_height, startingHeight, endingHeight)) {
                        if (x + dx == goal_x && y + dy == goal_y) {
                            Node n = routeMap[x+dx][y+dy];
                            n.cost = newCost;
                            n.heuristic = newHeur;
                            n.x = x + dx;
                            n.y = y + dy;
                            routeMap[x + dx][y + dy] = n;
                            routeMap[x + dx][y + dy].parent = &(routeMap[x][y]);
                            return std::make_pair(countPath(routeMap, routeMap[x + dx][y + dy], start_x, start_y), routeMap);
                        }
                        else if ((!possiblePath[x+dx][y+dy]) && notBlocked(routeMap, x+dx, y+dy)) {
                            newCost = routeMap[x][y].cost + 1;
                            newHeur = calcHeur(x + dx, y + dy, goal_x, goal_y);

                            if (routeMap[x + dx][y + dy].cost == -1 || routeMap[x + dx][y + dy].cost + routeMap[x + dx][y + dy].heuristic > newCost + newHeur) {
                                Node n = routeMap[x+dx][y+dy];
                                n.cost = newCost;
                                n.heuristic = newHeur;
                                n.x = x + dx;
                                n.y = y + dy;
                                n.parent = &(routeMap[x][y]);
                                routeMap[x + dx][y + dy] = n;
                                frontier.push(n);
                                possiblePath[x+dx][y+dy] = true;
                            }
                        }
                    }
                }
            }
        }
    }
    return std::make_pair(-1, routeMap);
}

//Initializes the map for across rows approach, only does from startingHeight to endingHeight
std::vector<std::vector<Node> > initializeMapVert(int grid[5058][5058], int image_height, int image_width, int startingHeight, int endingHeight) {
    std::vector<std::vector<Node> > routeMap(image_height, std::vector<Node>(image_width));
    for (int i = startingHeight; i < endingHeight; i++) {
        for (int j = 0; j < image_width; j++) {
            routeMap[i][j].cost = -1;
            routeMap[i][j].heuristic = -1;
            routeMap[i][j].parent = NULL;
            if (grid[i][j] == 0) {
                routeMap[i][j].blocked = false;
                routeMap[i][j].shadowed = false;
            } else {
                routeMap[i][j].blocked = true;
                routeMap[i][j].shadowed = true;
            }
            
        }
    }
    return routeMap;
}


//Generates a path from the destination to the start that was laid out by the A* algorithm
std::stack<Node> makePath(std::vector<std::vector<Node> > routeMap, Node dest) {
    std::stack<Node> path;
    int row = dest.x;
    int col = dest.y;
    int i = 0;
    while (!((*routeMap[row][col].parent).x == row && (*routeMap[row][col].parent).y == col)) { //The source parent points to itself
        int temp_row = (*routeMap[row][col].parent).x;
        int temp_col = (*routeMap[row][col].parent).y;
        path.push(routeMap[row][col]);
        row = temp_row;
        col = temp_col;
        
    }
    return path;
}

//Returns the path laid out by A* algorithm hitting each of the destinations in the form of a stack of vertices. 
std::stack<Node> getAStarPath(std::vector<std::vector<Node> > routeMap, std::vector<std::pair<int, int> > destinations, int numAntennas, int image_width, int image_height, int startingHeight, int endingHeight) {
    std::stack<Node> path;
    std::vector<std::vector<bool> > possiblePath(image_height, std::vector<bool>(image_width));
    int start_x = destinations[0].first;
    int start_y = destinations[0].second;
    Node start = routeMap[start_x][start_y];
    start.x = start_x;
    start.y = start.y;
    routeMap[start_x][start_y] = start;
    routeMap[start_x][start_y].parent = &(routeMap[start_x][start_y]);
    path.push(start);
    
    for (int dest = 0; dest < numAntennas; dest++) {
        start_x = destinations[dest].first;
        start_y = destinations[dest].second;
        
        start = routeMap[start_x][start_y];
        start.x = start_x;
        start.y = start.y;
        routeMap[start_x][start_y] = start;
        routeMap[start_x][start_y].parent = &(routeMap[start_x][start_y]);
        std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
        frontier.push(routeMap[start_x][start_y]);
        int goal_x = destinations[dest+1].first;
        int goal_y = destinations[dest+1].second;
        bool keepGoing = true;
        while (!frontier.empty() && keepGoing) {
            Node current = frontier.top();
            frontier.pop();
            
            // Check for goal
            int x = current.x;
            int y = current.y;

            int newHeur, newCost;
            // Expand to neighbors
            for (int dx = -1; dx < 2; dx++) { //Rows
                for (int dy = -1; dy < 2; dy++) { //Cols can only move forward
                    if ((dx == 0 || dy == 0) && keepGoing) { //Don't allow diagonals now
                        if (validDirection(x, y, dx, dy, image_width, image_height, startingHeight, endingHeight)) {
                            // printf("112 \n");
                            if (x + dx == goal_x && y + dy == goal_y) { //Found goal
                                Node n = routeMap[x+dx][y+dy];
                                n.cost = newCost;
                                n.heuristic = newHeur;
                                n.x = x + dx;
                                n.y = y + dy;
                                routeMap[x + dx][y + dy] = n;
                                routeMap[x + dx][y + dy].parent = &(routeMap[x][y]);
                                std::stack<Node> tempPath = makePath(routeMap, routeMap[x + dx][y + dy]);
                                while (!tempPath.empty()) {
                                    Node n = tempPath.top();
                                    tempPath.pop();
                                    path.push(n);
                                }
                                keepGoing = false;
                            }
                            else if ((!possiblePath[x+dx][y+dy]) && notBlocked(routeMap, x+dx, y+dy)) {
                                newCost = routeMap[x][y].cost + 1;
                                newHeur = calcHeur(x + dx, y + dy, goal_x, goal_y);
                                if (routeMap[x + dx][y + dy].cost == -1 || routeMap[x + dx][y + dy].cost + routeMap[x + dx][y + dy].heuristic > newCost + newHeur) {
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
                        }
                    }
                }
            }
        }
    }
    return path;
}




//Does A* algorithm, but to get to the corresponding y_goal, doesn't care about x_goal. 
std::pair<int, int> getAStarPathToNearestEdge(std::vector<std::vector<Node> > routeMap, int image_width, int image_height, int startingHeight, int endingHeight, int start_x, int start_y, int goal_y) {

    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
    std::vector<std::vector<bool> > possiblePath(image_height, std::vector<bool>(image_width));
    Node startNode = routeMap[start_x][start_y];
    startNode.x = start_x;
    startNode.y = start_y;
    startNode.parent = &startNode;
    routeMap[start_x][start_y] = startNode;
    routeMap[start_x][start_y].parent = &(routeMap[start_x][start_y]);
    frontier.push(startNode);

    while (!frontier.empty()) {
        Node current = frontier.top();
        frontier.pop();
    
        int x = current.x;
        int y = current.y;

        int newHeur, newCost;
        // Expand to neighbors
        for (int dx = -1; dx < 2; dx++) { //Rows
            for (int dy = -1; dy < 2; dy++) { //Cols
                if (dx == 0 || dy == 0) { //Don't allow diagonals now
                    if (validDirection(x, y, dx, dy, image_width, image_height, startingHeight, endingHeight)) {
                        // printf("112 \n");
                        if (y + dy == goal_y) { //Found goal
                            Node n = routeMap[x+dx][y+dy];
                            n.cost = newCost;
                            n.heuristic = newHeur;
                            n.x = x + dx;
                            n.y = y + dy;
                            routeMap[x + dx][y + dy] = n;
                            routeMap[x + dx][y + dy].parent = &(routeMap[x][y]);
                            return make_pair(countPath(routeMap, routeMap[x + dx][y + dy], start_x, start_y), x+dx);
                        }
                        else if ((!possiblePath[x+dx][y+dy]) && notBlocked(routeMap, x+dx, y+dy)) {
                            newCost = routeMap[x][y].cost + 1;
                            newHeur = calcHeur(x + dx, y + dy, x+dx, goal_y);
                            if (routeMap[x + dx][y + dy].cost == -1 || routeMap[x + dx][y + dy].cost + routeMap[x + dx][y + dy].heuristic > newCost + newHeur) {
                                Node n = routeMap[x+dx][y+dy];
                                n.cost = newCost;
                                n.heuristic = newHeur;
                                n.x = x + dx;
                                n.y = y + dy;
                                n.parent = &(routeMap[x][y]);
                                routeMap[x + dx][y + dy] = n;
                                frontier.push(n);
                                possiblePath[x+dx][y+dy] = true;
                            }
                        }
                    }
                }
            }
        }
    }
    return make_pair(-1, -1);
}

int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);
    int world_size;
    int world_rank;

    int starting_x = 0;

    const int image_height = 5058/16;
    const int image_width = 5058/16;

    const int numAntennas = 3;

    bool doVert = true; // Change to false to get horizontal parallelization
    
    // Get type of mode (Mostly ignored for now)
    if (argc >= 2) {
        for (int i = 0; i < argc; i++) {
            if (strcmp(argv[i],"b") == 0) { 
                doVert = false;
            }
        }
    }

    MPI_Comm_size(MPI_COMM_WORLD, &world_size);
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

    if (world_size == 1) {
        doVert = true;
    }

    //Initialize params based on mode
    int widthPerProc = image_width/world_size;
    int heightPerProc = image_height/world_size;

    if (widthPerProc * world_size < image_width) {
        widthPerProc += 1;
    }
    if (heightPerProc * world_size < image_height) { 
        heightPerProc += 1;
    }


    //Below are base cases to use, just don't include shadow.h
     // int grid[9][10]  
    //     = { { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
    //         { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
    //         { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
    //         { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
    //         { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
    //         { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
    //         { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
    //         { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
    //         { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 } };


    // int grid[9][10] = 
    //     { { 1, 0, 1, 1, 1, 1, 1, 1, 1, 1 },
    //         { 0, 1, 1, 1, 1, 1, 0, 1, 1, 1 },
    //         { 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 },
    //         { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
    //         { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
    //         { 1, 0, 1, 1, 1, 1, 1, 1, 1, 0 },
    //         { 1, 1, 1, 1, 0, 1, 1, 1, 0, 1 },
    //         { 0, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
    //         { 1, 1, 1, 0, 0, 1, 1, 1, 0, 1 } };
    //  int grid[image_height][image_width] = 
    //     { { 1 },
    //         { 0 },
    //         { 1 },
    //         { 0 },
    //         { 1 },
    //         { 0 },
    //         { 1},
    //         { 0 }};
    // for (int i = 0; i < image_height; i+= 2) {
    //     for (int j = 0; j < image_width; j++) {
    //         grid[i][j] = 1;
    //     }
    // }

    int startingWidth = widthPerProc * world_rank;
    if (doVert) {
        int startingWidth = 0;
    }
    int endingWidth = image_width;
    if (!doVert) {
        endingWidth = widthPerProc * (world_rank+1);
        endingWidth = std::min(endingWidth, image_width);
    }

    int startingHeight = heightPerProc * world_rank;
    int endingHeight = heightPerProc * (world_rank + 1);
    int distanceBetweenAntennas = image_width/numAntennas;
    startingHeight = std::min(startingHeight, image_height);
    endingHeight = std::min(endingHeight, image_height);
    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point initialTime;
    std::chrono::high_resolution_clock::time_point findAntennasTime;
    std::chrono::high_resolution_clock::time_point dataSearchTime;
    std::chrono::high_resolution_clock::time_point broadcastTime;
    std::chrono::high_resolution_clock::time_point endTime;
    std::chrono::duration<double, std::milli> spentInitializing;
    std::chrono::duration<double, std::milli> spentFindingAntennas;
    std::chrono::duration<double, std::milli> spendSearchingData;
    startTime = std::chrono::high_resolution_clock::now();
    

    if (doVert) { //Ensure start at 0
        startingWidth = 0;
    }
    

    int numAntennasPerProc = numAntennas / world_size;
    std::unordered_map<int, Node> came_from;
    std::unordered_map<int, float> cost_so_far;
    std::vector<std::vector<bool> > preventedPaths(image_height, std::vector<bool>(image_width));
    std::vector<std::vector<Node> > routeMap(image_height, std::vector<Node>(image_width)); 
    
    
    int minValuePath = INT_MAX;
    Node minStartingPath;
    std::stack<Node> finalPath;
    int localMinCount = -1;
    if (doVert) { //Doing across rows
        if (world_size != 0) {
            //First initialize map and find antenna locations
            routeMap = initializeMapVert(grid, image_height, image_width, startingHeight, endingHeight);
            initialTime = std::chrono::high_resolution_clock::now();   
            spentInitializing = initialTime - startTime;
            std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > antennaPair = findAntennasHeightNew(grid, routeMap, startingHeight, endingHeight, startingWidth, image_width, image_height, numAntennas);
            findAntennasTime = std::chrono::high_resolution_clock::now(); 
            spentFindingAntennas = findAntennasTime - initialTime;
            std::vector<std::vector<std::pair<int,int> > > antennaList = antennaPair.first;
            std::vector<int> counts = antennaPair.second;

            Node minNode;

            // Below is for printing antenna locations for images
            // for (int i = 0; i < numAntennas; i++) {
            //     printf("Rank %d Antenna %d Count %d \n", world_rank, i, counts[i]);
            //     // for (int j = 0; j < counts[i]; j++) {
            //     //     printf("Antenna %d count %d has x = %d y = %d \n", i, j, antennaList[i][j].first, antennaList[i][j].second);
            //     // }
            // }

            int dest = 0;
            int start = 0;
            int minTotalStartingCount = INT_MAX;
            std::vector<std::pair<int, int> > totalMinAntennas(numAntennas + 1);
            Node minStartingNode;

            //Find the antenna locations with the minimum path
            for (int si = 0; si < counts[0]; si++) { //Vary the starting row
                int startX = antennaList[0][si].first;
                int startY = antennaList[0][si].second;
                Node startingNode = routeMap[startX][startY];
                startingNode.x = startX;
                startingNode.y = startY;
                startingNode.parent = &(startingNode);
                routeMap[startX][startY] = startingNode;
                Node sourceNode = startingNode;
                std::vector<std::pair<int, int> > minAntennasPerStart(numAntennas + 1);
                int countPerStartingNode = 0;
                dest = 0;
                Node minFinDest;
                bool keepGoing = true;
                while (dest < numAntennas && keepGoing) { // Greedy algo to find the minimum route for a given starting node
                    dest++;
                    int min = INT_MAX;
                    Node minDest;
                    for (int i = 0; i < counts[dest]; i++) { // Check each potential antenna placement at a given site
                        std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
                        frontier.push(sourceNode);
                        int dest_x = antennaList[dest][i].first;
                        int dest_y = antennaList[dest][i].second;
                        std::pair<int, std::vector<std::vector<Node> > > AStarRes = doAStar(frontier, routeMap, preventedPaths, came_from, cost_so_far, image_width, image_height, startingHeight, endingHeight, sourceNode.x, sourceNode.y, dest_x, dest_y);
                        int c = AStarRes.first;
                        routeMap = AStarRes.second;
                        if (c > 0 && (c < min)) { // Check if min across different destinations
                            minDest = routeMap[dest_x][dest_y];
                            min = c;
                            minAntennasPerStart[dest] = std::make_pair(dest_x, dest_y);
                            if (dest == numAntennas - 1) {
                                routeMap[dest_x][dest_y].parent = &(routeMap[dest_x][dest_y]);
                                minFinDest = routeMap[dest_x][dest_y];
                            }
                        }
                    }
                    if (min != INT_MAX) { // Get min 
                        countPerStartingNode += min;
                        sourceNode = minDest;
                    } else {
                        keepGoing = false;
                    }
                    
                }

                if (countPerStartingNode > 0) { // Get distance from last node to the final one.
                    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
                    frontier.push(minFinDest);
                    std::pair<int, std::vector<std::vector<Node> > > AStarRes = doAStar(frontier, routeMap, preventedPaths, came_from, cost_so_far, image_width, image_height, startingHeight, endingHeight, minFinDest.x, minFinDest.y, startingNode.x, image_width - 1 );
                    int final_count = AStarRes.first;
                    routeMap = AStarRes.second;
                    if (final_count > 0) {
                        if (countPerStartingNode > 0 && (countPerStartingNode < minTotalStartingCount)) {
                            totalMinAntennas = minAntennasPerStart;
                            totalMinAntennas[0] = make_pair(startingNode.x, startingNode.y);
                            countPerStartingNode = final_count;
                            minTotalStartingCount = countPerStartingNode;
                            minStartingNode = startingNode;
                        }
                    }
                }
            }



            dataSearchTime = std::chrono::high_resolution_clock::now(); 
            spendSearchingData = dataSearchTime - findAntennasTime;

            // Now get the path 
            minValuePath = minTotalStartingCount;
            minStartingPath = minStartingNode;
            if (minValuePath == -1) {
                minValuePath = INT_MAX;
            }


            

            totalMinAntennas[numAntennas] = make_pair(totalMinAntennas[0].first, image_width - 1);

            
            routeMap = initializeMapVert(grid, image_height, image_width, startingHeight, endingHeight);
            std::stack<Node> path = getAStarPath(routeMap, totalMinAntennas, numAntennas, image_width, image_height, 0, image_height);
            
            
            // Now orient path in correct order and print if you so desire
            while (!path.empty()) {
                Node n = path.top();
                path.pop();
                finalPath.push(n);
                localMinCount++;
            }


        }
    }
    else { // Do parallelization across width
        routeMap = initializeMapVert(grid, image_height, image_width, 0, image_height);
        std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > antennaPair = findAntennasHeightAcrossWidth(grid, routeMap, startingWidth, endingWidth, image_width, image_height, numAntennasPerProc);
        std::vector<std::vector<std::pair<int,int> > > antennaList = antennaPair.first;
        std::vector<int> counts = antennaPair.second;
        std::vector<int> sendAntennas(counts[0]);
        int sendingCount = 0;
        for (int i = 0; i < counts[0]; i++) {
            std::pair<int, int> sending;
            if (world_rank != 0) {
                sending = getAStarPathToNearestEdge(routeMap, image_width, image_height, 0, image_height, antennaList[0][i].first, antennaList[0][i].second, startingWidth-1);
            } else {
                sending = getAStarPathToNearestEdge(routeMap, image_width, image_height, 0, image_height, antennaList[0][i].first, antennaList[0][i].second, 0);
            }
            if (sending.first != -1) {
                sendAntennas[sendingCount] = sending.second;
                sendingCount++;
            }
        }



        // Do the sendings and receivings of the initial 
        std::vector<int> destAntenna(image_width * image_height); 
        int destCount;
        MPI_Status status;
        
        if (world_rank == 0) { //Process 0 sends to last first
            MPI_Send(&sendingCount, 1, MPI_INT, world_size - 1, 0, MPI_COMM_WORLD);
            MPI_Send(&sendAntennas[0], sendingCount, MPI_INT, world_size-1, 0, MPI_COMM_WORLD);
            // printf("1040 %d \n", world_rank);
            MPI_Recv(&destCount, 1, MPI_INT, 1, 0, MPI_COMM_WORLD, &status);
            // printf("1042 %d %d\n", world_rank, destCount);
            MPI_Recv(&destAntenna[0], destCount, MPI_INT, 1, 0, MPI_COMM_WORLD, &status);
        } else if (world_rank == world_size - 1) { 
            MPI_Recv(&destCount, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, &status);
            MPI_Recv(&destAntenna[0], destCount, MPI_INT, 0, 0, MPI_COMM_WORLD, &status);
            // printf("1046 %d \n", world_rank);
            // printf("1048 %d %d\n", world_rank, destCount);
            MPI_Send(&sendingCount, 1, MPI_INT, world_rank-1, 0, MPI_COMM_WORLD);
            MPI_Send(&sendAntennas[0], sendingCount, MPI_INT, world_rank-1, 0, MPI_COMM_WORLD);

        } else if (world_rank % 2 == 0) { //Even ranks send back first
            MPI_Send(&sendingCount, 1, MPI_INT, world_rank - 1, 0, MPI_COMM_WORLD);
            MPI_Send(&sendAntennas[0], sendingCount, MPI_INT, world_rank-1, 0, MPI_COMM_WORLD);
            // printf("1053 %d \n", world_rank);
            MPI_Recv(&destCount, 1, MPI_INT, world_rank+1, 0, MPI_COMM_WORLD, &status);
            // printf("1057 %d %d\n", world_rank, destCount);
            MPI_Recv(&destAntenna[0], destCount, MPI_INT, world_rank+1, 0, MPI_COMM_WORLD, &status);
        } else { //odd ranks recieve first
            MPI_Recv(&destCount, 1, MPI_INT, world_rank + 1, 0, MPI_COMM_WORLD, &status);
            MPI_Recv(&destAntenna[0], destCount, MPI_INT, world_rank + 1, 0, MPI_COMM_WORLD, &status);
            // printf("1059 %d \n", world_rank);
            // printf("1063 %d %d\n", world_rank, destCount);
            MPI_Send(&sendingCount, 1, MPI_INT, world_rank-1, 0, MPI_COMM_WORLD);
            MPI_Send(&sendAntennas[0], sendingCount, MPI_INT, world_rank-1, 0, MPI_COMM_WORLD);
        }

        // printf("%d Rank has finished initial sends and recieves \n", world_rank);
        // Each element is the ((start_x,start_y), (end_x, end,y), cost)
        std::vector<std::pair<std::pair<std::pair<int, int>, std::pair<int,int> >, int> > minPaths(counts[0]);
        for (int i =0; i < counts[0]; i++) {
            minPaths[i].second = INT_MAX;
        }
        int dest = 0;
        int start = 0;
        int minTotalStartingCount = INT_MAX;
        std::vector<std::pair<int, int> > totalMinAntennas(numAntennas + 1);
        Node minStartingNode;
        // Find the antenna locations with the minimum path
        for (int si = 0; si < counts[0]; si++) { // Vary the starting row
            int startX = antennaList[0][si].first;
            int startY = antennaList[0][si].second;
            Node startingNode = routeMap[startX][startY];
            startingNode.x = startX;
            startingNode.y = startY;
            startingNode.parent = &(startingNode);
            routeMap[startX][startY] = startingNode;
            Node sourceNode = startingNode;
            std::vector<std::pair<int, int> > minAntennasPerStart(numAntennas + 1);
            int countPerStartingNode = 0;
            dest = 0;
            Node minFinDest;
            bool keepGoing = true;
            while (dest < numAntennasPerProc && keepGoing) { // Greedy algo to find the minimum route for a given starting node
                dest++;
                int min = INT_MAX;
                Node minDest;
                for (int i = 0; i < counts[dest]; i++) { // Check each potential antenna placement at a given site
                    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
                    frontier.push(sourceNode);
                    int dest_x = antennaList[dest][i].first;
                    int dest_y = antennaList[dest][i].second;
                    std::pair<int, std::vector<std::vector<Node> > > AStarRes = doAStar(frontier, routeMap, preventedPaths, came_from, cost_so_far, image_width, image_height, 0, image_height, sourceNode.x, sourceNode.y, dest_x, dest_y);
                    int c = AStarRes.first;
                    routeMap = AStarRes.second;

                    if (c > 0 && (c < min)) { // Check if min across different destinations
                        // printf("%d \n", world_rank);
                        minDest = routeMap[dest_x][dest_y];
                        min = c;
                        minAntennasPerStart[dest] = std::make_pair(dest_x, dest_y);
                        if (dest == numAntennas - 1) {
                            routeMap[dest_x][dest_y].parent = &(routeMap[dest_x][dest_y]);
                            minFinDest = routeMap[dest_x][dest_y];
                        }
                    }
                    // printf(" %d \n", min);
                }
                if (min != INT_MAX) { // Get min 
                    countPerStartingNode += min;
                    sourceNode = minDest;
                } else {
                    keepGoing = false;
                }
                
            }

            if (numAntennasPerProc < 2) {
                minFinDest = startingNode;
            }

            if (countPerStartingNode > 0 || numAntennasPerProc < 2) { // Get distance from last node to the final one.                
                int final_count = -1; 
                int minFinDestTot = -1;
                // printf("608 \n");
                for (int j = 0; j < destCount; j++) {
                    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
                    std::vector<std::vector<bool> > preventedPaths(image_height, std::vector<bool>(image_width));
                    frontier.push(minFinDest);
                    std::pair<int, std::vector<std::vector<Node> > > AStarRes = doAStar(frontier, routeMap, preventedPaths, came_from, cost_so_far, image_width, image_height, 0, image_height, minFinDest.x, minFinDest.y, destAntenna[j], endingWidth - 1); //Now need to iterate to each of the 
                    int minFinalCount = AStarRes.first;

                    routeMap = AStarRes.second;
                    if (minFinalCount > 0 && (final_count == -1 || minFinalCount < final_count)) {
                        minFinDestTot = j;
                        final_count = minFinalCount;
                    }
                }
                if (final_count > 0) {
                    minPaths[si].first.first.first = antennaList[0][si].first;
                    minPaths[si].first.first.second = antennaList[0][si].second;
                    minPaths[si].first.second.first = endingWidth;
                    minPaths[si].first.second.second = destAntenna[minFinDestTot];
                    minPaths[si].second = final_count;
                }
            }
        }

        // Find minimum path across antennas to destinations
        int numIters;
        if (world_rank == 0) {
            numIters = counts[0];
        }
        int minStart = -1;
        int minPathOverall = INT_MAX;
        int minForPath = INT_MAX;
        int recY;
        
        MPI_Barrier(MPI_COMM_WORLD);
        MPI_Bcast(&(numIters), 1, MPI_INT, 0, MPI_COMM_WORLD); // Get number of iterations
        for (int i = 0; i < numIters; i++) { 
            minForPath = INT_MAX;
            if (world_rank == 0) {
                MPI_Send(&(minPaths[i].first.second.second), 1, MPI_INT, 1, 0, MPI_COMM_WORLD);
                MPI_Send(&(minPaths[i].second), 1, MPI_INT, 1, 0, MPI_COMM_WORLD);
                MPI_Recv(&(minForPath), 1, MPI_INT, world_size- 1, 0, MPI_COMM_WORLD, &status);
                if (minForPath < minPathOverall) {
                    minStart = i;
                    minPathOverall = minForPath;
                }
            }
            else if (world_rank == world_size - 1) {
                MPI_Recv(&recY, 1, MPI_INT, world_rank - 1, 0, MPI_COMM_WORLD, &status);
                MPI_Recv(&minForPath, 1, MPI_INT, world_rank - 1,0, MPI_COMM_WORLD, &status);
                bool possible = true;
                for (int j = 0; j < counts[0]; j++) {
                    if (possible && minPaths[j].first.second.second == recY) {
                        minForPath += minPaths[j].second;
                        possible = false;
                    }
                }
                MPI_Send(&minForPath, 1, MPI_INT, 0, 0, MPI_COMM_WORLD);
            } else {
                MPI_Recv(&recY, 1, MPI_INT, world_rank - 1, 0, MPI_COMM_WORLD, &status);
                MPI_Recv(&minForPath, 1, MPI_INT, world_rank - 1,0, MPI_COMM_WORLD, &status);
                bool possible = true;
                int minDest = -1;
                for (int j = 0; j < counts[0]; j++) {
                    if (possible && minPaths[j].first.second.second == recY) {
                        minForPath += minPaths[j].second;
                        possible = false;
                        minDest = j;
                    }
                }
                MPI_Send(&minDest, 1, MPI_INT, world_rank + 1, 0, MPI_COMM_WORLD);
                MPI_Send(&minForPath, 1, MPI_INT, world_rank + 1, 0, MPI_COMM_WORLD);
            }
        }
        if (world_rank == 0) {
            printf("Found minimum path starting at (%d, %d) with cost %d \n", minPaths[minStart].first.first.first, minPaths[minStart].first.first.second, minPathOverall);
        }
    }
    
    MPI_Barrier(MPI_COMM_WORLD); // Make sure all threads are stopped


    if (doVert) {
        int procres[2];
        if (minValuePath == INT_MAX) { // if didn't find a min
            localMinCount = INT_MAX;
        }
        procres[0] = localMinCount;
        procres[1] = world_rank;
        int globalres[2];
        MPI_Allreduce(procres, globalres, 1, MPI_2INT, MPI_MINLOC, MPI_COMM_WORLD);
        MPI_Barrier(MPI_COMM_WORLD);
        if (world_rank == globalres[1]) {
            printf("The total minimum path was across %d and ", world_rank);
            // while (!finalPath.empty()) { //If need to print path (e.g. for graphit.py)
            //     Node n = finalPath.top();
            //     finalPath.pop();
            //     printf("(%d, %d)-> ", n.x, n.y);
            // }
            printf("\n with Cost: %d", localMinCount);
            printf("\n");
        }
    } 
    if (world_rank >= 0) {
        broadcastTime = std::chrono::high_resolution_clock::now();   
        std::chrono::duration<double, std::milli> spentBroadCasting = broadcastTime - dataSearchTime;
        endTime = std::chrono::high_resolution_clock::now();   
        std::chrono::duration<double, std::milli> s = endTime - startTime;
        printf("%d Ran in %.f milliseconds \n", world_rank, s.count());
        printf("%d Time spent %.f initializing \n", world_rank, spentInitializing.count());
        printf("%d Time spent %.f finding antennas \n", world_rank, spentFindingAntennas.count());
        printf("%d Time spent %.f searching for a path \n", world_rank, spendSearchingData.count());
       printf("%d Time spent %.f broadcasting \n", world_rank, spentBroadCasting.count());
    }

    MPI_Finalize();
    return 0;
}