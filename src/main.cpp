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


// Creating a shortcut for int, int pair type
typedef std::pair<int, int> Pair;

// typedef std::pair<std::vector<std::vector<Node> >, std::vector<int> > APair;




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


bool goodForAntenna(std::vector<std::vector<Node> > routeMap, int image_width, int image_height, int x, int y) {
    if (!routeMap[x][y].shadowed) {
        if (x + 1 < image_height && x - 1 >= 0) {
            if (routeMap[x+1][y].shadowed || routeMap[x-1][y].shadowed) {//Either top or bottom are shadowed
                return true;
            }
        }
    }
    return false;
}

std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > findAntennasHeight(std::vector<std::vector<Node> > routeMap, int startingHeight, int endingHeight, int startingWidth, int image_width, int image_height, int numAntennas) {
    std::vector<std::vector<std::pair<int,int> > > antennaList(numAntennas, std::vector<std::pair<int, int> >(endingHeight - startingHeight));
    std::vector<int> counts(numAntennas);
    int sepBetAnt = image_width/numAntennas;
    int count = 0;
    int currRow = 0;
    int sCol = startingWidth;
    int subCurr = 0;
    while (count < numAntennas) {
        printf("80 %d %d %d \n", count, sCol, sepBetAnt);
        int currCounts = 0;
        if (sCol < startingWidth || subCurr >= sepBetAnt) {
            std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
            return r;
        }
        for (int i = startingHeight; i < endingHeight; i++) {
            // printf("87 %d %d %d %d \n", i, sCol, routeMap[i][sCol].x, routeMap[i][sCol].y);
            if (goodForAntenna(routeMap, image_width, image_height, i, sCol)) {
                Node n = routeMap[i][sCol];
                // printf("89 %d %d \n", i, sCol);
                antennaList[count][currCounts].first = i;
                antennaList[count][currCounts].second = sCol;
                // printf("93 %d %d %d \n", count, antennaList[count][currCounts].first, antennaList[count][currCounts].second);
                currCounts++;
            }
        }
        if (currCounts > 0) {
            // printf("97 %d %d \n", i, sCol);
            counts[count] = currCounts;
            count++;
            currRow++;
            sCol += sepBetAnt;
            sCol = std::min(sCol, image_width);
            // printf("104 %d \n", sCol);
        } else {
            // printf("105 \n");
            subCurr++;
            sCol--;
        }
        // for (int i = 0; i < numAntennas; i++) {
        // for (int j = 0; j < counts[i]; j++) {
        //     printf("111 row %d x = %d y = %d \n", i, antennaList[i][j].first, antennaList[i][j].second);
        // }
    
    }
    // for (int i = 0; i < numAntennas; i++) {
    //     for (int j = 0; j < counts[i]; j++) {
    //         printf("111 row %d x = %d y = %d \n", i, antennaList[i][j].first, antennaList[i][j].second);
    //     }
    // }
    std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > r = make_pair(antennaList, counts);
    return r;
}

// Node findAntennaHeight(std::vector<std::vector<Node> > routeMap, int startingHeight, int endingHeight, int startingWidth, int image_width, int image_height, Node *unaccept, int unaccLen) {
//     Node possible[image_width];
//     for (int j = startingWidth; j < image_width; j++) { 
//         int count = 0;
//         for (int i = startingHeight; i < endingHeight; i++) {
//             if (goodForAntenna(routeMap, image_width, image_height, i, j, unaccept, unaccLen)) {
//                 possible[count] = routeMap[i][j];
//                 count++;
//             }
//         }

//         if (count > 0) {
//             return routeMap[count/2][j]; //Return midmost array
//         }
//     }
//     Node n;
//     n.x = -1;
//     n.y = -1;
//     return n;
// }


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
    printf("\n");
    return;
}


bool notBlocked(std::vector<std::vector<Node> > routeMap, int x, int y) {
    return (routeMap[x][y].blocked == false);
}

int countPath(std::vector<std::vector<Node> > routeMap, Node dest, int start_x, int start_y)
{
    int row = dest.x;
    int col = dest.y;
    // printf("176 CountPath %d %d \n", row, col);
    // stack<Pair> Path;
    int i = 0;
    // printf("179 %d \n", routeMap[row][col].parent);
    // while (!((*routeMap[row][col].parent).x == start_x && (*routeMap[row][col].parent).y == start_y)) { //The source parent points to itself
    while (!(row == start_x && col == start_y)) {    
        // Pair p = make_pair(row,col);
        // Path.push(p);
        // printf("182 %d %d %d %d \n", row, col, start_x, start_y);
        int temp_row = (*routeMap[row][col].parent).x;
        int temp_col = (*routeMap[row][col].parent).y;
        row = temp_row;
        col = temp_col;
        i++;
    }
    // printf("188 Returning from count path\n", row, col);
    return i;
}

std::pair<int, std::vector<std::vector<Node> > > doAStar(std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier,  
            std::vector<std::vector<Node> > routeMap, std::vector<std::vector<bool> > possiblePath,
            std::unordered_map<int, Node> came_from, std::unordered_map<int, float> cost_so_far, int image_width, int image_height, int start_x, int start_y, int goal_x, int goal_y) {
    // printf("96 \n");
    Node startNode = frontier.top();
    // printf("AStar 191 %d %d %d %d \n", startNode.x, startNode.y, goal_x, goal_y);
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
                            Node n = routeMap[x+dx][y+dy];
                            n.cost = newCost;
                            n.heuristic = newHeur;
                            n.x = x + dx;
                            n.y = y + dy;
                            routeMap[x + dx][y + dy] = n;
                            routeMap[x + dx][y + dy].parent = &(routeMap[x][y]);
                            // printf("HEREHERE %d %d %d %d %d %d \n", x + dx, x+dy, routeMap[x + dx][y + dy].x, routeMap[x][y],  routeMap[x][y].x,  routeMap[x][y].y);
                            // printPath(routeMap, routeMap[x + dx][y + dy]);
                            return std::make_pair(countPath(routeMap, routeMap[x + dx][y + dy], start_x, start_y), routeMap);
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
    // printf("Can't get a path under those conditions %d %d %d %d \n", startNode.x, startNode.y, goal_x, goal_y);
    return std::make_pair(-1, routeMap);
}

std::vector<std::vector<Node> > initializeMapVert(int grid[5058][5058], int image_height, int image_width, int startingHeight, int endingHeight) {
    std::vector<std::vector<Node> > routeMap(image_height, std::vector<Node>(image_width));
    // printf("244 %d %d \n", startingHeight, endingHeight);
    for (int i = startingHeight; i < endingHeight; i++) {
        for (int j = 0; j < image_width; j++) {
            routeMap[i][j].cost = -1;
            routeMap[i][j].heuristic = -1;
            routeMap[i][j].parent = NULL;
            // routeMap[i][j].x = i;
            // routeMap[i][j].y = j;
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

std::stack<Node> makePath(std::vector<std::vector<Node> > routeMap, Node dest) {
    std::stack<Node> path;
    int row = dest.x;
    int col = dest.y;
    // printf("176 CountPath %d %d \n", row, col);
    // stack<Pair> Path;
    int i = 0;
    // printf("179 %d \n", routeMap[row][col].parent);
    while (!((*routeMap[row][col].parent).x == row && (*routeMap[row][col].parent).y == col)) { //The source parent points to itself
        // Pair p = make_pair(row,col);
        // Path.push(p);
        // printf("182 %d \n", i);
        int temp_row = (*routeMap[row][col].parent).x;
        int temp_col = (*routeMap[row][col].parent).y;
        path.push(routeMap[row][col]);
        row = temp_row;
        col = temp_col;
        
    }
    // printf("188 Returning from count path\n", row, col);
    return path;
}


std::stack<Node> getAStarPath(std::vector<std::vector<Node> > routeMap, std::vector<std::pair<int, int> > destinations, int numAntennas, int image_width, int image_height) {
    // std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
    std::stack<Node> path;
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
        // printf("336 %d %d %d %d\n", start_x, start_y, goal_x, goal_y);
        bool keepGoing = true;
        while (!frontier.empty() && keepGoing) {
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
                    if ((dx == 0 || dy == 0) && keepGoing) { //Don't allow diagonals now
                        if (validDirection(x, y, dx, dy, image_width, image_height)) {
                            // printf("112 \n");
                            if (x + dx == goal_x && y + dy == goal_y) {
                                // printf("Found goal %d %d \n", x + dx, x + dy);
                                Node n = routeMap[x+dx][y+dy];
                                n.cost = newCost;
                                n.heuristic = newHeur;
                                n.x = x + dx;
                                n.y = y + dy;
                                routeMap[x + dx][y + dy] = n;
                                routeMap[x + dx][y + dy].parent = &(routeMap[x][y]);
                                // printf("HEREHERE %d %d %d %d %d %d \n", x + dx, x+dy, routeMap[x + dx][y + dy].x, routeMap[x][y],  routeMap[x][y].x,  routeMap[x][y].y);
                                // printPath(routeMap, routeMap[x + dx][y + dy]);
                                std::stack<Node> tempPath = makePath(routeMap, routeMap[x + dx][y + dy]);
                                while (!tempPath.empty()) {
                                    Node n = tempPath.top();
                                    tempPath.pop();
                                    path.push(n);
                                }
                                keepGoing = false;
                                // return std::make_pair(countPath(routeMap, routeMap[x + dx][y + dy]), routeMap);
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
    }
    return path;
}


int main(int argc, char** argv) {
    MPI_Init(&argc, &argv);

    // printf("143 \n");
    int world_size;
    int world_rank;
    // int image_width, image_height;
    // std::vector<std::vector<int> > moonMap(image_height, std::vector<Node>(image_width));
    int starting_x = 0;

    // image_height = 9;
    // image_width = 10;
    const int image_height = 5058;
    const int image_width = 5058;

    const int numAntennas = 4;

    bool doVert = true;
    
    //Get type of mode

    if (argc >= 2) {
        for (int i = 0; i < argc; i++) {
            if (strcmp(argv[i],"b") == 0) { 
                doVert = false;
            }
        }
    }

    MPI_Comm_size(MPI_COMM_WORLD, &world_size);
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

    //Initialize params based on mode

    int widthPerProc = image_width/world_size;
    int heightPerProc = image_height/world_size;

    if (widthPerProc * world_size < image_width) { //Probably want to change in future.
        widthPerProc += 1;
    }
    if (heightPerProc * world_size < image_height) { //Probably want to change in future.
        heightPerProc += 1;
    }

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
    //     // printf("%d %d %d %d %d \n", i, grid[i][0], grid[i][1], grid[i][2], grid[i][3]);
    // }

    // int startingWidth = widthPerProc * world_rank;
    int startingWidth = 0;
    int endingWidth = image_width;
    // int endingWidth = widthPerProc * (world_rank + 1);
    int startingHeight = heightPerProc * world_rank;
    int endingHeight = heightPerProc * (world_rank + 1);
    int distanceBetweenAntennas = image_width/numAntennas;
    startingHeight = std::min(startingHeight, image_height);
    endingHeight = std::min(endingHeight, image_height);
    std::chrono::high_resolution_clock::time_point startTime;
    std::chrono::high_resolution_clock::time_point endTime;
    if (world_rank == 0) {
        startTime = std::chrono::high_resolution_clock::now();
    }
    // printf("313 %d %d %d %d \n", world_rank, heightPerProc, startingHeight, endingHeight);

    if (doVert) { //to be replaced with separating based on height
        startingWidth = 0;
    }
    

    
    std::unordered_map<int, Node> came_from;
    std::unordered_map<int, float> cost_so_far;
    // std::vector<std::vector<Node> > routeMap(image_height, std::vector<Node>(image_width));
    std::vector<std::vector<bool> > preventedPaths;


   
    // int grid[image_height][image_width] = { { 1 }};
    std::vector<std::vector<Node> > routeMap; 
    
    //If parallel
    int minValuePath = INT_MAX;
    Node minStartingPath;
    std::stack<Node> finalPath;
    int localMinCount = -1;
    if (doVert) {
        if (world_size != 0) {
            //First initialize map and find antenna locations
            printf("%d Rank is starting \n", world_rank);
            routeMap = initializeMapVert(grid, image_height, image_width, startingHeight, endingHeight);
            printf("%d Rank has finished initialization \n", world_rank);
            std::pair<std::vector<std::vector<std::pair<int,int> > >, std::vector<int> > antennaPair = findAntennasHeight(routeMap, startingHeight, endingHeight, startingWidth, image_width, image_height, numAntennas);
            printf("%d Rank has found antennas \n", world_rank);
            std::vector<std::vector<std::pair<int,int> > > antennaList = antennaPair.first;
            std::vector<int> counts = antennaPair.second;

            Node minNode;
            // Below is for printing antennas for debugging

            // for (int i = 0; i < numAntennas; i++) {
            //     printf("Rank %d Antenna %d Count %d \n", world_rank, i, counts[i]);
            //     for (int j = 0; j < counts[i]; j++) {
            //         printf("Antenna %d count %d has x = %d y = %d \n", i, j, antennaList[i][j].first, antennaList[i][j].second);
            //     }
            // }
            int dest = 0;
            int start = 0;
            int minTotalStartingCount = INT_MAX;
            std::vector<std::pair<int, int> > totalMinAntennas(numAntennas + 1);
            Node minStartingNode;

            //Find the antenna locations with the minimum path
            for (int si = 0; si < counts[0]; si++) { //Vary the starting row
                printf("558 %d %d \n", world_rank, si);
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
                while (dest < numAntennas && keepGoing) { //Greedy algo to find the minimum route for a given starting node
                    dest++;
                    int min = INT_MAX;
                    Node minDest;
                    for (int i = 0; i < counts[dest]; i++) { //Check each potential antenna placement at a given site
                        // printf("577 %d %d \n", i, dest);
                        std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
                        // routeMap[sourceNode.x][sourceNode.y].parent = &(routeMap[sourceNode.x][sourceNode.y]);
                        // sourceNode.parent = &(sourceNode);
                        frontier.push(sourceNode);
                        int dest_x = antennaList[dest][i].first;
                        int dest_y = antennaList[dest][i].second;
                        // printf("582 %d %d \n", dest_x, dest_y);
                        std::pair<int, std::vector<std::vector<Node> > > AStarRes = doAStar(frontier, routeMap, preventedPaths, came_from, cost_so_far, image_width, image_height, sourceNode.x, sourceNode.y, dest_x, dest_y);
                        int c = AStarRes.first;
                        routeMap = AStarRes.second;
                        // printf("586 %d %d %d \n", dest_x, dest_y, c);
                        if (c > 0 && (c < min)) { //Check if min across different destinations
                            minDest = routeMap[dest_x][dest_y];
                            min = c;
                            minAntennasPerStart[dest] = std::make_pair(dest_x, dest_y);
                            if (dest == numAntennas - 1) {
                                routeMap[dest_x][dest_y].parent = &(routeMap[dest_x][dest_y]);
                                minFinDest = routeMap[dest_x][dest_y];
                            }
                        }
                        // printf("593 %d \n", min);
                    }
                    if (min != INT_MAX) { //Get min 
                        countPerStartingNode += min;
                        sourceNode = minDest;
                    } else {
                        keepGoing = false;
                    }
                    
                }
                // printf("601 \n");

                if (countPerStartingNode > 0) { //Get distance from last node to the final one.
                    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > frontier;
                    frontier.push(minFinDest);
                    // printf("608 \n");
                    std::pair<int, std::vector<std::vector<Node> > > AStarRes = doAStar(frontier, routeMap, preventedPaths, came_from, cost_so_far, image_width, image_height, minFinDest.x, minFinDest.y, startingNode.x, image_width - 1 );
                    // printf("609 \n");
                    int final_count = AStarRes.first;
                    routeMap = AStarRes.second;
                    // printf("619 %d %d %d %d %d\n", final_count, minFinDest.x, minFinDest.y, startingNode.x, image_width - 1);
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
                // printf("621 \n");
            }

            //Now get the path 
            minValuePath = minTotalStartingCount;
            minStartingPath = minStartingNode;
            if (minValuePath == -1) {
                minValuePath = INT_MAX;
            }
            totalMinAntennas[numAntennas] = make_pair(totalMinAntennas[0].first, image_width - 1);

            // printf("The antenna locations for rank %d are: ", world_rank);
            // for (int i = 0; i < numAntennas; i++) {
            //     printf("(%d, %d), ", totalMinAntennas[i].first, totalMinAntennas[i].second);
            // }
            // printf("\n");
            
            routeMap = initializeMapVert(grid, image_height, image_width, startingHeight, endingHeight);
            std::stack<Node> path = getAStarPath(routeMap, totalMinAntennas, numAntennas, image_width, image_height);
            
            
            //Now orient path in correct order and print if you so desire
            // printf("%d The path is: ", world_rank);
            while (!path.empty()) {
                Node n = path.top();
                path.pop();
                // printf("(%d, %d)-> ", n.x, n.y);
                finalPath.push(n);
                localMinCount++;
            }
            // printf("\n");
            // printf("%d Which has a length of %d \n", world_rank, minValuePath);

        }
    }
    
    // Your code here
    MPI_Barrier(MPI_COMM_WORLD);
    // printf("502 %d \n", world_rank);
    int procres[2];
    if (minValuePath == INT_MAX) { //if didn't find a min
        localMinCount = INT_MAX;
    }
    // printf("700 %d %d \n", world_rank, localMinCount);
    procres[0] = localMinCount;
    procres[1] = world_rank;
    int globalres[2];
    MPI_Allreduce(procres, globalres, 1, MPI_2INT, MPI_MINLOC, MPI_COMM_WORLD);
    
    // printf("509 %d \n", world_rank);
    if (world_rank == globalres[1]) {
        // printf("511 %d %d %d \n", world_rank, image_width - 1, minStartingPath.x);
        // printf("533 %d %d %d \n", routeMap[minStartingPath.x][image_width-1].x, minStartingPath.y, globalres[0]);
        
        printf("The total minimum path was: ");
        // while (!finalPath.empty()) {
        //     Node n = finalPath.top();
        //     finalPath.pop();
        //     printf("(%d, %d)-> ", n.x, n.y);
        // }
        printf("\n with Cost: %d", localMinCount);
        printf("\n");
        // printPath(routeMap, routeMap[minStartingPath.x][image_width-1]);
        // printf("513 %d \n", world_rank);
    }
    if (world_rank == 0) {
        endTime = std::chrono::high_resolution_clock::now();   
        std::chrono::duration<double, std::milli> s = endTime - startTime;
        printf("Ran in %.f milliseconds \n", s.count());
       
    }

    MPI_Finalize();
    return 0;
}