/*
 *  graph_solver.cpp
 *  PathFinder
 *
 *  Created by peter on 2/02/10.
 *
 *  http://compprog.wordpress.com/2007/12/01/one-source-shortest-path-dijkstras-algorithm/
 *  Based on http://snippets.dzone.com/posts/show/4832
 *
 */
 
#include <limits.h> 
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>
#include <iomanip>
#include <algorithm>

using namespace std; 

#define CHECK_LOGIC false

static const int WIDTH = 3;
static const string the_tile_fn = "table_tile_floor.txt";

static const int INFINITY = INT_MAX/2;

struct Graph {
    int  _number_edges;
    int  _number_nodes;
    vector<vector<int> > _dist;
    Graph(int number_edges);
};

Graph::Graph(int number_edges) {
    _number_nodes = -1;
    _number_edges = number_edges;
    _dist.resize(_number_edges);
    vector<vector<int> >::iterator it;
    for (it = _dist.begin(); it != _dist.end(); it++) {
        (*it).resize(_number_edges, 0);
    }
}

/*
 * Print a list of distances
 */
void printDistances(const vector<int> shortest_dist, int number_nodes) {
    for (int i = 1; i <= number_nodes; ++i)
        cout << setw(9) << i << " ";
    cout << endl;
    for (int i = 1; i <= number_nodes; ++i)
        cout << setw(9) << shortest_dist[i] << " ";
    cout << endl;
}

template <class T>
static void show(const vector<T> v, const string name) {
    cout << name << ": ";
    for (int i = 0; i < v.size(); i++)
        cout << v[i] << ", ";
    cout << endl;
}

/*
 *  Given a list of edge lengths dist[][] and a source node s, 
 *  return list of min distances from s to ith node 
 */
vector<int> dijkstra(int s, const Graph& graph) {
    int number_nodes = graph._number_nodes;
    vector<vector<int> > dist = graph._dist;
    vector<bool> visited(graph._number_edges + 1, false);
    vector<int> shortest_dist(graph._number_edges + 1, INFINITY);   // shortest_dist[i] = shortest distance from s to i
#if CHECK_LOGIC    
    show(visited, "visited");
    show(shortest_dist, "shortest_dist");
#endif
    shortest_dist[s] = 0;

    for (int k = 1; k <= number_nodes; ++k) {
        int mini = -1;                              
        for (int i = 1; i <= number_nodes; ++i) {
            if (!visited[i] && ((mini == -1) || (shortest_dist[i] < shortest_dist[mini]))) {
                mini = i;
            }
        }
        // mini is now closest non-visited node to s
        visited[mini] = true;                          
        // Replace any shortest paths sa-i that are shorter through s-mini-i
        for (int i = 1; i <= number_nodes; ++i) {
            if (dist[mini][i] > 0) {
                if (shortest_dist[mini] + dist[mini][i] < shortest_dist[i]) 
                    shortest_dist[i] = shortest_dist[mini] + dist[mini][i];
            }
        }
    }
#if CHECK_LOGIC    
    for (int i = 1; i <= number_nodes; ++i) {
        if (!visited[i]) 
            cerr << setw(2) << i << " not visited" << endl;
        if (shortest_dist[i] == INFINITY)
           cerr << "shortest_dist[" << setw(2) << i << "] = INFINITY" << endl;
        assert(shortest_dist[i] >= 0);
    }
#endif
    return shortest_dist;
}

struct Coord { int _x, _y; };


int coordToNodeNum(int width, int x, int y) {
    return 1 + y * width + x;
}

Coord nodeNumToCoord(int width, int nodeNum) {
    Coord coord;
    coord._y = (nodeNum-1) / width;
    coord._x = (nodeNum-1) % width;
    return coord;
}

/*
 *  Greate a graph to represent a tiled floor. Graph links 
 *  adjacent tiles.
 *
 *      *-*-*
 *      | | |
 *      *-*-*
 *      | | |
 *      *-*-*      
 */
void makeTileFloor(const string tile_fn, int width, int height) throw (int) {
    ofstream file;
    int   error = 0;
    file.open(tile_fn.c_str() , ios_base::out | ios_base::trunc);
    if (!file.is_open()) {
        cerr << "Could not open " << tile_fn << endl;
        error = 1;
    }
    else {
        file << (width)*(height-1) + (width-1)*(height) << endl;
        int distance = 1;
        for (int y = 0; y < height; y++) {
            for (int x = 1; x < width; x++) {
                file << coordToNodeNum(width, x-1,y) << " " <<  coordToNodeNum(width, x,y) << " " << distance << endl;
            }
        }
        for (int x = 0; x < width; x++) {
            for (int y = 1; y < height; y++) {
                file << coordToNodeNum(width, x,y-1) << " " <<  coordToNodeNum(width, x,y) << " " << distance << endl;
            }
        }
        file.close();
    }
    if (error != 0)
        throw error;
}

/*
 * Print out a grid
 */
void printGrid(int width, int height, const vector<int> shortest_dist)  {
#if CHECK_LOGIC
    show(shortest_dist, "printGrid");
    cout << "-------------------------" << endl;
#endif
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int n =  shortest_dist[coordToNodeNum(width, x, y)];
            cout << setw(3) << (n == INFINITY ? -1 : n); 
        }
        cout << endl;
    }
}



/*
 * Read a graph file
 * Each line is "node1 node2 distance"
 */
Graph  readGraphFile(const string fn) throw(int) {
    int error = 0;
    Graph graph(0);
    ifstream file;
    file.open(fn.c_str(), ios_base::in);
    if (!file.is_open()) {
        cerr << "Could not open " << fn << endl;
        error = 2;
    }
    else {
        string s;
        getline(file, s);
        int number_edges = atoi(s.c_str());
        cout << "number_edges = " << number_edges << endl;
        graph = Graph(number_edges);
        for (int i = 0; i < number_edges; ++i) {
            getline(file, s);
            istringstream iss(s);
            vector<int> v;
            // not the fastest method ... 
            copy(istream_iterator<int>(iss), istream_iterator<int>(), back_inserter(v));
            if (v.size() < 3) {
                cerr << "Bad line [" << i << "] " << v.size() << " entries" << endl;
                cerr << "Line format: node1 node2 distance" << endl;
                error = 3;
                break;
            }
            cout << v[0] << " " << v[1] << " " << v[2] << endl;
            assert(v[0] >= 0 && v[1] >= 0 && v[2] > 0);
            graph._dist[v[0]][v[1]] = v[2];
            graph._number_nodes = max(v[0], max(v[1], graph._number_nodes));
        }
        file.close();
    }
    if (error != 0)
        throw error;
    return graph;
}

/*
 *  Returns shortest path in graph specified in fn with starting point s
 */
vector<int> findShortestPath(const string fn, int s)  {
    Graph graph = readGraphFile(fn);
    vector<int> shortestDist = dijkstra(s, graph);
    printDistances(shortestDist, graph._number_nodes);
    return shortestDist;
}
 

 
void shortest_path_test() {
    int width = WIDTH, height = WIDTH;
    
    try {
        makeTileFloor(the_tile_fn, width, height);
        Graph graph = readGraphFile(the_tile_fn);
 
        int s = width*height;
        cout << "====================== source = " << s << endl;
        vector<int> shortest_dist = dijkstra(s, graph);
        printGrid(width, height, shortest_dist);
 
        for (int s = 1; s <= width * height; s++) {
            cout << "====================== source = " << s << endl;
            shortest_dist = dijkstra(s, graph);
            printGrid(width, height, shortest_dist);
        }
    }
    catch (int e) {
        cerr << "Caught exception: " << e << endl;
    }
     
 }
