#include <iostream>
#include <vector>
#include <queue>
#include <cstdlib>
#include <ctime>
#include <iomanip>
using namespace std;

// Map size
const int MAP_SIZE = 10;

// Meaning of each coordinate
const int EMPTY = -1; // Traversable
const int OBSTACLE = -2; // Obstacle

// Structure to represent a coordinate
struct Point {
    int y, x;
    Point(int y, int x) : y(y), x(x) {}
};

// Node structure for A* algorithm
struct Node {
    Point point;
    int g_cost, h_cost, f_cost; // g: distance from start, h: heuristic (distance to goal), f: g + h
    Node* parent;
    Node(Point point, int g, int h, Node* parent)
        : point(point), g_cost(g), h_cost(h), f_cost(g + h), parent(parent) {}
};

// Heuristic (distance to goal) calculation: Manhattan distance
int calculate_heuristic(Point a, Point b) {
    return abs(a.y - b.y) + abs(a.x - b.x);
}

// Function to print the map
void print_map(const vector<vector<int>>& map) {
    for (const auto& row : map) {
        for (const auto& cell : row) {
            if (cell == EMPTY)
                cout << "□ "; // Traversable
            else if (cell == OBSTACLE)
                cout << "■ "; // Obstacle
            else
                cout << "* "; // Path display
        }
        cout << '\n';
    }
}

// Function to initialize the map: randomly place obstacles
void initialize_map(vector<vector<int>>& map) {
    srand(time(0)); // Initialize random number generator
    for (int i = 0; i < MAP_SIZE; ++i) {
        for (int j = 0; j < MAP_SIZE; ++j) {
            map[i][j] = EMPTY; // By default, traversable
        }
    }
    int obstacles = (MAP_SIZE * MAP_SIZE) * 20 / 100; // 20% of the total map
    while (obstacles > 0) {
        int y = rand() % MAP_SIZE;
        int x = rand() % MAP_SIZE;
        if (map[y][x] == EMPTY && !(y == 0 && x == 0)) { // The start point cannot be an obstacle
            map[y][x] = OBSTACLE;
            obstacles--;
        }
    }
}

// Search the path using A* algorithm
bool find_path(vector<vector<int>>& map, Point start, Point end) {
    vector<Point> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}}; // Up, Down, Left, Right
    vector<vector<bool>> visited(MAP_SIZE, vector<bool>(MAP_SIZE, false));

    // Priority queue: sorted by f_cost (total cost)
    auto compare = [](Node* a, Node* b) { return a->f_cost > b->f_cost; };
    priority_queue<Node*, vector<Node*>, decltype(compare)> open_list(compare);

    // Initialize the start node
    Node* start_node = new Node(start, 0, calculate_heuristic(start, end), nullptr);
    open_list.push(start_node);

    while (!open_list.empty()) {
        Node* current = open_list.top();
        open_list.pop();
        Point p = current->point;

        if (visited[p.y][p.x])
            continue;
        visited[p.y][p.x] = true;

        // If the destination is reached
        if (p.y == end.y && p.x == end.x) {
            Node* temp = current;
            int step = 1;
            while (temp) { // Display the path as numbers
                map[temp->point.y][temp->point.x] = step++;
                temp = temp->parent;
            }
            return true;
        }

        // Explore all neighbors of the current node
        for (const auto& dir : directions) {
            int ny = p.y + dir.y;
            int nx = p.x + dir.x;

            // Check if the coordinates are valid
            if (ny >= 0 && ny < MAP_SIZE && nx >= 0 && nx < MAP_SIZE &&
                map[ny][nx] == EMPTY && !visited[ny][nx]) {
                Node* neighbor = new Node(Point(ny, nx), current->g_cost + 1,
                                          calculate_heuristic(Point(ny, nx), end),
                                          current);
                open_list.push(neighbor);
            }
        }
    }
    return false; // No path found
}


int main() {
    vector<vector<int>> map(MAP_SIZE, vector<int>(MAP_SIZE));

    // Initialize the map
    initialize_map(map);

    // Print the initial map
    cout << '\n' << "Hello! This program is <A* NAVIGATION> that finds the path to the destination!" << '\n';
    cout << '\n' << "    <Full Map>" << '\n';
    print_map(map);
    cout << '\n' << "□: Traversable Area" << '\n' << "■: Obstacle" << '\n';

    // Get the destination from the user
    Point start(0, 0), end(0, 0);
    while (true) {
        cout << '\n' << "Starting the pathfinding using A* algorithm." << '\n';
        cout << "Your car is currently at (0, 0)." << '\n';
        cout << "Enter the y and x coordinates of the desired destination. (0-9, e.g., 5 5): ";
        cin >> end.y >> end.x;

        // Input validation
        if (cin.fail() || end.y < 0 || end.y >= MAP_SIZE || end.x < 0 || end.x >= MAP_SIZE ) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << '\n' << "!!! ERROR: Exceeded map boundaries.." << '\n';
        }   
        else if (map[end.y][end.x] == OBSTACLE) {
            cout << '\n' << "!!! ERROR: The destination cannot be set in an area with an obstacle." << '\n';
        }
        else {
            break;
        }
    }

    // Run the A* algorithm
    if (find_path(map, start, end)) {
        cout << '\n' << "<Pathfinding Completed>" << '\n';
        print_map(map);
        cout << '\n' << "*: Path" << '\n';

    } else {
        cout << '\n' << "No path exists to the destination." << '\n';
    }


    cout << '\n';
    return 0;
}
