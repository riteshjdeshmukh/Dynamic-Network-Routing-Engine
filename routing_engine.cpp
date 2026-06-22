#include <iostream>

// Global constants for static allocation to maintain cache locality
const int MAX_NODES = 10000;
const int MAX_EDGES = 40000; // Assuming ~4 edges per node on a grid
const double INF = 1e9;      // Representing infinity for unvisited states

// ============================================================================
// 1. GRAPH STRUCTURES (Contiguous Memory Layout)
// ============================================================================
struct Edge {
    int to_node;
    double distance;
    double speed_multiplier; // 1.0 = clear, 0.2 = traffic jam
    int next_edge_idx;       // Index of the next edge in the adjacency chain
};

struct Node {
    int first_edge_idx;      // Head of the edge chain (-1 if no edges)
    double x, y;             // Spatial coordinates for heuristic calculation
};

class Graph {
public:
    Node nodes[MAX_NODES];
    Edge edges[MAX_EDGES];
    int num_nodes;
    int edge_counter;

    Graph() {
        num_nodes = 0;
        edge_counter = 0;
    }

    void init_nodes(int count) {
        num_nodes = count;
        for (int i = 0; i < num_nodes; ++i) {
            nodes[i].first_edge_idx = -1;
            nodes[i].x = 0.0;
            nodes[i].y = 0.0;
        }
    }

    void set_node_coordinates(int node_id, double x, double y) {
        if (node_id >= 0 && node_id < num_nodes) {
            nodes[node_id].x = x;
            nodes[node_id].y = y;
        }
    }

    void add_edge(int u, int v, double distance) {
        if (edge_counter >= MAX_EDGES) return;

        // Populate edge info and insert at the head of node u's edge list
        edges[edge_counter].to_node = v;
        edges[edge_counter].distance = distance;
        edges[edge_counter].speed_multiplier = 1.0; // default clear
        edges[edge_counter].next_edge_idx = nodes[u].first_edge_idx;
        
        nodes[u].first_edge_idx = edge_counter;
        edge_counter++;
    }
};

// ============================================================================
// 2. CUSTOM INDEXED MIN-HEAP (Priority Queue)
// ============================================================================
struct HeapNode {
    int node_id;
    double priority; // Key: accumulated cost + heuristic
};

class IndexedMinHeap {
private:
    HeapNode heap[MAX_NODES];
    int node_to_heap_idx[MAX_NODES]; // Maps Node ID -> Position inside heap array
    int heap_size;

    void swap_nodes(int idx_a, int idx_b) {
        // Update index tracking arrays first
        node_to_heap_idx[heap[idx_a].node_id] = idx_b;
        node_to_heap_idx[heap[idx_b].node_id] = idx_a;

        // Swap actual data positions
        HeapNode temp = heap[idx_a];
        heap[idx_a] = heap[idx_b];
        heap[idx_b] = temp;
    }

    void up_heapify(int idx) {
        while (idx > 0) {
            int parent = (idx - 1) / 2;
            if (heap[idx].priority < heap[parent].priority) {
                swap_nodes(idx, parent);
                idx = parent;
            } else {
                break;
            }
        }
    }

    void down_heapify(int idx) {
        int smallest = idx;
        int left = 2 * idx + 1;
        int right = 2 * idx + 2;

        if (left < heap_size && heap[left].priority < heap[smallest].priority) {
            smallest = left;
        }
        if (right < heap_size && heap[right].priority < heap[smallest].priority) {
            smallest = right;
        }

        if (smallest != idx) {
            swap_nodes(idx, smallest);
            down_heapify(smallest);
        }
    }

public:
    IndexedMinHeap() {
        heap_size = 0;
        for (int i = 0; i < MAX_NODES; ++i) {
            node_to_heap_idx[i] = -1; // -1 means not in heap
        }
    }

    bool is_empty() {
        return heap_size == 0;
    }

    bool contains(int node_id) {
        return node_to_heap_idx[node_id] != -1;
    }

    void insert(int node_id, double priority) {
        if (contains(node_id)) return;

        heap[heap_size].node_id = node_id;
        heap[heap_size].priority = priority;
        node_to_heap_idx[node_id] = heap_size;
        
        heap_size++;
        up_heapify(heap_size - 1);
    }

    HeapNode extract_min() {
        if (is_empty()) return { -1, -1.0 };

        HeapNode min_node = heap[0];
        node_to_heap_idx[min_node.node_id] = -1; // Removed from heap tracking

        if (heap_size > 1) {
            heap[0] = heap[heap_size - 1];
            node_to_heap_idx[heap[0].node_id] = 0;
            heap_size--;
            down_heapify(0);
        } else {
            heap_size--;
        }

        return min_node;
    }

    void decrease_key(int node_id, double new_priority) {
        int idx = node_to_heap_idx[node_id];
        if (idx == -1) return; // Node isn't in heap

        if (new_priority < heap[idx].priority) {
            heap[idx].priority = new_priority;
            up_heapify(idx);
        }
    }
};

// ============================================================================
// 3. ROUTING ENGINE (A* Implementation)
// ============================================================================
class RoutingEngine {
private:
    // Custom inline absolute value function since we avoid <cmath>
    double abs_val(double val) {
        return (val < 0) ? -val : val;
    }

    // Manhattan distance heuristic for grid networks
    double get_heuristic(Graph& graph, int current, int target) {
        double dx = abs_val(graph.nodes[current].x - graph.nodes[target].x);
        double dy = abs_val(graph.nodes[current].y - graph.nodes[target].y);
        return dx + dy;
    }

public:
    double travel_distances[MAX_NODES];
    int parent_track[MAX_NODES];

    bool compute_shortest_path(Graph& graph, int start_node, int target_node) {
        IndexedMinHeap min_heap;

        // Initialize state tracking arrays
        for (int i = 0; i < graph.num_nodes; ++i) {
            travel_distances[i] = INF;
            parent_track[i] = -1;
        }

        travel_distances[start_node] = 0.0;
        min_heap.insert(start_node, get_heuristic(graph, start_node, target_node));

        while (!min_heap.is_empty()) {
            HeapNode current = min_heap.extract_min();
            int u = current.node_id;

            // Early exit if destination is found
            if (u == target_node) {
                return true;
            }

            // Traverse the contiguous edge list
            int edge_idx = graph.nodes[u].first_edge_idx;
            while (edge_idx != -1) {
                Edge& edge = graph.edges[edge_idx];
                int v = edge.to_node;

                // Adjust cost based on simulated speed/traffic factors
                double dynamic_cost = edge.distance * (1.0 / edge.speed_multiplier);
                double alternate_distance = travel_distances[u] + dynamic_cost;

                if (alternate_distance < travel_distances[v]) {
                    travel_distances[v] = alternate_distance;
                    parent_track[v] = u;

                    double priority_key = alternate_distance + get_heuristic(graph, v, target_node);
                    
                    if (min_heap.contains(v)) {
                        min_heap.decrease_key(v, priority_key);
                    } else {
                        min_heap.insert(v, priority_key);
                    }
                }
                edge_idx = edge.next_edge_idx; // Move down memory list
            }
        }

        return false; // Path not found
    }

    void print_path(int target_node) {
        if (parent_track[target_node] == -1) {
            std::cout << target_node;
            return;
        }
        print_path(parent_track[target_node]);
        std::cout << " -> " << target_node;
    }
};

// ============================================================================
// 4. MAIN SIMULATION & EXECUTION LOOP
// ============================================================================
int main() {
    Graph city_map;
    RoutingEngine router;

    // Create a simple 3x3 Grid Network (9 Nodes total)
    city_map.init_nodes(9);
    
    // Set up geometric tracking points (x, y)
    city_map.set_node_coordinates(0, 0, 2); city_map.set_node_coordinates(1, 1, 2); city_map.set_node_coordinates(2, 2, 2);
    city_map.set_node_coordinates(3, 0, 1); city_map.set_node_coordinates(4, 1, 1); city_map.set_node_coordinates(5, 2, 1);
    city_map.set_node_coordinates(6, 0, 0); city_map.set_node_coordinates(7, 1, 0); city_map.set_node_coordinates(8, 2, 0);

    // Add horizontal and vertical layout edges (Bi-directional)
    city_map.add_edge(0, 1, 1.0); city_map.add_edge(1, 0, 1.0);
    city_map.add_edge(1, 2, 1.0); city_map.add_edge(2, 1, 1.0);
    city_map.add_edge(3, 4, 1.0); city_map.add_edge(4, 3, 1.0);
    city_map.add_edge(4, 5, 1.0); city_map.add_edge(5, 4, 1.0);
    city_map.add_edge(6, 7, 1.0); city_map.add_edge(7, 6, 1.0);
    city_map.add_edge(7, 8, 1.0); city_map.add_edge(8, 7, 1.0);

    city_map.add_edge(0, 3, 1.0); city_map.add_edge(3, 0, 1.0);
    city_map.add_edge(3, 6, 1.0); city_map.add_edge(6, 3, 1.0);
    city_map.add_edge(1, 4, 1.0); city_map.add_edge(4, 1, 1.0);
    city_map.add_edge(4, 7, 1.0); city_map.add_edge(7, 4, 1.0);
    city_map.add_edge(2, 5, 1.0); city_map.add_edge(5, 2, 1.0);
    city_map.add_edge(5, 8, 1.0); city_map.add_edge(8, 5, 1.0);

    int start = 0;  // Top-Left corner
    int target = 8; // Bottom-Right corner

    std::cout << "--- Run 1: Clear Conditions ---" << std::endl;
    if (router.compute_shortest_path(city_map, start, target)) {
        std::cout << "Optimal Cost: " << router.travel_distances[target] << std::endl;
        std::cout << "Path: ";
        router.print_path(target);
        std::cout << "\n\n";
    }

    // SIMULATING LIVE TRAFFIC CHANGE:
    // Create heavy congestion on the traditional path elements (Node 1 and Node 4)
    std::cout << "--- Run 2: Simulating Heavy Traffic Congestion at Node 1 and 4 ---" << std::endl;
    
    // Cycle through internal contiguous edges array to artificially reduce speeds
    for (int i = 0; i < city_map.edge_counter; ++i) {
        if (city_map.edges[i].to_node == 1 || city_map.edges[i].to_node == 4) {
            city_map.edges[i].speed_multiplier = 0.1; // Drops speed capacity to 10%
        }
    }

    // Dynamic Re-routing execution
    if (router.compute_shortest_path(city_map, start, target)) {
        std::cout << "New Cost (Traffic Adjusted): " << router.travel_distances[target] << std::endl;
        std::cout << "Dynamic Adjusted Path: ";
        router.print_path(target);
        std::cout << "\n";
    }

    return 0;
}
