# High-Performance Dynamic Network Routing Engine (C++)

A cache-friendly, low-latency spatial routing engine designed to execute dynamic path calculations (A* Search) under hardware-level optimization constraints. 

##  Key Engineering Highlights

* **Contiguous Memory Layout:** Eliminated dynamic heap allocation and pointer-chasing by structuring the entire graph layout within flat primitive arrays (`nodes` and `edges`). This maximizes CPU cache locality (L1/L2 data caching).
* **Custom Indexed Min-Heap:** Because standard `std::priority_queue` lacks an efficient element update mechanism, I implemented a Binary Min-Heap from scratch paired with a lookup array (`node_to_heap_idx`). This allows for an instantaneous $O(1)$ lookup and $O(\log V)$ `decrease_key` operation during graph exploration.
* **A* Search Optimization:** Utilizes a Manhattan distance heuristic to guide the search space directionally toward the destination, drastically reducing redundant node evaluations compared to standard Dijkstra.
* **Dynamic Simulation Loop:** Features a real-time state adjustment layer where edge capacities (`speed_multiplier`) can be restricted programmatically to simulate gridlock, triggering sub-millisecond dynamic re-routing.

##  Performance Architecture

### Graph Structure
Instead of utilizing standard `std::vector<Edge*>`, memory is allocated uniformly up front:

```cpp
struct Edge {
    int to_node;
    double distance;
    double speed_multiplier; 
    int next_edge_idx; // Head insertion index-pointer
};
