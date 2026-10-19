import heapq
import json
import math
import random
import os

def load_graph_from_geojson(geojson_filename="karhutla_network.geojson"):
    """Loads node and edge spatial data directly from a Karhutla GeoJSON file."""
    # Resolve file path relative to the script location
    base_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(base_dir, geojson_filename)

    with open(file_path, 'r', encoding='utf-8') as f:
        geojson_data = json.load(f)
    
    nodes = {}
    edges_list = []
    
    # Separate Point features (Nodes/Hotspots) and LineString features (Road Edges)
    for feature in geojson_data.get('features', []):
        geom_type = feature.get('geometry', {}).get('type')
        props = feature.get('properties', {})
        
        if geom_type == 'Point':
            if 'id' in props:
                nodes[props['id']] = props
        elif geom_type == 'LineString':
            if 'from' in props and 'to' in props and 'distance_km' in props:
                edges_list.append(props)
                
    # Construct adjacency list (undirected graph representation)
    graph = {node_id: [] for node_id in nodes}
    for edge in edges_list:
        u = edge['from']
        v = edge['to']
        cost = float(edge['distance_km'])
        
        # Roads are treated as bidirectional corridors
        graph[u].append((v, cost))
        graph[v].append((u, cost))
            
    return nodes, graph

def dijkstra_shortest_path(start, goal, graph):
    """Computes the shortest path between two nodes using Dijkstra's algorithm."""
    pq = [(0.0, start, [start])]
    visited = set()
    
    while pq:
        cost, current, path = heapq.heappop(pq)
        if current == goal:
            return cost, path
            
        if current in visited:
            continue
        visited.add(current)
            
        for neighbor, weight in graph.get(current, []):
            if neighbor not in visited:
                heapq.heappush(pq, (cost + weight, neighbor, path + [neighbor]))
                
    return float('inf'), []

def simulated_annealing_patrol(
    start_node, 
    patrol_targets, 
    graph, 
    initial_temp=1000.0, 
    cooling_rate=0.995, 
    min_temp=1e-3, 
    max_iter=3000, 
    seed=42
):
    """
    Optimizes a multi-checkpoint circular patrol tour (TSP) using Simulated Annealing:
    Start -> Visit all patrol_targets -> Return to Start.
    """
    if seed is not None:
        random.seed(seed)

    # 1. Precompute shortest travel paths across all points of interest
    all_poi = [start_node] + patrol_targets
    dist_matrix = {a: {} for a in all_poi}
    path_matrix = {a: {} for a in all_poi}

    for a in all_poi:
        for b in all_poi:
            if a == b:
                dist_matrix[a][b] = 0.0
                path_matrix[a][b] = [a]
            elif b not in dist_matrix[a]:
                cost, path = dijkstra_shortest_path(a, b, graph)
                dist_matrix[a][b] = cost
                dist_matrix[b][a] = cost
                path_matrix[a][b] = path
                path_matrix[b][a] = path[::-1]

    def calculate_total_cost(tour):
        full_loop = [start_node] + tour + [start_node]
        return sum(dist_matrix[full_loop[i]][full_loop[i + 1]] for i in range(len(full_loop) - 1))

    # 2. Initialize with a random candidate solution
    current_solution = list(patrol_targets)
    random.shuffle(current_solution)
    current_cost = calculate_total_cost(current_solution)

    best_solution = list(current_solution)
    best_cost = current_cost
    initial_cost = current_cost

    temp = initial_temp

    # 3. Annealing Optimization Loop
    for _ in range(max_iter):
        if temp < min_temp:
            break

        # 2-Opt neighborhood operator (reverse random sub-segment)
        i, j = sorted(random.sample(range(len(current_solution)), 2))
        candidate = current_solution[:i] + current_solution[i:j + 1][::-1] + current_solution[j + 1:]
        candidate_cost = calculate_total_cost(candidate)

        delta = candidate_cost - current_cost

        # Metropolis acceptance criterion
        if delta < 0 or math.exp(-delta / temp) > random.random():
            current_solution = candidate
            current_cost = candidate_cost

            if current_cost < best_cost:
                best_solution = list(current_solution)
                best_cost = current_cost

        # Exponential temperature decay
        temp *= cooling_rate

    # Assemble full circular tour
    optimal_tour = [start_node] + best_solution + [start_node]
    return initial_cost, best_cost, optimal_tour, path_matrix, dist_matrix

# ==========================================
# SCRIPT EXECUTION
# ==========================================
if __name__ == "__main__":
    nodes, graph = load_graph_from_geojson("karhutla_network.geojson")
    
    start_node = "POSKO_INDUK"       # Base station command post
    
    # 7 High-priority wildfire monitoring checkpoints
    patrol_targets = [
        "RSUD_DORIS",
        "KALAMPANGAN",
        "TARUNA",
        "JABIREN",
        "SEBANGAU_POST",
        "BUNTOI",
        "PILANG"
    ]
    
    print("Executing Simulated Annealing using GeoJSON network data...")
    print(f"Base Station (Origin & Finish) : {nodes.get(start_node, {}).get('name')} ({start_node})")
    print(f"Patrol Target Checkpoints      : {len(patrol_targets)} locations\n")
    
    initial_c, optimal_c, optimal_tour, path_matrix, dist_matrix = simulated_annealing_patrol(
        start_node=start_node,
        patrol_targets=patrol_targets,
        graph=graph,
        initial_temp=1000.0,
        cooling_rate=0.995,
        max_iter=3000,
        seed=42
    )
    
    reduction_pct = ((initial_c - optimal_c) / initial_c) * 100.0
    
    print("=" * 68)
    print("WILDFIRE PATROL ROUTE OPTIMIZATION RESULTS (SIMULATED ANNEALING)")
    print("=" * 68)
    print(f"Initial Random Tour Cost   : {initial_c:.2f} km")
    print(f"Optimized Patrol Cost (SA) : {optimal_c:.2f} km")
    print(f"Tour Distance Improvement  : {reduction_pct:.2f}%")
    print("\nCheckpoint Visit Sequence:")
    for idx, node_id in enumerate(optimal_tour, 1):
        loc = nodes.get(node_id, {})
        print(f"  {idx}. [{node_id}] {loc.get('name')} ({loc.get('regency')})")
        
    print("\nDetailed Graph Traversal Segments:")
    for i in range(len(optimal_tour) - 1):
        u, v = optimal_tour[i], optimal_tour[i + 1]
        step_dist = dist_matrix[u][v]
        detailed_nodes = path_matrix[u][v]
        print(f"  * [{u} -> {v}] ({step_dist:.1f} km): {' -> '.join(detailed_nodes)}")
    print("=" * 68)