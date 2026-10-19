import heapq
import json

def load_graph_from_geojson(geojson_filename="karhutla_network.geojson"):
    """Memuat data node dan edge langsung dari file GeoJSON Karhutla."""
    with open(geojson_filename, 'r', encoding='utf-8') as f:
        geojson_data = json.load(f)
    
    nodes = {}
    edges_list = []
    
    # Memisahkan fitur Point (Node) dan LineString (Edge) dari GeoJSON
    for feature in geojson_data.get('features', []):
        geom_type = feature.get('geometry', {}).get('type')
        props = feature.get('properties', {})
        
        if geom_type == 'Point':
            # Deteksi apakah ini Node/Posko atau Hotspot
            if 'id' in props:
                nodes[props['id']] = props
        elif geom_type == 'LineString':
            # Ini adalah jalur jalan (edge)
            if 'from' in props and 'to' in props and 'distance_km' in props:
                edges_list.append(props)
                
    # Membangun struktur adjacency list (grafik)
    graph = {node_id: [] for node_id in nodes}
    for edge in edges_list:
        u = edge['from']
        v = edge['to']
        cost = float(edge['distance_km'])
        
        # Asumsikan jalan bersifat dua arah (bidirectional)
        graph[u].append((v, cost))
        graph[v].append((u, cost))
            
    return nodes, graph

def uniform_cost_search(start, goal, graph):
    """Implementasi Uniform Cost Search (UCS)."""
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
                new_cost = cost + weight
                heapq.heappush(pq, (new_cost, neighbor, path + [neighbor]))
                
    return float('inf'), []

# ==========================================
# EKSEKUSI PROGRAM
# ==========================================
if __name__ == "__main__":
    # Pastikan file karhutla_network.geojson ada di folder yang sama
    nodes, graph = load_graph_from_geojson("karhutla_network.geojson")
    
    start_node = "POSKO_INDUK"       # Posko Induk Palangka Raya
    goal_node = "KUALA_KAPUAS"       # Kota Kuala Kapuas
    
    print(f"Menjalankan UCS menggunakan format GeoJSON...")
    print(f"Titik Asal  : {nodes.get(start_node, {}).get('name')} ({start_node})")
    print(f"Titik Tujuan: {nodes.get(goal_node, {}).get('name')} ({goal_node})\n")
    
    total_cost, optimal_path = uniform_cost_search(start_node, goal_node, graph)
    
    if total_cost != float('inf'):
        print("="*60)
        print("HASIL PENCARIAN RUTE OPTIMAL (UCS - GEOJSON)")
        print("="*60)
        print(f"Total Jarak Akumulasi g(n) : {total_cost:.2f} km")
        print(f"Jumlah Simpul Dilewati   : {len(optimal_path)} titik")
        print("\nUrutan Jalur Evakuasi:")
        for idx, node_id in enumerate(optimal_path, 1):
            loc = nodes.get(node_id, {})
            print(f"  {idx}. [{node_id}] {loc.get('name')} - Wilayah: {loc.get('regency')}")
        print("="*60)
    else:
        print("Maaf, jalur evakuasi tidak ditemukan!")