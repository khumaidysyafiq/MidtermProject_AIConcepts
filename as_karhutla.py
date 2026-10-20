#!/usr/bin/env python3
"""
=============================================================================
STANDALONE WEIGHTED A* SEARCH ALGORITHM (ZERO DEPENDENCIES)
Problem: Wildfire Peatland Emergency Routing (Palangka Raya - Pulang Pisau)
=============================================================================
Usage:
    python weighted_astar_standalone.py
    python weighted_astar_standalone.py POSKO_INDUK PULANG_PISAU 1.35
    python weighted_astar_standalone.py JABIREN RSUD_DORIS 1.35
"""

import math
import heapq
import time
import sys

# =============================================================================
# 1. EMBEDDED GEOSPATIAL NETWORK (18 NODES & 25 ROAD EDGES)
# =============================================================================

NODES = {
    "POSKO_INDUK":    {"name": "Posko Induk Palangka Raya",       "lat": -2.2078, "lon": 113.9165, "type": "command_post"},
    "RSUD_DORIS":     {"name": "RSUD dr. Doris Sylvanus",         "lat": -2.2150, "lon": 113.9210, "type": "hospital"},
    "SABARU":         {"name": "Sabaru (Pintu TN Sebangau)",      "lat": -2.2610, "lon": 113.8820, "type": "checkpoint"},
    "BERENG_BENGKEL": {"name": "Bereng Bengkel",                  "lat": -2.2740, "lon": 113.9310, "type": "village"},
    "KALAMPANGAN":    {"name": "Kalampangan Agropolitan",         "lat": -2.2925, "lon": 113.9870, "type": "village"},
    "TARUNA":         {"name": "Taruna Checkpoint",               "lat": -2.3610, "lon": 114.0450, "type": "checkpoint"},
    "SEBANGAU_POST":  {"name": "Pos Penjagaan TN Sebangau",      "lat": -2.3500, "lon": 113.8500, "type": "shelter"},
    "JABIREN":        {"name": "Jabiren Peatland Center",         "lat": -2.4830, "lon": 114.1250, "type": "village"},
    "PILANG":         {"name": "Desa Pilang",                     "lat": -2.4200, "lon": 114.2100, "type": "village"},
    "HENDA":          {"name": "Desa Henda",                      "lat": -2.5700, "lon": 114.1750, "type": "village"},
    "BUNTOI":         {"name": "Desa Adat Buntoi",                "lat": -2.6850, "lon": 114.1950, "type": "shelter"},
    "PULANG_PISAU":   {"name": "Kota Pulang Pisau",               "lat": -2.7450, "lon": 114.2500, "type": "command_post"},
    "MINTIN":         {"name": "Mintin Relief Depot",             "lat": -2.8100, "lon": 114.2900, "type": "water_source"},
    "BUKIT_RAWI":     {"name": "Simpang Bukit Rawi",              "lat": -2.1400, "lon": 114.0100, "type": "checkpoint"},
    "TUWUNG":         {"name": "Desa Tuwung",                     "lat": -2.0400, "lon": 114.0800, "type": "village"},
    "ANJIR_KALAMPAN": {"name": "Anjir Kalampan",                  "lat": -2.7200, "lon": 114.3300, "type": "water_source"},
    "KAPUAS_BARAT":   {"name": "Mandomai (Kapuas Barat)",         "lat": -2.8800, "lon": 114.3500, "type": "shelter"},
    "KUALA_KAPUAS":   {"name": "Kota Kuala Kapuas",               "lat": -3.0080, "lon": 114.3850, "type": "hospital"}
}

# 25 Undirected Edges: (u, v, distance_km, road_type, name)
EDGES_RAW = [
    ("POSKO_INDUK",    "RSUD_DORIS",      1.1,  "highway",    "Jl. Tjilik Riwut - Jl. Tambun Bungai"),
    ("RSUD_DORIS",     "BERENG_BENGKEL",  7.6,  "highway",    "Trans Kalimantan Poros Tengah"),
    ("BERENG_BENGKEL", "KALAMPANGAN",     6.7,  "highway",    "Trans Kalimantan Utama (Palangka - Basarang)"),
    ("KALAMPANGAN",    "TARUNA",          10.2, "highway",    "Trans Kalimantan Poros Tengah"),
    ("TARUNA",         "JABIREN",         16.4, "highway",    "Trans Kalimantan (Jembatan Tumbang Nusa)"),
    ("JABIREN",        "HENDA",           11.2, "highway",    "Trans Kalimantan Poros Selatan"),
    ("HENDA",          "BUNTOI",          13.1, "highway",    "Trans Kalimantan Poros Selatan"),
    ("BUNTOI",         "PULANG_PISAU",    9.2,  "highway",    "Jl. Trans Kalimantan - Jembatan Basarang"),
    ("PULANG_PISAU",   "MINTIN",          8.6,  "highway",    "Trans Kalimantan Basarang - Kapuas"),
    ("MINTIN",         "KAPUAS_BARAT",    10.1, "highway",    "Trans Kalimantan Lintas Selatan"),
    ("KAPUAS_BARAT",   "KUALA_KAPUAS",    14.7, "highway",    "Trans Kalimantan Poros Selatan"),
    ("POSKO_INDUK",    "BUKIT_RAWI",      13.2, "secondary",  "Jl. Palangka Raya - Bukit Rawi"),
    ("BUKIT_RAWI",     "TUWUNG",          13.8, "secondary",  "Jl. Poros Utara Kahayan"),
    ("BUKIT_RAWI",     "PILANG",          38.5, "secondary",  "Jalur Alternatif Tangkahen - Pilang"),
    ("PILANG",         "TARUNA",          19.8, "peat_trail", "Jalur Rawa Gambut Pilang - Taruna"),
    ("PILANG",         "JABIREN",         12.5, "peat_trail", "Jalur Kanal Tua Pilang - Jabiren"),
    ("PILANG",         "BUNTOI",          30.3, "secondary",  "Jl. Poros Timur Kahayan Hilir"),
    ("POSKO_INDUK",    "SABARU",          7.1,  "secondary",  "Jl. RTA Milono - Sabaru"),
    ("SABARU",         "SEBANGAU_POST",   10.7, "peat_trail", "Jalur Patroli Gambut Taman Nasional"),
    ("SEBANGAU_POST",  "JABIREN",         34.4, "peat_trail", "Kanal Eks-PLG Blok C Sebangau"),
    ("KALAMPANGAN",    "SABARU",          12.3, "secondary",  "Jl. Mahir Mahar Lingkar Luar"),
    ("BERENG_BENGKEL", "PILANG",          34.2, "canal",      "Alur Sungai Kahayan (Kanal Bereng)"),
    ("HENDA",          "ANJIR_KALAMPAN",  20.8, "peat_trail", "Tanggul Kanal Primer Petak Kapuas"),
    ("ANJIR_KALAMPAN", "PULANG_PISAU",    9.8,  "secondary",  "Jl. Irigasi Anjir Kalampan"),
    ("ANJIR_KALAMPAN", "KAPUAS_BARAT",    18.4, "peat_trail", "Tanggul Kanal Anjir Kalampan Timur")
]

# Active Wildfire Hotspots (Sensor VIIRS / MODIS)
HOTSPOTS = [
    {"id": "HOTSPOT_JABIREN",      "lat": -2.4780, "lon": 114.1150, "frp_mw": 420.0, "radius_km": 6.5},
    {"id": "HOTSPOT_SEBANGAU",     "lat": -2.3300, "lon": 113.8650, "frp_mw": 180.0, "radius_km": 4.0},
    {"id": "HOTSPOT_KALAMPANGAN",  "lat": -2.3100, "lon": 113.9980, "frp_mw": 95.0,  "radius_km": 2.8}
]

# Dense Smoke Zones (ISPU / PM2.5 Hazard Plumes)
SMOKE_ZONES = [
    {"id": "SMOKE_PLUME_JABIREN",  "lat": -2.4600, "lon": 114.1000, "radius_km": 12.0, "mult": 3.5},
    {"id": "SMOKE_PLUME_SEBANGAU", "lat": -2.3100, "lon": 113.8750, "radius_km": 8.0,  "mult": 2.2},
    {"id": "SMOKE_PLUME_CENTRAL",  "lat": -2.2500, "lon": 113.9500, "radius_km": 9.0,  "mult": 1.5}
]

ROAD_FACTORS = {
    "highway": 1.00,
    "secondary": 1.35,
    "canal": 1.75,
    "peat_trail": 2.20
}

# =============================================================================
# 2. MATHEMATICAL HEURISTIC & COST FUNCTIONS
# =============================================================================

def haversine(lat1, lon1, lat2, lon2):
    """Calculates straight-line great-circle distance in kilometers."""
    R = 6371.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2)**2
    return 2.0 * R * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))

def compute_step_cost(u, v, dist_km, rtype, alpha=1.2, beta=0.8):
    """
    Computes Hazard-Aware Step Cost:
    c(u, v) = distance * road_factor * (1 + alpha * P_fire + beta * P_smoke)
    """
    u_node, v_node = NODES[u], NODES[v]
    mid_lat = (u_node["lat"] + v_node["lat"]) / 2.0
    mid_lon = (u_node["lon"] + v_node["lon"]) / 2.0

    # Fire radiation penalty
    p_fire = 0.0
    for spot in HOTSPOTS:
        d = haversine(mid_lat, mid_lon, spot["lat"], spot["lon"])
        r = spot["radius_km"]
        if d <= r:
            p_fire += (spot["frp_mw"] / 40.0) * (1.0 + (r - d) * 1.5)
        elif d <= r * 2.0:
            p_fire += (spot["frp_mw"] / 100.0) * ((r * 2.0 - d) / r)

    # Smoke plume penalty
    p_smoke = 0.0
    for smoke in SMOKE_ZONES:
        d = haversine(mid_lat, mid_lon, smoke["lat"], smoke["lon"])
        r = smoke["radius_km"]
        if d <= r:
            p_smoke += smoke["mult"] * ((r - d) / r)

    w_road = ROAD_FACTORS.get(rtype, 1.0)
    multiplier = 1.0 + (alpha * p_fire) + (beta * p_smoke)
    return round(dist_km * w_road * multiplier, 3)

# Build Graph Adjacency
ADJ = {nid: [] for nid in NODES}
for u, v, dist, rtype, name in EDGES_RAW:
    cost = compute_step_cost(u, v, dist, rtype)
    ADJ[u].append((v, cost, dist, name))
    ADJ[v].append((u, cost, dist, name))

# =============================================================================
# 3. STANDALONE WEIGHTED A* SEARCH ALGORITHM
# =============================================================================

def weighted_astar(start_node, goal_node, weight=1.35):
    """
    Weighted A* Search Algorithm.
    Evaluation Function: f(n) = g(n) + w * h(n)
    """
    if start_node not in NODES or goal_node not in NODES:
        raise ValueError(f"Unknown start or goal node: {start_node} -> {goal_node}")

    t0 = time.perf_counter()

    goal_lat = NODES[goal_node]["lat"]
    goal_lon = NODES[goal_node]["lon"]

    def h(n):
        return haversine(NODES[n]["lat"], NODES[n]["lon"], goal_lat, goal_lon)

    counter = 0
    start_h = h(start_node)
    # Entry in Min-Heap: (f_cost, tie_breaker, g_cost, node_id, path, edges)
    frontier = [(0.0 + weight * start_h, counter, 0.0, start_node, [start_node], [])]
    heapq.heapify(frontier)

    cost_so_far = {start_node: 0.0}
    explored = set()
    nodes_expanded = 0

    while frontier:
        f_cost, _, g_cost, curr, path, edges = heapq.heappop(frontier)

        if curr in explored:
            continue
        explored.add(curr)
        nodes_expanded += 1

        # Goal test upon expansion
        if curr == goal_node:
            t1 = time.perf_counter()
            total_dist = sum(e[2] for e in edges)
            return {
                "success": True,
                "start": start_node,
                "goal": goal_node,
                "weight": weight,
                "total_cost": round(g_cost, 2),
                "total_dist_km": round(total_dist, 2),
                "hops": len(path) - 1,
                "nodes_expanded": nodes_expanded,
                "runtime_ms": round((t1 - t0) * 1000.0, 3),
                "path": path,
                "edges": edges
            }

        for neighbor, step_cost, dist, road_name in ADJ[curr]:
            new_g = g_cost + step_cost
            if neighbor not in cost_so_far or new_g < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_g
                h_val = h(neighbor)
                f_val = new_g + (weight * h_val)
                counter += 1
                new_edge_info = (curr, neighbor, dist, step_cost, road_name)
                heapq.heappush(frontier, (f_val, counter, new_g, neighbor, path + [neighbor], edges + [new_edge_info]))

    return {"success": False, "nodes_expanded": nodes_expanded}

# =============================================================================
# 4. CLI INPUT & OUTPUT RUNNER
# =============================================================================

if __name__ == "__main__":
    # If arguments are provided via CLI, use them directly
    if len(sys.argv) > 1:
        start = sys.argv[1].upper()
        goal  = sys.argv[2].upper() if len(sys.argv) > 2 else "PULANG_PISAU"
        w     = float(sys.argv[3]) if len(sys.argv) > 3 else 1.35
    else:
        # Interactive mode with instant default
        print("=" * 70)
        print("  STANDALONE WEIGHTED A* SEARCH (NO SERVERS / NO WEB / PURE PYTHON)")
        print("=" * 70)
        print("  Available Quick Skenario:")
        print("    [1] Skenario 1: POSKO_INDUK -> PULANG_PISAU (Default)")
        print("    [2] Skenario 2: JABIREN     -> RSUD_DORIS")
        print("    [3] Skenario 3: MINTIN      -> SEBANGAU_POST")
        print("    [4] Custom Nodes Input")
        print("-" * 70)

        choice = ""
        if sys.stdin.isatty():
            try:
                choice = input("  Select option [1-4] (Press ENTER for default Skenario 1): ").strip()
            except (EOFError, KeyboardInterrupt):
                choice = "1"
        else:
            choice = "1"

        if choice == "2":
            start, goal, w = "JABIREN", "RSUD_DORIS", 1.35
        elif choice == "3":
            start, goal, w = "MINTIN", "SEBANGAU_POST", 1.35
        elif choice == "4":
            print("  Available Nodes:", ", ".join(sorted(NODES.keys())))
            start = input("  Enter Start Node: ").strip().upper()
            goal  = input("  Enter Goal Node : ").strip().upper()
            w_str = input("  Enter Weight w (default 1.35): ").strip()
            w = float(w_str) if w_str else 1.35
        else:
            start, goal, w = "POSKO_INDUK", "PULANG_PISAU", 1.35

    print("\n" + "=" * 70)
    print(f"  RUNNING WEIGHTED A* SEARCH")
    print("=" * 70)
    print(f"  Start Node : {start} ({NODES.get(start, {}).get('name', 'Unknown')})")
    print(f"  Goal Node  : {goal} ({NODES.get(goal, {}).get('name', 'Unknown')})")
    print(f"  Weight (w) : {w} (f(n) = g(n) + {w} * h(n))")
    print("-" * 70)

    result = weighted_astar(start, goal, weight=w)

    if result["success"]:
        print(f"  [STATUS]           : SUCCESS (Optimal Path Found)")
        print(f"  [CUMULATIVE COST]  : {result['total_cost']}")
        print(f"  [TOTAL DISTANCE]   : {result['total_dist_km']} km")
        print(f"  [TOTAL HOPS]       : {result['hops']} segments")
        print(f"  [NODES EXPANDED]   : {result['nodes_expanded']} nodes")
        print(f"  [RUNTIME]          : {result['runtime_ms']} ms")
        print("-" * 70)
        print("  OPTIMAL PATH SEQUENCE:")
        for idx, nid in enumerate(result["path"]):
            print(f"    ({idx + 1}) {nid:16} -> {NODES[nid]['name']}")
        print("-" * 70)
        print("  STEP-BY-STEP ROAD SEGMENTS:")
        for idx, (u, v, dist, cost, rname) in enumerate(result["edges"]):
            print(f"    Step {idx + 1}: {u:14} -> {v:14} | Dist: {dist:4.1f} km | Cost: {cost:6.2f} | {rname}")
        print("=" * 70)
    else:
        print("  [STATUS] : FAILED - No path exists between nodes.")
