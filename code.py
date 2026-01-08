#① 
import osmnx as ox  
import geopandas as gpd 
import matplotlib.pyplot as plt 
from geopy.geocoders import Nominatim 
import pandas as pd 
import heapq 
import time 
import math 
from collections import defaultdict 
import sys 
 
#② 
def get_lat_lon(address): 
    try: 
        location = geolocator.geocode(address) 
        time.sleep(1) 
        if location: 
            return location.latitude, location.longitude 
    except: 
        pass 
    return None, None 
geolocator = Nominatim(user_agent="kanagawa_locator") 
 
#③ 
algo = input("使用する経路探索アルゴリズムを選択してください（dijkstra / bellman / astar）: ").lower() 
if algo not in {"dijkstra", "bellmanford", "astar"}: 
    raise ValueError("有効なアルゴリズム名を入力してください") 
address = input("通報地点の住所を入力してください: ") 
report_lat, report_lon = get_lat_lon(address) 
if report_lat is None: 
    raise ValueError("ジオコーディングに失敗しました。") 
 
#④ 
area_list = "Yokohama, Kanagawa, Japan" 
graph_path = "kanagawa_combined.graphml" 
try: 
    G = ox.load_graphml(graph_path) 
    G = ox.convert.to_undirected(G) 
except: 
    G = ox.graph_from_place(area_list, network_type="drive", simplify=True) 
    G = ox.convert.to_undirected(G) 
    ox.save_graphml(G, filepath=graph_path) 
report_node = ox.nearest_nodes(G, report_lon, report_lat) 
 
#⑤ 
tags = {"amenity": "fire_station"} 
fire_gdf = ox.features_from_place(area_list, tags) 
fire_points = fire_gdf[fire_gdf.geometry.type == "Point"] 
fire_stations = [] 
for geom in fire_points.geometry: 
    lat, lon = geom.y, geom.x 
    try: 
        node = ox.nearest_nodes(G, lon, lat) 
        fire_stations.append({"node": node, "lat": lat, "lon": lon}) 
    except: 
        continue 
fire_nodes = [fs["node"] for fs in fire_stations] 
 
#⑥ 
df = pd.read_csv("hospitals.csv", encoding="shift_jis") 
df = df.dropna(subset=["latitude", "longitude"]) 
hospital_info = [] 
for _, row in df.iterrows(): 
    lat, lon = row["longitude"], row["latitude"] 
    try: 
        node = ox.nearest_nodes(G, lon, lat) 
        hospital_info.append({ 
            "node": node, 
            "name": row["name"], 
            "lat": lat, 
            "lon": lon 
        }) 
    except: 
        continue 
hospital_nodes = [h["node"] for h in hospital_info] 
 
#⑦ 
def build_adjacency_dict(G): 
    adj = {} 
    for u, v, data in G.edges(data=True): 
        dist = data.get('length', 1.0) 
        adj.setdefault(u, {})[v] = dist 
        adj.setdefault(v, {})[u] = dist 
    return adj 
graph_dict = build_adjacency_dict(G) 
 
#⑧ 
def dijkstra(graph, start): 
    dist = {n: float('inf') for n in graph} 
    prev = {} 
    dist[start] = 0 
    queue = [(0, start)] 
    while queue: 
        d, u = heapq.heappop(queue) 
        if d > dist[u]: 
            continue 
        for v in graph[u]: 
            alt = d + graph[u][v] 
            if alt < dist[v]: 
                dist[v] = alt 
                prev[v] = u 
                heapq.heappush(queue, (alt, v)) 
    return dist, prev 
 
#⑨ 
def bellman_ford(graph, start): 
    dist = {n: float('inf') for n in graph} 
    prev = {} 
    dist[start] = 0 
    for _ in range(len(graph) - 1): 
        for u in graph: 
            for v in graph[u]: 
                alt = dist[u] + graph[u][v] 
                if alt < dist[v]: 
                    dist[v] = alt 
                    prev[v] = u 
    return dist, prev 
 
#⑩ 
def heuristic(a, b): 
    y1, x1 = G.nodes[a]['y'], G.nodes[a]['x'] 
    y2, x2 = G.nodes[b]['y'], G.nodes[b]['x'] 
    return math.hypot(x1 - x2, y1 - y2) 
 
def a_star(graph, start, goal): 
    open_set = [(0, start)] 
    came_from = {} 
    g_score = {n: float('inf') for n in graph} 
    g_score[start] = 0 
    f_score = {n: float('inf') for n in graph} 
    f_score[start] = heuristic(start, goal) 
    while open_set: 
        _, current = heapq.heappop(open_set) 
        if current == goal: 
            break 
        for neighbor in graph[current]: 
            tentative = g_score[current] + graph[current][neighbor] 
            if tentative < g_score[neighbor]: 
                came_from[neighbor] = current 
                g_score[neighbor] = tentative 
                f_score[neighbor] = tentative + heuristic(neighbor, goal) 
                heapq.heappush(open_set, (f_score[neighbor], neighbor)) 
    return g_score, came_from 
 
def reconstruct_path(prev, start, goal): 
    path = [] 
    cur = goal 
    while cur != start: 
        path.append(cur) 
        cur = prev.get(cur) 
        if cur is None: 
            return [] 
    path.append(start) 
    return path[::-1] 
 
#⑪ 
min_total_distance = float('inf') 
best_fire_node = None 
best_hospital_node = None 
best_paths = () 
start_time = time.time() 
 
for fire in fire_nodes: 
    if algo == "astar": 
        _, prev_f2r = a_star(graph_dict, fire, report_node) 
    else: 
        dist_f, prev_f2r = ( 
            dijkstra(graph_dict, fire) if algo == "dijkstra" else bellman_ford(graph_dict, fire) 
        ) 
    path_f2r = reconstruct_path(prev_f2r, fire, report_node) 
    if not path_f2r: 
        continue 
    dist_f2r = sum(graph_dict[u][v] for u, v in zip(path_f2r[:-1], path_f2r[1:])) 
 
    if algo == "astar": 
        _, prev_r2h = a_star(graph_dict, report_node, hospital_nodes[0]) 
        dist_r = dict((n, float('inf')) for n in graph_dict) 
        dist_r[report_node] = 0 
        for h in hospital_nodes: 
            d, prev = a_star(graph_dict, report_node, h) 
            path = reconstruct_path(prev, report_node, h) 
            dist_r[h] = sum(graph_dict[u][v] for u, v in zip(path[:-1], path[1:])) 
    else:  
        dist_r, prev_r2h = ( 
            dijkstra(graph_dict, report_node) if algo == "dijkstra" else bellman_ford(graph_dict, 
report_node) 
        ) 
    for hospital in hospital_nodes: 
        dist_r2h = dist_r.get(hospital, float('inf')) 
        total_dist = dist_f2r + dist_r2h 
        if total_dist < min_total_distance: 
            min_total_distance = total_dist 
            best_fire_node = fire 
            best_hospital_node = hospital 
            best_paths = (prev_f2r, prev_r2h) 
elapsed_time = time.time() - start_time 
path_fire_to_report = reconstruct_path(best_paths[0], best_fire_node, report_node) 
path_report_to_hosp = reconstruct_path(best_paths[1], report_node, best_hospital_node) 
full_path = path_fire_to_report + path_report_to_hosp[1:] 
 
#⑫ 
def calc_length(G, path): 
    def euclidean(y1, x1, y2, x2): 
        R = 6371000 
        phi1, phi2 = map(math.radians, [y1, y2]) 
        dphi = math.radians(y2 - y1) 
        dlambda = math.radians(x2 - x1) 
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2 
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)) 
    return sum(euclidean(G.nodes[u]['y'], G.nodes[u]['x'], G.nodes[v]['y'], 
G.nodes[v]['x']) for u, v in zip(path[:-1], path[1:])) 
 
hospital_index = hospital_nodes.index(best_hospital_node) 
hospital_name = df.iloc[hospital_index]["name"] 
 
#⑬ 
print("\n【Information】") 
print(f"Algorithm Used: {algo.upper()}") 
print(f"Nearest Hospital: {hospital_name}") 
print(f"FireStation → Point: {int(calc_length(G,path_fire_to_report))} m") 
print(f"Point → Hospital: {int(calc_length(G,path_report_to_hosp[1:]))} m") 
total_distance = calc_length(G,path_fire_to_report) + 
calc_length(G,path_report_to_hosp[1:]) 
print(f"Total Distance: {int(total_distance)} m") 
print(f"Estimated Time: {round(total_distance / 40 / 1000 * 60 + 5, 1)} min") 
print(f"Computation Time: {elapsed_time:.3f} sec") 
 
#⑭ 
rx, ry = G.nodes[report_node]['x'], G.nodes[report_node]['y'] 
fx, fy = G.nodes[best_fire_node]['x'], G.nodes[best_fire_node]['y'] 
hx, hy = G.nodes[best_hospital_node]['x'], G.nodes[best_hospital_node]['y'] 
mid_lat = (G.nodes[report_node]['y'] + G.nodes[best_hospital_node]['y']) / 2 
mid_lon = (G.nodes[report_node]['x'] + G.nodes[best_hospital_node]['x']) / 2 
bbox_large = ox.utils_geo.bbox_from_point((mid_lat, mid_lon), dist=5000) 
bbox_zoom = ox.utils_geo.bbox_from_point((G.nodes[report_node]['y'], 
G.nodes[report_node]['x']), dist=1000) 
route_x = [G.nodes[n]['x'] for n in full_path] 
route_y = [G.nodes[n]['y'] for n in full_path] 
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(20, 10)) 
ox.plot_graph(G, ax=ax1, bbox=bbox_large, node_size=0, edge_color="gray", 
show=False, close=False) 
ax1.plot(route_x, route_y, color="black", linewidth=2, label="Route") 
ax1.scatter(rx, ry, c='blue', s=100, label='Point', zorder=5) 
ax1.scatter(fx, fy, c='red', s=100, label='FireStation', zorder=5) 
ax1.scatter(hx, hy, c='green', s=100, label='Hospital', zorder=5) 
ax1.set_title("Whole Map", fontsize=16) 
ax1.legend() 
ox.plot_graph(G, ax=ax2, bbox=bbox_zoom, node_size=0, edge_color="gray", 
show=False, close=False) 
ax2.plot(route_x, route_y, color="black", linewidth=2, label="Route") 
ax2.scatter(rx, ry, c='blue', s=100, label='Point', zorder=5) 
ax2.scatter(fx, fy, c='red', s=100, label='FireStation', zorder=5) 
ax2.scatter(hx, hy, c='green', s=100, label='Hospital', zorder=5) 
ax2.set_title("Zoomed Map", fontsize=16) 
ax2.legend() 
plt.tight_layout() 
plt.show()
