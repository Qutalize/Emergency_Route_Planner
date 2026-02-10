!pip install matplotlib
!pip install osmnx==2.0.3

# 0. ライブラリのインポート
import osmnx as ox
import networkx as nx
import heapq
import time
import math
import pandas as pd
import matplotlib.pyplot as plt
from geopy.geocoders import Nominatim
from shapely.geometry import Point, LineString

# 1. 共通処理
def get_dist_meters(lat1, lon1, lat2, lon2):
    """ 2点間の直線距離を計算 """
    R = 6371000
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2) * math.sin(dlambda/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c

def heuristic(u, v, G):
    if 'x' not in G.nodes[u] or 'x' not in G.nodes[v]:
        return 0
    dy = (G.nodes[u]['y'] - G.nodes[v]['y']) * 111000
    dx = (G.nodes[u]['x'] - G.nodes[v]['x']) * 91000
    return math.sqrt(dx*dx + dy*dy)

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]

def run_search(algo_name, G, start, goal, time_limit):
    """ 経路探索実行 (Time Budget対応) """
    start_time = time.time()
    dist = {n: float('inf') for n in G.nodes}
    dist[start] = 0
    came_from = {}
    
    success = False
    closest_node = start
    min_h = heuristic(start, goal, G)

    # Dijkstra / A*
    if algo_name in ["Dijkstra", "A*"]:
        pq = [(0, start)]
        while pq:
            if time.time() - start_time > time_limit:
                break
            
            _, u = heapq.heappop(pq)
            
            if u == goal:
                success = True
                break
            
            h_u = heuristic(u, goal, G)
            if h_u < min_h:
                min_h = h_u
                closest_node = u
            
            for v, edge_data in G[u].items():
                weight = min(d.get('length', 1) for d in edge_data.values())
                alt = dist[u] + weight
                if alt < dist[v]:
                    dist[v] = alt
                    came_from[v] = u
                    priority = alt
                    if algo_name == "A*":
                        priority += heuristic(v, goal, G)
                    heapq.heappush(pq, (priority, v))

    # Bellman-Ford
    elif algo_name == "Bellman-Ford":
        edges = []
        for u, v, d in G.edges(data=True):
            edges.append((u, v, d.get('length', 1)))
            
        for _ in range(len(G.nodes) - 1):
            if time.time() - start_time > time_limit:
                break
            changed = False
            for u, v, w in edges:
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    came_from[v] = u
                    changed = True
                    h_v = heuristic(v, goal, G)
                    if h_v < min_h:
                        min_h = h_v
                        closest_node = v
            if not changed:
                if dist[goal] != float('inf'): success = True
                break

    elapsed = time.time() - start_time
    target = goal if success else closest_node
    path = []
    if target in came_from or target == start:
        path = reconstruct_path(came_from, target)
        
    total_len = 0
    if path:
        for u, v in zip(path[:-1], path[1:]):
            edge_data = G.get_edge_data(u, v)
            if edge_data:
                total_len += min(d.get('length', 0) for d in edge_data.values())

    return path, success, elapsed, total_len

# 2. メイン処理
print("設定")
geolocator = Nominatim(user_agent="colab_router_viz_fix")
default_addr = "Kawasaki Station, Kanagawa, Japan"
address = input(f"通報地点 (例: {default_addr}): ") or default_addr

# 2.0 通報地点の特定
try:
    loc = geolocator.geocode(address)
    if not loc: raise ValueError("住所が見つかりません")
    report_lat, report_lon = loc.latitude, loc.longitude
    print(f"通報地点: {loc.address}")
except Exception as e:
    print(f"エラー: {e}")
    report_lat, report_lon = 35.5313, 139.6970

limit_str = input("探索制限時間[秒] (例: 1.0): ") or "1.0"
time_limit = float(limit_str)
print("\n最寄りの施設座標を計算中...")

# 2.1 病院 (from CSV)
hosp_target_lat, hosp_target_lon = report_lat, report_lon
hosp_name = "Unknown"
try:
    try: df = pd.read_csv("hospitals.csv", encoding="utf-8")
    except: df = pd.read_csv("hospitals.csv", encoding="shift_jis")
    
    min_dist = float('inf')
    for _, row in df.iterrows():
        val1 = row.get('latitude', row.get('lat', None))
        val2 = row.get('longitude', row.get('lon', None))
        try:
            val1 = float(val1)
            val2 = float(val2)
            if math.isnan(val1) or math.isnan(val2): continue
        except: continue
        
        if val1 > 100: lon, lat = val1, val2
        else: lat, lon = val1, val2
            
        d = get_dist_meters(report_lat, report_lon, lat, lon)
        if d < min_dist:
            min_dist = d
            hosp_target_lat, hosp_target_lon = lat, lon
            hosp_name = row.get('name', 'CSV Hospital')
    print(f"最寄り病院特定: {hosp_name} (直線距離 {int(min_dist)}m)")
except:
    pass

# 2.2 消防署 (from OSM Features API)
print("最寄りの消防署を検索中...")
fire_target_lat, fire_target_lon = report_lat, report_lon
fire_name = "Unknown Fire Station"
try:
    tags = {'amenity': 'fire_station'}
    fire_pois = ox.features_from_point((report_lat, report_lon), tags=tags, dist=5000)
    if not fire_pois.empty:
        min_dist = float('inf')
        for idx, row in fire_pois.iterrows():
            geom = row.geometry
            if geom.geom_type != 'Point': geom = geom.centroid
            d = get_dist_meters(report_lat, report_lon, geom.y, geom.x)
            if d < min_dist:
                min_dist = d
                fire_target_lat, fire_target_lon = geom.y, geom.x
                fire_name = row.get('name', 'Nearest FireStation')
        print(f"最寄り消防署特定: {fire_name} (直線距離 {int(min_dist)}m)")
except:
    pass

# 3. 地図データ取得
print(f"\n半径5kmの地図データをダウンロード中...")
try:
    G = ox.graph_from_point((report_lat, report_lon), dist=5000, network_type='drive')
    G = ox.convert.to_undirected(G)
    for i, data in G.nodes(data=True):
        data['x'] = data.get('x', 0)
        data['y'] = data.get('y', 0)
    print(f"グラフ準備完了: {len(G.nodes)} nodes")
except Exception as e:
    print(f"地図取得エラー: {e}")
    raise e

# ノードマッピング
report_node = ox.nearest_nodes(G, report_lon, report_lat)
hosp_node = ox.nearest_nodes(G, hosp_target_lon, hosp_target_lat)
fire_node = ox.nearest_nodes(G, fire_target_lon, fire_target_lat)

# 4. アルゴリズム実行 & 表作成
algos = ["Dijkstra", "A*", "Bellman-Ford"]
results_list = []
paths_for_plot = {}

print(f"\n経路探索開始 (制限時間: {time_limit}秒/区間)...")

for algo in algos:
    # ①Fire -> Report
    path1, ok1, t1, d1 = run_search(algo, G, fire_node, report_node, time_limit)
    # ②Report -> Hospital
    path2, ok2, t2, d2 = run_search(algo, G, report_node, hosp_node, time_limit)
    
    total_dist = d1 + d2
    est_time_min = round(total_dist / 40 / 1000 * 60 + 5, 1)
    
    full_path = []
    if path1: full_path.extend(path1)
    if path2: full_path.extend(path2[1:])
    
    status_str = "Complete"
    if not ok1: status_str = "Timeout (Fire->Point)"
    if not ok2: status_str = "Timeout (Point->Hosp)"
    if not ok1 and not ok2: status_str = "Timeout (Both)"

    results_list.append({
        "Algorithm": algo,
        "Status": status_str,
        "Fire->Point (m)": int(d1),
        "Point->Hosp (m)": int(d2),
        "Total Dist (m)": int(total_dist),
        "Est Time (min)": est_time_min,
        "Comp Time (sec)": round(t1+t2, 4)
    })
    
    paths_for_plot[algo] = {"path": full_path, "ok": ok1 and ok2}

# 5.表出力
df_results = pd.DataFrame(results_list)
print("\n" + "="*80)
print(f"探索結果一覧 (最寄り病院: {hosp_name})")
print("="*80)
print(df_results.to_markdown(index=False))
print("="*80)

# 6. 可視化
bbox_5km = ox.utils_geo.bbox_from_point((report_lat, report_lon), dist=3000)
fig, axes = plt.subplots(3, 1, figsize=(10, 24))
plt.subplots_adjust(hspace=0.3)
fx, fy = G.nodes[fire_node]['x'], G.nodes[fire_node]['y']
rx, ry = G.nodes[report_node]['x'], G.nodes[report_node]['y']
hx, hy = G.nodes[hosp_node]['x'], G.nodes[hosp_node]['y']

for ax, algo in zip(axes, algos):
    res = paths_for_plot[algo]
    path = res["path"]
    ox.plot_graph(G, ax=ax, bbox=bbox_5km, node_size=0, 
                  edge_color='#bbbbbb', edge_linewidth=0.8,
                  show=False, close=False)
    if path and len(path) > 1:
        route_x = [G.nodes[n]['x'] for n in path]
        route_y = [G.nodes[n]['y'] for n in path]
        ax.plot(route_x, route_y, color="white", linewidth=5, alpha=0.7, zorder=4)
        ax.plot(route_x, route_y, color="blue", linewidth=2.5, label="Route", zorder=5)
        if not res["ok"]:
            ax.scatter(route_x[-1], route_y[-1], c='orange', s=200, marker='X', label='Timeout', zorder=7)

    ax.scatter(fx, fy, c='red', s=150, label='Fire Station', zorder=6, edgecolors='white')
    ax.scatter(rx, ry, c='gold', s=150, label='Incident Point', zorder=6, edgecolors='black')
    ax.scatter(hx, hy, c='green', s=150, label='Hospital', zorder=6, edgecolors='white')
    ax.set_title(f"Algorithm: {algo}", fontsize=16, fontweight='bold')
    ax.legend(loc='upper right', frameon=True, framealpha=0.9)

plt.show()
