from flask import Flask, request, jsonify
from flask_cors import CORS
import os
import json
import math
import networkx as nx

app = Flask(__name__)
CORS(app)

# =========================
# 1. ESP32 指令中繼站
# =========================

latest_cmd = {
    "seq": 0,
    "dir": ""
}

@app.route("/")
def home():
    return "ShadeBike command server is running."

@app.route("/set")
def set_command():
    global latest_cmd

    direction = request.args.get("dir", "")

    if direction not in ["L", "R", "S", "A", "X"]:
        return jsonify({
            "ok": False,
            "message": "Invalid direction"
        }), 400

    latest_cmd["seq"] += 1
    latest_cmd["dir"] = direction

    print(f"New command: seq={latest_cmd['seq']}, dir={latest_cmd['dir']}")

    return jsonify({
        "ok": True,
        "seq": latest_cmd["seq"],
        "dir": latest_cmd["dir"]
    })

@app.route("/get")
def get_command():
    return jsonify(latest_cmd)

@app.route("/get_plain")
def get_plain():
    return f"{latest_cmd['seq']},{latest_cmd['dir']}"


# =========================
# 2. 路網資料讀取
# =========================

NODE_FILE = "node.geojson"
ROAD_FILE_8AM = "校內路網_8AM_完成.geojson"

nodes = {}
G_8AM = nx.Graph()


def web_mercator_to_lonlat(x, y):
    lon = x / 20037508.34 * 180
    lat = y / 20037508.34 * 180
    lat = 180 / math.pi * (
        2 * math.atan(math.exp(lat * math.pi / 180)) - math.pi / 2
    )
    return lon, lat


def load_nodes():
    global nodes

    with open(NODE_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    for feature in data["features"]:
        p = feature["properties"]

        node_id = int(p["FID"])
        lon = float(p["X"])
        lat = float(p["Y"])

        nodes[node_id] = {
            "lon": lon,
            "lat": lat
        }

    print(f"Loaded nodes: {len(nodes)}")


def load_road_network():
    global G_8AM

    with open(ROAD_FILE_8AM, "r", encoding="utf-8") as f:
        data = json.load(f)

    edge_count = 0
    missing_node_edges = []

    for feature in data["features"]:
        p = feature["properties"]

        road_id = int(p["FID"])
        start_node = int(p["start_node"])
        final_node = int(p["final_node"])

        length_m = float(p["Shape_Leng"])
        cost = float(p["Cost"])

        if start_node not in nodes or final_node not in nodes:
            missing_node_edges.append({
                "road_id": road_id,
                "start_node": start_node,
                "final_node": final_node
            })
            continue

        # 讀取道路線段 geometry
        coords_3857 = feature["geometry"]["coordinates"]

        geometry_lonlat = []
        for coord in coords_3857:
            x = coord[0]
            y = coord[1]
            lon, lat = web_mercator_to_lonlat(x, y)
            geometry_lonlat.append([lat, lon])  # Leaflet 使用 [lat, lon]

        # 無向圖：代表道路兩方向都可以走
        G_8AM.add_edge(
            start_node,
            final_node,
            weight=cost,
            road_id=road_id,
            length_m=length_m,
            cost=cost,
            geometry=geometry_lonlat
        )

        edge_count += 1

    print(f"Loaded 8AM road edges: {edge_count}")

    if missing_node_edges:
        print("Missing node edges:")
        print(missing_node_edges)


# =========================
# 3. 路徑計算工具
# =========================

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)

    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(d_phi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2
    )

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c


def find_nearest_node(lat, lon):
    nearest_id = None
    nearest_dist = float("inf")

    for node_id, info in nodes.items():
        d = haversine_m(lat, lon, info["lat"], info["lon"])

        if d < nearest_dist:
            nearest_dist = d
            nearest_id = node_id

    return nearest_id, nearest_dist


def bearing_deg(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_lon = math.radians(lon2 - lon1)

    y = math.sin(d_lon) * math.cos(phi2)
    x = (
        math.cos(phi1) * math.sin(phi2)
        - math.sin(phi1) * math.cos(phi2) * math.cos(d_lon)
    )

    brng = math.degrees(math.atan2(y, x))
    return (brng + 360) % 360


def angle_diff_deg(a, b):
    diff = (b - a + 540) % 360 - 180
    return diff


def decide_action(prev_node, current_node, next_node):
    prev_info = nodes[prev_node]
    curr_info = nodes[current_node]
    next_info = nodes[next_node]

    b1 = bearing_deg(
        prev_info["lat"], prev_info["lon"],
        curr_info["lat"], curr_info["lon"]
    )

    b2 = bearing_deg(
        curr_info["lat"], curr_info["lon"],
        next_info["lat"], next_info["lon"]
    )

    diff = angle_diff_deg(b1, b2)

    # 角度門檻之後可以再調
    if diff > 35:
        return "R"
    elif diff < -35:
        return "L"
    else:
        return "S"


def build_route_points(path):
    route = []

    for i, node_id in enumerate(path):
        info = nodes[node_id]

        if i == 0:
            action = "S"
        elif i == len(path) - 1:
            action = "A"
        else:
            action = decide_action(path[i - 1], path[i], path[i + 1])

        route.append({
            "node_id": node_id,
            "lat": info["lat"],
            "lon": info["lon"],
            "action": action
        })

    return route


def build_route_geometry(path):
    full_geometry = []

    for i in range(len(path) - 1):
        u = path[i]
        v = path[i + 1]

        edge_data = G_8AM.get_edge_data(u, v)

        if edge_data is None:
            continue

        geom = edge_data.get("geometry", [])

        if not geom:
            continue

        # 判斷 geometry 方向是否跟 path 方向一致
        start_geom = geom[0]
        end_geom = geom[-1]

        u_info = nodes[u]

        d_start_to_u = haversine_m(
            start_geom[0], start_geom[1],
            u_info["lat"], u_info["lon"]
        )

        d_end_to_u = haversine_m(
            end_geom[0], end_geom[1],
            u_info["lat"], u_info["lon"]
        )

        # 如果線段尾端比較接近 u，代表 geometry 方向反了，要反轉
        if d_end_to_u < d_start_to_u:
            geom = list(reversed(geom))

        # 避免相鄰線段接點重複
        if len(full_geometry) > 0:
            full_geometry.extend(geom[1:])
        else:
            full_geometry.extend(geom)

    return full_geometry


# =========================
# 4. /route API
# =========================

@app.route("/route")
def route():
    try:
        start_lat = float(request.args.get("start_lat"))
        start_lon = float(request.args.get("start_lon"))
        dest_lat = float(request.args.get("dest_lat"))
        dest_lon = float(request.args.get("dest_lon"))
    except:
        return jsonify({
            "ok": False,
            "message": "Missing or invalid parameters. Use start_lat, start_lon, dest_lat, dest_lon."
        }), 400

    start_node, start_dist = find_nearest_node(start_lat, start_lon)
    dest_node, dest_dist = find_nearest_node(dest_lat, dest_lon)

    if start_node is None or dest_node is None:
        return jsonify({
            "ok": False,
            "message": "Could not find nearest node."
        }), 400

    try:
        path = nx.shortest_path(
            G_8AM,
            source=start_node,
            target=dest_node,
            weight="weight"
        )

        total_cost = nx.shortest_path_length(
            G_8AM,
            source=start_node,
            target=dest_node,
            weight="weight"
        )

    except nx.NetworkXNoPath:
        return jsonify({
            "ok": False,
            "message": "No path found between start and destination.",
            "start_node": start_node,
            "dest_node": dest_node
        }), 400

    route_points = build_route_points(path)
    route_geometry = build_route_geometry(path)

    return jsonify({
        "ok": True,
        "time_slot": "8AM",
        "start_node": start_node,
        "dest_node": dest_node,
        "start_distance_m": start_dist,
        "dest_distance_m": dest_dist,
        "node_path": path,
        "total_cost": total_cost,
        "route": route_points,
        "route_geometry": route_geometry
    })


# =========================
# 5. 啟動時讀取資料
# =========================

load_nodes()
load_road_network()


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5050))
    app.run(host="0.0.0.0", port=port)
