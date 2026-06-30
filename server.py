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

NODE_FILE = "node_new.geojson"

ROAD_FILES = {
    "8AM": "校內路網_8AM_完成.geojson",
    "5PM": "校內路網_5PM_完成.geojson"
}

nodes = {}
GRAPHS = {}

START_VIRTUAL = "__start__"
DEST_VIRTUAL = "__dest__"


def web_mercator_to_lonlat(x, y):
    lon = x / 20037508.34 * 180
    lat = y / 20037508.34 * 180
    lat = 180 / math.pi * (
        2 * math.atan(math.exp(lat * math.pi / 180)) - math.pi / 2
    )
    return lon, lat


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


def polyline_length_m(geom):
    total = 0.0

    for i in range(len(geom) - 1):
        total += haversine_m(
            geom[i][0], geom[i][1],
            geom[i + 1][0], geom[i + 1][1]
        )

    return total


def cumulative_distances(geom):
    cum = [0.0]
    total = 0.0

    for i in range(len(geom) - 1):
        seg_len = haversine_m(
            geom[i][0], geom[i][1],
            geom[i + 1][0], geom[i + 1][1]
        )
        total += seg_len
        cum.append(total)

    return cum


def point_at_distance(geom, cum, target_dist):
    if len(geom) == 0:
        return None

    if target_dist <= 0:
        return geom[0]

    if target_dist >= cum[-1]:
        return geom[-1]

    for i in range(len(cum) - 1):
        if cum[i] <= target_dist <= cum[i + 1]:
            seg_len = cum[i + 1] - cum[i]

            if seg_len == 0:
                return geom[i]

            t = (target_dist - cum[i]) / seg_len

            lat = geom[i][0] + t * (geom[i + 1][0] - geom[i][0])
            lon = geom[i][1] + t * (geom[i + 1][1] - geom[i][1])

            return [lat, lon]

    return geom[-1]


def slice_geometry_by_distance(geom, d1, d2):
    if not geom or len(geom) < 2:
        return []

    cum = cumulative_distances(geom)

    reverse = False

    if d1 > d2:
        d1, d2 = d2, d1
        reverse = True

    d1 = max(0.0, min(d1, cum[-1]))
    d2 = max(0.0, min(d2, cum[-1]))

    start_pt = point_at_distance(geom, cum, d1)
    end_pt = point_at_distance(geom, cum, d2)

    sliced = [start_pt]

    for i in range(1, len(geom) - 1):
        if d1 < cum[i] < d2:
            sliced.append(geom[i])

    sliced.append(end_pt)

    if reverse:
        sliced = list(reversed(sliced))

    return sliced


def load_nodes():
    global nodes

    with open(NODE_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)

    nodes = {}

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


def load_road_network(time_slot, road_file):
    G = nx.Graph()

    with open(road_file, "r", encoding="utf-8") as f:
        data = json.load(f)

    edge_count = 0
    missing_node_edges = []

    for feature in data["features"]:
        p = feature["properties"]

        road_id = int(p["FID"])

        # 新版資料已經可以直接對 node_new.geojson 的 FID
        # 不要再 -1
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

        coords_3857 = feature["geometry"]["coordinates"]

        geometry_lonlat = []

        for coord in coords_3857:
            x = coord[0]
            y = coord[1]
            lon, lat = web_mercator_to_lonlat(x, y)
            geometry_lonlat.append([lat, lon])  # Leaflet 使用 [lat, lon]

        if len(geometry_lonlat) < 2:
            continue

        # 確保 geometry 方向是 start_node -> final_node
        start_info = nodes[start_node]

        d_geom_start_to_start_node = haversine_m(
            geometry_lonlat[0][0], geometry_lonlat[0][1],
            start_info["lat"], start_info["lon"]
        )

        d_geom_end_to_start_node = haversine_m(
            geometry_lonlat[-1][0], geometry_lonlat[-1][1],
            start_info["lat"], start_info["lon"]
        )

        if d_geom_end_to_start_node < d_geom_start_to_start_node:
            geometry_lonlat = list(reversed(geometry_lonlat))

        geom_length = polyline_length_m(geometry_lonlat)

        if geom_length <= 0:
            geom_length = length_m

        cost_per_m = cost / geom_length if geom_length > 0 else 1.0

        # 如果同一對 start/final 已經有 edge，保留 cost 較低的那條
        if G.has_edge(start_node, final_node):
            old_cost = G[start_node][final_node].get("cost", float("inf"))

            if old_cost <= cost:
                continue

        G.add_edge(
            start_node,
            final_node,
            weight=cost,
            road_id=road_id,
            start_node=start_node,
            final_node=final_node,
            length_m=length_m,
            geom_length_m=geom_length,
            cost=cost,
            cost_per_m=cost_per_m,
            geometry=geometry_lonlat
        )

        edge_count += 1

    print(f"Loaded {time_slot} road edges: {edge_count}")

    if missing_node_edges:
        print(f"Missing node edges in {time_slot}:")
        print(missing_node_edges)
    else:
        print(f"{time_slot}: all road edges match node_new.geojson")

    return G


def load_all_road_networks():
    global GRAPHS

    GRAPHS = {}

    for time_slot, road_file in ROAD_FILES.items():
        if os.path.exists(road_file):
            GRAPHS[time_slot] = load_road_network(time_slot, road_file)
        else:
            print(f"Road file not found for {time_slot}: {road_file}")


# =========================
# 3. 最近道路點投影工具
# =========================

def latlon_to_local_xy(lat, lon, lat0, lon0):
    R = 6371000
    x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R
    return x, y


def local_xy_to_latlon(x, y, lat0, lon0):
    R = 6371000
    lat = lat0 + math.degrees(y / R)
    lon = lon0 + math.degrees(x / (R * math.cos(math.radians(lat0))))
    return lat, lon


def project_point_to_segment(point_lat, point_lon, a_lat, a_lon, b_lat, b_lon):
    lat0 = point_lat
    lon0 = point_lon

    px, py = latlon_to_local_xy(point_lat, point_lon, lat0, lon0)
    ax, ay = latlon_to_local_xy(a_lat, a_lon, lat0, lon0)
    bx, by = latlon_to_local_xy(b_lat, b_lon, lat0, lon0)

    abx = bx - ax
    aby = by - ay

    ab_len2 = abx * abx + aby * aby

    if ab_len2 == 0:
        return {
            "lat": a_lat,
            "lon": a_lon,
            "t": 0.0,
            "distance_m": haversine_m(point_lat, point_lon, a_lat, a_lon)
        }

    apx = px - ax
    apy = py - ay

    t = (apx * abx + apy * aby) / ab_len2
    t = max(0.0, min(1.0, t))

    qx = ax + t * abx
    qy = ay + t * aby

    q_lat, q_lon = local_xy_to_latlon(qx, qy, lat0, lon0)

    d = haversine_m(point_lat, point_lon, q_lat, q_lon)

    return {
        "lat": q_lat,
        "lon": q_lon,
        "t": t,
        "distance_m": d
    }


def find_nearest_point_on_edges(G, lat, lon):
    best = None

    for u, v, data in G.edges(data=True):
        geom = data.get("geometry", [])

        if len(geom) < 2:
            continue

        cum = cumulative_distances(geom)

        for i in range(len(geom) - 1):
            a = geom[i]
            b = geom[i + 1]

            projection = project_point_to_segment(
                lat, lon,
                a[0], a[1],
                b[0], b[1]
            )

            seg_len = haversine_m(a[0], a[1], b[0], b[1])
            along_m = cum[i] + projection["t"] * seg_len

            candidate = {
                "lat": projection["lat"],
                "lon": projection["lon"],
                "distance_m": projection["distance_m"],
                "u": u,
                "v": v,
                "road_id": data.get("road_id"),
                "edge_cost": data.get("cost"),
                "edge_length_m": data.get("length_m"),
                "geom_length_m": data.get("geom_length_m"),
                "cost_per_m": data.get("cost_per_m"),
                "along_m": along_m,
                "geometry": geom
            }

            if best is None or candidate["distance_m"] < best["distance_m"]:
                best = candidate

    return best


def add_virtual_snap_node(G_temp, virtual_id, snap):
    lat = snap["lat"]
    lon = snap["lon"]

    u = snap["u"]
    v = snap["v"]

    geom = snap["geometry"]
    total_len = snap["geom_length_m"]
    along = snap["along_m"]
    cost_per_m = snap["cost_per_m"]

    G_temp.add_node(
        virtual_id,
        lat=lat,
        lon=lon,
        node_type="virtual_snap"
    )

    dist_to_u = along
    dist_to_v = max(0.0, total_len - along)

    cost_to_u = dist_to_u * cost_per_m
    cost_to_v = dist_to_v * cost_per_m

    geom_to_u = slice_geometry_by_distance(geom, along, 0.0)
    geom_to_v = slice_geometry_by_distance(geom, along, total_len)

    if len(geom_to_u) < 2:
        geom_to_u = [[lat, lon], [nodes[u]["lat"], nodes[u]["lon"]]]

    if len(geom_to_v) < 2:
        geom_to_v = [[lat, lon], [nodes[v]["lat"], nodes[v]["lon"]]]

    G_temp.add_edge(
        virtual_id,
        u,
        weight=cost_to_u,
        cost=cost_to_u,
        length_m=dist_to_u,
        road_id=snap["road_id"],
        geometry=geom_to_u,
        edge_type="virtual_to_u"
    )

    G_temp.add_edge(
        virtual_id,
        v,
        weight=cost_to_v,
        cost=cost_to_v,
        length_m=dist_to_v,
        road_id=snap["road_id"],
        geometry=geom_to_v,
        edge_type="virtual_to_v"
    )


def add_direct_snap_edge_if_same_road(G_temp, start_snap, dest_snap):
    same_edge = (
        start_snap["road_id"] == dest_snap["road_id"]
        and {start_snap["u"], start_snap["v"]} == {dest_snap["u"], dest_snap["v"]}
    )

    if not same_edge:
        return

    geom = start_snap["geometry"]
    cost_per_m = start_snap["cost_per_m"]

    d1 = start_snap["along_m"]
    d2 = dest_snap["along_m"]

    direct_geom = slice_geometry_by_distance(geom, d1, d2)
    direct_len = abs(d2 - d1)

    if len(direct_geom) < 2:
        direct_geom = [
            [start_snap["lat"], start_snap["lon"]],
            [dest_snap["lat"], dest_snap["lon"]]
        ]

    direct_cost = direct_len * cost_per_m

    G_temp.add_edge(
        START_VIRTUAL,
        DEST_VIRTUAL,
        weight=direct_cost,
        cost=direct_cost,
        length_m=direct_len,
        road_id=start_snap["road_id"],
        geometry=direct_geom,
        edge_type="virtual_direct_same_edge"
    )


# =========================
# 4. 路徑輸出工具
# =========================

def get_node_position(G, node_id):
    if node_id in G.nodes:
        data = G.nodes[node_id]
        return data["lat"], data["lon"]

    if isinstance(node_id, int) and node_id in nodes:
        return nodes[node_id]["lat"], nodes[node_id]["lon"]

    raise KeyError(f"Node position not found: {node_id}")


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


def decide_action_by_coords(prev_pt, curr_pt, next_pt):
    b1 = bearing_deg(
        prev_pt["lat"], prev_pt["lon"],
        curr_pt["lat"], curr_pt["lon"]
    )

    b2 = bearing_deg(
        curr_pt["lat"], curr_pt["lon"],
        next_pt["lat"], next_pt["lon"]
    )

    diff = angle_diff_deg(b1, b2)

    if diff > 35:
        return "R"
    elif diff < -35:
        return "L"
    else:
        return "S"


def build_route_points(G_temp, path):
    route = []
    temp_points = []

    for node_id in path:
        lat, lon = get_node_position(G_temp, node_id)

        if node_id == START_VIRTUAL:
            label = "S"
        elif node_id == DEST_VIRTUAL:
            label = "E"
        else:
            label = str(node_id)

        temp_points.append({
            "node_id": label,
            "raw_node_id": str(node_id),
            "lat": lat,
            "lon": lon
        })

    for i, point in enumerate(temp_points):
        if i == 0:
            action = "S"
        elif i == len(temp_points) - 1:
            action = "A"
        else:
            action = decide_action_by_coords(
                temp_points[i - 1],
                temp_points[i],
                temp_points[i + 1]
            )

        route.append({
            "node_id": point["node_id"],
            "raw_node_id": point["raw_node_id"],
            "lat": point["lat"],
            "lon": point["lon"],
            "action": action
        })

    return route


def build_route_geometry_from_temp_graph(G_temp, path):
    full_geometry = []

    for i in range(len(path) - 1):
        u = path[i]
        v = path[i + 1]

        edge_data = G_temp.get_edge_data(u, v)

        if edge_data is None:
            continue

        geom = edge_data.get("geometry", [])

        if not geom or len(geom) < 2:
            continue

        # 複製一份，避免改到 graph 原始 geometry
        geom = [list(pt) for pt in geom]

        u_lat, u_lon = get_node_position(G_temp, u)
        v_lat, v_lon = get_node_position(G_temp, v)

        # 確保 geometry 方向是 u -> v
        d_start_to_u = haversine_m(
            geom[0][0], geom[0][1],
            u_lat, u_lon
        )

        d_end_to_u = haversine_m(
            geom[-1][0], geom[-1][1],
            u_lat, u_lon
        )

        if d_end_to_u < d_start_to_u:
            geom = list(reversed(geom))

        # 重要：
        # 強制每一段 geometry 的起點、終點對齊 path 的 u、v
        # 避免藍線跳過中間節點，例如 4 -> E
        geom[0] = [u_lat, u_lon]
        geom[-1] = [v_lat, v_lon]

        if len(full_geometry) > 0:
            full_geometry.extend(geom[1:])
        else:
            full_geometry.extend(geom)

    return full_geometry


def build_edge_info(G_temp, path):
    edge_info = []

    for i in range(len(path) - 1):
        u = path[i]
        v = path[i + 1]

        e = G_temp.get_edge_data(u, v)

        if not e:
            continue

        edge_info.append({
            "from": str(u),
            "to": str(v),
            "road_id": e.get("road_id"),
            "length_m": e.get("length_m"),
            "cost": e.get("cost"),
            "edge_type": e.get("edge_type", "normal")
        })

    return edge_info


# =========================
# 5. /route API
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

    time_slot = request.args.get("time_slot", "8AM").upper()

    if time_slot not in GRAPHS:
        return jsonify({
            "ok": False,
            "message": f"Invalid or unavailable time_slot: {time_slot}. Use 8AM or 5PM.",
            "available_time_slots": list(GRAPHS.keys())
        }), 400

    G_base = GRAPHS[time_slot]

    start_snap = find_nearest_point_on_edges(G_base, start_lat, start_lon)
    dest_snap = find_nearest_point_on_edges(G_base, dest_lat, dest_lon)

    if start_snap is None or dest_snap is None:
        return jsonify({
            "ok": False,
            "message": "Could not snap start or destination to road network."
        }), 400

    G_temp = G_base.copy()

    # 把原本 nodes 座標加進 graph node attributes
    for node_id, info in nodes.items():
        if node_id in G_temp.nodes:
            G_temp.nodes[node_id]["lat"] = info["lat"]
            G_temp.nodes[node_id]["lon"] = info["lon"]
            G_temp.nodes[node_id]["node_type"] = "real_node"

    add_virtual_snap_node(G_temp, START_VIRTUAL, start_snap)
    add_virtual_snap_node(G_temp, DEST_VIRTUAL, dest_snap)
    add_direct_snap_edge_if_same_road(G_temp, start_snap, dest_snap)

    try:
        path = nx.shortest_path(
            G_temp,
            source=START_VIRTUAL,
            target=DEST_VIRTUAL,
            weight="weight"
        )

        total_cost = nx.shortest_path_length(
            G_temp,
            source=START_VIRTUAL,
            target=DEST_VIRTUAL,
            weight="weight"
        )

    except nx.NetworkXNoPath:
        return jsonify({
            "ok": False,
            "message": "No path found between start snap and destination snap.",
            "time_slot": time_slot,
            "start_snap": {
                "lat": start_snap["lat"],
                "lon": start_snap["lon"],
                "distance_m": start_snap["distance_m"],
                "road_id": start_snap["road_id"],
                "edge_u": start_snap["u"],
                "edge_v": start_snap["v"]
            },
            "dest_snap": {
                "lat": dest_snap["lat"],
                "lon": dest_snap["lon"],
                "distance_m": dest_snap["distance_m"],
                "road_id": dest_snap["road_id"],
                "edge_u": dest_snap["u"],
                "edge_v": dest_snap["v"]
            }
        }), 400

    route_points = build_route_points(G_temp, path)
    route_geometry = build_route_geometry_from_temp_graph(G_temp, path)
    edge_info = build_edge_info(G_temp, path)

    return jsonify({
        "ok": True,
        "time_slot": time_slot,

        "start_snap": {
            "lat": start_snap["lat"],
            "lon": start_snap["lon"],
            "distance_m": start_snap["distance_m"],
            "road_id": start_snap["road_id"],
            "edge_u": start_snap["u"],
            "edge_v": start_snap["v"],
            "along_m": start_snap["along_m"]
        },

        "dest_snap": {
            "lat": dest_snap["lat"],
            "lon": dest_snap["lon"],
            "distance_m": dest_snap["distance_m"],
            "road_id": dest_snap["road_id"],
            "edge_u": dest_snap["u"],
            "edge_v": dest_snap["v"],
            "along_m": dest_snap["along_m"]
        },

        "node_path": [str(n) for n in path],
        "total_cost": total_cost,
        "route": route_points,
        "route_geometry": route_geometry,
        "edge_info": edge_info
    })


# =========================
# 6. 啟動時讀取資料
# =========================

load_nodes()
load_all_road_networks()


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 5050))
    app.run(host="0.0.0.0", port=port)
