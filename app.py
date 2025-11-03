# Tên file: app.py
# (Phiên bản cập nhật - Thêm API lấy tất cả node)

import osmnx as ox
import math
import random 
from heapq import heappush, heappop
from flask import Flask, jsonify
from flask_cors import CORS

print("Bắt đầu khởi động server...")

# === TẢI BẢN ĐỒ ===
print("Đang tải file bản đồ phuong_tuong_mai.graphml...")
G = ox.load_graphml("phuong_tuong_mai.graphml")

# === LỌC RA CÁC NGÃ GIAO ===
print("Đang lọc ra các ngã giao (junctions)...")
degrees = G.degree()
junctions = [node for node, degree in degrees if degree != 2]
print(f"Đã tìm thấy {len(junctions)} ngã giao. Server đã sẵn sàng!")


# === KHỞI TẠO FLASK (Web Server) ===
app = Flask(__name__)
CORS(app) # Cho phép web front-end của bạn gọi API này

# === HÀM A* VÀ HAVERSINE ===
def haversine(node1, node2):
    lat1, lon1 = G.nodes[node1]["y"], G.nodes[node1]["x"]
    lat2, lon2 = G.nodes[node2]["y"], G.nodes[node2]["x"]
    R = 6371e3
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def a_star(graph, start, goal):
    open_set = []
    heappush(open_set, (0, start))
    came_from = {}
    g_score = {n: float("inf") for n in graph.nodes}
    g_score[start] = 0
    f_score = {n: float("inf") for n in graph.nodes}
    f_score[start] = haversine(start, goal)

    while open_set:
        _, current = heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1] # Trả về đường đi

        for neighbor in graph.neighbors(current):
            edge_data = graph[current][neighbor][0]
            weight = edge_data.get("length", haversine(current, neighbor))
            tentative_g = g_score[current] + weight
            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + haversine(neighbor, goal)
                heappush(open_set, (f_score[neighbor], neighbor))
    return None # Không tìm thấy đường

# === API TÌM ĐƯỜNG NGẪU NHIÊN ===
@app.route('/api/random-junction-path', methods=['GET'])
def find_random_path_api():
    try:
        start_node = random.choice(junctions)
        end_node = random.choice(junctions)
        while start_node == end_node:
            end_node = random.choice(junctions)

        path_nodes = a_star(G, start_node, end_node)

        if path_nodes:
            path_coords = [(G.nodes[n]["y"], G.nodes[n]["x"]) for n in path_nodes]
            start_coord = (G.nodes[start_node]["y"], G.nodes[start_node]["x"])
            end_coord = (G.nodes[end_node]["y"], G.nodes[end_node]["x"])
            
            return jsonify({
                "status": "success",
                "path": path_coords,
                "start_point": start_coord,
                "end_point": end_coord
            })
        else:
            return jsonify({"status": "error", "message": "Không tìm thấy đường đi"}), 404

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

# ===  API LẤY TẤT CẢ CÁC NODE ===
@app.route('/api/all-nodes', methods=['GET'])
def get_all_nodes():
    try:
        # Lấy tọa độ [lat, lon] của TẤT CẢ các node
        # G.nodes(data=True) trả về (node_id, data_dict)
        all_nodes_coords = [(data["y"], data["x"]) for _, data in G.nodes(data=True)]
        
        return jsonify({
            "status": "success",
            "nodes": all_nodes_coords
        })
    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500


# === CHẠY SERVER ===
if __name__ == '__main__':
    app.run(debug=True, port=5000)
