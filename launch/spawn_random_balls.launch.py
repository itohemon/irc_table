import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # モデルファイルのパス設定
    model_base_path = os.path.join(get_package_share_directory('irc_table'),
                                   'models')
    color_models = {
        "red": os.path.join(model_base_path, "ball_red", "model.sdf"),
        "yellow": os.path.join(model_base_path, "ball_yellow", "model.sdf"),
        "blue": os.path.join(model_base_path, "ball_blue", "model.sdf"),
    }

    # 8つの矩形エリア [ (左下x, 左下y, 右上x, 右上y), ... ]
    areas = [
        (1.000, -0.980, 1.300, -1.340),   # D1
        (1.000, -1.340, 1.300, -1.700),   # D2
        (0.700, -0.980, 1.000, -1.340),   # D3
        (0.700, -1.340, 1.000, -1.700),   # D4
        (0.400, -0.980, 0.700, -1.340),   # B1
        (0.400, -1.340, 0.700, -1.700),   # B2
        (0.100, -0.980, 0.400, -1.340),   # C1
        (0.100, -1.340, 0.400, -1.700),   # C2
    ]

    # 各色の割り当て（各5個）
    color_list = ["red"] * 5 + ["yellow"] * 5 + ["blue"] * 5
    random.shuffle(color_list)  # 配置をランダム化

    ball_radius = 0.033
    safe_distance = ball_radius * 2  # 最低距離を2倍程度にして重なり防止

    area_ball_map = {i: [] for i in range(len(areas))}  # 各エリアの配置済みボール座標
    nodes = []

    for i, color in enumerate(color_list):
        model_file = color_models[color]

        # 配置可能なエリア（2個未満のエリア）
        candidate_area_indices = [idx for idx, balls in area_ball_map.items() if len(balls) < 2]
        if not candidate_area_indices:
            print("Error: 全エリアが上限に達しています。")
            break

        # ランダムなエリア選択と重なりチェック
        placed = False
        while not placed and candidate_area_indices:
            area_idx = random.choice(candidate_area_indices)
            x0, y0, x1, y1 = areas[area_idx]

            # ランダムな位置を試行
            for _ in range(20):  # 試行上限（安全距離確保のため）
                x = random.uniform(x0, x1)
                y = random.uniform(y0, y1)

                if all(((x - px)**2 + (y - py)**2)**0.5 >= safe_distance for px, py in area_ball_map[area_idx]):
                    area_ball_map[area_idx].append((x, y))
                    placed = True
                    break

            if not placed:
                candidate_area_indices.remove(area_idx)  # このエリアを候補から除外

        if not placed:
            print(f"Warning: {color} ボールを配置できませんでした。")
            continue

        z = ball_radius + 0.29

        nodes.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', f'ball_{i}_{color}',
                    '-file', model_file,
                    '-x', str(round(x, 3)),
                    '-y', str(round(y, 3)),
                    '-z', str(z) 
                ],
                output='screen'
            )
        )

    return LaunchDescription(nodes)


