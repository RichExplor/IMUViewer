# -*- coding: utf-8 -*-
"""
OBJ 模型加载器 — 针对 pyqtgraph.opengl 的 GLMeshItem 优化

功能:
1. 解析 OBJ 文件中的顶点(v)、法线(vn)、面片(f)、材质(usemtl)
2. 按材质分组生成 gl.MeshData 对象列表
3. 自动居中 + 缩放模型以适配指定显示范围
4. 材质名 → RGBA 颜色映射
5. .npy 缓存机制: 首次解析后保存，后续直接加载缓存（秒级启动）
6. 跳过线段(l)和纹理坐标(vt)等不支持的元素
"""

import os
import hashlib
import numpy as np
import pyqtgraph.opengl as gl


# ==================== 材质名 → 颜色映射表 ====================

# 关键词 → (R, G, B, A)  映射，按优先级从高到低匹配
MATERIAL_COLOR_MAP = [
    # 轮胎 / 橡胶 — 深黑
    (['tyre', 'tire'], (0.12, 0.12, 0.12, 1.0)),
    # 黑色系列
    (['black_cable', 'black_fuel_tank', 'black_strip', 'black_vent',
      'black_side_vent', 'black'], (0.10, 0.10, 0.10, 1.0)),
    # 刹车盘 — 深灰金属
    (['brake_disk'], (0.25, 0.25, 0.28, 1.0)),
    # 卡钳 — 红色
    (['cALLIPERS', 'caliper'], (0.75, 0.15, 0.15, 1.0)),
    # 轮毂银色
    (['rim_silver', 'aluminium', 'aluminiumm'], (0.75, 0.75, 0.78, 1.0)),
    # 轮毂蓝色
    (['rim_blue'], (0.2, 0.3, 0.7, 1.0)),
    # 螺母 — 暗银
    (['Nut'], (0.55, 0.55, 0.58, 1.0)),
    # 轮毂关节 — 深灰
    (['wheel_joint'], (0.35, 0.35, 0.38, 1.0)),
    # 引擎 / 排气 — 深灰金属
    (['ENGINE', 'exhaust'], (0.30, 0.30, 0.32, 1.0)),
    # 白色车身
    (['WHITE'], (0.92, 0.92, 0.92, 1.0)),
    # 红色玻璃 / 红色部件
    (['red_glass'], (0.7, 0.15, 0.15, 0.6)),
    (['red'], (0.75, 0.2, 0.2, 1.0)),
    # 蓝色 / 海军蓝
    (['BLUE'], (0.2, 0.35, 0.75, 1.0)),
    (['NAVY'], (0.12, 0.15, 0.35, 1.0)),
    # 前挡风玻璃
    (['glass_front', 'front_window'], (0.55, 0.65, 0.8, 0.35)),
    # 车灯玻璃
    (['headlight_glass'], (0.7, 0.75, 0.8, 0.5)),
    # 车灯方框
    (['headlight_square'], (0.6, 0.6, 0.55, 1.0)),
    # 格栅 / 通风口 — 深色
    (['GRILL'], (0.08, 0.08, 0.08, 1.0)),
    # 银色装饰条
    (['silver_door_strip', 'silver_trim', 'Trim'], (0.70, 0.70, 0.72, 1.0)),
    # 光泽面
    (['glossy'], (0.65, 0.65, 0.68, 1.0)),
    # 反光涂层
    (['reflective_coat'], (0.6, 0.6, 0.65, 1.0)),
    # 白色支架
    (['white_holders'], (0.85, 0.85, 0.85, 1.0)),
    # 后视镜
    (['Mirrors'], (0.5, 0.5, 0.55, 1.0)),
    # 路缘轮
    (['WHEEL_CURB'], (0.4, 0.4, 0.42, 1.0)),
]

# 默认颜色（未匹配到任何关键词时使用）
DEFAULT_COLOR = (0.5, 0.5, 0.52, 1.0)


def get_material_color(material_name: str) -> tuple:
    """根据材质名返回 RGBA 颜色元组"""
    if not material_name or material_name.lower() == 'none':
        return DEFAULT_COLOR
    name_lower = material_name.lower()
    for keywords, color in MATERIAL_COLOR_MAP:
        for kw in keywords:
            if kw.lower() in name_lower:
                return color
    return DEFAULT_COLOR


# ==================== OBJ 解析核心 ====================

def _compute_obj_hash(obj_path: str) -> str:
    """计算 OBJ 文件的 MD5 哈希（仅取前10MB + 文件大小，避免大文件全量哈希太慢）"""
    h = hashlib.md5()
    file_size = os.path.getsize(obj_path)
    h.update(str(file_size).encode())
    with open(obj_path, 'rb') as f:
        # 读取头部 1MB
        chunk = f.read(1024 * 1024)
        h.update(chunk)
        # 读取尾部 1MB
        f.seek(max(0, file_size - 1024 * 1024))
        chunk = f.read(1024 * 1024)
        h.update(chunk)
    return h.hexdigest()


def _parse_obj_raw(obj_path: str):
    """
    纯 Python 逐行解析 OBJ 文件，提取顶点、法线、面片和材质分组。

    返回:
        vertices: np.ndarray, shape=(N, 3), float32
        normals:  np.ndarray, shape=(M, 3), float32
        face_groups: list of (material_name, face_indices_array)
            face_indices_array: np.ndarray, shape=(K, 3) or (K, 4), int32 (0-based vertex indices)
        face_normal_groups: list of (material_name, normal_indices_array)
            normal_indices_array: np.ndarray, same shape as face_indices_array, int32 (0-based)
    """
    vertices = []
    normals = []
    current_material = '__default__'

    # 临时存储: {material_name: [face_vertex_indices_list], ...}
    mat_faces = {}
    mat_face_normals = {}

    with open(obj_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            prefix = line[0]

            if prefix == 'v':
                if line[1] == ' ':
                    # 顶点: v x y z
                    parts = line.split()
                    vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
                elif line[1] == 'n' and line[2] == ' ':
                    # 法线: vn nx ny nz
                    parts = line.split()
                    normals.append((float(parts[1]), float(parts[2]), float(parts[3])))
                # 跳过 vt (纹理坐标)

            elif prefix == 'f' and line[1] == ' ':
                # 面片: f v1//vn1 v2//vn2 v3//vn3 [v4//vn4]
                parts = line.split()[1:]  # 跳过 'f'
                face_vi = []
                face_ni = []
                for p in parts:
                    # 支持 v, v/vt, v//vn, v/vt/vn 格式
                    indices = p.split('/')
                    vi = int(indices[0]) - 1  # OBJ 索引从1开始
                    face_vi.append(vi)
                    ni = -1
                    if len(indices) >= 3 and indices[2]:
                        ni = int(indices[2]) - 1
                    face_ni.append(ni)

                # 将多边形面片三角化（扇形分割）
                if len(face_vi) >= 3:
                    if current_material not in mat_faces:
                        mat_faces[current_material] = []
                        mat_face_normals[current_material] = []
                    # 三角化: 对于 n 边形，生成 n-2 个三角形
                    for i in range(1, len(face_vi) - 1):
                        tri_v = [face_vi[0], face_vi[i], face_vi[i + 1]]
                        tri_n = [face_ni[0], face_ni[i], face_ni[i + 1]]
                        mat_faces[current_material].append(tri_v)
                        mat_face_normals[current_material].append(tri_n)

            elif prefix == 'u' and line.startswith('usemtl '):
                current_material = line[7:].strip()

            # 跳过 o, g, s, l, mtllib 等其他行

    vertices = np.array(vertices, dtype=np.float32) if vertices else np.zeros((0, 3), dtype=np.float32)
    normals = np.array(normals, dtype=np.float32) if normals else np.zeros((0, 3), dtype=np.float32)

    face_groups = []
    face_normal_groups = []
    for mat_name in mat_faces:
        face_groups.append((mat_name, np.array(mat_faces[mat_name], dtype=np.int32)))
        face_normal_groups.append((mat_name, np.array(mat_face_normals[mat_name], dtype=np.int32)))

    return vertices, normals, face_groups, face_normal_groups


def _center_and_scale(vertices: np.ndarray, target_size: float = 6.0) -> np.ndarray:
    """
    将顶点居中到原点并缩放，使模型最大维度等于 target_size。
    同时将 Blender 坐标系 (Y 朝上) 转换为 pyqtgraph 坐标系 (Z 朝上)，
    并使汽车车头对准 X 轴正方向。

    坐标系映射:
        Blender  (X右, Y上, Z前/车头) →
        pyqtgraph (X前/车头, Y右, Z上)

    即: Blender Z→pyqtgraph X, Blender X→pyqtgraph Y, Blender Y→pyqtgraph Z

    Args:
        vertices: (N, 3) 顶点数组 (Blender 坐标系: X右, Y上, Z前)
        target_size: 缩放后模型的最大维度（适配相机距离12）

    Returns:
        居中+缩放后的顶点数组 (pyqtgraph 坐标系: X前, Y右, Z上)
    """
    if vertices.shape[0] == 0:
        return vertices

    # Blender Y-up Z-forward → pyqtgraph Z-up X-forward:
    # Blender (X, Y, Z) → pyqtgraph (Z, X, Y)
    vertices = vertices[:, [2, 0, 1]]

    # 计算质心
    center = (vertices.max(axis=0) + vertices.min(axis=0)) / 2.0
    # 居中
    vertices = vertices - center
    # 计算最大跨度
    extent = vertices.max(axis=0) - vertices.min(axis=0)
    max_extent = extent.max()
    if max_extent > 0:
        scale = target_size / max_extent
        vertices = vertices * scale

    # 让车身最低点落在 Z = 0
    min_z = vertices[:, 2].min()
    vertices[:, 2] -= min_z

    return vertices


def _build_mesh_data_list(vertices, normals, face_groups, face_normal_groups):
    """
    将解析结果按材质分组构建 gl.MeshData 对象列表。

    返回:
        mesh_data_list: [(gl.MeshData, material_name, rgba_color), ...]
    """
    mesh_data_list = []

    for i, (mat_name, face_indices) in enumerate(face_groups):
        if face_indices.shape[0] == 0:
            continue

        # 提取该材质组使用的顶点
        unique_vi = np.unique(face_indices)
        vi_map = {old_idx: new_idx for new_idx, old_idx in enumerate(unique_vi)}
        sub_verts = vertices[unique_vi]

        # 重映射面片索引
        sub_faces = np.vectorize(lambda x: vi_map[x])(face_indices)

        # 构建 MeshData（不传法线，让 pyqtgraph 自动计算面法线）
        # OBJ 的顶点法线索引与顶点索引不一定对齐，直接传容易出错
        md = gl.MeshData(vertexes=sub_verts, faces=sub_faces)

        color = get_material_color(mat_name)
        mesh_data_list.append((md, mat_name, color))

    return mesh_data_list


def _save_cache(cache_path: str, vertices, normals, face_groups, face_normal_groups, obj_hash: str):
    """将解析结果保存为 .npy 缓存"""
    # 保存为单个 .npz 文件
    save_dict = {
        'obj_hash': np.array([obj_hash]),  # 用于验证缓存有效性
        'vertices': vertices,
        'normals': normals,
    }
    # 将 face_groups 展平保存
    for i, (mat_name, face_indices) in enumerate(face_groups):
        save_dict[f'face_mat_{i}'] = np.array([mat_name])
        save_dict[f'face_idx_{i}'] = face_indices
    for i, (mat_name, normal_indices) in enumerate(face_normal_groups):
        save_dict[f'fnorm_mat_{i}'] = np.array([mat_name])
        save_dict[f'fnorm_idx_{i}'] = normal_indices
    save_dict['num_face_groups'] = np.array([len(face_groups)])
    save_dict['num_fnorm_groups'] = np.array([len(face_normal_groups)])

    np.savez_compressed(cache_path, **save_dict)


def _load_cache(cache_path: str, obj_hash: str):
    """
    从 .npz 缓存加载解析结果。

    如果缓存不存在或哈希不匹配，返回 None。
    """
    if not os.path.exists(cache_path):
        return None

    try:
        data = np.load(cache_path, allow_pickle=True)
        cached_hash = str(data['obj_hash'][0])
        if cached_hash != obj_hash:
            return None

        vertices = data['vertices']
        normals = data['normals']

        num_fg = int(data['num_face_groups'][0])
        num_fng = int(data['num_fnorm_groups'][0])

        face_groups = []
        for i in range(num_fg):
            mat_name = str(data[f'face_mat_{i}'][0])
            face_indices = data[f'face_idx_{i}']
            face_groups.append((mat_name, face_indices))

        face_normal_groups = []
        for i in range(num_fng):
            mat_name = str(data[f'fnorm_mat_{i}'][0])
            normal_indices = data[f'fnorm_idx_{i}']
            face_normal_groups.append((mat_name, normal_indices))

        return vertices, normals, face_groups, face_normal_groups
    except Exception:
        return None


def load_obj_as_mesh_items(obj_path: str, target_size: float = 6.0, cache_dir: str = None):
    """
    加载 OBJ 文件并返回 pyqtgraph GLMeshItem 列表。

    Args:
        obj_path: OBJ 文件路径
        target_size: 模型缩放后的最大维度（默认6，适配相机距离12）
        cache_dir: 缓存目录，默认为 OBJ 文件同目录

    Returns:
        list of gl.GLMeshItem: 可直接添加到 GLViewWidget 的网格项列表
    """
    obj_path = os.path.abspath(obj_path)
    if cache_dir is None:
        cache_dir = os.path.dirname(obj_path)

    # 缓存路径
    obj_hash = _compute_obj_hash(obj_path)
    cache_path = os.path.join(cache_dir, 'car_obj_cache.npz')

    # 尝试加载缓存
    cached = _load_cache(cache_path, obj_hash)
    if cached is not None:
        vertices, normals, face_groups, face_normal_groups = cached
        print(f"[OBJ Loader] 从缓存加载成功: {cache_path}")
    else:
        print(f"[OBJ Loader] 正在解析 OBJ 文件: {obj_path} (首次加载可能需要数秒)...")
        vertices, normals, face_groups, face_normal_groups = _parse_obj_raw(obj_path)
        print(f"[OBJ Loader] 解析完成: {vertices.shape[0]} 顶点, {normals.shape[0]} 法线, "
              f"{len(face_groups)} 材质组")
        # 保存缓存
        _save_cache(cache_path, vertices, normals, face_groups, face_normal_groups, obj_hash)
        print(f"[OBJ Loader] 缓存已保存: {cache_path}")

    # 居中 + 缩放
    vertices = _center_and_scale(vertices, target_size)

    # 构建 MeshData 列表
    mesh_data_list = _build_mesh_data_list(vertices, normals, face_groups, face_normal_groups)
    print(f"[OBJ Loader] 生成 {len(mesh_data_list)} 个 GLMeshItem")

    # 创建 GLMeshItem 列表
    mesh_items = []
    for md, mat_name, color in mesh_data_list:
        item = gl.GLMeshItem(
            meshdata=md,
            smooth=True,
            color=color,
            shader='shaded',
            glOptions='opaque' if color[3] >= 1.0 else 'translucent'
        )
        mesh_items.append(item)

    return mesh_items
