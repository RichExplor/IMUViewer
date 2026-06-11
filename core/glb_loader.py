# -*- coding: utf-8 -*-
"""
GLB/GLTF 模型加载器 — 基于 trimesh，针对 pyqtgraph.opengl 的 GLMeshItem 优化

功能:
1. 使用 trimesh 加载 GLB/GLTF 文件，自动解析网格、材质和颜色
2. 从 GLB 材质中提取 PBR 基础颜色(baseColorFactor)和透明度
3. 支持顶点颜色(vertex_colors)
4. 自动居中 + 缩放模型以适配指定显示范围（多部件统一变换）
5. 坐标系转换: Blender Y-up → pyqtgraph Z-up
6. 返回可直接添加到 GLViewWidget 的 GLMeshItem 列表
"""

import os
import numpy as np
import trimesh
import pyqtgraph.opengl as gl


# ==================== 材质颜色提取 ====================

def _extract_material_color(mesh) -> tuple:
    """
    从 trimesh 网格对象中提取 RGBA 颜色。
    优先级:
    1. PBR metallic-roughness 的 baseColorFactor
    2. 双边材质(doubleSided) + diffuse
    3. 顶点颜色的平均值
    4. 默认灰色
    """
    # 尝试从 PBR metallic-roughness 获取基础颜色
    if hasattr(mesh, 'visual') and hasattr(mesh.visual, 'material'):
        material = mesh.visual.material

        # PBR baseColorFactor (GLB 最常见的颜色定义方式)
        if hasattr(material, 'baseColorFactor'):
            bcf = material.baseColorFactor
            if bcf is not None:
                if isinstance(bcf, (list, tuple, np.ndarray)):
                    bcf = np.array(bcf, dtype=np.float32)
                    if bcf.ndim == 0:
                        pass
                    elif len(bcf) >= 4:
                        if bcf.max() > 1.0:
                            bcf = bcf / 255.0
                        return (float(bcf[0]), float(bcf[1]), float(bcf[2]), float(bcf[3]))
                    elif len(bcf) == 3:
                        if bcf.max() > 1.0:
                            bcf = bcf / 255.0
                        return (float(bcf[0]), float(bcf[1]), float(bcf[2]), 1.0)

        # 尝试从 baseColorTexture 获取平均颜色（可选）
        if hasattr(material, 'baseColorTexture'):
            try:
                tex = material.baseColorTexture
                if tex is not None and hasattr(tex, 'image'):
                    img = np.array(tex.image)
                    if img.ndim >= 2:
                        avg_color = img.reshape(-1, img.shape[-1]).mean(axis=0)
                        if len(avg_color) >= 4:
                            if avg_color.max() > 1.0:
                                avg_color = avg_color / 255.0
                            return (float(avg_color[0]), float(avg_color[1]), float(avg_color[2]), float(avg_color[3]))
                        elif len(avg_color) == 3:
                            if avg_color.max() > 1.0:
                                avg_color = avg_color / 255.0
                            return (float(avg_color[0]), float(avg_color[1]), float(avg_color[2]), 1.0)
            except Exception:
                pass

        # 尝试从 diffuse 获取 (旧版 glTF 1.0)
        if hasattr(material, 'diffuse'):
            diffuse = material.diffuse
            if diffuse is not None:
                diffuse = np.array(diffuse, dtype=np.float32)
                if len(diffuse) >= 4:
                    return (float(diffuse[0]), float(diffuse[1]), float(diffuse[2]), float(diffuse[3]))
                elif len(diffuse) == 3:
                    return (float(diffuse[0]), float(diffuse[1]), float(diffuse[2]), 1.0)

    # 尝试使用顶点颜色的平均值
    if hasattr(mesh, 'visual') and hasattr(mesh.visual, 'vertex_colors'):
        try:
            vc = mesh.visual.vertex_colors
            if vc is not None and len(vc) > 0:
                avg = vc.mean(axis=0)
                if len(avg) >= 4:
                    if avg.max() > 1.0:
                        avg = avg / 255.0
                    return (float(avg[0]), float(avg[1]), float(avg[2]), float(avg[3]))
        except Exception:
            pass

    # 默认灰色
    return (0.5, 0.5, 0.52, 1.0)


# ==================== 顶点颜色提取 ====================

def _extract_vertex_colors(mesh, vertex_count: int) -> np.ndarray | None:
    """从 trimesh 网格中提取顶点颜色数组 (N,4) float32，值域 [0,1]"""
    try:
        if hasattr(mesh, 'visual') and hasattr(mesh.visual, 'vertex_colors'):
            vc = mesh.visual.vertex_colors
            if vc is not None and len(vc) == vertex_count:
                return vc.astype(np.float32) / 255.0
    except Exception:
        pass

    # 从面颜色插值到顶点颜色
    try:
        if hasattr(mesh, 'visual') and hasattr(mesh.visual, 'face_colors'):
            fc = mesh.visual.face_colors
            if fc is not None and len(fc) > 0 and hasattr(mesh, 'faces'):
                vc = np.zeros((vertex_count, 4), dtype=np.float32)
                counts = np.zeros(vertex_count, dtype=np.float32)
                fc_float = fc.astype(np.float32) / 255.0
                for fi, face in enumerate(mesh.faces):
                    for vi in face:
                        vc[vi] += fc_float[fi]
                        counts[vi] += 1
                mask = counts > 0
                vc[mask] /= counts[mask, np.newaxis]
                return vc
    except Exception:
        pass

    return None


# ==================== 坐标系转换与统一缩放 ====================

def _unified_center_and_scale(vertices_list: list, target_size: float = 7.0) -> list:
    """
    对多个网格的顶点统一进行居中、缩放，并将最低点置于 Z=0。
    增加翻转 Z 轴以修正上下颠倒问题。

    即: Blender X→pyqtgraph Y, Blender Y→pyqtgraph X, Blender Z→pyqtgraph -Z
    """
    if not vertices_list:
        return vertices_list

    # 1. 坐标系转换: Blender (X, Y, Z) -> pyqtgraph (Y, X, Z) 车头朝X轴正方向
    transformed = [v[:, [1, 0, 2]] for v in vertices_list]

    # 2. 合并所有顶点计算全局边界
    all_verts = np.vstack(transformed)
    if all_verts.shape[0] == 0:
        return transformed

    global_min = all_verts.min(axis=0)
    global_max = all_verts.max(axis=0)
    center = (global_min + global_max) / 2.0
    extent = global_max - global_min
    max_extent = extent.max()
    scale = target_size / max_extent if max_extent > 0 else 1.0

    # 3. 居中 + 缩放
    result = []
    for verts in transformed:
        verts_centered = verts - center
        verts_scaled = verts_centered * scale
        result.append(verts_scaled)

    # 4. 翻转 Z 轴（修正上下颠倒）
    for i in range(len(result)):
        result[i][:, 2] = -result[i][:, 2]

    # 5. 重新调整最低点到 Z=0
    all_z = np.concatenate([v[:, 2] for v in result])
    min_z = all_z.min()
    if min_z < 0:
        for i in range(len(result)):
            result[i][:, 2] -= min_z

    return result


# ==================== 解析 GLB 文件 ====================

def _parse_glb_with_trimesh(glb_path: str):
    """
    使用 trimesh 解析 GLB 文件，提取所有网格的原始顶点、面片、材质颜色和顶点颜色。

    Returns:
        all_vertices: list of np.ndarray, 每个网格的顶点数组（原始 Blender 坐标系）
        all_faces: list of np.ndarray, 每个网格的面片索引数组
        all_colors: list of tuple, 每个网格的 RGBA 颜色
        all_vertex_colors_list: list of np.ndarray or None, 顶点颜色
        mesh_names: list of str, 网格名称列表
    """
    scene = trimesh.load(glb_path, force='scene')

    all_vertices = []
    all_faces = []
    all_colors = []
    all_vertex_colors_list = []
    mesh_names = []

    if isinstance(scene, trimesh.Trimesh):
        # 单个网格
        mesh = scene
        vertices = np.array(mesh.vertices, dtype=np.float32)
        faces = np.array(mesh.faces, dtype=np.int32)
        color = _extract_material_color(mesh)
        vertex_colors = _extract_vertex_colors(mesh, len(vertices))

        all_vertices.append(vertices)
        all_faces.append(faces)
        all_colors.append(color)
        all_vertex_colors_list.append(vertex_colors)
        mesh_names.append(mesh.metadata.get('name', 'mesh_0'))

    elif isinstance(scene, trimesh.Scene):
        # 场景包含多个网格
        for i, (name, mesh) in enumerate(scene.geometry.items()):
            if not isinstance(mesh, trimesh.Trimesh):
                continue

            vertices = np.array(mesh.vertices, dtype=np.float32)
            faces = np.array(mesh.faces, dtype=np.int32)
            color = _extract_material_color(mesh)
            vertex_colors = _extract_vertex_colors(mesh, len(vertices))

            all_vertices.append(vertices)
            all_faces.append(faces)
            all_colors.append(color)
            all_vertex_colors_list.append(vertex_colors)
            mesh_names.append(name if name else f'mesh_{i}')
    else:
        print(f"[GLB Loader] 警告: 未知模型类型 {type(scene)}")

    return all_vertices, all_faces, all_colors, all_vertex_colors_list, mesh_names


# ==================== 核心: 加载 GLB 并返回 GLMeshItem 列表 ====================

def load_glb_as_mesh_items(glb_path: str, target_size: float = 7.0) -> list:
    """
    加载 GLB/GLTF 文件并返回 pyqtgraph GLMeshItem 列表。

    使用 trimesh 解析 GLB 文件，提取每个网格的顶点、面片和材质颜色，
    然后统一进行坐标系转换、居中、缩放，最后转换为 pyqtgraph 的 GLMeshItem 对象。

    Args:
        glb_path: GLB/GLTF 文件路径
        target_size: 模型缩放后的最大维度（默认7，适配相机距离7）

    Returns:
        list of gl.GLMeshItem: 可直接添加到 GLViewWidget 的网格项列表
    """
    glb_path = os.path.abspath(glb_path)
    print(f"[GLB Loader] 正在使用 trimesh 解析 GLB 文件: {glb_path} ...")

    # 解析原始数据（顶点为 Blender 坐标系）
    all_vertices, all_faces, all_colors, all_vertex_colors_list, mesh_names = \
        _parse_glb_with_trimesh(glb_path)
    print(f"[GLB Loader] 解析完成: {len(all_vertices)} 个网格")

    # 统一变换：坐标系转换 + 整体居中 + 缩放 + 最低点归零
    all_vertices_transformed = _unified_center_and_scale(all_vertices, target_size)

    # 创建 GLMeshItem 列表
    mesh_items = []
    for i in range(len(all_vertices_transformed)):
        verts = all_vertices_transformed[i].astype(np.float32)
        faces = all_faces[i].astype(np.int32)
        color = all_colors[i]
        vertex_colors = all_vertex_colors_list[i]

        if verts.shape[0] == 0 or faces.shape[0] == 0:
            continue

        # 构建 MeshData
        md = gl.MeshData(vertexes=verts, faces=faces)

        # 如果有顶点颜色，设置到 MeshData
        if vertex_colors is not None and len(vertex_colors) == verts.shape[0]:
            try:
                md.setVertexColors(vertex_colors.astype(np.float32))
                # 使用白色底，让顶点颜色主导
                item_color = (1.0, 1.0, 1.0, 1.0)
            except Exception:
                vertex_colors = None
                item_color = color
        else:
            item_color = color

        # 决定渲染模式
        gl_options = 'translucent' if item_color[3] < 1.0 else 'opaque'

        item = gl.GLMeshItem(
            meshdata=md,
            smooth=True,
            color=item_color,
            shader='shaded',
            glOptions=gl_options,
        )
        mesh_items.append(item)

    print(f"[GLB Loader] 生成 {len(mesh_items)} 个 GLMeshItem")
    return mesh_items


# ==================== 便捷函数 ====================

def load_model_as_mesh_items(model_path: str, target_size: float = 7.0) -> list:
    """
    统一的模型加载入口，根据文件扩展名自动选择加载器。

    支持: .glb, .gltf, .obj
    """
    ext = os.path.splitext(model_path)[1].lower()
    if ext in ('.glb', '.gltf'):
        return load_glb_as_mesh_items(model_path, target_size)
    elif ext == '.obj':
        # 假设 obj_loader 在同一包中，可根据实际路径调整
        from core.obj_loader import load_obj_as_mesh_items
        return load_obj_as_mesh_items(model_path, target_size)
    else:
        raise ValueError(f"不支持的模型格式: {ext}，仅支持 .glb/.gltf/.obj")