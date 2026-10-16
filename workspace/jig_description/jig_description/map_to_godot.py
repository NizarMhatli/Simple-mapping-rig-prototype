#!/usr/bin/env python3
"""
Convert RTAB-Map / Nav2 map exports into Godot-ready glTF (.glb) files.

Inputs
  --map2d  map_2d.yaml   Nav2 map (yaml + pgm) from map_saver_cli
  --cloud  map_3d.ply    3D point cloud from rtabmap-export

Outputs (in --out, default ./godot)
  map_2d.png             occupancy image (free = white, occupied = dark, unknown = transparent)
  map_2d_floor.glb       floor quad textured with map_2d.png, at true scale/position
  map_2d_walls.glb       occupied cells extruded to --wall-height (good for collisions)
  map_3d_points.glb      downsampled point cloud, height-coloured (glTF POINTS)
  map_3d_mesh.glb        Poisson surface mesh (needs open3d)
  map_info.json          resolution, origin, bounds

All files share RTAB-Map's `map` frame, so they line up when dropped into one
Godot scene. Coordinates are converted from ROS (x fwd, y left, z up) to glTF
(y up): (x, y, z)_ros -> (x, z, -y)_gltf. 1 unit = 1 metre.

Install:  pip install trimesh pillow pyyaml numpy open3d
          (open3d is optional; without it the mesh step is skipped)
"""
import argparse
import json
import os
import sys

import numpy as np
import trimesh
import yaml
from PIL import Image

try:
    import open3d as o3d
except ImportError:
    o3d = None

ROS_TO_GLTF = np.array([[1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, -1, 0, 0],
                        [0, 0, 0, 1]], dtype=float)


# ─────────────────────────────── 2D ───────────────────────────────
def load_2d(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img_path = meta['image']
    if not os.path.isabs(img_path):
        img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
    img = np.asarray(Image.open(img_path).convert('L'), dtype=float)
    p = img / 255.0 if meta.get('negate', 0) else (255.0 - img) / 255.0
    occ = p > meta.get('occupied_thresh', 0.65)
    free = p < meta.get('free_thresh', 0.25)
    ox, oy, oyaw = meta['origin']
    if abs(oyaw) > 1e-6:
        print(f'[warn] map origin yaw {oyaw} ignored', file=sys.stderr)
    return occ, free, float(meta['resolution']), float(ox), float(oy)


def export_2d(yaml_path, out, wall_height):
    occ, free, res, ox, oy = load_2d(yaml_path)
    H, W = occ.shape
    w_m, h_m = W * res, H * res

    # Image: row 0 = top = max y (same as the pgm)
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[free] = (235, 235, 235, 255)
    rgba[occ] = (40, 40, 40, 255)
    png = os.path.join(out, 'map_2d.png')
    Image.fromarray(rgba, 'RGBA').save(png)

    # Floor quad (ROS coords, z = 0), UV (0,0) = bottom-left of the image
    v = np.array([[ox, oy, 0], [ox + w_m, oy, 0], [ox + w_m, oy + h_m, 0], [ox, oy + h_m, 0]])
    f = np.array([[0, 1, 2], [0, 2, 3]])
    uv = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=float)
    mat = trimesh.visual.material.PBRMaterial(
        baseColorTexture=Image.open(png), alphaMode='BLEND', doubleSided=True,
        metallicFactor=0.0, roughnessFactor=1.0)
    floor = trimesh.Trimesh(v, f, visual=trimesh.visual.TextureVisuals(uv=uv, material=mat),
                            process=False)
    floor.apply_transform(ROS_TO_GLTF)
    floor.export(os.path.join(out, 'map_2d_floor.glb'))

    # Walls: merge horizontal runs of occupied cells per row into boxes
    boxes = []
    for r in range(H):
        row = occ[r]
        c = 0
        while c < W:
            if not row[c]:
                c += 1
                continue
            start = c
            while c < W and row[c]:
                c += 1
            x0, x1 = ox + start * res, ox + c * res
            y0 = oy + (H - 1 - r) * res
            box = trimesh.creation.box(extents=(x1 - x0, res, wall_height))
            box.apply_translation(((x0 + x1) / 2, y0 + res / 2, wall_height / 2))
            boxes.append(box)
    if boxes:
        walls = trimesh.util.concatenate(boxes)
        walls.visual.face_colors = (120, 120, 130, 255)
        walls.apply_transform(ROS_TO_GLTF)
        walls.export(os.path.join(out, 'map_2d_walls.glb'))
    print(f'2D: {W}x{H} cells @ {res} m, {int(occ.sum())} occupied, {len(boxes)} wall boxes')
    return {'resolution': res, 'origin_xy': [ox, oy], 'size_m': [w_m, h_m],
            'cells': [W, H], 'wall_height': wall_height}


# ─────────────────────────────── 3D ───────────────────────────────
def height_colours(z):
    t = (z - z.min()) / max(1e-6, np.ptp(z))
    # blue -> cyan -> green -> yellow -> red
    stops = np.array([[0, 0, 255], [0, 255, 255], [0, 255, 0], [255, 255, 0], [255, 0, 0]], float)
    i = np.clip((t * 4).astype(int), 0, 3)
    a = (t * 4 - i)[:, None]
    rgb = stops[i] * (1 - a) + stops[i + 1] * a
    return np.hstack([rgb, np.full((len(z), 1), 255)]).astype(np.uint8)


def voxel_down(p, voxel):
    keys = np.floor(p / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return p[np.sort(idx)]


_PLY_TYPES = {'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1',
              'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2',
              'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4',
              'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8'}


def read_ply_xyz(path):
    """Read vertex x/y/z from a PLY file (ascii or binary).

    Written for PCL/RTAB-Map PLYs, which add extra elements (e.g. `camera`)
    that trimesh's loader can't parse. Only the vertex element is read.
    """
    with open(path, 'rb') as f:
        if f.readline().strip() != b'ply':
            raise ValueError(f'{path} is not a PLY file')
        fmt, elements, cur = None, [], None
        while True:
            line = f.readline()
            if not line:
                raise ValueError('PLY header has no end_header')
            tok = line.decode('ascii', 'replace').split()
            if not tok or tok[0] in ('comment', 'obj_info'):
                continue
            if tok[0] == 'format':
                fmt = tok[1]
            elif tok[0] == 'element':
                cur = {'name': tok[1], 'count': int(tok[2]), 'props': [], 'has_list': False}
                elements.append(cur)
            elif tok[0] == 'property':
                if tok[1] == 'list':
                    cur['has_list'] = True
                else:
                    cur['props'].append((tok[2], _PLY_TYPES[tok[1]]))
            elif tok[0] == 'end_header':
                break
        data_start = f.tell()

        vi = next(i for i, e in enumerate(elements) if e['name'] == 'vertex')
        v = elements[vi]
        if fmt == 'ascii':
            skip = sum(e['count'] for e in elements[:vi])
            for _ in range(skip):
                f.readline()
            names = [n for n, _ in v['props']]
            rows = np.loadtxt(f, max_rows=v['count'], ndmin=2)
            return rows[:, [names.index(c) for c in ('x', 'y', 'z')]].astype(float)

        endian = '<' if fmt == 'binary_little_endian' else '>'
        offset = data_start
        for e in elements[:vi]:
            if e['has_list']:
                raise ValueError(f"can't skip list element '{e['name']}' before vertex")
            offset += e['count'] * np.dtype([(n, endian + t) for n, t in e['props']]).itemsize
        dt = np.dtype([(n, endian + t) for n, t in v['props']])
        f.seek(offset)
        arr = np.frombuffer(f.read(dt.itemsize * v['count']), dtype=dt, count=v['count'])
        return np.column_stack([arr['x'], arr['y'], arr['z']]).astype(float)


def export_3d(ply, out, voxel, poisson_depth, max_tris, no_mesh):
    pts = read_ply_xyz(ply)
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = voxel_down(pts, voxel)
    cols = height_colours(pts[:, 2])

    pc = trimesh.PointCloud(pts.copy(), colors=cols)
    pc.apply_transform(ROS_TO_GLTF)
    pc.export(os.path.join(out, 'map_3d_points.glb'))
    info = {'points': int(len(pts)), 'voxel': voxel,
            'bounds_ros_min': pts.min(0).tolist(), 'bounds_ros_max': pts.max(0).tolist()}
    print(f'3D: {len(pts)} points after {voxel} m voxel filter')

    if no_mesh:
        return info
    if o3d is None:
        print('[info] open3d not installed -> skipping map_3d_mesh.glb', file=sys.stderr)
        return info

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 4, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(15)
    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=poisson_depth)
    dens = np.asarray(dens)
    mesh.remove_vertices_by_mask(dens < np.quantile(dens, 0.05))     # drop Poisson "balloon"
    mesh = mesh.crop(pcd.get_axis_aligned_bounding_box())
    if len(mesh.triangles) > max_tris:
        mesh = mesh.simplify_quadric_decimation(max_tris)
    mesh.remove_degenerate_triangles()
    mesh.remove_unreferenced_vertices()

    v = np.asarray(mesh.vertices)
    tm = trimesh.Trimesh(v, np.asarray(mesh.triangles),
                         vertex_colors=height_colours(v[:, 2]), process=False)
    tm.apply_transform(ROS_TO_GLTF)
    tm.export(os.path.join(out, 'map_3d_mesh.glb'))
    info['mesh_triangles'] = int(len(tm.faces))
    print(f'3D mesh: {len(tm.faces)} triangles')
    return info


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map2d', help='Nav2 map yaml (from map_saver_cli)')
    ap.add_argument('--cloud', help='3D cloud .ply (from rtabmap-export)')
    ap.add_argument('--out', default='godot')
    ap.add_argument('--wall-height', type=float, default=1.0)
    ap.add_argument('--voxel', type=float, default=0.05)
    ap.add_argument('--poisson-depth', type=int, default=9)
    ap.add_argument('--max-tris', type=int, default=200000)
    ap.add_argument('--no-mesh', action='store_true')
    a = ap.parse_args()
    if not (a.map2d or a.cloud):
        ap.error('give --map2d and/or --cloud')

    os.makedirs(a.out, exist_ok=True)
    info = {'frame': 'map', 'units': 'metres', 'axes': 'glTF y-up; ros (x,y,z) -> (x,z,-y)'}
    if a.map2d:
        info['map_2d'] = export_2d(a.map2d, a.out, a.wall_height)
    if a.cloud:
        info['map_3d'] = export_3d(a.cloud, a.out, a.voxel, a.poisson_depth, a.max_tris, a.no_mesh)
    json.dump(info, open(os.path.join(a.out, 'map_info.json'), 'w'), indent=2)
    print(f'Godot files written to {os.path.abspath(a.out)}')


if __name__ == '__main__':
    main()