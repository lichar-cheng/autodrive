from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np


DEFAULT_PCD_DOWNSAMPLE_DIR = Path("/tmp/pcd_cache")
DEFAULT_MIN_VOXEL_SIZE = 0.01


def _pcd_numpy_dtype_from_header(header_lines: list[str]) -> tuple[np.dtype, int]:
    """
    根据 PCD header 构造 numpy structured dtype。

    当前支持 DATA binary PCD。
    例如 Point-LIO 输出可能包含：
    x y z intensity normal_x normal_y normal_z curvature

    本函数只解析完整字段结构；后续读取后只取 x/y/z 做降采样。
    """
    fields: dict[str, Any] = {}
    npoints = 0

    for line in header_lines:
        parts = line.split()
        if not parts:
            continue

        key = parts[0].upper()

        if key == "FIELDS":
            fields["names"] = parts[1:]
        elif key == "SIZE":
            fields["sizes"] = [int(x) for x in parts[1:]]
        elif key == "TYPE":
            fields["types"] = parts[1:]
        elif key == "COUNT":
            fields["counts"] = [int(x) for x in parts[1:]]
        elif key == "POINTS":
            npoints = int(parts[1])
        elif key == "DATA":
            if len(parts) < 2 or parts[1].lower() != "binary":
                raise ValueError(f"only DATA binary PCD is supported, got: {line}")

    names = fields.get("names", [])
    sizes = fields.get("sizes", [])
    types = fields.get("types", [])
    counts = fields.get("counts", [1] * len(names))

    if not names or not sizes or not types:
        raise ValueError("invalid PCD header: missing FIELDS/SIZE/TYPE")

    if not {"x", "y", "z"}.issubset(set(names)):
        raise ValueError(f"PCD must contain x/y/z fields, got: {names}")

    type_map = {
        ("F", 4): np.float32,
        ("F", 8): np.float64,
        ("U", 1): np.uint8,
        ("U", 2): np.uint16,
        ("U", 4): np.uint32,
        ("I", 1): np.int8,
        ("I", 2): np.int16,
        ("I", 4): np.int32,
    }

    dtype_fields = []
    for name, size, field_type, count in zip(names, sizes, types, counts):
        np_type = type_map.get((field_type.upper(), int(size)))
        if np_type is None:
            raise ValueError(
                f"unsupported PCD field type: name={name} "
                f"type={field_type} size={size}"
            )

        if int(count) == 1:
            dtype_fields.append((name, np_type))
        else:
            dtype_fields.append((name, np_type, (int(count),)))

    return np.dtype(dtype_fields), npoints


def _read_pcd_binary_xyz(path: Path) -> np.ndarray:
    """
    读取 binary PCD，并只返回 xyz 三列。

    返回 shape=(N, 3) 的 float32 numpy array。
    会过滤 NaN / Inf 点。
    """
    header_lines: list[str] = []

    with path.open("rb") as file:
        while True:
            raw_line = file.readline()
            if not raw_line:
                raise ValueError("invalid PCD: missing DATA line")

            line = raw_line.decode("ascii", errors="strict").rstrip("\n")
            header_lines.append(line)

            if line.upper().startswith("DATA "):
                break

        dtype, npoints = _pcd_numpy_dtype_from_header(header_lines)
        raw = file.read()

    points = np.frombuffer(raw, dtype=dtype, count=npoints)
    if len(points) <= 0:
        return np.empty((0, 3), dtype=np.float32)

    xyz = np.column_stack(
        [points["x"], points["y"], points["z"]]
    ).astype(np.float32, copy=False)

    finite = np.isfinite(xyz).all(axis=1)
    return xyz[finite]


def _voxel_centroids_xyz(xyz: np.ndarray, voxel_size: float) -> np.ndarray:
    """
    voxel centroid 降采样。

    同一个 voxel 内的点取平均值，只输出一个 centroid。
    """
    if xyz.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    voxel_size = max(DEFAULT_MIN_VOXEL_SIZE, float(voxel_size))

    keys = np.floor(xyz / voxel_size).astype(np.int64)
    keys -= keys.min(axis=0)

    ranges = keys.max(axis=0) + 1
    flat_keys = (
        keys[:, 0] * (ranges[1] * ranges[2])
        + keys[:, 1] * ranges[2]
        + keys[:, 2]
    )

    _, inverse = np.unique(flat_keys, return_inverse=True)
    group_count = int(inverse.max()) + 1

    sums = np.zeros((group_count, 3), dtype=np.float64)
    counts = np.zeros(group_count, dtype=np.int64)

    np.add.at(sums, inverse, xyz)
    np.add.at(counts, inverse, 1)

    return (sums / counts[:, None]).astype(np.float32)


def _write_pcd_binary_xyz(path: Path, xyz: np.ndarray, source_note: str = "") -> None:
    """
    写出 xyz-only binary PCD。

    输出文件只包含 x/y/z 三个 float32 字段。
    """
    xyz = xyz.astype(np.float32, copy=False)
    point_count = int(len(xyz))

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        f"# {source_note}\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {point_count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {point_count}\n"
        "DATA binary\n"
    )

    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("wb") as file:
        file.write(header.encode("ascii"))
        file.write(xyz.tobytes())


def downsample_pcd_to_cache(
    source_path: Path,
    voxel_size: float,
    cache_dir: Path = DEFAULT_PCD_DOWNSAMPLE_DIR,
) -> tuple[Path, dict[str, Any]]:
    """
    将原始 PCD 降采样到缓存文件，并返回降采样后的文件路径和元信息。

    参数：
    - source_path: 原始 PCD 文件路径
    - voxel_size: voxel 边长，单位 m
    - cache_dir: 降采样结果缓存目录

    注意：
    降采样失败时会抛异常，由调用方决定返回 500 还是其它错误。
    """
    source_path = Path(source_path)
    cache_dir = Path(cache_dir)

    source_stat = source_path.stat()
    voxel_size = max(DEFAULT_MIN_VOXEL_SIZE, float(voxel_size))

    cache_name = (
        f"{source_path.stem}"
        f"_mtime{int(source_stat.st_mtime)}"
        f"_size{int(source_stat.st_size)}"
        f"_voxel{voxel_size:.3f}"
        ".pcd"
    )
    cache_path = cache_dir / cache_name

    if cache_path.exists() and cache_path.stat().st_size > 0:
        return cache_path, {
            "cached": True,
            "voxel_size": voxel_size,
            "source_size": int(source_stat.st_size),
            "output_size": int(cache_path.stat().st_size),
        }

    started_at = time.monotonic()

    xyz = _read_pcd_binary_xyz(source_path)
    source_points = int(len(xyz))

    centroids = _voxel_centroids_xyz(xyz, voxel_size)
    output_points = int(len(centroids))

    note = (
        f"downsampled voxel={voxel_size}m "
        f"source={source_path.name} "
        f"source_points={source_points} "
        f"output_points={output_points}"
    )
    _write_pcd_binary_xyz(cache_path, centroids, source_note=note)

    output_size = int(cache_path.stat().st_size)

    return cache_path, {
        "cached": False,
        "voxel_size": voxel_size,
        "source_points": source_points,
        "output_points": output_points,
        "source_size": int(source_stat.st_size),
        "output_size": output_size,
        "elapsed_sec": round(time.monotonic() - started_at, 3),
    }


__all__ = ["downsample_pcd_to_cache"]