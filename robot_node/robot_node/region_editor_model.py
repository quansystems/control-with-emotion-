"""Manual region selection with occupancy, overlap and catalog validation."""

from dataclasses import dataclass
import math
from pathlib import Path
import re

import cv2
import numpy as np

from robot_node.partition_io import write_partition
from robot_node.semantic_map import RegionCatalog


@dataclass
class Selection:
    id: str
    name: str
    aliases: list
    mask: np.ndarray
    polygon: list = None
    goal_policy: str = 'center'


def polygon_mask(grid, vertices):
    points = np.asarray(vertices, dtype=float)
    if (points.ndim != 2 or points.shape[0] < 3 or points.shape[1] != 2 or
            not np.isfinite(points).all()):
        raise ValueError('Vùng cần ít nhất 3 đỉnh hợp lệ.')
    if (np.any(points < 0) or np.any(points[:, 0] >= grid.width) or
            np.any(points[:, 1] >= grid.height)):
        raise ValueError('Các đỉnh phải nằm trong bản đồ.')
    points = np.floor(points).astype(np.int32)
    if len(set(map(tuple, points))) < 3 or abs(cv2.contourArea(points)) < 0.5:
        raise ValueError('Vùng quá nhỏ hoặc các đỉnh nằm trên cùng một đường thẳng.')
    # Explicitly reject self-intersection rather than apply an ambiguous fill rule.
    def cross(a, b, c):
        return int(b[0]-a[0])*int(c[1]-a[1]) - int(b[1]-a[1])*int(c[0]-a[0])
    def on_segment(a, b, c):
        return (cross(a, b, c) == 0 and min(a[0], b[0]) <= c[0] <= max(a[0], b[0])
                and min(a[1], b[1]) <= c[1] <= max(a[1], b[1]))
    def intersects(a, b, c, d):
        ac, ad, ca, cb = cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b)
        return ((ac*ad < 0 and ca*cb < 0) or on_segment(a, b, c) or
                on_segment(a, b, d) or on_segment(c, d, a) or on_segment(c, d, b))
    count = len(points)
    for i in range(count):
        for j in range(i+1, count):
            if j == i+1 or (i == 0 and j == count-1):
                continue
            if intersects(points[i], points[(i+1) % count], points[j], points[(j+1) % count]):
                raise ValueError('Các cạnh của vùng không được tự cắt nhau.')
    mask = np.zeros(grid.free.shape, dtype=np.uint8)
    cv2.fillPoly(mask, [points], 1)
    return mask.astype(bool) & grid.free


class RegionDocument:
    def __init__(self, grid, output, clearance=0.3):
        if not math.isfinite(clearance) or clearance <= 0:
            raise ValueError('Khoảng cách an toàn phải lớn hơn 0 và hữu hạn.')
        self.grid, self.output, self.clearance = grid, Path(output).resolve(), clearance
        self.entries = []

    def load(self):
        config, raster = self.output/'regions.yaml', self.output/'labels.npy'
        if not config.exists() and not raster.exists():
            return False
        if not config.is_file() or not raster.is_file():
            raise ValueError('Cần cả regions.yaml và labels.npy để mở lại vùng đã lưu.')
        catalog = RegionCatalog(config, map_override=self.grid.filename)
        labels = np.load(raster, allow_pickle=False)
        if (labels.shape != self.grid.free.shape or not np.issubdtype(labels.dtype, np.integer)
                or np.any(labels < 0) or np.any(labels > len(catalog.regions))
                or np.any((labels > 0) & ~self.grid.free)):
            raise ValueError('labels.npy không khớp bản đồ hoặc danh sách vùng.')
        entries = []
        for index, region in enumerate(catalog.regions, 1):
            mask = labels == index
            u, v = self.grid.world_to_pixel(region['goal']['x'], region['goal']['y'])
            if not mask.any() or not mask[v, u]:
                raise ValueError(f'Đích của vùng {region["id"]} không nằm trong vùng đã lưu.')
            entries.append(Selection(region['id'], region['name'], region.get('aliases', []),
                                     mask, region.get('polygon_pixels'),
                                     region.get('goal_policy', 'max_clearance')))
        self.entries, self.clearance = entries, catalog.clearance
        return True

    def next_id(self):
        existing = {entry.id for entry in self.entries}
        for index in range(1, len(existing)+28):
            candidate = chr(64+index) if index <= 26 else f'R{index}'
            if candidate not in existing:
                return candidate

    def labels(self, entries=None):
        labels = np.zeros(self.grid.free.shape, dtype=np.int32)
        for index, entry in enumerate(self.entries if entries is None else entries, 1):
            labels[entry.mask] = index
        return labels

    def _persist(self, entries):
        if not entries:
            raise ValueError('Cần giữ ít nhất một vùng để điều hướng.')
        metadata = [{'id': e.id, 'name': e.name, 'aliases': e.aliases,
                     'polygon_pixels': e.polygon, 'goal_policy': e.goal_policy} for e in entries]
        regions = write_partition(self.grid, self.labels(entries), metadata,
                                  self.clearance, self.output)
        self.entries = entries  # Commit working state only after a successful save.
        return regions

    def save_region(self, region_id, name, aliases, vertices=None, editing_id=None):
        region_id, name = region_id.strip(), name.strip()
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_-]*', region_id):
            raise ValueError('ID phải bắt đầu bằng chữ, chỉ gồm chữ, số, _ hoặc -.')
        if not name:
            raise ValueError('Hãy đặt tên cho vùng trước khi lưu.')
        aliases = [alias.strip() for alias in aliases if alias.strip()]
        current = next((e for e in self.entries if e.id == editing_id), None)
        if editing_id is not None and current is None:
            raise ValueError('Vùng đang sửa không còn tồn tại.')
        if any(e.id == region_id and e is not current for e in self.entries):
            raise ValueError(f'ID {region_id} đã tồn tại.')
        if vertices is not None:
            mask = polygon_mask(self.grid, vertices)
            polygon = [[int(x), int(y)] for x, y in np.floor(vertices)]
        elif current is not None:
            mask, polygon = current.mask.copy(), current.polygon
        else:
            raise ValueError('Hãy chọn vùng trên bản đồ trước khi lưu.')
        if not np.any(mask & (self.grid.clearance >= self.clearance)):
            raise ValueError(f'Vùng không có đích trống cách vật cản ít nhất {self.clearance:g} m.')
        for entry in self.entries:
            if entry is not current and np.any(mask & entry.mask):
                raise ValueError(f'Vùng đang chọn chồng lên vùng {entry.id} ({entry.name}).')
        policy = 'center' if vertices is not None or current is None else current.goal_policy
        selection = Selection(region_id, name, aliases, mask, polygon, policy)
        entries = [selection if entry is current else entry for entry in self.entries]
        if current is None:
            entries.append(selection)
        return self._persist(entries)

    def delete_region(self, region_id):
        if not any(e.id == region_id for e in self.entries):
            raise ValueError('Chọn một vùng để xóa.')
        return self._persist([entry for entry in self.entries if entry.id != region_id])
