"""Validated persistence shared by the editor and automatic partitioning CLI."""

import os
from pathlib import Path
from tempfile import TemporaryDirectory

import cv2
import numpy as np
import yaml

from robot_node.semantic_map import RegionCatalog, region_records


PALETTE = [(75, 185, 95), (245, 160, 65), (100, 100, 240), (185, 95, 190)]


def preview_image(grid, labels, regions, scale=5):
    preview = cv2.cvtColor(grid.image, cv2.COLOR_GRAY2BGR)
    for index, region in enumerate(regions, 1):
        mask = labels == index
        color = np.array(PALETTE[(index-1) % len(PALETTE)])
        preview[mask] = (0.4*preview[mask] + 0.6*color).astype(np.uint8)
    preview = cv2.resize(preview, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    for region in regions:
        u, v = grid.world_to_pixel(region['goal']['x'], region['goal']['y'])
        point = (round(scale*(u+0.5)), round(scale*(v+0.5)))
        cv2.circle(preview, point, 5, (0, 0, 0), -1)
        cv2.putText(preview, region['id'], (point[0]+7, point[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
    return preview


def write_partition(grid, labels, metadata, clearance, output, source='manual'):
    """Validate all outputs before replacing saved files; catalog commits last."""
    regions = region_records(grid, labels, metadata, clearance)
    for region, entry in zip(regions, metadata):
        if entry.get('polygon_pixels') is not None:
            region['polygon_pixels'] = entry['polygon_pixels']
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = {'schema_version': 1, 'frame_id': 'map', 'partition_method': source,
              'map_yaml': os.path.relpath(grid.filename, output),
              'map_fingerprint': grid.fingerprint,
              'goal_clearance_m': clearance, 'regions': regions}
    files = ['labels.npy', 'partition_preview.png', 'regions.yaml']
    with TemporaryDirectory(prefix='.regions-', dir=output) as temporary:
        stage = Path(temporary)
        (stage/'regions.yaml').write_text(yaml.safe_dump(
            config, allow_unicode=True, sort_keys=False), encoding='utf-8')
        RegionCatalog(stage/'regions.yaml', map_override=grid.filename)
        np.save(stage/'labels.npy', labels)
        if not cv2.imwrite(str(stage/'partition_preview.png'),
                           preview_image(grid, labels, regions)):
            raise OSError('Không thể ghi ảnh phân vùng.')
        backups = {name: (output/name).read_bytes() if (output/name).exists() else None
                   for name in files}
        replaced = []
        try:
            for name in files:
                os.replace(stage/name, output/name)
                replaced.append(name)
        except OSError:
            for name in reversed(replaced):
                if backups[name] is None:
                    (output/name).unlink()
                else:
                    (stage/name).write_bytes(backups[name])
                    os.replace(stage/name, output/name)
            raise
    return regions
