"""Stable test catalog independent of the operator's editable production regions."""

import cv2
import numpy as np
import pytest
import yaml

from robot_node.semantic_map import OccupancyMap


@pytest.fixture
def sample_catalog_file(tmp_path):
    image = np.full((60, 80), 254, np.uint8)
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    cv2.imwrite(str(tmp_path/'map.pgm'), image)
    map_file = tmp_path/'map.yaml'
    map_file.write_text(yaml.safe_dump(dict(image='map.pgm', mode='trinary',
        resolution=0.1, origin=[-1.0, -2.0, 0.0], negate=0,
        occupied_thresh=0.65, free_thresh=0.25)))
    grid = OccupancyMap(map_file)
    names = ['Phòng học', 'Phòng thí nghiệm', 'Hành lang']
    regions = []
    for region_id, name, u in zip('ABC', names, [15, 40, 65]):
        x, y = grid.pixel_to_world(u, 30)
        regions.append(dict(id=region_id, name=name, aliases=[], outlines=[],
                            goal=dict(x=x, y=y, yaw=0.0)))
    path = tmp_path/'regions.yaml'
    path.write_text(yaml.safe_dump(dict(schema_version=1, frame_id='map',
        map_yaml='map.yaml', map_fingerprint=grid.fingerprint,
        goal_clearance_m=0.3, regions=regions), allow_unicode=True))
    return path
