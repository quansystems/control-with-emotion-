"""Geometry and command validation independent of ROS and speech models."""

from copy import deepcopy

import cv2
import numpy as np
import pytest
import yaml

from robot_node.semantic_map import OccupancyMap, RegionCatalog, partition, region_records


@pytest.fixture
def catalog(sample_catalog_file):
    return RegionCatalog(sample_catalog_file)


def make_map(tmp_path, image, **kwargs):
    cv2.imwrite(str(tmp_path / 'map.pgm'), image)
    meta = dict(image='map.pgm', resolution=0.1, origin=[-1, -2, 0],
                negate=0, occupied_thresh=0.65, free_thresh=0.25)
    meta.update(kwargs)
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump(meta))
    return OccupancyMap(path)


@pytest.mark.parametrize('command,expected', [
    ('Đi đến khu vực A.', ['A']),
    ('Di chuyển từ khu vực A đến khu vực B.', ['A', 'B']),
    ('Đi đến phòng thí nghiệm.', ['B']),
    ('Đi từ phòng A qua hành lang C rồi đến phòng B', ['A', 'C', 'B']),
    ('di chuyen den phong thi nghiem', ['B']),
    ('Đi đến A qua C rồi đến A', ['A', 'C', 'A']),
    ('Đi đến phòng bê', ['B']),
    ('Đi đến vùng xê', ['C']),
    ('Dừng lại', []), ('Hủy điều hướng', []),
])
def test_commands(catalog, command, expected):
    if expected is None:
        with pytest.raises(ValueError):
            catalog.parse(command)
    else:
        assert catalog.parse(command) == expected


@pytest.mark.parametrize('command', [
    'Đi đến khu vực Z', 'Đi từ A qua khu vực Z rồi đến B',
    'Đi từ A đến Z', 'Đi đến A và phòng Z', 'Đi từ A',
    'phòng A', 'Không đi đến phòng A', '', 'Đi đến A rồi đến',
])
def test_reject_entire_invalid_route(catalog, command):
    with pytest.raises(ValueError):
        catalog.parse(command)


@pytest.mark.parametrize('yaw', [0, 0.3, -1.57])
def test_pixel_world_roundtrip_including_rotated_origin(tmp_path, yaw):
    grid = make_map(tmp_path, np.full((11, 13), 254, np.uint8), origin=[-1, 2, yaw])
    for pixel in [(0, 0), (12, 10), (5, 7)]:
        assert grid.world_to_pixel(*grid.pixel_to_world(*pixel)) == pixel
    assert grid.pixel_to_world(0, 10) == pytest.approx([
        -1 + 0.05*np.cos(yaw) - 0.05*np.sin(yaw),
        2 + 0.05*np.sin(yaw) + 0.05*np.cos(yaw)])


def test_segmentation_never_crosses_wall_or_unknown(tmp_path):
    image = np.full((30, 30), 254, np.uint8)
    image[:, 15] = 0
    image[:3, :] = 127
    grid = make_map(tmp_path, image)
    seeds = [{'id': 'A', 'name': 'A', 'seed_pixel': [5, 10]},
             {'id': 'B', 'name': 'B', 'seed_pixel': [22, 10]}]
    labels, _ = partition(grid, seeds)
    assert np.all(labels[:3] == 0)
    assert np.all(labels[:, 15] == 0)
    assert np.all(labels[3:, :15] == 1)
    assert np.all(labels[3:, 16:] == 2)
    regions = region_records(grid, labels, seeds, 0.3)
    for region in regions:
        assert grid.valid_goal(region['goal']['x'], region['goal']['y'], 0.3)


def test_automatic_opening_separates_narrow_door(tmp_path):
    image = np.zeros((50, 80), np.uint8)
    image[3:47, 3:36] = 254
    image[3:47, 44:77] = 254
    image[24:27, 36:44] = 254
    grid = make_map(tmp_path, image)
    labels, seeds = partition(grid, opening_radius=0.3, min_area=0.5)
    assert len(seeds) == 2
    assert np.all(labels[grid.free] > 0)


def test_obstacle_seed_and_no_safe_goal_are_errors(tmp_path):
    grid = make_map(tmp_path, np.zeros((10, 10), np.uint8))
    with pytest.raises(ValueError, match='known free'):
        partition(grid, [{'id': 'A', 'seed_pixel': [1, 1]}])
    with pytest.raises(ValueError, match='No region seeds'):
        partition(grid)
    grid = make_map(tmp_path, np.full((10, 10), 254, np.uint8))
    labels, seeds = partition(grid, [{'id': 'A', 'name': 'A', 'seed_pixel': [5, 5]}])
    with pytest.raises(ValueError, match='no goal'):
        region_records(grid, labels, seeds, 2.0)


def test_negate_and_unknown_threshold(tmp_path):
    grid = make_map(tmp_path, np.array([[0, 127, 255]], np.uint8), negate=1)
    assert grid.free.tolist() == [[True, False, False]]


def test_catalog_rejects_changed_map_and_alias_collision(tmp_path, catalog):
    data = deepcopy(catalog.data)
    data['map_yaml'] = str(catalog.grid.filename)
    path = tmp_path / 'regions.yaml'
    data['map_fingerprint'] = 'wrong'
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='Map changed'):
        RegionCatalog(path)
    data['map_fingerprint'] = catalog.grid.fingerprint
    data['regions'][1]['aliases'].append('Phòng học')
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='Ambiguous'):
        RegionCatalog(path)
