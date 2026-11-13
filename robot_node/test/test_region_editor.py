"""Manual selection persists exactly the chosen free cells and valid Nav2 goals."""

from pathlib import Path

import cv2
import numpy as np
import pytest
import yaml

from robot_node import partition_io
from robot_node.region_editor_model import RegionDocument, polygon_mask
from robot_node.semantic_map import OccupancyMap, RegionCatalog, select_region_goal


A = [[3, 3], [28, 3], [28, 27], [3, 27]]
B = [[40, 3], [65, 3], [65, 27], [40, 27]]
C = [[3, 38], [30, 38], [30, 64], [3, 64]]


@pytest.fixture
def document(tmp_path):
    image = np.full((70, 90), 254, np.uint8)
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    image[12:17, 12:17] = 0
    image[6:9, 6:9] = 127
    cv2.imwrite(str(tmp_path/'map.pgm'), image)
    config = dict(image='map.pgm', mode='trinary', resolution=0.1,
                  origin=[-1, -2, 0.3], negate=0, occupied_thresh=0.65, free_thresh=0.25)
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(config))
    return RegionDocument(OccupancyMap(tmp_path/'map.yaml'), tmp_path/'output', 0.2)


def test_save_unicode_regions_load_and_voice_route(document):
    document.save_region('A', 'Phòng học', ['lớp học'], A)
    document.save_region('B', 'Phòng thí nghiệm', ['phòng lab'], B)
    document.save_region('C', 'Hành lang', [], C)
    labels = np.load(document.output/'labels.npy')
    assert np.array_equal(labels == 1, polygon_mask(document.grid, A))
    assert np.all(labels[~document.grid.free] == 0)
    assert labels[30, 35] == 0  # Never expand a user's polygon to the rest of the map.
    catalog = RegionCatalog(document.output/'regions.yaml')
    assert catalog.parse('Đi từ phòng học qua hành lang rồi đến phòng lab') == ['A', 'C', 'B']
    for index, region in enumerate(catalog.regions, 1):
        u, v = document.grid.world_to_pixel(region['goal']['x'], region['goal']['y'])
        assert labels[v, u] == index
        assert document.grid.valid_goal(region['goal']['x'], region['goal']['y'], 0.2)
    reopened = RegionDocument(document.grid, document.output)
    assert reopened.load()
    assert reopened.entries[0].polygon == A
    assert np.array_equal(reopened.labels(), labels)
    assert reopened.clearance == 0.2
    assert (document.output/'partition_preview.png').is_file()


@pytest.mark.parametrize('vertices', [
    [], [[1, 1], [2, 2]], [[2, 2], [5, 5], [8, 8]],
    [[-1, 3], [10, 3], [10, 10]], [[2, 2], [100, 2], [10, 20]],
    [[3, 3], [20, float('nan')], [10, 20]],
    [[3, 3], [23, 25], [3, 20], [25, 3], [15, 28]],
])
def test_invalid_geometry_cannot_be_saved(document, vertices):
    with pytest.raises(ValueError):
        document.save_region('A', 'Phòng học', [], vertices)
    assert not document.entries
    assert not (document.output/'regions.yaml').exists()


@pytest.mark.parametrize('kwargs', [
    dict(region_id='B', name='Phòng khác', aliases=[], vertices=A),  # overlap
    dict(region_id='A', name='Phòng khác', aliases=[], vertices=B),  # duplicate ID
    dict(region_id='B', name='Phong hoc', aliases=[], vertices=B),  # same spoken name
    dict(region_id='B', name='Phòng khác', aliases=['phòng A'], vertices=B),
    dict(region_id='B', name=' ', aliases=[], vertices=B),
    dict(region_id='1', name='Phòng khác', aliases=[], vertices=B),
    dict(region_id='B', name='Phòng khác', aliases=[], vertices=None),
    dict(region_id='B', name='Phòng khác', aliases=[], vertices=[[0, 0], [5, 0], [5, 1], [0, 1]]),
])
def test_invalid_edit_preserves_previous_saved_files(document, kwargs):
    document.save_region('A', 'Phòng học', [], A)
    before = {p.name: p.read_bytes() for p in document.output.iterdir() if p.is_file()}
    with pytest.raises(ValueError):
        document.save_region(**kwargs)
    assert len(document.entries) == 1
    assert before == {p.name: p.read_bytes() for p in document.output.iterdir() if p.is_file()}


def test_rename_redraw_and_delete_reindex_masks(document):
    document.save_region('A', 'Phòng học', [], A)
    document.save_region('B', 'Phòng lab', [], B)
    document.save_region('C', 'Hành lang', [], C)
    replacement = [[5, 4], [26, 4], [26, 25], [5, 25]]
    document.save_region('X', 'Phòng mới', ['lớp mới'], replacement, editing_id='A')
    document.save_region('B', 'Phòng thí nghiệm', ['lab'], editing_id='B')
    document.delete_region('B')
    catalog = RegionCatalog(document.output/'regions.yaml')
    assert list(catalog.by_id) == ['X', 'C']
    assert catalog.parse('Đi từ lớp mới đến hành lang') == ['X', 'C']
    labels = np.load(document.output/'labels.npy')
    assert np.array_equal(labels == 1, polygon_mask(document.grid, replacement))
    assert np.array_equal(labels == 2, polygon_mask(document.grid, C))
    document.delete_region('C')
    with pytest.raises(ValueError, match='ít nhất một'):
        document.delete_region('X')
    assert len(RegionCatalog(document.output/'regions.yaml').regions) == 1


def test_failed_file_commit_restores_previous_snapshot(document, monkeypatch):
    document.save_region('A', 'Phòng học', [], A)
    before = {p.name: p.read_bytes() for p in document.output.iterdir() if p.is_file()}
    original = partition_io.os.replace
    failed = False
    def fail_catalog_once(source, destination):
        nonlocal failed
        if Path(destination).name == 'regions.yaml' and not failed:
            failed = True
            raise OSError('simulated disk failure')
        return original(source, destination)
    monkeypatch.setattr(partition_io.os, 'replace', fail_catalog_once)
    with pytest.raises(OSError):
        document.save_region('B', 'Phòng lab', [], B)
    assert len(document.entries) == 1
    assert before == {p.name: p.read_bytes() for p in document.output.iterdir() if p.is_file()}


def test_load_rejects_changed_raster(document):
    document.save_region('A', 'Phòng học', [], A)
    np.save(document.output/'labels.npy', np.zeros((1, 1), dtype=np.int32))
    with pytest.raises(ValueError, match='không khớp'):
        RegionDocument(document.grid, document.output).load()


def test_goal_is_nearest_safe_cell_to_polygon_center(document):
    document.save_region('B', 'B', [], B)
    catalog = RegionCatalog(document.output/'regions.yaml')
    goal = catalog.by_id['B']['goal']
    u, v = catalog.grid.world_to_pixel(goal['x'], goal['y'])
    assert abs(u-52.5) <= 0.5
    assert v == 15
    assert catalog.by_id['B']['goal_policy'] == 'center'
    # Even if physical-map clearance is larger near one side, do not drift there.
    previous_u, previous_v = select_region_goal(document.grid, polygon_mask(document.grid, B),
                                                document.clearance, 'max_clearance')
    assert (previous_u-52.5)**2 + (previous_v-15)**2 > (u-52.5)**2 + (v-15)**2


def test_obstructed_center_projects_to_nearest_safe_cell_and_survives_edit(document):
    document.save_region('A', 'A', [], A)
    old_goal = RegionCatalog(document.output/'regions.yaml').by_id['A']['goal']
    document.save_region('A', 'Renamed area', [], editing_id='A')
    reopened = RegionDocument(document.grid, document.output)
    reopened.load()
    assert reopened.entries[0].goal_policy == 'center'
    catalog = RegionCatalog(document.output/'regions.yaml')
    assert catalog.by_id['A']['goal'] == old_goal
    u, v = document.grid.world_to_pixel(old_goal['x'], old_goal['y'])
    mask = polygon_mask(document.grid, A)
    candidates = mask & (document.grid.clearance >= document.clearance)
    rows, columns = np.nonzero(candidates)
    distances = (columns-15.5)**2 + (rows-15)**2
    assert (u-15.5)**2 + (v-15)**2 == pytest.approx(distances.min())
