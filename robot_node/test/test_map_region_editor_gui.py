"""Exercise real mouse coordinates, zoom and Save in Qt's offscreen platform."""

import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import cv2
import numpy as np
import pytest
import yaml
from PyQt5.QtCore import QLibraryInfo, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

from robot_node.map_region_editor import RegionEditor
from robot_node.region_editor_model import RegionDocument
from robot_node.semantic_map import OccupancyMap, RegionCatalog


@pytest.fixture
def editor(tmp_path):
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    app = QApplication.instance() or QApplication([])
    image = np.full((60, 80), 254, np.uint8)
    image[[0, -1], :] = 0
    image[:, [0, -1]] = 0
    cv2.imwrite(str(tmp_path/'map.pgm'), image)
    (tmp_path/'map.yaml').write_text(yaml.safe_dump(dict(
        image='map.pgm', resolution=0.1, origin=[-1, -2, 0], negate=0,
        occupied_thresh=0.65, free_thresh=0.25)))
    document = RegionDocument(OccupancyMap(tmp_path/'map.yaml'), tmp_path/'output', 0.2)
    window = RegionEditor(document)
    window.show()
    app.processEvents()
    window.canvas.fit_map()
    try:
        yield app, window, document
    finally:
        window.close()
        app.processEvents()


def click_vertex(canvas, u, v):
    position = canvas.mapFromScene(u+0.5, v+0.5)
    QTest.mouseClick(canvas.viewport(), Qt.LeftButton, pos=position)


def test_mouse_polygon_named_save_then_reload(editor):
    app, window, document = editor
    window.canvas.scale(1.2, 1.2)  # View coordinates must still map to the same pixels.
    vertices = [[10, 10], [30, 10], [30, 30], [10, 30]]
    for u, v in vertices:
        click_vertex(window.canvas, u, v)
    QTest.keyClick(window.canvas, Qt.Key_Return)
    assert window.vertices == vertices
    assert not (document.output/'regions.yaml').exists()
    window.name_field.setText('Phòng học')
    window.alias_field.setText('lớp học; lớp A')
    QTest.mouseClick(window.save_button, Qt.LeftButton)
    assert 'Đã lưu vùng A' in window.hint.text()
    assert len(document.entries) == 1
    assert RegionCatalog(document.output/'regions.yaml').parse('Đi đến lớp học') == ['A']
    window.reload_document()
    window.region_list.setCurrentRow(0)
    assert window.name_field.text() == 'Phòng học'
    window.name_field.setText('Phòng nghiên cứu')
    QTest.mouseClick(window.save_button, Qt.LeftButton)
    assert RegionCatalog(document.output/'regions.yaml').parse('Đi đến phòng nghiên cứu') == ['A']


def test_rectangle_drag_finish_and_unsaved_cancel(editor):
    app, window, document = editor
    window.mode.setCurrentIndex(1)
    canvas = window.canvas
    first = canvas.mapFromScene(10.5, 10.5)
    last = canvas.mapFromScene(30.5, 30.5)
    QTest.mousePress(canvas.viewport(), Qt.LeftButton, pos=first)
    QTest.mouseMove(canvas.viewport(), last)
    QTest.mouseRelease(canvas.viewport(), Qt.LeftButton, pos=last)
    assert window.vertices == [[10, 10], [30, 10], [30, 30], [10, 30]]
    window.name_field.setText('Phòng A')
    assert window.save_region()
    saved = (document.output/'regions.yaml').read_bytes()
    window.new_region()
    assert window.id_field.text() == 'B'
    window.new_document()
    assert not document.entries
    assert (document.output/'regions.yaml').read_bytes() == saved
    window.reload_document()
    assert len(document.entries) == 1
