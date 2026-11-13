"""Desktop editor: draw a region, give it a name and explicitly save it."""

import argparse
import math
import os
from pathlib import Path
import sys

import cv2
import numpy as np
from PyQt5.QtCore import QLibraryInfo, QPointF, Qt, pyqtSignal
from PyQt5.QtGui import QBrush, QColor, QFont, QImage, QPainter, QPen, QPixmap, QPolygonF
from PyQt5.QtWidgets import (
    QApplication, QComboBox, QDoubleSpinBox, QFormLayout, QGraphicsScene,
    QGraphicsView, QHBoxLayout, QLabel, QLineEdit, QListWidget, QMainWindow,
    QMessageBox, QPushButton, QSplitter, QVBoxLayout, QWidget,
)

from robot_node.region_editor_model import RegionDocument
from robot_node.semantic_map import OccupancyMap, select_region_goal


COLORS = [(75, 195, 120), (80, 155, 245), (240, 125, 100), (180, 110, 230)]


class MapCanvas(QGraphicsView):
    selection_changed = pyqtSignal(object)
    hint_changed = pyqtSignal(str)

    def __init__(self, document):
        super().__init__()
        self.document = document
        self.setScene(QGraphicsScene(self))
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setBackgroundBrush(QColor('#202b36'))
        self.setRenderHint(QPainter.Antialiasing)
        self.mode = 'polygon'
        self.points, self.draft_items = [], []
        self.anchor = None
        self.finished = False
        self.selected_id = None
        self.rebuild()

    def rebuild(self):
        self.scene().clear()
        self.draft_items = []
        grid = self.document.grid
        image = np.repeat(grid.image[:, :, None], 3, axis=2).copy()
        qimage = QImage(image.data, grid.width, grid.height, image.strides[0],
                        QImage.Format_RGB888).copy()
        self.scene().addPixmap(QPixmap.fromImage(qimage))
        rgba = np.zeros((grid.height, grid.width, 4), dtype=np.uint8)
        for index, entry in enumerate(self.document.entries):
            rgba[entry.mask] = (*COLORS[index % len(COLORS)], 100)
        overlay = QImage(rgba.data, grid.width, grid.height, rgba.strides[0],
                         QImage.Format_RGBA8888).copy()
        self.scene().addPixmap(QPixmap.fromImage(overlay))
        for index, entry in enumerate(self.document.entries):
            color = QColor(*COLORS[index % len(COLORS)])
            pen = QPen(Qt.white if entry.id == self.selected_id else color)
            pen.setCosmetic(True)
            pen.setWidth(3 if entry.id == self.selected_id else 1)
            contours, _ = cv2.findContours(entry.mask.astype(np.uint8), cv2.RETR_LIST,
                                          cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                self.scene().addPolygon(QPolygonF([
                    QPointF(float(p[0][0])+0.5, float(p[0][1])+0.5) for p in contour]), pen)
            u, v = select_region_goal(grid, entry.mask, self.document.clearance,
                                      entry.goal_policy, entry.polygon)
            radius = max(0.6, grid.width/250)
            self.scene().addEllipse(u+0.5-radius, v+0.5-radius, 2*radius, 2*radius,
                                    QPen(Qt.black, 0), QBrush(color))
            label = self.scene().addSimpleText(f'{entry.id}: {entry.name}')
            face = QFont()
            face.setPixelSize(max(3, round(grid.width/65)))
            face.setBold(True)
            label.setFont(face)
            label.setBrush(QBrush(QColor('#101820')))
            label.setPos(max(0, min(u+2, grid.width-label.boundingRect().width())), v-4)
        self.setSceneRect(0, 0, grid.width, grid.height)
        self.draw_draft()

    def fit_map(self):
        self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fit_map()

    def wheelEvent(self, event):
        factor = 1.2 if event.angleDelta().y() > 0 else 1/1.2
        if 0.05 < self.transform().m11()*factor < 150:
            self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
            self.scale(factor, factor)

    def clear_draft(self):
        self.points, self.anchor, self.finished = [], None, False
        self.draw_draft()
        self.selection_changed.emit(None)

    def draw_draft(self, cursor=None):
        for item in self.draft_items:
            self.scene().removeItem(item)
        self.draft_items = []
        points = list(self.points)
        if cursor is not None and not self.finished and self.mode == 'polygon':
            points.append(cursor)
        pen = QPen(QColor('#edaa17'))
        pen.setCosmetic(True)
        pen.setWidth(2)
        if len(points) >= 2:
            polygon = QPolygonF([QPointF(x+0.5, y+0.5) for x, y in points])
            item = self.scene().addPolygon(polygon, pen, QBrush(QColor(255, 200, 40, 65)))
            self.draft_items.append(item)
        radius = max(0.4, self.document.grid.width/400)
        for x, y in self.points:
            self.draft_items.append(self.scene().addEllipse(
                x+0.5-radius, y+0.5-radius, 2*radius, 2*radius, pen, QBrush(Qt.yellow)))

    def point_at(self, position, clamp=False):
        point = self.mapToScene(position)
        grid = self.document.grid
        u, v = math.floor(point.x()), math.floor(point.y())
        if clamp:
            return [min(max(0, u), grid.width-1), min(max(0, v), grid.height-1)]
        return [u, v] if 0 <= u < grid.width and 0 <= v < grid.height else None

    def finish_polygon(self):
        if len(self.points) < 3:
            self.hint_changed.emit('Hãy chọn ít nhất 3 đỉnh để khép vùng.')
            return
        self.finished = True
        self.draw_draft()
        self.selection_changed.emit(list(self.points))

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.finish_polygon()
            return
        point = self.point_at(event.pos())
        if event.button() != Qt.LeftButton or point is None:
            return super().mousePressEvent(event)
        if self.finished:
            self.clear_draft()
        if self.mode == 'rectangle':
            self.anchor = point
            self.points = [point]
        elif not self.points or point != self.points[-1]:
            self.points.append(point)
        self.draw_draft()

    def mouseMoveEvent(self, event):
        point = self.point_at(event.pos(), clamp=self.anchor is not None)
        if self.mode == 'rectangle' and self.anchor is not None and point is not None:
            u, v = self.anchor
            x, y = point
            self.points = [[u, v], [x, v], [x, y], [u, y]]
            self.draw_draft()
        elif point is not None:
            self.draw_draft(point)
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.mode == 'rectangle' and self.anchor is not None:
            u, v = self.anchor
            x, y = self.point_at(event.pos(), clamp=True)
            self.points = [[u, v], [x, v], [x, y], [u, y]]
            self.anchor = None
            self.finished = True
            self.draw_draft()
            self.selection_changed.emit(list(self.points))
            return
        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if self.mode == 'polygon' and event.button() == Qt.LeftButton:
            point = self.point_at(event.pos())
            if point is not None and (not self.points or point != self.points[-1]):
                self.points.append(point)
            self.finish_polygon()
            return
        super().mouseDoubleClickEvent(event)

    def keyPressEvent(self, event):
        if event.key() in (Qt.Key_Return, Qt.Key_Enter):
            self.finish_polygon()
        elif event.key() == Qt.Key_Escape:
            self.clear_draft()
        elif event.key() == Qt.Key_Backspace and self.points:
            self.points.pop()
            self.finished = False
            self.draw_draft()
            self.selection_changed.emit(None)
        else:
            super().keyPressEvent(event)


class RegionEditor(QMainWindow):
    def __init__(self, document):
        super().__init__()
        self.document, self.editing_id, self.vertices = document, None, None
        self.setWindowTitle('TurtleBot4 — Chọn vùng và đặt tên trên bản đồ')
        self.resize(1280, 820)
        self.canvas = MapCanvas(document)
        self.region_list = QListWidget()
        self.region_list.currentRowChanged.connect(self.select_region)
        self.id_field, self.name_field, self.alias_field = QLineEdit(), QLineEdit(), QLineEdit()
        self.name_field.setPlaceholderText('Ví dụ: Phòng học')
        self.alias_field.setPlaceholderText('Ví dụ: lớp học; phòng A')
        self.mode = QComboBox()
        self.mode.addItem('Đa giác — bấm các đỉnh', 'polygon')
        self.mode.addItem('Hình chữ nhật — kéo chuột', 'rectangle')
        self.mode.currentIndexChanged.connect(self.change_mode)
        self.clearance = QDoubleSpinBox()
        self.clearance.setRange(0.01, 10.0)
        self.clearance.setDecimals(2)
        self.clearance.setSingleStep(0.05)
        self.clearance.setValue(document.clearance)
        self.hint = QLabel('')
        self.hint.setWordWrap(True)
        self.hint.setMinimumHeight(70)
        self.hint.setStyleSheet('padding: 10px; background: #e8f0f9; color: #173651;')
        self.canvas.selection_changed.connect(self.on_selection)
        self.canvas.hint_changed.connect(self.hint.setText)
        form = QFormLayout()
        form.addRow('Cách chọn:', self.mode)
        form.addRow('ID vùng:', self.id_field)
        form.addRow('Tên vùng:', self.name_field)
        form.addRow('Tên gọi khác (;):', self.alias_field)
        form.addRow('Cách vật cản (m):', self.clearance)
        side = QWidget()
        side.setMinimumWidth(340)
        side.setMaximumWidth(460)
        layout = QVBoxLayout(side)
        title = QLabel('CHỌN VÙNG → ĐẶT TÊN → LƯU VÙNG')
        title.setStyleSheet('font-weight: bold; font-size: 15px; padding: 8px 0;')
        layout.addWidget(title)
        layout.addWidget(QLabel('Các vùng hiện có:'))
        layout.addWidget(self.region_list, 1)
        row = QHBoxLayout()
        self.add_button = QPushButton('Vùng mới')
        self.redraw_button = QPushButton('Vẽ lại vùng')
        self.add_button.clicked.connect(self.new_region)
        self.redraw_button.clicked.connect(self.redraw_region)
        row.addWidget(self.add_button)
        row.addWidget(self.redraw_button)
        layout.addLayout(row)
        layout.addLayout(form)
        self.save_button = QPushButton('Lưu vùng')
        self.save_button.setMinimumHeight(44)
        self.save_button.setStyleSheet('background: #217b50; color: white; font-weight: bold;')
        self.save_button.clicked.connect(self.save_region)
        layout.addWidget(self.save_button)
        layout.addWidget(self.hint)
        delete = QPushButton('Xóa vùng đang chọn và lưu')
        delete.clicked.connect(self.delete_region)
        layout.addWidget(delete)
        blank = QPushButton('Bộ vùng mới (vẽ lại toàn bộ)')
        blank.clicked.connect(self.new_document)
        layout.addWidget(blank)
        reload_button = QPushButton('Mở lại dữ liệu đã lưu')
        reload_button.clicked.connect(self.reload_document)
        layout.addWidget(reload_button)
        fit = QPushButton('Vừa bản đồ')
        fit.clicked.connect(self.canvas.fit_map)
        layout.addWidget(fit)
        note = QLabel('Đa giác: bấm từng đỉnh, Enter/chuột phải để khép vùng.\n'
                      'Backspace: bỏ đỉnh cuối · Esc: hủy nét vẽ\n'
                      'Cuộn chuột: phóng to/thu nhỏ.\n'
                      'Chấm màu: đích điều hướng tự chọn trong vùng.')
        note.setWordWrap(True)
        layout.addWidget(note)
        split = QSplitter()
        split.addWidget(self.canvas)
        split.addWidget(side)
        split.setStretchFactor(0, 1)
        self.setCentralWidget(split)
        self.refresh_list()
        self.new_region()

    def refresh_list(self, selected=None):
        self.region_list.blockSignals(True)
        self.region_list.clear()
        for entry in self.document.entries:
            self.region_list.addItem(f'{entry.id} — {entry.name}')
        index = next((i for i, e in enumerate(self.document.entries) if e.id == selected), -1)
        self.region_list.setCurrentRow(index)
        self.region_list.blockSignals(False)
        self.canvas.selected_id = selected
        self.canvas.rebuild()
        self.statusBar().showMessage(f'{len(self.document.entries)} vùng • {self.document.output}')

    def new_region(self):
        self.editing_id = None
        self.region_list.setCurrentRow(-1)
        self.canvas.selected_id = None
        self.canvas.clear_draft()
        self.canvas.rebuild()
        self.id_field.setText(self.document.next_id())
        self.name_field.clear()
        self.alias_field.clear()
        self.hint.setText('Vẽ vùng trên bản đồ, nhập tên rồi nhấn Lưu vùng. '
                          'Nếu muốn thay toàn bộ bản mẫu, chọn Bộ vùng mới.')

    def select_region(self, index):
        if not 0 <= index < len(self.document.entries):
            return
        entry = self.document.entries[index]
        self.editing_id = entry.id
        self.id_field.setText(entry.id)
        self.name_field.setText(entry.name)
        self.alias_field.setText('; '.join(entry.aliases))
        self.canvas.clear_draft()
        self.canvas.selected_id = entry.id
        self.canvas.rebuild()
        self.hint.setText('Có thể đổi tên rồi Lưu vùng, hoặc chọn Vẽ lại vùng để đổi đường biên.')

    def change_mode(self):
        self.canvas.mode = self.mode.currentData()
        self.canvas.clear_draft()

    def redraw_region(self):
        if self.editing_id is None:
            self.hint.setText('Chọn một vùng trong danh sách trước khi vẽ lại.')
            return
        self.canvas.clear_draft()
        self.hint.setText(f'Vẽ đường biên mới cho {self.editing_id}, sau đó nhấn Lưu vùng.')

    def on_selection(self, vertices):
        self.vertices = vertices
        if vertices is not None:
            self.hint.setText(f'Đã chọn {len(vertices)} đỉnh. Nhập tên và nhấn Lưu vùng.')
            self.name_field.setFocus()

    def save_region(self):
        if self.canvas.points and not self.canvas.finished:
            self.hint.setText('Hãy khép vùng bằng Enter/chuột phải trước khi lưu.')
            return False
        try:
            self.document.clearance = self.clearance.value()
            self.document.save_region(self.id_field.text(), self.name_field.text(),
                                      self.alias_field.text().replace(',', ';').split(';'),
                                      self.vertices, self.editing_id)
        except (ValueError, OSError) as exc:
            self.hint.setText(f'Chưa lưu: {exc}')
            return False
        region_id = self.id_field.text().strip()
        self.editing_id = region_id
        self.canvas.clear_draft()
        self.refresh_list(region_id)
        self.hint.setText(f'Đã lưu vùng {region_id}. Dữ liệu sẵn sàng cho điều hướng giọng nói. '
                          'Chọn Vùng mới để vẽ vùng tiếp theo.')
        return True

    def delete_region(self):
        try:
            self.document.delete_region(self.editing_id)
        except (ValueError, OSError) as exc:
            self.hint.setText(f'Chưa xóa: {exc}')
            return
        self.refresh_list()
        self.new_region()
        self.hint.setText('Đã xóa vùng và cập nhật dữ liệu trên đĩa.')

    def new_document(self):
        self.document.entries = []
        self.refresh_list()
        self.new_region()
        self.hint.setText('Bộ vùng mới đang trống. Vẽ và Lưu vùng đầu tiên để thay dữ liệu trên đĩa. '
                          'Mở lại dữ liệu đã lưu để quay lại bộ cũ trước khi lưu.')

    def reload_document(self):
        try:
            loaded = self.document.load()
        except (ValueError, OSError) as exc:
            self.hint.setText(f'Không thể mở lại: {exc}')
            return
        self.clearance.setValue(self.document.clearance)
        self.refresh_list()
        self.new_region()
        self.hint.setText('Đã mở lại các vùng đã lưu.' if loaded else 'Chưa có vùng đã lưu.')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', required=True, help='ROS occupancy-map YAML')
    parser.add_argument('--output', required=True, help='Folder for regions.yaml and labels.npy')
    parser.add_argument('--clearance', type=float, default=0.3)
    options = parser.parse_args(args)
    # OpenCV pip wheels can override Qt's plugin path with their private copy.
    os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = QLibraryInfo.location(QLibraryInfo.PluginsPath)
    os.environ.pop('QT_QPA_FONTDIR', None)
    app = QApplication.instance() or QApplication(sys.argv[:1])
    try:
        document = RegionDocument(OccupancyMap(options.map), options.output, options.clearance)
        document.load()
        editor = RegionEditor(document)
        editor.show()
        editor.canvas.fit_map()
    except (ValueError, OSError, KeyError) as exc:
        QMessageBox.critical(None, 'Không thể mở bản đồ', str(exc))
        return 1
    return app.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
