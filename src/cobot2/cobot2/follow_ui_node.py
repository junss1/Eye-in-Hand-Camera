# follow_ui_node v0.101 2026-02-04
# [이번 버전에서 수정된 사항]
# - (기능구현) LIVE/HISTORY 탭 추가: 실시간(annotated 영상+이벤트 로그) + DB 기록 조회(이벤트 선택 시 스냅샷 표시)
# - (버그수정) QLabel이 pixmap sizeHint를 따라가며 UI가 계속 커지는 현상 방지(영상/프리뷰 QLabel sizePolicy=Ignored, minimumSize 설정)
# - (기능구현) HISTORY에서 log_root_dir 아래 세션 폴더 자동 탐색 및 DB(read-only)로 events/snapshots 조인 조회
# - (기능구현) ROS spin을 Qt 타이머로 처리하여 UI/ROS 간섭 최소화(별도 스레드 없이 spin_once)

"""follow_ui_node

- LIVE 탭:
  - /follow/annotated_image (sensor_msgs/Image) 구독 → 왼쪽 영상 표시
  - /follow/ui_event (std_msgs/String) 구독 → 오른쪽 로그 리스트에 [시간] 메시지 추가

- HISTORY 탭:
  - log_root_dir 아래 세션 폴더(YYYYMMDD_HHMMSS_xxxxxxxx) 자동 탐색
  - 선택한 세션의 follow.db(read-only)에서 events + snapshots 조인 조회
  - 이벤트 클릭 시 해당 스냅샷 이미지 미리보기 표시

주의:
- DB는 UI에서 "읽기 전용"으로만 접근(Writer는 follow_logger_node).
- 이미지(스냅샷)는 DB가 아니라 파일로 저장되어 있어 경로(img_path)를 읽어 표시한다.
"""

from __future__ import annotations

import os
import sqlite3
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QLineEdit,
    QMessageBox,
)


def _local_time_str(ts: Optional[float] = None) -> str:
    if ts is None:
        ts = time.time()
    return time.strftime("%H:%M:%S", time.localtime(ts))


def _bgr_to_qpixmap(bgr: np.ndarray) -> QPixmap:
    if bgr is None:
        return QPixmap()

    if bgr.ndim == 2:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_GRAY2RGB)
    else:
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    h, w = rgb.shape[:2]
    qimg = QImage(rgb.data, w, h, int(rgb.strides[0]), QImage.Format_RGB888)
    return QPixmap.fromImage(qimg)


@dataclass
class HistoryRow:
    event_id: int
    ts_ns: int
    event: str
    img_path: Optional[str]


class FollowUiNode(Node):
    def __init__(self, ui: "MainWindow") -> None:
        super().__init__("follow_ui_node")
        self._ui = ui
        self._bridge = CvBridge()

        self.declare_parameter("annotated_image_topic", "/follow/annotated_image")
        self.declare_parameter("ui_event_topic", "/follow/ui_event")

        self._annotated_topic = str(self.get_parameter("annotated_image_topic").value)
        self._ui_event_topic = str(self.get_parameter("ui_event_topic").value)

        self.create_subscription(Image, self._annotated_topic, self._on_annotated_image, 10)
        self.create_subscription(String, self._ui_event_topic, self._on_ui_event, 50)

        self.get_logger().info(
            f"[FOLLOW_UI] subscribed annotated='{self._annotated_topic}', ui_event='{self._ui_event_topic}'"
        )

    def _on_annotated_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        if bgr is None:
            return
        self._ui.set_live_image(bgr)

    def _on_ui_event(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return
        self._ui.append_live_log(text)


class MainWindow(QWidget):
    def __init__(self) -> None:
        super().__init__()

        # -------------------------
        # Parameters (UI-side)
        # -------------------------
        self._log_root_dir = Path.home() / "logs" / "follow_runs"

        # live cache
        self._last_live_bgr: Optional[np.ndarray] = None

        # -------------------------
        # Window
        # -------------------------
        self.setWindowTitle("Follow UI (Live + History)")
        self.resize(1280, 720)

        root = QVBoxLayout(self)
        self._tabs = QTabWidget()
        root.addWidget(self._tabs)

        # LIVE tab
        self._tab_live = QWidget()
        self._tabs.addTab(self._tab_live, "LIVE")
        self._build_live_tab()

        # HISTORY tab
        self._tab_hist = QWidget()
        self._tabs.addTab(self._tab_hist, "HISTORY")
        self._build_history_tab()

        # initial
        self.refresh_sessions()

    # -------------------------
    # LIVE UI
    # -------------------------
    def _build_live_tab(self) -> None:
        layout = QHBoxLayout(self._tab_live)

        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter)

        # left: image
        left = QWidget()
        llay = QVBoxLayout(left)
        self._live_image = QLabel("(waiting annotated image...)")

        self._live_image.setAlignment(Qt.AlignCenter)
        # 중요: QLabel은 pixmap이 설정되면 sizeHint가 pixmap 크기로 커질 수 있음.
        # layout이 그 sizeHint를 따라가며 "가만히 있어도" 창/스플리터가 커지는 현상을 막기 위해 Ignored로 둔다.
        self._live_image.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self._live_image.setMinimumSize(1, 1)
        self._live_image.setStyleSheet("background-color: #111; color: #bbb; border: 1px solid #333;")
        llay.addWidget(self._live_image)

        splitter.addWidget(left)

        # right: log list
        right = QWidget()
        rlay = QVBoxLayout(right)
        self._live_log = QListWidget()
        self._live_log.setWordWrap(True)
        rlay.addWidget(self._live_log)

        btn_row = QHBoxLayout()
        self._btn_clear_live = QPushButton("Clear")
        self._btn_clear_live.clicked.connect(self._live_log.clear)
        btn_row.addWidget(self._btn_clear_live)
        btn_row.addStretch(1)
        rlay.addLayout(btn_row)

        splitter.addWidget(right)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

    def set_live_image(self, bgr: np.ndarray) -> None:
        self._last_live_bgr = bgr
        pix = _bgr_to_qpixmap(bgr)
        if pix.isNull():
            return
        # fit to label keeping aspect
        scaled = pix.scaled(self._live_image.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._live_image.setPixmap(scaled)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        # refresh live pixmap scaling on resize
        if self._last_live_bgr is not None:
            self.set_live_image(self._last_live_bgr)
        # refresh history snapshot scaling
        self._refresh_history_snapshot_scale()

    def append_live_log(self, text: str) -> None:
        ts = _local_time_str()
        item = QListWidgetItem(f"[{ts}] {text}")
        self._live_log.addItem(item)
        self._live_log.scrollToBottom()

    # -------------------------
    # HISTORY UI
    # -------------------------
    def _build_history_tab(self) -> None:
        layout = QVBoxLayout(self._tab_hist)

        # top controls
        top = QHBoxLayout()

        top.addWidget(QLabel("Log root:"))

        self._hist_root_edit = QLineEdit(str(self._log_root_dir))
        self._hist_root_edit.setReadOnly(False)
        top.addWidget(self._hist_root_edit, 3)

        self._btn_refresh_sessions = QPushButton("Refresh")
        self._btn_refresh_sessions.clicked.connect(self.refresh_sessions)
        top.addWidget(self._btn_refresh_sessions)

        top.addWidget(QLabel("Session:"))

        self._session_combo = QComboBox()
        self._session_combo.currentIndexChanged.connect(self._on_session_changed)
        top.addWidget(self._session_combo, 2)

        layout.addLayout(top)

        # splitter: left list / right preview
        splitter = QSplitter(Qt.Horizontal)
        layout.addWidget(splitter, 1)

        # left: event list
        left = QWidget()
        llay = QVBoxLayout(left)

        self._hist_list = QListWidget()
        self._hist_list.itemSelectionChanged.connect(self._on_history_item_selected)
        llay.addWidget(self._hist_list, 1)

        splitter.addWidget(left)

        # right: preview
        right = QWidget()
        rlay = QVBoxLayout(right)

        self._hist_preview = QLabel("(select an event)")
        self._hist_preview.setAlignment(Qt.AlignCenter)
        # LIVE와 동일 이유: pixmap sizeHint로 인해 레이아웃이 커지는 현상 방지
        self._hist_preview.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        self._hist_preview.setMinimumSize(1, 1)
        self._hist_preview.setStyleSheet("background-color: #111; color: #bbb; border: 1px solid #333;")
        rlay.addWidget(self._hist_preview, 1)

        self._hist_info = QLabel("")
        self._hist_info.setWordWrap(True)
        self._hist_info.setStyleSheet("color: #ddd;")
        rlay.addWidget(self._hist_info, 0)

        btn_row = QHBoxLayout()
        self._btn_open_file = QPushButton("Open file")
        self._btn_open_file.clicked.connect(self._open_selected_snapshot_external)
        btn_row.addWidget(self._btn_open_file)
        btn_row.addStretch(1)
        rlay.addLayout(btn_row)

        splitter.addWidget(right)

        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        # state
        self._history_rows: List[HistoryRow] = []
        self._history_row_by_item: Dict[int, HistoryRow] = {}
        self._selected_snapshot_bgr: Optional[np.ndarray] = None
        self._selected_snapshot_path: Optional[str] = None

    def refresh_sessions(self) -> None:
        root = Path(self._hist_root_edit.text()).expanduser()
        self._log_root_dir = root

        self._session_combo.blockSignals(True)
        self._session_combo.clear()

        if not root.exists():
            self._session_combo.addItem("(no root)", "")
            self._session_combo.blockSignals(False)
            return

        sessions = sorted([p for p in root.iterdir() if p.is_dir()], reverse=True)
        if not sessions:
            self._session_combo.addItem("(no sessions)", "")
            self._session_combo.blockSignals(False)
            return

        for p in sessions[:200]:
            db = p / "follow.db"
            if db.exists():
                self._session_combo.addItem(p.name, str(p))
        self._session_combo.blockSignals(False)

        if self._session_combo.count() > 0:
            self._session_combo.setCurrentIndex(0)
            self._on_session_changed()

    def _on_session_changed(self) -> None:
        sess_path = self._session_combo.currentData()
        if not sess_path:
            return
        sess_dir = Path(str(sess_path))
        db_path = sess_dir / "follow.db"
        if not db_path.exists():
            return

        rows = self._load_history_rows(db_path)
        self._history_rows = rows
        self._history_row_by_item.clear()
        self._hist_list.clear()

        for r in rows:
            t = time.strftime("%H:%M:%S", time.localtime(r.ts_ns / 1e9))
            has_img = "🖼" if r.img_path else ""
            item = QListWidgetItem(f"[{t}] {r.event} {has_img}")
            self._hist_list.addItem(item)
            self._history_row_by_item[id(item)] = r

        self._hist_preview.setText("(select an event)")
        self._hist_info.setText(f"Loaded {len(rows)} events from {db_path}")
        self._selected_snapshot_bgr = None
        self._selected_snapshot_path = None

    def _load_history_rows(self, db_path: Path) -> List[HistoryRow]:
        # read-only connection (SQLite URI)
        uri = f"file:{db_path.as_posix()}?mode=ro"
        conn = sqlite3.connect(uri, uri=True)
        try:
            cur = conn.cursor()
            cur.execute(
                """
                SELECT e.id AS event_id,
                       e.ts_ns,
                       e.event,
                       (SELECT s.img_path FROM snapshots s WHERE s.event_id = e.id ORDER BY s.id DESC LIMIT 1) AS img_path
                FROM events e
                ORDER BY e.id DESC
                LIMIT 500;
                """
            )
            out: List[HistoryRow] = []
            for event_id, ts_ns, event, img_path in cur.fetchall():
                out.append(HistoryRow(int(event_id), int(ts_ns), str(event), img_path if img_path else None))
            return out
        finally:
            conn.close()

    def _on_history_item_selected(self) -> None:
        items = self._hist_list.selectedItems()
        if not items:
            return
        item = items[0]
        row = self._history_row_by_item.get(id(item))
        if row is None:
            return

        self._selected_snapshot_bgr = None
        self._selected_snapshot_path = None

        t_full = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(row.ts_ns / 1e9))
        info = f"event_id={row.event_id}\n{t_full}\n{row.event}"

        if row.img_path and Path(row.img_path).exists():
            self._selected_snapshot_path = row.img_path
            bgr = cv2.imread(row.img_path, cv2.IMREAD_COLOR)
            if bgr is not None:
                self._selected_snapshot_bgr = bgr
                self._set_history_preview_image(bgr)
                info += f"\n{row.img_path}"
            else:
                self._hist_preview.setText("(failed to load image)")
        else:
            self._hist_preview.setText("(no snapshot)")
            if row.img_path:
                info += f"\n(missing file) {row.img_path}"

        self._hist_info.setText(info)

    def _set_history_preview_image(self, bgr: np.ndarray) -> None:
        pix = _bgr_to_qpixmap(bgr)
        if pix.isNull():
            return
        scaled = pix.scaled(self._hist_preview.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self._hist_preview.setPixmap(scaled)

    def _refresh_history_snapshot_scale(self) -> None:
        if self._selected_snapshot_bgr is not None:
            self._set_history_preview_image(self._selected_snapshot_bgr)

    def _open_selected_snapshot_external(self) -> None:
        if not self._selected_snapshot_path:
            QMessageBox.information(self, "Info", "No snapshot selected.")
            return
        path = self._selected_snapshot_path
        # xdg-open
        os.system(f'xdg-open "{path}" >/dev/null 2>&1 &')

    # -------------------------
    # Small helpers
    # -------------------------
    def show_error(self, msg: str) -> None:
        QMessageBox.critical(self, "Error", msg)


def main(args=None) -> None:
    # Qt app
    app = QApplication([])
    ui = MainWindow()
    ui.show()

    # ROS
    rclpy.init(args=args)
    node = FollowUiNode(ui)

    # Spin ROS in Qt timer
    timer = QTimer()
    timer.setInterval(10)  # ms
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    timer.start()

    try:
        app.exec_()
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()