# follow_logger_node v0.000 2026-02-04
# [이번 버전에서 수정된 사항]
# - (기능구현) /follow/ui_event 이벤트를 SQLite(events 테이블)에 저장
# - (기능구현) 이벤트 수신 시점에 최신 /follow/annotated_image 프레임을 스냅샷(JPG/PNG)으로 저장하고 DB(snapshots 테이블)에 경로 기록
# - (기능구현) 세션 단위 폴더/DB 생성 및 sessions 테이블에 start/end 기록
# - (기능구현) SQLite WAL 모드 적용(PRAGMA)으로 write 중 read 가능하게 구성

"""
follow_logger_node
- 목적: UI로 흘러가는 이벤트(/follow/ui_event)를 DB로 남기고,
        (선택) 이벤트 순간의 annotated 프레임(/follow/annotated_image)을 스냅샷 파일로 저장한다.
- 원칙:
  1) 이미지 자체를 DB에 넣지 않는다(용량/성능 문제). DB에는 파일 경로만 저장.
  2) SQLite는 로컬 파일 기반이며, WAL 모드로 기록 안정성/동시 읽기 성능을 확보한다.
  3) ROS 콜백에서는 최소 작업만 하고, DB write는 전용 워커 스레드로 처리한다.
"""

from __future__ import annotations

import json
import queue
import re
import signal
import sqlite3
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge


def now_ns() -> int:
    return time.time_ns()


def sanitize_filename(s: str, max_len: int = 60) -> str:
    s = s.strip()
    s = re.sub(r"\s+", "_", s)
    s = re.sub(r"[^A-Za-z0-9_\-가-힣]+", "", s)
    if not s:
        s = "event"
    return s[:max_len]


SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS sessions (
    session_id TEXT PRIMARY KEY,
    start_ts_ns INTEGER NOT NULL,
    end_ts_ns INTEGER,
    note TEXT
);

CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL,
    ts_ns INTEGER NOT NULL,
    event TEXT NOT NULL,
    source TEXT DEFAULT 'unknown',
    meta_json TEXT,
    FOREIGN KEY(session_id) REFERENCES sessions(session_id)
);

CREATE INDEX IF NOT EXISTS idx_events_ts ON events(ts_ns);
CREATE INDEX IF NOT EXISTS idx_events_session ON events(session_id);

CREATE TABLE IF NOT EXISTS snapshots (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id INTEGER NOT NULL,
    ts_ns INTEGER NOT NULL,
    img_path TEXT NOT NULL,
    width INTEGER,
    height INTEGER,
    FOREIGN KEY(event_id) REFERENCES events(id)
);

CREATE INDEX IF NOT EXISTS idx_snapshots_event ON snapshots(event_id);
CREATE INDEX IF NOT EXISTS idx_snapshots_ts ON snapshots(ts_ns);
"""


@dataclass
class EventRecord:
    ts_ns: int
    event: str
    source: str
    meta_json: Optional[str] = None


@dataclass
class SnapshotRecord:
    ts_ns: int
    img: np.ndarray  # BGR
    width: int
    height: int


class FollowLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("follow_logger_node")

        # Params
        self.declare_parameter("ui_event_topic", "/follow/ui_event")
        self.declare_parameter("annotated_image_topic", "/follow/annotated_image")
        self.declare_parameter("log_root_dir", str(Path.home() / "logs" / "follow_runs"))
        self.declare_parameter("session_note", "")
        self.declare_parameter("event_source", "unknown")

        self.declare_parameter("snapshot_enable", True)
        self.declare_parameter("snapshot_on_events", [])  # 빈 리스트면 모든 이벤트 스냅샷
        self.declare_parameter("snapshot_format", "jpg")  # jpg|png
        self.declare_parameter("jpg_quality", 90)
        self.declare_parameter("snapshot_max_age_sec", 0.25)

        self.declare_parameter("frame_cache_enable", True)

        self.declare_parameter("sqlite_busy_timeout_ms", 3000)
        self.declare_parameter("sqlite_wal", True)
        self.declare_parameter("sqlite_synchronous", "NORMAL")  # OFF|NORMAL|FULL

        # Load
        self._ui_event_topic = str(self.get_parameter("ui_event_topic").value)
        self._annotated_image_topic = str(self.get_parameter("annotated_image_topic").value)
        self._log_root_dir = Path(str(self.get_parameter("log_root_dir").value)).expanduser()
        self._session_note = str(self.get_parameter("session_note").value)
        self._event_source = str(self.get_parameter("event_source").value)

        self._snapshot_enable = bool(self.get_parameter("snapshot_enable").value)
        self._snapshot_on_events: List[str] = list(self.get_parameter("snapshot_on_events").value)
        self._snapshot_format = str(self.get_parameter("snapshot_format").value).lower().strip()
        self._jpg_quality = int(self.get_parameter("jpg_quality").value)
        self._snapshot_max_age_sec = float(self.get_parameter("snapshot_max_age_sec").value)

        self._frame_cache_enable = bool(self.get_parameter("frame_cache_enable").value)

        self._sqlite_busy_timeout_ms = int(self.get_parameter("sqlite_busy_timeout_ms").value)
        self._sqlite_wal = bool(self.get_parameter("sqlite_wal").value)
        self._sqlite_synchronous = str(self.get_parameter("sqlite_synchronous").value).upper().strip()

        if self._snapshot_format not in ("jpg", "png"):
            self.get_logger().warn(f"snapshot_format='{self._snapshot_format}' invalid -> force 'jpg'")
            self._snapshot_format = "jpg"

        # Session paths
        self._session_id = str(uuid.uuid4())
        ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        self._session_dir = self._log_root_dir / f"{ts}_{self._session_id[:8]}"
        self._snap_dir = self._session_dir / "snapshots"
        self._session_dir.mkdir(parents=True, exist_ok=True)
        self._snap_dir.mkdir(parents=True, exist_ok=True)

        self._db_path = self._session_dir / "follow.db"

        # DB worker
        self._q: "queue.Queue[Tuple[EventRecord, Optional[SnapshotRecord]]]" = queue.Queue(maxsize=2000)
        self._stop_ev = threading.Event()
        self._db_thread = threading.Thread(target=self._db_worker, daemon=True)

        # frame cache
        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._last_frame_bgr: Optional[np.ndarray] = None
        self._last_frame_ts_ns: int = 0
        self._last_frame_wh: Tuple[int, int] = (0, 0)

        # Subs
        self.create_subscription(String, self._ui_event_topic, self._on_ui_event, 50)
        self.create_subscription(Image, self._annotated_image_topic, self._on_annotated_image, 10)

        # Start
        self._db_thread.start()

        self.get_logger().info(
            f"[FOLLOW_LOGGER] session_id={self._session_id} db={self._db_path} snap_dir={self._snap_dir}"
        )
        self.get_logger().info(
            f"[FOLLOW_LOGGER] topics: ui_event='{self._ui_event_topic}', annotated_image='{self._annotated_image_topic}'"
        )

        # Signals
        signal.signal(signal.SIGINT, self._sig_handler)
        signal.signal(signal.SIGTERM, self._sig_handler)

        # session start marker
        self._enqueue_system_event("SESSION_START", meta={"note": self._session_note})

    def _on_annotated_image(self, msg: Image) -> None:
        if not self._frame_cache_enable:
            return
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"[FOLLOW_LOGGER] imgmsg_to_cv2 failed: {e}")
            return

        if img is None:
            return

        ts_ns = now_ns()
        h, w = img.shape[:2]
        with self._frame_lock:
            self._last_frame_bgr = img
            self._last_frame_ts_ns = ts_ns
            self._last_frame_wh = (w, h)

    def _on_ui_event(self, msg: String) -> None:
        text = (msg.data or "").strip()
        if not text:
            return

        ts_ns = now_ns()
        event = EventRecord(ts_ns=ts_ns, event=text, source=self._event_source, meta_json=None)

        snap: Optional[SnapshotRecord] = None
        if self._snapshot_enable and self._should_snapshot(text):
            snap = self._make_snapshot_if_fresh(ts_ns)

        try:
            self._q.put_nowait((event, snap))
        except queue.Full:
            self.get_logger().warn("[FOLLOW_LOGGER] queue full -> drop event")

    def _should_snapshot(self, event_text: str) -> bool:
        if not self._snapshot_on_events:
            return True
        return event_text in self._snapshot_on_events

    def _make_snapshot_if_fresh(self, event_ts_ns: int) -> Optional[SnapshotRecord]:
        with self._frame_lock:
            img = None if self._last_frame_bgr is None else self._last_frame_bgr.copy()
            frame_ts_ns = self._last_frame_ts_ns
            w, h = self._last_frame_wh

        if img is None or frame_ts_ns <= 0:
            return None

        age_sec = abs(event_ts_ns - frame_ts_ns) / 1e9
        if age_sec > self._snapshot_max_age_sec:
            return None

        return SnapshotRecord(ts_ns=event_ts_ns, img=img, width=w, height=h)

    def _db_worker(self) -> None:
        conn = sqlite3.connect(str(self._db_path), check_same_thread=False)
        conn.execute("PRAGMA foreign_keys=ON;")
        conn.execute(f"PRAGMA busy_timeout={self._sqlite_busy_timeout_ms};")

        if self._sqlite_wal:
            try:
                conn.execute("PRAGMA journal_mode=WAL;")
            except Exception:
                pass

        if self._sqlite_synchronous in ("OFF", "NORMAL", "FULL"):
            try:
                conn.execute(f"PRAGMA synchronous={self._sqlite_synchronous};")
            except Exception:
                pass

        conn.executescript(SCHEMA_SQL)
        conn.commit()

        conn.execute(
            "INSERT OR REPLACE INTO sessions(session_id, start_ts_ns, end_ts_ns, note) VALUES(?, ?, ?, ?)",
            (self._session_id, now_ns(), None, self._session_note),
        )
        conn.commit()

        while not self._stop_ev.is_set():
            try:
                event, snap = self._q.get(timeout=0.2)
            except queue.Empty:
                continue

            try:
                ev_id = self._insert_event(conn, event)
                if snap is not None:
                    path = self._save_snapshot_file(snap, event.event)
                    if path is not None:
                        self._insert_snapshot(conn, ev_id, snap.ts_ns, path, snap.width, snap.height)
                conn.commit()
            except Exception as e:
                try:
                    conn.rollback()
                except Exception:
                    pass
                self.get_logger().error(f"[FOLLOW_LOGGER] DB write failed: {e}")

        # end session
        try:
            conn.execute("UPDATE sessions SET end_ts_ns=? WHERE session_id=?", (now_ns(), self._session_id))
            conn.commit()
        except Exception:
            pass

        try:
            conn.close()
        except Exception:
            pass

    def _insert_event(self, conn: sqlite3.Connection, event: EventRecord) -> int:
        cur = conn.execute(
            "INSERT INTO events(session_id, ts_ns, event, source, meta_json) VALUES(?, ?, ?, ?, ?)",
            (self._session_id, event.ts_ns, event.event, event.source, event.meta_json),
        )
        return int(cur.lastrowid)

    def _insert_snapshot(
        self,
        conn: sqlite3.Connection,
        event_id: int,
        ts_ns: int,
        img_path: str,
        width: int,
        height: int,
    ) -> None:
        conn.execute(
            "INSERT INTO snapshots(event_id, ts_ns, img_path, width, height) VALUES(?, ?, ?, ?, ?)",
            (event_id, ts_ns, img_path, width, height),
        )

    def _save_snapshot_file(self, snap: SnapshotRecord, event_text: str) -> Optional[str]:
        ts_tag = time.strftime("%Y%m%d_%H%M%S", time.localtime(snap.ts_ns / 1e9))
        ms = int((snap.ts_ns % 1_000_000_000) / 1_000_000)
        name = sanitize_filename(event_text)
        fname = f"{ts_tag}_{ms:03d}_{name}.{self._snapshot_format}"
        path = self._snap_dir / fname

        if self._snapshot_format == "jpg":
            params = [int(cv2.IMWRITE_JPEG_QUALITY), int(max(0, min(100, self._jpg_quality)))]
        else:
            params = [int(cv2.IMWRITE_PNG_COMPRESSION), 3]

        try:
            ok = cv2.imwrite(str(path), snap.img, params)
        except Exception:
            ok = False

        if not ok:
            return None
        return str(path)

    def _enqueue_system_event(self, name: str, meta: Optional[dict] = None) -> None:
        ts_ns = now_ns()
        meta_json = json.dumps(meta, ensure_ascii=False) if meta is not None else None
        event = EventRecord(ts_ns=ts_ns, event=name, source="system", meta_json=meta_json)
        try:
            self._q.put_nowait((event, None))
        except queue.Full:
            pass

    def _sig_handler(self, signum, frame) -> None:
        self._stop_ev.set()

    def destroy_node(self) -> bool:
        self._stop_ev.set()
        try:
            self._db_thread.join(timeout=2.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._enqueue_system_event("SESSION_END")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()