# ugv_common/survey.py
from __future__ import annotations
from dataclasses import dataclass, field
from typing import List, Optional
import csv, json, datetime
from .geodesy import GeoOrigin, wgs84_to_enu

@dataclass
class Waypoint:
    # Local ENU (map) coordinates
    x: float
    y: float
    z: float = 0.0
    speed_mps: float = 0.7
    hold_sec: float = 0.0
    yaw_rad: Optional[float] = None
    tolerance_m: float = 0.5
    id: Optional[str] = None

@dataclass
class Survey:
    name: str
    started_at: datetime.datetime
    origin: GeoOrigin
    waypoints: List[Waypoint] = field(default_factory=list)
    notes: str = ""

    # ---------- Factories ----------
    @classmethod
    def from_csv(cls, name: str, filepath: str, origin: GeoOrigin) -> "Survey":
        wps: List[Waypoint] = []
        with open(filepath, "r", newline="") as f:
            reader = csv.DictReader(filter(lambda row: row[0] != "#", f))
            for row in reader:
                lat = float(row["lat"]); lon = float(row["lon"])
                alt = float(row.get("alt_m", 0.0))
                speed = float(row.get("speed_mps", 0.7))
                hold  = float(row.get("hold_sec", 0.0))
                yaw   = row.get("yaw_deg"); yaw_rad = None if yaw in (None, "") else float(yaw) * 3.1415926535/180.0
                tol   = float(row.get("tolerance_m", 0.5))
                x, y, z = wgs84_to_enu(lat, lon, alt, origin)
                wps.append(Waypoint(x=x, y=y, z=z, speed_mps=speed, hold_sec=hold,
                                    yaw_rad=yaw_rad, tolerance_m=tol))
        return cls(name=name, started_at=datetime.datetime.utcnow(), origin=origin, waypoints=wps)

    @classmethod
    def from_json(cls, name: str, filepath: str, origin: GeoOrigin) -> "Survey":
        with open(filepath, "r") as f:
            data = json.load(f)
        wps: List[Waypoint] = []
        frame = data.get("frame", "wgs84").lower()
        for i, w in enumerate(data["waypoints"]):
            if frame == "wgs84":
                x, y, z = wgs84_to_enu(w["lat"], w["lon"], w.get("alt", 0.0), origin)
            else:
                x, y, z = w["x"], w["y"], w.get("z", 0.0)
            wps.append(Waypoint(
                x=x, y=y, z=z,
                speed_mps=w.get("speed", 0.7),
                hold_sec=w.get("hold", 0.0),
                yaw_rad=(None if w.get("yaw") is None else float(w["yaw"])),
                tolerance_m=w.get("tol", 0.5),
                id=w.get("id", f"wp{i:03d}")
            ))
        return cls(name=name, started_at=datetime.datetime.utcnow(), origin=origin, waypoints=wps)

    # ---------- Introspection ----------
    def next_waypoint(self, index: int) -> Optional[Waypoint]:
        return self.waypoints[index] if 0 <= index < len(self.waypoints) else None

    def count(self) -> int:
        return len(self.waypoints)