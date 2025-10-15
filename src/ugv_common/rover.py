from dataclasses import dataclass, field, asdict
from typing import Optional
import time, math
from .geodesy import GeoOrigin, wgs84_to_enu
from .types import wrap_angle

@dataclass
class RoverState:
    origin: Optional[GeoOrigin] = None
    # ENU/map position
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    # Heading & speed
    yaw_rad: float = 0.0
    speed_mps: float = 0.0
    # Last geo fix
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    alt_m:  Optional[float] = None
    hacc_m: Optional[float] = None
    fix_ok: bool = False
    # housekeeping
    last_update_s: float = field(default_factory=lambda: time.time())

    def set_origin_if_needed(self, lat: float, lon: float, alt: float):
        if self.origin is None:
            self.origin = GeoOrigin(lat, lon, alt)

    def update_from_gnss(self,
                         lat_deg: float, lon_deg: float, alt_m: float,
                         hacc_m: Optional[float] = None,
                         cog_rad: Optional[float] = None,
                         gspeed_mps: Optional[float] = None,
                         fix_ok: bool = True):
        self.set_origin_if_needed(lat_deg, lon_deg, alt_m)
        self.lat_deg, self.lon_deg, self.alt_m = lat_deg, lon_deg, alt_m
        self.hacc_m = hacc_m
        self.fix_ok = bool(fix_ok)
        if self.origin:
            self.x_m, self.y_m, self.z_m = wgs84_to_enu(lat_deg, lon_deg, alt_m, self.origin)
        if gspeed_mps is not None:
            self.speed_mps = float(gspeed_mps)
        if cog_rad is not None:
            self.yaw_rad = wrap_angle(float(cog_rad))
        self.last_update_s = time.time()

    def to_dict(self):
        d = asdict(self)
        if self.origin:
            d["origin"] = {"lat_deg": self.origin.lat_deg, "lon_deg": self.origin.lon_deg, "alt_m": self.origin.alt_m}
        return d