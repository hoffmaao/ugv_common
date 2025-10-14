from dataclasses import dataclass
import math

@dataclass
class GeoOrigin:
    lat_deg: float
    lon_deg: float
    alt_m: float = 0.0

def wgs84_to_enu(lat_deg: float, lon_deg: float, alt_m: float,
                 origin: GeoOrigin) -> tuple[float, float, float]:
    """
    Simple local-tangent-plane ENU approximation using equirectangular for small areas.
    For production, swap with geographiclib or pyproj for accuracy over larger areas.
    """
    # Constants
    R_EARTH = 6378137.0  # meters
    lat0 = math.radians(origin.lat_deg)
    lon0 = math.radians(origin.lon_deg)
    lat  = math.radians(lat_deg)
    lon  = math.radians(lon_deg)
    dlat = lat - lat0
    dlon = lon - lon0
    x_e = dlon * math.cos(lat0) * R_EARTH     # East
    y_n = dlat * R_EARTH                      # North
    z_u = alt_m - origin.alt_m                # Up
    return (x_e, y_n, z_u)