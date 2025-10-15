from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable, Sequence, Tuple, Union, Optional

# Geo stack
try:
    import geopandas as gpd
    from shapely.geometry import Point
    from pyproj import CRS
except Exception as e:
    raise ImportError(
        "ugv_common.geodesy requires GeoPandas, Shapely and pyproj. "
        "Install extras with: pip install 'ugv-common[geo]'"
    ) from e

Number = Union[int, float]
US_SURVEY_FOOT_TO_M = 1200.0 / 3937.0  # exact factor

@dataclass
class GeoOrigin:
    lat_deg: float
    lon_deg: float
    alt_m: float = 0.0

def _as_arrays(x: Union[Number, Sequence[Number]]) -> list[float]:
    if isinstance(x, (list, tuple)):
        return list(map(float, x))
    return [float(x)]

def make_local_aeqd_crs(origin: GeoOrigin) -> CRS:
    """
    Local Azimuthal-Equidistant CRS centered at mission origin.
    Returns a pyproj.CRS usable with GeoPandas .to_crs().
    """
    proj4 = (
        f"+proj=aeqd +lat_0={origin.lat_deg} +lon_0={origin.lon_deg} "
        "+x_0=0 +y_0=0 +datum=WGS84 +units=m +no_defs"
    )
    return CRS.from_proj4(proj4)

def wgs84_to_enu_gpd(
    lat: Union[Number, Sequence[Number]],
    lon: Union[Number, Sequence[Number]],
    alt: Optional[Union[Number, Sequence[Number]]] = None,
    origin: Optional[GeoOrigin] = None,
) -> Tuple[list[float], list[float], Optional[list[float]]]:
    """
    Project WGS84 lon/lat to local ENU in meters using GeoPandas (AEQD@origin).
    If origin is None, the first point is used as the origin.
    Returns (x_east_m, y_north_m, z_up_m_or_None)
    """
    lats = _as_arrays(lat); lons = _as_arrays(lon)
    if len(lats) != len(lons):
        raise ValueError("lat and lon must have the same length")
    if origin is None:
        if not lats:
            return [], [], None
        origin = GeoOrigin(lat_deg=lats[0], lon_deg=lons[0], alt_m=float(_as_arrays(alt or [0.0])[0]))

    # Build GeoSeries in EPSG:4326 (WGS84)
    gdf = gpd.GeoDataFrame(geometry=gpd.points_from_xy(lons, lats), crs="EPSG:4326")
    local_crs = make_local_aeqd_crs(origin)
    gdf_local = gdf.to_crs(local_crs)
    x = gdf_local.geometry.x.tolist()
    y = gdf_local.geometry.y.tolist()

    z = None
    if alt is not None:
        alts = _as_arrays(alt)
        if len(alts) not in (1, len(lats)):
            raise ValueError("alt must be scalar or same length as lat/lon")
        if len(alts) == 1:
            alts = alts * len(lats)
        z = [a - origin.alt_m for a in alts]

    return x, y, z

def project_points(
    lat: Union[Number, Sequence[Number]],
    lon: Union[Number, Sequence[Number]],
    epsg: int,
    to_meters: bool = False,
) -> Tuple[list[float], list[float]]:
    """
    Project WGS84 lon/lat to a target EPSG (e.g., 3031, 3413, 2260, 2263).
    If the target CRS uses US survey feet (e.g., 2260/2263), set to_meters=True to convert outputs to meters.
    """
    lats = _as_arrays(lat); lons = _as_arrays(lon)
    gdf = gpd.GeoDataFrame(geometry=gpd.points_from_xy(lons, lats), crs="EPSG:4326")
    target = CRS.from_epsg(epsg)
    gdf_p = gdf.to_crs(target)
    x = gdf_p.geometry.x.tolist()
    y = gdf_p.geometry.y.tolist()

    unit = ""
    try:
        # pyproj >= 3.6
        unit = (target.axis_info[0].unit_name or "").lower()
    except Exception:
        pass

    if to_meters and ("foot" in unit or "ft" in unit):
        x = [xi * US_SURVEY_FOOT_TO_M for xi in x]
        y = [yi * US_SURVEY_FOOT_TO_M for yi in y]
    return x, y

def select_crs_for_location(lat_deg: float, lon_deg: float) -> int:
    """
    Heuristic CRS selector:
    - lat <= -60 → EPSG:3031 (Antarctica)
    - lat >=  60 → EPSG:3413 (Arctic/Greenland)
    - NY region quick check:
        * Long Island NYC bbox → EPSG:2263 (ftUS)
        * New York East bbox   → EPSG:2260 (ftUS)
    - otherwise None (use local ENU/AEQD or UTM)
    """
    if lat_deg <= -60.0:
        return 3031
    if lat_deg >= 60.0:
        return 3413

    # Long Island (EPSG:2263) WGS84 bounds (approx per EPSG): lon [-74.26, -71.8], lat [40.47, 41.30]
    if (-74.26 <= lon_deg <= -71.8) and (40.47 <= lat_deg <= 41.30):
        return 2263
    # New York East (EPSG:2260) WGS84 bounds: lon [-75.87, -73.23], lat [40.88, 45.02]
    if (-75.87 <= lon_deg <= -73.23) and (40.88 <= lat_deg <= 45.02):
        return 2260

    return 0  # unknown / use local ENU