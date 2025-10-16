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
    """
    Represents a geographic origin with latitude, longitude, and optional altitude.

    Attributes:
        lat_deg (float): Latitude in degrees.
        lon_deg (float): Longitude in degrees.
        alt_m (float): Altitude in meters (default is 0.0).
    """
    lat_deg: float
    lon_deg: float
    alt_m: float = 0.0

def _as_arrays(x: Union[Number, Sequence[Number]]) -> list[float]:
    """
    Converts a number or sequence of numbers into a list of floats.

    Args:
        x (Union[Number, Sequence[Number]]): A single number or a sequence of numbers.

    Returns:
        list[float]: A list of floats.
    """
    if isinstance(x, (list, tuple)):
        return list(map(float, x))
    return [float(x)]

def make_local_aeqd_crs(origin: GeoOrigin) -> CRS:
    """
    Creates a local Azimuthal-Equidistant CRS centered at the given geographic origin.

    Args:
        origin (GeoOrigin): The geographic origin for the CRS.

    Returns:
        CRS: A pyproj.CRS object usable with GeoPandas .to_crs().
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
    Projects WGS84 longitude/latitude to local ENU coordinates in meters using GeoPandas.

    Args:
        lat (Union[Number, Sequence[Number]]): Latitude(s) in degrees.
        lon (Union[Number, Sequence[Number]]): Longitude(s) in degrees.
        alt (Optional[Union[Number, Sequence[Number]]], optional): Altitude(s) in meters. Defaults to None.
        origin (Optional[GeoOrigin], optional): The geographic origin for the projection. Defaults to None.

    Returns:
        Tuple[list[float], list[float], Optional[list[float]]]:
            x_east_m (list[float]): Easting coordinates in meters.
            y_north_m (list[float]): Northing coordinates in meters.
            z_up_m_or_None (Optional[list[float]]): Up coordinates in meters or None if altitude is not provided.
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
    Projects WGS84 longitude/latitude to a target EPSG coordinate reference system.

    Args:
        lat (Union[Number, Sequence[Number]]): Latitude(s) in degrees.
        lon (Union[Number, Sequence[Number]]): Longitude(s) in degrees.
        epsg (int): EPSG code of the target CRS.
        to_meters (bool, optional): Whether to convert outputs to meters if the target CRS uses US survey feet. Defaults to False.

    Returns:
        Tuple[list[float], list[float]]:
            x (list[float]): Projected x-coordinates.
            y (list[float]): Projected y-coordinates.
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
    Selects an appropriate EPSG code for a given geographic location based on latitude and longitude.

    Args:
        lat_deg (float): Latitude in degrees.
        lon_deg (float): Longitude in degrees.

    Returns:
        int: EPSG code for the selected CRS. Returns 0 if no suitable CRS is found.
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