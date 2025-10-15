# ugv_common

Shared utilities for UGV rover deployments:

- **Rover state** (`RoverState`): pose, velocity, and GNSS updates
- **Survey & waypoints** (`Survey`, `Waypoint`): load from GeoJSON (LineString / MultiLineString / Points), densify, and manage missions
- **Navigation** (`Navigator`): pure‑pursuit steering and distance/bearing helpers
- **Geodesy** (`geodesy`): WGS84 → **local ENU** via GeoPandas/pyproj, and support for common CRSs (EPSG:3031, 3413, and NY State Plane)

This package is imported by both:
- `ugv_ws` (ROS 2 nodes wrap these classes and publish standard messages)
- `ugv_jetson` (web app displays live state & loads mission files)

---

## Installation

```bash
# System deps (recommended on Ubuntu/Jetson)
sudo apt-get update
sudo apt-get install -y gdal-bin libgdal-dev

# Python deps
pip install -U pip
pip install -e ".[geo]"   # installs geopandas, pyproj, shapely as well