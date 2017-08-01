import pyproj

def is_valid_lla(lat, lon, alt):
    return (-180 <= lon <= 180) and (-90 <= lat <= 90)

class WorldPoint(object):
    """A 3D point in the world defined with Lat/Lon/Alt or ENU cartesian."""
    ORIGIN_LAT = 34.41631195
    ORIGIN_LON = -119.84327558
    ORIGIN_ALT = 17
    WGS84_PROJ = pyproj.Proj('+proj=latlong +datum=WGS84')
    LOCAL_PROJ = pyproj.Proj(('+proj=tmerc +ellps=WGS84 +datum=WGS84 +k_0=1 '
                              '+lon_0={0} +lat_0={1} +x_0=0 +y_0=0 +axis=enu '
                              '+units=meter').format(ORIGIN_LON, ORIGIN_LAT))
    def __init__(self, x, y, z, use_lla=False):
        if use_lla:
            self.lat, self.lon, self.alt = y, x, z
            if not is_valid_lla(self.lat, self.lon, self.alt):
                raise ValueError("LLA values: {} out of range.".format(
                    (self.lat, self.lon, self.alt)))
            self.x, self.y, self.z = pyproj.transform(WorldPoint.WGS84_PROJ,
                WorldPoint.LOCAL_PROJ, self.lon, self.lat, self.alt)
            self.z -= WorldPoint.ORIGIN_ALT
        else:
            self.x, self.y, self.z = x, y, z
            self.lon, self.lat, self.alt = pyproj.transform(
                WorldPoint.LOCAL_PROJ, WorldPoint.WGS84_PROJ, x, y, z)
            self.alt += WorldPoint.ORIGIN_ALT

    @classmethod
    def from_lla(cls, lat, lon, alt):
        return cls(lon, lat, alt, use_lla=True)

    @classmethod
    def from_enu(cls, x, y, z):
        return cls(x, y, z, use_lla=False)
