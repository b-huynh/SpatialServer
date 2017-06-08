import PIL.Image
import PIL.ExifTags	

# from https://gist.github.com/erans/983821
def convert_to_degress(value):
    """Helper function to convert the GPS coordinates stored in the EXIF to degress in float format"""
    d0 = value[0][0]
    d1 = value[0][1]
    d = float(d0) / float(d1)

    m0 = value[1][0]
    m1 = value[1][1]
    m = float(m0) / float(m1)

    s0 = value[2][0]
    s1 = value[2][1]
    s = float(s0) / float(s1)

    return d + (m / 60.0) + (s / 3600.0)

def get_exif(fpath):
	img = PIL.Image.open(fpath)
	exif_data = img._getexif()

	exif = {
		PIL.ExifTags.TAGS[k]: v
		for k, v in img._getexif().items()
		if k in PIL.ExifTags.TAGS
	}
	return exif

def hasGPSInfo(exif):
	return "GPSInfo" in exif

def get_lat_lon(exif):
	if not hasGPSInfo(exif):
		return None
	else:
		gps_info = exif["GPSInfo"]

	def get_or_none(data, key):
		return data[key] if key in data else None

	latitude_ref = get_or_none(gps_info, 1)
	latitude = get_or_none(gps_info, 2)
	longitude_ref = get_or_none(gps_info, 3)
	longitude = get_or_none(gps_info, 4)

	if not latitude_ref or not latitude or not longitude_ref or not longitude:
		print("Cannot parse GPS data:\n{}".format(gps_info))
		return None

	lat = convert_to_degress(latitude)
	if latitude_ref == "S":
		lat = 0 - lat
	lon = convert_to_degress(longitude)
	if longitude_ref == "W":
		lon = 0 - lon
	return (lat, lon)

exif = get_exif("antipasto.jpg")
if hasGPSInfo(exif):
	lat, lon = get_lat_lon(exif)
	print("{0:.8f}, {1:.8f}".format(lat, lon))	
