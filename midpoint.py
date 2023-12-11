import math

def midpoint(lat1, lat2, lng1, lng2):
    dLon = math.radians(lng2-lng1)

    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lng1 = math.radians(lng1)

    bx = math.cos(lat2)*math.cos(dLon)
    by = math.cos(lat2)*math.sin(dLon)
    lat3 = math.atan2(math.sin(lat1) + math.sin(lat2), math.sqrt((math.cos(lat1) + bx) * (math.cos(lat1) + bx) + by * by))
    lon3 = lng1 + math.atan2(by, math.cos(lat1) + bx)
    
    lat3 = math.degrees(lat3)
    lon3 = math.degrees(lon3)

    # print(f"Lat: {lat3}, lng = {lon3}")
    return lat3, lon3