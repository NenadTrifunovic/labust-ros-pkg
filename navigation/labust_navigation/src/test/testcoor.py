#!/usr/bin/env python
from pysatel import coord
from math import radians, degrees
from scipy import mat, cos, sin, arctan, sqrt, pi, arctan2, linalg
import math
import geodesy
from pyproj import Proj, transform

deg2rad = math.pi/180;
rad2deg = 180/math.pi;
radius = 6378137;
ratio = 0.99664719;

def mpdlat(lat):
    return (111132.954 - 559.822*math.cos(2*lat*deg2rad) + 1.175*math.cos(4*lat*deg2rad))

def mpdlon(lat):
    return (radius*math.cos(math.atan(ratio*math.tan(lat*deg2rad)))*deg2rad);

def deg2meter(difflat, difflon, lat):
    return difflat*mpdlat(lat),difflon*mpdlon(lat);

def meter2deg(x, y, lat):
    return x/mpdlat(lat),y/mpdlon(lat);
    
def ecef2enu(X, Y, Z, lat, lon, alt):
    X0, Y0, Z0 = coord.geodetic2ecef(lat, lon, alt)
    lat, lon = radians(lat), radians(lon)
    mx = mat('[%f %f %f; %f %f %f; %f %f %f]' %
        (-sin(lon), -sin(lat) * cos(lon), cos(lat) * cos(lon), cos(lon),
         -sin(lat) * sin(lon), cos(lat) * sin(lon), 0, cos(lat), sin(lat)))
    geo = mat('[%f; %f; %f]' % (X0, Y0, Z0))
    res = mat('[%f; %f; %f]' % (X, Y, Z))
    enu = mx.transpose()*(res - geo)
    return enu[1], enu[0], -enu[2]

def ned2ecef(lat, lon, alt, n, e, d):
    X0, Y0, Z0 = coord.geodetic2ecef(lat, lon, alt)
    lat, lon = radians(lat), radians(lon)
    
    pitch = math.pi/2 + lat
    yaw = -lon 
    
    my = mat('[%f %f %f; %f %f %f; %f %f %f]' %
        (cos(pitch), 0, -sin(pitch),
         0,1,0,
         sin(pitch), 0, cos(pitch)))
    
    mz = mat('[%f %f %f; %f %f %f; %f %f %f]' %
        (cos(yaw), sin(yaw),0,
         -sin(yaw),cos(yaw),0,
         0,0,1))
    
    mr = mat('[%f %f %f; %f %f %f; %f %f %f]' %
        (-cos(lon)*sin(lat), -sin(lon), -cos(lat) * cos(lon), 
         -sin(lat)*sin(lon), cos(lon), -sin(lon)*cos(lat),
         cos(lat), 0, -sin(lat)))
    
    geo = mat('[%f; %f; %f]' % (X0, Y0, Z0))
    ned = mat('[%f; %f; %f]' % (n, e, d))
    res = mr*ned + geo
    return res[0], res[1], res[2]  

def ecef2ned(lat, lon, alt, X, Y, Z):
    X0, Y0, Z0 = coord.geodetic2ecef(lat, lon, alt)
    lat, lon = radians(lat), radians(lon)
    
    mr = mat('[%f %f %f; %f %f %f; %f %f %f]' %
        (-cos(lon)*sin(lat), -sin(lon), -cos(lat) * cos(lon), 
         -sin(lat)*sin(lon), cos(lon), -sin(lon)*cos(lat),
         cos(lat), 0, -sin(lat)))
    
    geo = mat('[%f; %f; %f]' % (X0, Y0, Z0))
    res = mat('[%f; %f; %f]' % (X, Y, Z))
    res = mr.transpose()*(res - geo)
    return float(res[0]), res[1], res[2]   
    
if __name__ == "__main__":
    print("Hello")
    latInit = 42;
    lonInit= 16;
    altInit = 1000;
    
    xof = 1000;
    yof = 5000;
    zof = 1000;
    
    #IST conversion
    xg,yg,zg = coord.geodetic2ecef(latInit,lonInit, altInit/1000)
    print("IST Geo -> ECEF: {0} {1} {2}".format(xg,yg,zg))
    a,b,c = ecef2enu(xg, yg, zg, latInit, lonInit, altInit/1000)
    print("IST ECEF -> NED: {0} {1} {2}".format(a*1000,b*1000,c*1000))
    xg,yg,zg = coord.enu2ecef(latInit, lonInit, altInit/1000, a, b, c)
    print("IST NED -> ECEF: {0} {1} {2}".format(xg,yg,zg))
    latR,lonR = coord.ecef2geodetic(xg,yg,zg)
    print("IST ECEF -> Geo: {0} {1}".format(latR,lonR))
    
    xg,yg,zg = coord.enu2ecef(latInit, lonInit, altInit/1000, xof/1000,yof/1000,zof/1000)
    print("IST NED -> ECEF 1: {0} {1} {2}".format(xg,yg,zg))
    latR,lonR = coord.ecef2geodetic(xg,yg,zg)
    print("IST NED -> Geo: {0} {1}".format(latR,lonR))
    xg,yg,zg = ned2ecef(latInit, lonInit, altInit/1000, xof/1000,yof/1000,zof/1000)
    print("IST NED -> ECEF 2: {0} {1} {2}".format(xg,yg,zg))
    a,b,c = ecef2ned(latInit, lonInit, altInit/1000, xg, yg, zg)
    print("IST Geo -> NED 2: {0} {1} {2}".format(a*1000,b*1000,c*1000))
    #xg,yg,zg = coord.geodetic2ecef(latR,lonR, zof)
    a,b,c = ecef2enu(xg, yg, zg, latInit, lonInit, altInit/1000)
    print("IST Geo -> NED: {0} {1} {2}".format(a*1000,b*1000,c*1000))
       
    #LABUST conversions
    xg,yg,zg = coord.geodetic2ecef(latInit,lonInit, altInit)
    dLat,dLon = meter2deg(xof, yof, latInit)
    latR = latInit + dLat
    lonR = lonInit + dLon
    print("LABUST NED -> Geo: {0} {1}".format(latR,lonR))
    dLat = latR - latInit
    dLon = lonR - lonInit      
    dx,dy = deg2meter(dLat, dLon, latInit)
    print("LABUST Geo -> NED: {0} {1}".format(dx,dy))
       
       
    #Proj conversions     
    p1 = Proj(proj='latlong',ellps='WGS84', datum='WGS84')
    p2 = Proj(proj='geocent',  ellps='WGS84', datum='WGS84')
    p3 = Proj(proj="sterea", lat_0=latInit, lon_0=lonInit, k_0=1.0, x_0=0, y_0=0, ellps="WGS84", datum="WGS84")
    xg,yg,zg = transform(p1,p2,lonInit,latInit,altInit, radians=False)
    print("Proj Geo -> ECEF: {0} {1} {2}".format(xg,yg,zg))
    b,a,c = transform(p3,p2,yof,xof,zof, radians=False)
    print("Proj NED -> Geo: {0} {1} {2}".format(a,b,c))
    a,b,c = transform(p2,p1,b,a,c, radians=False)
    print("Proj Geo -> NED: {0} {1} {2}".format(a,b,c))
    #xg,yg,zg = coord.enu2ecef(latInit, lonInit, altInit, a, b, c)
    #print("Proj NED -> ECEF: {0} {1} {2}".format(xg,yg,zg))
    lonR,latR,altR = transform(p2,p3,xg,yg,zg, radians=False)
    print("Proj ECEF -> Geo: {0} {1}".format(latR,lonR))
    
   
    
    ##xg,yg,zg = 
    ##b,a,c = transform(p3,p1,1,0,0, radians=False)
    #print("PyProj: {0} {1} {2}".format(xg,yg,zg))
    #print("PyProj NED: {0} {1} {2}".format(a,b,c))
    
    