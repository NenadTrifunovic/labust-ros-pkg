#!/bin/bash
# Setup desired geoid, gravity and magnetic models
geolib_link=https://sf.net/projects/geographiclib/files
geolib_dir=/usr/share/geographiclib
mkdir -p $geolib_dir
cd $geolib_dir 

wget $geolib_link/magnetic-distrib/emm2015.tar.bz2 
tar xvjf emm2015.tar.bz2

wget $geolib_link/geoids-distrib/egm96-15.tar.bz2 
tar xvjf egm96-15.tar.bz2

wget $geolib_link/gravity-distrib/egm96.tar.bz2
tar xvjf egm96.tar.bz2

rm *.tar.bz2