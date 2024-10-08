"""
GPS Point (Lat & Lon) Container
Author: Matthew Lauriault
Created: 10/8/24
"""



class GPSList:
        
        def __init__(self):
                self.latitudes = []
                self.longitudes = []
                self.update()
        
        def append(self, lat: float, lon: float):
                # if this is the first update
                if len(self) == 0:
                        self.new = True
                # if new gps point is different from the last one
                else:
                        self.new = lat != self.currentLat() or lon != self.currentLon()
                # if new data -> add and indicate it
                if self.new:
                        self.latitudes.append(lat)
                        self.longitudes.append(lon)
                        self.new = True

        def update(self):
                self.new = False

        def currentLat(self) -> float:
                return self.latitudes[-1]
        
        def currentLon(self) -> float:
                return self.longitudes[-1]

        def __len__(self) -> int:
                if len(self.latitudes) != len(self.longitudes):
                        raise Exception("latitude and longitude lists have unequal lengths.")
                return len(self.latitudes)