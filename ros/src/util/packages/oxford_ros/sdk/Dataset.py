
import os
import re
from datetime import datetime as dt


class Dataset:
    
#     poseModeGps = 1
#     poseModeIns = 2
#     poseModeVO  = 3
    
    def __init__ (self, datadir):
        if (not os.path.isdir(datadir)):
            raise IOError 
        self.path = datadir

    def getTimestamp (self, name, raw=False):
        abspath = self.path + '/' + name + '.timestamps'
        fd = open(abspath)
        tslist = []
        for l in fd:
            tokens = l.split()
            if raw==True:
                tslist.append(tokens[0])
            else:
                datetime = dt.utcfromtimestamp(int(tokens[0])/1000000)
                tslist.append(datetime)
        return tslist
    
    # Returns file names
    def getStereo (self):
        timestamps = self.getTimestamp('stereo', raw=True)
        fileList = []
        ctrname = self.path + '/stereo/centre'
        lftname = self.path + '/stereo/left'
        rhtname = self.path + '/stereo/right'
        for ts in timestamps :
            rcatch = {
                    'center' : ctrname + '/' + ts + '.png',
                    'left'   : lftname + '/' + ts + '.png',
                    'right'  : rhtname + '/' + ts + '.png'
                }
            fileList.append(rcatch)
        return fileList
    
    def getMainLidar (self):
        pass
    
    def getGps (self):
        pass
    
    def getIns (self):
        pass