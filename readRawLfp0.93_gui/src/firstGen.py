# -*- coding: utf-8 -*-
'''
Created on 2015. 1. 26.
Last Modified at 2015. 2. 3.

@author: Young Hyun Jo, CIPLab at Yonsei University
@email: heyday097@gmail.com
'''
import json
import numpy as np
import readRawLfp

class LightFieldFromRawImage():
    def __init__(self, rawImage):
        print()
        
    
class Metadata():
    def __init__(self, metadata):
        metadata = json.loads(metadata)  
        
    
class PrivateMetadata():
    def __init__(self, privateMetadata):
        privateMetadata = json.loads(privateMetadata)  
        
    
class FrameMetadata():
    def __init__(self, frameMetadata):
        lfpFrameMetadata = json.loads(frameMetadata)  
        
        # # image
        self.imgWidth = lfpFrameMetadata["image"]["width"]
        self.imgHeight = lfpFrameMetadata["image"]["height"]
        self.imgOrientation = lfpFrameMetadata["image"]["orientation"]
        
        self.blackR = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["black"]["r"]
        self.blackGr = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["black"]["gr"]
        self.blackGb = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["black"]["gb"]
        self.blackB = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["black"]["b"]
        self.whiteR = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["white"]["r"]
        self.whiteGr = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["white"]["gr"]
        self.whiteGb = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["white"]["gb"]
        self.whiteB = lfpFrameMetadata["image"]["rawDetails"]["pixelFormat"]["white"]["b"]
        
        self.bitsPerPixel = lfpFrameMetadata["image"]["rawDetails"]["pixelPacking"]["bitsPerPixel"]
        self.endianness = lfpFrameMetadata["image"]["rawDetails"]["pixelPacking"]["endianness"]
        
        self.ratio = self.bitsPerPixel / 8.0
        
        self.mosaicTile = lfpFrameMetadata["image"]["rawDetails"]["mosaic"]["tile"]
        self.mosaicUpperLeft = lfpFrameMetadata["image"]["rawDetails"]["mosaic"]["upperLeftPixel"]
        
        self.ccm = np.array(lfpFrameMetadata["image"]["color"]["ccmRgbToSrgbArray"])
        self.ccm = self.ccm.reshape((3, 3))
    
        self.gamma = lfpFrameMetadata["image"]["color"]["gamma"]
        
        self.wbGainR = lfpFrameMetadata["image"]["color"]["whiteBalanceGain"]["r"]
        self.wbGainGr = lfpFrameMetadata["image"]["color"]["whiteBalanceGain"]["gr"]
        self.wbGainGb = lfpFrameMetadata["image"]["color"]["whiteBalanceGain"]["gb"]
        self.wbGainB = lfpFrameMetadata["image"]["color"]["whiteBalanceGain"]["b"]
        
        # # devices
        self.clock = lfpFrameMetadata["devices"]["clock"]["zuluTime"]
        
        self.iso = lfpFrameMetadata["devices"]["sensor"]["iso"]
        
        self.analogGainR = lfpFrameMetadata["devices"]["sensor"]["analogGain"]["r"]
        self.analogGainGr = lfpFrameMetadata["devices"]["sensor"]["analogGain"]["gr"]
        self.analogGainGb = lfpFrameMetadata["devices"]["sensor"]["analogGain"]["gb"]
        self.analogGainB = lfpFrameMetadata["devices"]["sensor"]["analogGain"]["b"]
        
        self.pixelPitch = lfpFrameMetadata["devices"]["sensor"]["pixelPitch"]
        
        self.infinityLambda = lfpFrameMetadata["devices"]["lens"]["infinityLambda"]
        self.focalLength = lfpFrameMetadata["devices"]["lens"]["focalLength"]
        self.exitPupilOffsetZ = lfpFrameMetadata["devices"]["lens"]["exitPupilOffset"]["z"]
        
        self.tiling = lfpFrameMetadata["devices"]["mla"]["tiling"]
        self.lensPitch = lfpFrameMetadata["devices"]["mla"]["lensPitch"]
        self.rotation = lfpFrameMetadata["devices"]["mla"]["rotation"]
        self.scaleFactorX = lfpFrameMetadata["devices"]["mla"]["scaleFactor"]["x"]
        self.scaleFactorY = lfpFrameMetadata["devices"]["mla"]["scaleFactor"]["y"]
        self.sensorOffsetX = lfpFrameMetadata["devices"]["mla"]["sensorOffset"]["x"]
        self.sensorOffsetY = lfpFrameMetadata["devices"]["mla"]["sensorOffset"]["y"]
        self.sensorOffsetZ = lfpFrameMetadata["devices"]["mla"]["sensorOffset"]["z"]
    


def splitLfp(fileName):
    f = open(fileName, "rb")
    
    lfpP = 0
    lfmP = 0
    lfcP = [0, 0, 0, 0]
    lfcNo = 0
    fileP = 0
    
    # find start position of each section
    while True:
        s = f.read(16)
        l = len(s) 
        if l == 0: 
            break  # it is clear that this is EOF
        else:
            if ord(s[0]) == 0x89 and s[1] == "L" and s[2] == "F" and s[3] == "P":
                lfpP = fileP
            elif ord(s[0]) == 0x89 and s[1] == "L" and s[2] == "F" and s[3] == "M":
                lfmP = fileP
            elif ord(s[0]) == 0x89 and s[1] == "L" and s[2] == "F" and s[3] == "C":
                lfcP[lfcNo] = fileP
                lfcNo += 1

        fileP += 16
        
    # check the file have each sections
    if lfpP != 0 or lfmP == 0 or lfcP[0] == 0:
        print("Not a valid LFP 1.0 file")
        f.close()
        return -1, -1, -1, -1
        
    # get LFM
    f.seek(lfmP + 16 * 6)
    extMetadata = f.read(lfcP[0] - (lfmP + 16 * 6))

    # trim (delete last 0's) to json parsing
    l = len(extMetadata)
    while True:
        if ord(extMetadata[l - 1]) == 0:
            l -= 1
        else:
            break
    extMetadata = extMetadata[:l]

    # check each ref value (버전이 두가지인듯..)
    lfpMetadata = json.loads(extMetadata)  
    
    frameMetadataRef = {}
    imageRef = {}
    privateMetadataRef = {}
    
    try:
        frameMetadataRef = lfpMetadata["picture"]["frameArray"][0]["frame"]["metadataRef"]
        imageRef = lfpMetadata["picture"]["frameArray"][0]["frame"]["imageRef"]
        try:
            privateMetadataRef = lfpMetadata["picture"]["frameArray"][0]["frame"]["privateMetadataRef"]
        except: 
            extPrivateMetadata = {}   
    except:
        frameMetadataRef = lfpMetadata["frames"][0]["frame"]["metadataRef"]
        imageRef = lfpMetadata["frames"][0]["frame"]["imageRef"]
        try:
            privateMetadataRef = lfpMetadata["frames"][0]["frame"]["privateMetadataRef"]
        except: 
            extPrivateMetadata = {}

    # there is no raw image
    if imageRef == 0:
        f.close()
        return -1, -1, -1, -1
    
    # get data
    for i in range(0, lfcNo):
        f.seek(lfcP[i] + 16 * 1)  # get sha1
        extSha1 = f.read(16 * 3 - 3)
        
        if lfcP[i + 1] == 0:  # move to eof
            f.seek(0, 2)
            lfcP[i + 1] = f.tell()
        
        f.seek(lfcP[i] + 16 * 6)  # get each part of data
        if extSha1 == frameMetadataRef:
            extFrameMetadata = f.read(lfcP[i + 1] - (lfcP[i] + 16 * 6))
        elif extSha1 == privateMetadataRef:
            extPrivateMetadata = f.read(lfcP[i + 1] - (lfcP[i] + 16 * 6))
        elif extSha1 == imageRef:
            extImg = f.read(lfcP[i + 1] - (lfcP[i] + 16 * 6))
    
    f.close()
    
    # trim (delete last 0's)
    l = len(extFrameMetadata)
    while True:
        if ord(extFrameMetadata[l - 1]) == 0:
            l -= 1
        else:
            break
    extFrameMetadata = extFrameMetadata[:l]
                            
    if privateMetadataRef:
        l = len(extPrivateMetadata)
        while True:
            if ord(extPrivateMetadata[l - 1]) == 0:
                l -= 1
            else:
                break
        extPrivateMetadata = extPrivateMetadata[:l]
    
    return extMetadata, extImg, extFrameMetadata, extPrivateMetadata

