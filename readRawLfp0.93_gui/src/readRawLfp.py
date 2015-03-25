# -*- coding: utf-8 -*-
'''
Created on 2015. 2. 28.
Last Modified at 2015. 2. 28.

@author: Young Hyun Jo, CIPLab at Yonsei University
@email: heyday097@gmail.com
'''

import gc
import math

import cv2
import wx

import firstGen
import numpy as np


global startX, startY, deltaX, deltaY, rotMat, centerX, centerY
global mlCountTop, mlCountLeft
global u_left, u_right, v_top, v_bottom, x_left, x_right, y_top, y_bottom
global bgr
    

def L(u, v, x, y):  # 0 <= x,y,u,v <= 1
    global startX, startY, deltaX, deltaY, rotMat, centerX, centerY
    global mlCountTop, mlCountLeft
    global u_left, u_right, v_top, v_bottom, x_left, x_right, y_top, y_bottom
    global bgr
 
    u = (u * u_right)
    v = (v * v_bottom)  # 0 <= u,v <= 6
    x = (x * x_right)
    y = (y * y_bottom)  # 0 <= x,y <= 
     
    px = (u - 3) + startX + ((round(x) + mlCountLeft) * deltaX)
    if (round(y) + mlCountTop) % 2 == 1:
        px += (deltaX / 2)
    py = (v - 3) + startY + ((round(y) + mlCountTop) * deltaY)
      
    px -= startX
    py -= startY
      
    newP = np.dot(rotMat, np.array([[px], [py]]))
      
    px = newP[0][0]
    py = newP[1][0]
       
    px += startX
    py += startY
     
#     return py, px
    return bgr[py][px]


# def L(u, v, x, y, _y):  # 0 <= u,v <= 6  0 <= x,y
#     global startX, startY, deltaX, deltaY, rotMat, centerX, centerY
#     global mlCountTop, mlCountLeft
#     global u_left, u_right, v_top, v_bottom, x_left, x_right, y_top, y_bottom
#     global bgr
# 
# #     u = (u * u_right)
# #     v = (v * v_bottom)  # 0 <= u,v <= 6
# #     x = (x * x_right)
# #     y = (y * y_bottom)  # 0 <= x,y <= 
#     
#     # do not use the different y-axis value
#     if (round(y) + mlCountTop) % 2 == _y % 2:
#         return 0, np.zeros((3)) 
#     
#     px = (u - 3) + startX + ((x + mlCountLeft) * deltaX)
#     if (round(y) + mlCountTop) % 2 == 1:
#         px += (deltaX / 2)
#     py = (v - 3) + startY + ((y + mlCountTop) * deltaY)
#      
#     px -= startX
#     py -= startY
#      
#     newP = np.dot(rotMat, np.array([[px], [py]]))
#      
#     px = newP[0][0]
#     py = newP[1][0]
#       
#     px += startX
#     py += startY
#     
# #     return py, px
#     return 1, bgr[py][px]


def postRefocus(alpha, x=0, y=0):
    global u_left, u_right, v_top, v_bottom, x_left, x_right, y_top, y_bottom
    global bgr
     
    alpha = (float)(alpha)
    xx_right = (float)(x_right)
    yy_bottom = (float)(y_bottom)
    uu_right = (float)(u_right)
    vv_bottom = (float)(v_bottom)
     
    if x == 0 and y == 0:
        ySize = y_bottom + 1
        xSize = x_right + 1
    else:
        ySize = y
        xSize = x
        xx_right = (float)(x - 1)
        yy_bottom = (float)(y - 1)
#     ySize = (y_bottom + 1) * alpha
#     xSize = (x_right + 1) * alpha
#     ySize = int(ySize)
#     xSize = int(xSize)
    imgRefocused = np.zeros((ySize, xSize, 3), dtype=np.uint8)
     
    alphaTerm = (1 - (1 / alpha))
      
    for y in range(0, ySize):
        setStatusText("y:%d" % (y))
         
        yy = y / yy_bottom
        yyyTerm = (yy / alpha)
                     
        for x in range(0, xSize):
            imgVal = np.zeros((3))
            cnt = 0
             
            xx = x / xx_right
            xxxTerm = (xx / alpha)
                     
            for v in range(0, v_bottom + 1):
                vv = v / vv_bottom
                     
                for u in range(0, u_right + 1):
                    uu = u / uu_right
                     
#                     xxx = uu * (1 - (1 / alpha)) + (xx / alpha)
#                     yyy = vv * (1 - (1 / alpha)) + (yy / alpha)
                    xxx = uu * alphaTerm + xxxTerm
                    yyy = vv * alphaTerm + yyyTerm
                     
                    if xxx < 0 or xxx > 1:
                        continue                        
                    if yyy < 0 or yyy > 1:
                        continue                        
                    if uu < 0 or uu > 1:
                        continue                        
                    if vv < 0 or vv > 1:
                        continue
                         
                    imgVal += L(uu, vv, xxx, yyy)
                    cnt += 1
             
            imgRefocused[y][x] = (imgVal / (float)(cnt))
     
    return imgRefocused

# def postRefocus(alpha, x=0, y=0):
#     global mlCountTop, mlCountLeft
#     global u_left, u_right, v_top, v_bottom, x_left, x_right, y_top, y_bottom
#     global bgr
#     
#     alpha = (float)(alpha)
#     
#     if x == 0 and y == 0:
#         ySize = y_bottom + 1
#         xSize = x_right + 1
#     else:
#         ySize = y
#         xSize = x
# 
#     imgRefocused = np.zeros((ySize, xSize, 3), dtype=np.uint8)
#     
#     alphaTerm = (1 - (1 / alpha))
#      
#     for y in range(0, ySize):
#         setStatusText("y:%d" % (y))
#         
#         for x in range(0, xSize):
#             imgVal = np.zeros((3))
#             cnt = 0
#             
#             for v in range(0, v_bottom + 1):
#                 yyy = v * alphaTerm + y
#                 
#                 for u in range(0, u_right + 1):
#                     xxx = u * alphaTerm + x
#                     
#                     if xxx < mlCountLeft or xxx > x_right:
#                         continue                        
#                     if yyy < mlCountTop or yyy > y_bottom:
#                         continue                        
#                         
#                     t, tt = L(u, v, x, y, y)
#                     
#                     cnt += t
#                     imgVal += tt
#             
#             imgRefocused[y][x] = (imgVal / (float)(cnt))
#     
#     return imgRefocused


def subAperture(u, v):  # inputs are 0<=u,v<=1
    global u_left, u_right, v_top, v_bottom, x_left, x_right, y_top, y_bottom
    
    u = u * 6
    v = v * 6
    xx_right = (float)(x_right)
    yy_bottom = (float)(y_bottom)
    uu_right = (float)(u_right)
    vv_bottom = (float)(v_bottom)
    
    ySize = y_bottom + 1
    xSize = x_right + 1
    imgSubAperture = np.zeros((ySize, xSize, 3), dtype=np.uint8)
    
    for y in range(0, ySize):
        for x in range(0, xSize):
            uu = u / uu_right
            vv = v / vv_bottom
            xx = x / xx_right
            yy = y / yy_bottom
            
#             py, px= L(uu, vv, xx, yy)
#             imgSubAperture[y][x]=bgr[py][px]
#             
#             bgr[py][px][0]=0
#             bgr[py][px][1]=0
#             bgr[py][px][2]=0
            
            imgSubAperture[y][x] = L(uu, vv, xx, yy)
            
    return imgSubAperture


def radianToDegree(rad):
    return rad * 180 / math.pi

def degreeToRadian(deg):
    return deg * math.pi / 180.0


def setStatusText(text):
    global gui
    
    gui.statusbar.SetStatusText(text)
    
    
# def doRefocus(alpha):
#     global gui
#     global _save
#     
#     a = alpha
#     wow = postRefocus(a, 328, 328)
#     wow = wow.astype(np.uint8)
#     cv2.imshow("refocus alpha: " + str(a), wow)
#     if _save:
#         cv2.imwrite(gui.filePath[:-4] + "_refocus_" + str(a) + ".tif", wow)
#         
#     return
def doRefocus(l):
    global  x_right, y_bottom, horizontalMLs, verticalMLs
    
    global gui
    global _save
    global imgSubAperture, frameMetadata
    
    # refucusing by sub aperture image
    
    # i do not know how it works clearly
    lensF = frameMetadata.focalLength
    lensExitPupilOffsetZ = frameMetadata.exitPupilOffsetZ
    mlaSensorOffsetZ = frameMetadata.sensorOffsetZ
    
    t = mlaSensorOffsetZ / lensExitPupilOffsetZ
    refocusPlaneOffsetZ = (l * t)  # distance from original focus plane to refocus plane
    
    slopeDegree = (refocusPlaneOffsetZ / (lensExitPupilOffsetZ / 2)) * 90 + 90
    multU = 1 / math.tan(degreeToRadian(slopeDegree)) * int(horizontalMLs / 2)
    multV = 1 / math.tan(degreeToRadian(slopeDegree)) * int(verticalMLs / 2)
#     
#     alpha = deltaZ / lensExitPupilOffsetZ
#     
#     slope = 1 - (1 / alpha)
    
    
    newX = int(x_right + round(abs(multU)))
    newY = int(y_bottom + round(abs(multV)))
    lastImage = np.zeros((y_bottom + round(abs(multV) + 10) * 2, x_right + round(abs(multU) + 10) * 2 , 3))
    
    for yy in range(0, 7):
        for xx in range(0, 7):  
            
            resized = cv2.resize(imgSubAperture[yy][xx], (newX, newY)) 
            
            u = (xx - 3) / 3.0
            v = (yy - 3) / 3.0
            
            t = np.zeros((y_bottom + round(abs(multV) + 10) * 2, x_right + round(abs(multU) + 10) * 2 , 3))
            
            startX = round(abs(multU) + u * multU)
            startY = round(abs(multV) + v * multV)
#             if (xx == 0and yy == 0) or (xx == 6 and yy == 6):
#                 print("u=" + str(xx) + ",v=" + str(yy) + " startX:" + str(startX) + " startY:" + str(startY) + " newY:" + str(newY) + " t's y:" + str(y_bottom + round(abs(multV) + 1) * 2))
            
            if startX < 0 or startY < 0: 
                print("err")
                
            t[startY:startY + newY, startX:startX + newX] = resized
            
            
#             if fx >= 1 and fy >= 1:
#                 diffY = t.shape[0] - y_bottom
#                 diffX = t.shape[1] - x_right
#                 
#                 startY = round(diffY / 2)
#                 startX = round(diffX / 2)
#                 
#                 t = t[startY:startY + y_bottom, startX:startX + x_right]  # Crop from x, y, w, h -> 100, 200, 300, 400
#             
#             elif fx >= 1 and fy < 1:
#                 diffY = y_bottom - t.shape[0]
#                 diffX = t.shape[1] - x_right
#                 
#                 startY = round(diffY / 2)
#                 startX = round(diffX / 2)
#                 
#                 tt = np.zeros((y_bottom, x_right, 3))
#                 tt[startY:startY + t.shape[0], :] = t[:, startX:startX + x_right]
#                 t = tt
#                 
#             elif fx < 1 and fy >= 1:
#                 diffY = t.shape[0] - y_bottom
#                 diffX = x_right - t.shape[1]
#                 
#                 startY = round(diffY / 2)
#                 startX = round(diffX / 2)
#                 
#                 tt = np.zeros((y_bottom, x_right, 3))
#                 tt[:, startX:startX + t.shape[1]] = t[startY:startY + y_bottom, :]
#                 t = tt
#                 
#             elif fx < 1 and fy < 1:
#                 diffY = y_bottom - t.shape[0]
#                 diffX = x_right - t.shape[1]
#                 
#                 startY = round(diffY / 2)
#                 startX = round(diffX / 2)
#                 
#                 tt = np.zeros((y_bottom, x_right, 3))
#                 tt[startY:startY + t.shape[0], startX:startX + t.shape[1]] = t
#                 t = tt
                
            lastImage += t
            
    lastImage = np.rint(lastImage / 49.0).astype(np.uint8)
#     cv2.imshow("refocused", lastImage)

    # show on GUI
    rgbb = lastImage
#     rgbb[:, :, [2, 0]] = rgbb[:, :, [0, 2]]  # bgr to rgb
    gui.ShowRefocusImage(rgbb)
    
    if _save:
        cv2.imwrite(gui.filePath[:-4] + "_refocus_" + str(l) + ".tif", lastImage)
    


def processLfp(fileName):
    global startX, startY, deltaX, deltaY, rotMat, centerX, centerY
    global mlCountTop, mlCountLeft
    global u_left, u_right, v_top, v_bottom, x_left, x_right, y_top, y_bottom, horizontalMLs, verticalMLs
    global bgr
    
    global gui
    global  _save, _whiteBalanceGain, _demosaicing, _colorCorrection, _gammaCorrection
    
    global imgSubAperture, frameMetadata
    
    gc.collect()  # garbage collect
    
    targetRes = (1080, 1080)  # (y,x)
    numSubAperture = (7, 7)  # use 7x7 subaperture (y,x)
    
    # split lfp
    setStatusText("Open File : " + fileName)
    lfpMetadata, lfpRawImg, lfpFrameMetadata, lfpPrivateMetadata = firstGen.splitLfp(fileName)
    if lfpMetadata == -1:  # not a valid lfp file
        setStatusText("Invalid LFP file (Cannot open it)")
        return
    
    # save to file
    if _save:
        f = open(fileName[:-4] + "_metadata.json", "wb")
        f.write(lfpMetadata)
        f.close()
        
        f = open(fileName[:-4] + "_frameMedatada.json", "wb")
        f.write(lfpFrameMetadata)
        f.close()
        
        f = open(fileName[:-4] + "_raw.raw", "wb")
        f.write(lfpRawImg)
        f.close()
        
        if lfpPrivateMetadata:
            f = open(fileName[:-4] + "_privateMetadata.json", "wb")
            f.write(lfpPrivateMetadata)
            f.close()
    
    # get metadata values
    metadata = firstGen.Metadata(lfpMetadata)    
    frameMetadata = firstGen.FrameMetadata(lfpFrameMetadata)
    privateMetadata = {}
    if lfpPrivateMetadata:
        privateMetadata = firstGen.PrivateMetadata(lfpPrivateMetadata)
    
    # check bits per pixel, endianness
    if frameMetadata.bitsPerPixel != 12:
        setStatusText("only 12 bits per pixel is possible, but : " + frameMetadata.bitsPerPixel)
        return
    if frameMetadata.endianness != "big":
        setStatusText("only big-endianness is possible, but : " + frameMetadata.endianness)
        return
    # check mosaic tile
    if frameMetadata.mosaicTile != "r,gr:gb,b":
        setStatusText("unavailable mosaic tile : " + frameMetadata.mosaicTile)
        return
    if frameMetadata.mosaicUpperLeft != "b":
        setStatusText("upper left pixel of mosaic tile is not B, but : " + frameMetadata.mosaicUpperLeft)
        return
    # check mla
    if frameMetadata.tiling != "hexUniformRowMajor":
        setStatusText("unavailable tilling : " + frameMetadata.tiling)
        return


    # show on GUI
    gui.panelMetadataText.SetValue(lfpMetadata)
    gui.panelFrameMetadataText.SetValue(lfpFrameMetadata)
    gui.panelPrivateMetadataText.SetValue(lfpPrivateMetadata)
    
    
    # get raw bayer pattern value
    bayerR = np.zeros((frameMetadata.imgHeight, frameMetadata.imgWidth), dtype=np.uint16)
    bayerGr = np.zeros((frameMetadata.imgHeight, frameMetadata.imgWidth), dtype=np.uint16)
    bayerGb = np.zeros((frameMetadata.imgHeight, frameMetadata.imgWidth), dtype=np.uint16)
    bayerB = np.zeros((frameMetadata.imgHeight, frameMetadata.imgWidth), dtype=np.uint16)
    
    setStatusText("get raw bayer value")
    for y in range(0, frameMetadata.imgHeight):
        for x in range(0, frameMetadata.imgWidth, 2):
            ptr = int(y * frameMetadata.imgWidth * frameMetadata.ratio + x * frameMetadata.ratio)
            firstByte = ord(lfpRawImg[ptr])
            secondByte = ord(lfpRawImg[ptr + 1])
            thirdByte = ord(lfpRawImg[ptr + 2])
            
            if (y % 2 == 0):
                read2Bytes = firstByte << 8 | secondByte;
                read12Bits = read2Bytes >> 4
                bayerB[y][x] = read12Bits
                
                read2Bytes = secondByte << 8 | thirdByte;
                read12Bits = read2Bytes & 0b0000111111111111
                bayerGb[y][x + 1] = read12Bits
            else:  # if (y % 2 == 1):
                read2Bytes = firstByte << 8 | secondByte;
                read12Bits = read2Bytes >> 4
                bayerGr[y][x] = read12Bits
                
                read2Bytes = secondByte << 8 | thirdByte;
                read12Bits = read2Bytes & 0b0000111111111111
                bayerR[y][x + 1] = read12Bits
    
    # stretch from black(168)- white(4095) to black(0) - white(1)
    bayerR = np.where(bayerR < frameMetadata.blackR, 0, bayerR - frameMetadata.blackR)
    bayerGr = np.where(bayerGr < frameMetadata.blackGr, 0, bayerGr - frameMetadata.blackGr)
    bayerGb = np.where(bayerGb < frameMetadata.blackGb, 0, bayerGb - frameMetadata.blackGb)
    bayerB = np.where(bayerB < frameMetadata.blackB, 0, bayerB - frameMetadata.blackB)
    
    bayerR = bayerR / float(frameMetadata.whiteR - frameMetadata.blackR) 
    bayerGr = bayerGr / float(frameMetadata.whiteGr - frameMetadata.blackGr)
    bayerGb = bayerGb / float(frameMetadata.whiteGb - frameMetadata.blackGb)
    bayerB = bayerB / float(frameMetadata.whiteB - frameMetadata.blackB)
        
    # white balance gain
    if _whiteBalanceGain:
        setStatusText("white balance gain")
        bayerR *= frameMetadata.wbGainR
        bayerGr *= frameMetadata.wbGainGr
        bayerGb *= frameMetadata.wbGainGb
        bayerB *= frameMetadata.wbGainB

    bayer = bayerR + bayerGr + bayerGb + bayerB

    # correction : over 1 -> set to 1
    bayer = np.where(bayer > 1, 1, bayer)
    bayer = np.rint(bayer * 255).astype(np.uint8)
    
    # demosaicing
    if _demosaicing:
        setStatusText("demosaicing (cv2.cvtColor)")
        bgr = cv2.cvtColor(bayer, cv2.COLOR_BAYER_RG2BGR)
    else:
        bgr = np.zeros((frameMetadata.imgHeight, frameMetadata.imgWidth, 3))
        bgr[:, :, 0] = bayer
        bgr[:, :, 1] = bayer
        bgr[:, :, 2] = bayer
    
    # color correction
    if _colorCorrection:
        setStatusText("color correction")
        bgr = bgr / 255.0
        rows, cols = bgr.shape[0:2]
        bgr = np.reshape(bgr, (rows * cols, 3))  # 3280x3280x3 -> 3280*3280x3
        bgr = np.transpose(bgr)  # 3280*3280x3 -> 3x3280*3280
        bgr[[2, 0], :] = bgr[[0, 2], :]  # bgr to rgb
        bgr = np.dot(frameMetadata.ccm, bgr)
        bgr[[2, 0], :] = bgr[[0, 2], :]  # rgb to bgr
        bgr = np.transpose(bgr)  # 3x3280*3280 -> 3280*3280x3
        bgr = np.reshape(bgr, (rows, cols, 3))  # 3280*3280x3 -> 3280x3280x3
        
        # correction : over 1 -> set to 1
        bgr = np.where(bgr > 1, 1, bgr)
        bgr = np.where(bgr < 0, 0, bgr)
    
    # gamma correction
    if _gammaCorrection:
        setStatusText("gamma correction")
        bgr = np.power(bgr, frameMetadata.gamma)
        bgr = np.rint(bgr * 255).astype(np.uint8)
    
    # save file
    if _save:
        setStatusText("save raw sensor image")
        cv2.imwrite(fileName[:-4] + "_rawImage.tif", bgr)
        
        
    # show on GUI
    rgb = bgr
    rgb[:, :, [2, 0]] = rgb[:, :, [0, 2]]  # bgr to rgb
    gui.ShowRawSensorImage(rgb)
    
    
    # raw sensor image rotation
    setStatusText("rotation and resize (fit microlens diameter to 10px)")
    rotRadian = frameMetadata.rotation
    rotDegree = radianToDegree(rotRadian)
    
    centerX = frameMetadata.imgWidth / 2.0
    centerY = frameMetadata.imgHeight / 2.0
    
    startX = centerX + (float)(frameMetadata.sensorOffsetX / frameMetadata.pixelPitch)
    startY = centerY + (float)(frameMetadata.sensorOffsetY / frameMetadata.pixelPitch)
    
    rotMat = np.array([[math.cos(rotRadian) , -math.sin(rotRadian)], \
                       [math.sin(rotRadian), math.cos(rotRadian)]])
    
    newY = startX * math.sin(rotRadian) + startY * math.cos(rotRadian)
    resizePadding = int(round(abs(startY - newY)))
    
    imgCenter = tuple(np.array(bgr.shape[0:2]) / 2)
    rotMatrix = cv2.getRotationMatrix2D(imgCenter, rotDegree, 1.0)
    imgNewSize = tuple(np.array(bgr.shape[0:2]) + resizePadding)
    bgr = cv2.warpAffine(bgr, rotMatrix, imgNewSize, flags=cv2.INTER_LINEAR)
    
    deltaX = (float)(frameMetadata.lensPitch / frameMetadata.pixelPitch)
    # scale factor ??
#     deltaX *= frameMetadata.scaleFactorX
    
    # resize the image (fit ML to 10px)
    widthOfMicrolens = 10.0
    resizeRatio = widthOfMicrolens / deltaX
    
    bgr = cv2.resize(bgr, (0, 0), fx=resizeRatio, fy=resizeRatio)  # resize image
    resizePadding = round(resizePadding * resizeRatio)  # re arrange the values
    centerX *= resizeRatio 
    centerY *= resizeRatio
    startX *= resizeRatio
    startY *= resizeRatio
    
    deltaX = widthOfMicrolens  # deltaX is changed to 10
    
    deltaY = deltaX / 2.0 * math.sqrt(3)
    # scale factor ??
#     deltaY *= frameMetadata.scaleFactorY
    
    verticalMLs = int(bgr.shape[1] / deltaY)  # the number of MLs ; v=378.74, h=328
    horizontalMLs = int(bgr.shape[0] / deltaX)


###############
    mlCountLeft = -int(horizontalMLs / 2)  # -162
    mlCountRight = int(horizontalMLs / 2) - 1  # 161
    mlCountTop = -int(verticalMLs / 2) + 1  # -186
    mlCountBottom = int(verticalMLs / 2)  # 187
    # x_right=324, y_bottom=374
    x_left = 0
    x_right = mlCountRight - mlCountLeft + 1
    y_top = 0
    y_bottom = mlCountBottom - mlCountTop + 1
###############    
    

    # make dot at center pixel of MLs
    setStatusText("make sub aperture image (" + str(numSubAperture[1]) + "x" + str(numSubAperture[0]) + ")")
#     centerPixel = np.zeros((y_bottom, x_right, 2))
    
    targetResSubAperture = np.zeros((numSubAperture[0] , numSubAperture[1] , targetRes[0], targetRes[0], 3), dtype=np.uint8)
    
    imgSubAperture = np.zeros((numSubAperture[0] , numSubAperture[1] , verticalMLs + 1, horizontalMLs + 1, 3), dtype=np.uint8)
    
    resizedVertex = np.zeros((numSubAperture[0] , numSubAperture[1] , verticalMLs + 1, horizontalMLs + 1, 2))
    resizedVertex -= 1
    
    for y in range(0, verticalMLs + 1):
        for x in range(0, horizontalMLs + 1):
            
            yy = y - round(verticalMLs / 2)
            xx = x - round(horizontalMLs / 2)
            
            cY = round(startY + yy * deltaY)
            cX = round(startX + xx * deltaX)
            
            if yy % 2 == 1:
                cX += (deltaX / 2.0)
                
            # boundary check
            if cY < 0 or cY >= bgr.shape[0]:
                continue
            if cX < 0 or cX >= bgr.shape[1]:
                continue
                
            # calculate resized vertex
            halfV = (numSubAperture[0] - 1) / 2.0
            halfU = (numSubAperture[1] - 1) / 2.0
            for v in range(0, numSubAperture[0]):
                for u in range(0, numSubAperture[1]):
                    resizedVertex[v][u][y][x] = ((cY + (v - halfV)) / float(bgr.shape[0]) * targetRes[0], (cX + (u - halfU)) / float(bgr.shape[1]) * targetRes[1])

#                     resizedVertex[v][u][y][x] = (y, x)

                    pY = cY + (v - halfV)
                    pX = cX + (u - halfU)
                    if pY >= 0 and pY < bgr.shape[0] and pX >= 0 and pX < bgr.shape[1]:
                        imgSubAperture[v][u][y][x] = bgr[pY][pX]
            
            bgr[cY][cX] = (0, 0, 0)
    
#     for v in range(0, numSubAperture[0]):
#         for u in range(0, numSubAperture[1]):
#             cv2.imwrite(fileName[:-4] + "_subAperture_v" + str(v) + "u" + str(u) + ".tif", imgSubAperture[v][u])
#             if u == 0 and v == 0:
#                 print(resizedVertex[v][u])
                
    cv2.imwrite(fileName[:-4] + "_rawRotatedAndResized.tif", bgr)
    
    
    
#     # make target resolution subaperture image (barycentric interpolation)
#     for vvv in range(0, numSubAperture[0]):
#         for uuu in range(0, numSubAperture[1]):
#             print(uuu + vvv * 7)
#             
#             for yyy in range(0, targetRes[0]):
#                 for xxx in range(0, targetRes[1]):
#                     
#                     p1 = np.array([-1, -1])
#                     p2 = np.array([-1, -1])
#                     p3 = np.array([-1, -1])
#                     for yyyy in range(0, verticalMLs + 1):
#                         for xxxx in range(0, horizontalMLs + 1):
#                             if p1[0] != -1 and p1[1] != -1:
#                                 break
# 
#                             if p1[1] == -1 and xxx >= resizedVertex[vvv][uuu][yyyy][xxxx][1]:
#                                 p1[1] = resizedVertex[vvv][uuu][yyyy][xxxx][1]
#                                 p2[1] = resizedVertex[vvv][uuu][yyyy - 1][xxxx][1]
#                                 p3[1] = resizedVertex[vvv][uuu][yyyy][xxxx - 1][1]
#                             if p1[0] == -1 and yyy >= resizedVertex[vvv][uuu][yyyy][xxxx][0]:
#                                 p1[0] = resizedVertex[vvv][uuu][yyyy][xxxx][0]
#                                 p2[0] = resizedVertex[vvv][uuu][yyyy - 1][xxxx][0]
#                                 p3[0] = resizedVertex[vvv][uuu][yyyy][xxxx - 1][0]
#                                 
#                         if p1[0] != -1 and p1[1] != -1:
#                             break
#                         
#                                  
#                     if (p1[0] != -1 and p1[1] != -1) and not((p1 == p2).all()) and not((p1 == p3).all()) and not((p2 == p3).all()):
#                         A = np.matrix([[p1[1], p2[1], p3[1]], \
#                              [p1[0], p2[0], p3[0]], \
#                              [1, 1, 1]])
#                         print(A)
#                         b = np.array([[xxx], [yyy], [1]])
#                         x = np.dot(np.linalg.inv(A), b)
#                         
#                         I = x[0] * imgSubAperture[vvv][uuu][p1[0]][p1[1]] \
#                         + x[1] * imgSubAperture[vvv][uuu][p2[0]][p2[1]] \
#                         + x[2] * imgSubAperture[vvv][uuu][p3[0]][p3[1]]
#                         
#                         targetResSubAperture[vvv][uuu][yyy][xxx] = I
#             
#             
#             cv2.imwrite(fileName[:-4] + "_targetSubAperture_u" + str(uuu) + "v" + str(vvv) + ".tif", targetResSubAperture[vvv][uuu])
#             
        
        
        
#     doRefocus(_alpha)
    
    return


    
class ShowGui(wx.Frame):
    def __init__(self, parent, title, size):
        global _lambda, _save, _whiteBalanceGain, _demosaicing, _colorCorrection, _gammaCorrection
    
        self.title = title
        self.filePath = ""
        self.size = size
        super(ShowGui, self).__init__(parent, title=title, size=size)
        
        self.Bind(wx.EVT_SIZE, self.OnResize)
        
        self.panelMenuSize = [150, 300]  # [x,y]
        # panel menu
        self.panelMenu = wx.Panel(self, -1, pos=(0, 0), size=self.panelMenuSize)
        panelMenuMetadata = wx.RadioButton(self.panelMenu, label="File metadata", pos=(10, 10))
        panelMenuFrameMetadata = wx.RadioButton(self.panelMenu, label="Frame Metadata", pos=(panelMenuMetadata.GetPosition() + (0, 20)))
        panelMenuPrivateMetadata = wx.RadioButton(self.panelMenu, label="Private Metadata", pos=(panelMenuFrameMetadata.GetPosition() + (0, 20)))
        panelMenuRawSensorImage = wx.RadioButton(self.panelMenu, label="Raw Sensor Image", pos=(panelMenuPrivateMetadata.GetPosition() + (0, 20)))
        panelMenuRefocusImage = wx.RadioButton(self.panelMenu, label="Refocusing Image", pos=(panelMenuRawSensorImage.GetPosition() + (0, 20)))

        self.Bind(wx.EVT_RADIOBUTTON, self.SwitchPanelToMetadata, panelMenuMetadata)
        self.Bind(wx.EVT_RADIOBUTTON, self.SwitchPanelToFrameMetadata, panelMenuFrameMetadata)
        self.Bind(wx.EVT_RADIOBUTTON, self.SwitchPanelToPrivateMetadata, panelMenuPrivateMetadata)
        self.Bind(wx.EVT_RADIOBUTTON, self.SwitchPanelToRawSensorImage, panelMenuRawSensorImage)
        self.Bind(wx.EVT_RADIOBUTTON, self.SwitchPanelToRefocusImage, panelMenuRefocusImage)
        
        # panel metadata
        self.panelMetadata = wx.Panel(self, -1, pos=(self.panelMenu.GetPosition() + (self.panelMenu.GetSize().x + 0, 0)))
        self.panelMetadataText = wx.TextCtrl(self.panelMetadata, style=wx.TE_MULTILINE | wx.TE_AUTO_SCROLL)
        
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panelMetadataText, 1, wx.EXPAND)
        self.panelMetadata.SetSizer(sizer)
                
        # panel frame metadata
        self.panelFrameMetadata = wx.Panel(self, -1, pos=(self.panelMenu.GetPosition() + (self.panelMenu.GetSize().x + 0, 0)))
        self.panelFrameMetadataText = wx.TextCtrl(self.panelFrameMetadata, style=wx.TE_MULTILINE | wx.TE_AUTO_SCROLL)
        
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panelFrameMetadataText, 1, wx.EXPAND)
        self.panelFrameMetadata.SetSizer(sizer)
        
        # panel private metadata
        self.panelPrivateMetadata = wx.Panel(self, -1, pos=(self.panelMenu.GetPosition() + (self.panelMenu.GetSize().x + 0, 0)))
        self.panelPrivateMetadataText = wx.TextCtrl(self.panelPrivateMetadata, style=wx.TE_MULTILINE | wx.TE_AUTO_SCROLL)
        
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panelPrivateMetadataText, 1, wx.EXPAND)
        self.panelPrivateMetadata.SetSizer(sizer)
        
        # panel raw sensor image
        self.panelRawSensorImage = wx.Panel(self, -1, pos=(self.panelMenu.GetPosition() + (self.panelMenu.GetSize().x + 0, 0)))
        self.panelRawSensorImageImage = wx.EmptyImage(1, 1)
        self.panelRawSensorImageBitmap = wx.StaticBitmap(self.panelRawSensorImage)
        
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panelRawSensorImageBitmap, 1, wx.EXPAND)
        self.panelRawSensorImage.SetSizer(sizer)
        
        # panel refocus image
        self.panelRefocusImage = wx.Panel(self, -1, pos=(self.panelMenu.GetPosition() + (self.panelMenu.GetSize().x + 0, 0)))
        
        self.panelRefocusImageController = wx.Panel(self.panelRefocusImage, -1, pos=(0, 0), size=(400, 30))
        self.panelRefocusImageText = wx.StaticText(self.panelRefocusImageController, label="lambda: 0.00", pos=(0, 10), size=(200, 20))
        sld = wx.Slider(self.panelRefocusImageController, value=0, minValue=-2000, maxValue=2000, pos=(200 , 10), size=(200, 20), style=wx.SL_HORIZONTAL)
        _lambda = 0
        sld.Bind(wx.EVT_SCROLL, self.OnSliderScroll)
        
        self.panelRefocusImageImage = wx.EmptyImage(1, 1)
        self.panelRefocusImageBitmap = wx.StaticBitmap(self.panelRefocusImage, -1, pos=(0, 30))
        
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.panelRefocusImageBitmap)  # , 1, wx.EXPAND)
        self.panelRefocusImage.SetSizer(sizer)
        
        
        # menu
        menubar = wx.MenuBar()
        
        # file menu
        fileMenu = wx.Menu()
        fileMenu.AppendSeparator()

        fileImport = wx.Menu()
        fileImportLfp = wx.MenuItem(fileImport, wx.ID_ANY, "From LFP file\tCtrl+O")
        fileImport.AppendItem(fileImportLfp)
        fileMenu.AppendMenu(wx.ID_ANY, "&Import", fileImport)
         
        fileExport = wx.Menu()
        fileExportImage = wx.MenuItem(fileExport, wx.ID_ANY, "Image\tCtrl+S")
        fileExport.AppendItem(fileExportImage)
        fileMenu.AppendMenu(wx.ID_ANY, "&Export", fileExport)
        fileMenu.AppendSeparator()
        
        fileQuit = wx.MenuItem(fileMenu, wx.ID_ANY, "&Quit\tCtrl+Q")
        fileMenu.AppendItem(fileQuit)
        
        self.Bind(wx.EVT_MENU, self.OpenLfp, fileImportLfp)
        self.Bind(wx.EVT_MENU, self.SaveImage, fileExportImage)
        self.Bind(wx.EVT_MENU, self.OnQuit, fileQuit)
        
#         # option menu
#         optionMenu = wx.Menu()
#         
#         self.save = optionMenu.Append(wx.ID_ANY, "Save splitted data", "Save splitted data", kind=wx.ITEM_CHECK)
#         optionMenu.AppendSeparator()
#         self.wbg = optionMenu.Append(wx.ID_ANY, "White balance gain", "White balance gain", kind=wx.ITEM_CHECK)
#         self.demo = optionMenu.Append(wx.ID_ANY, "Demosaicing", "Demosaicing", kind=wx.ITEM_CHECK)
#         self.cc = optionMenu.Append(wx.ID_ANY, "Color correction", "Color correction", kind=wx.ITEM_CHECK)
#         self.gc = optionMenu.Append(wx.ID_ANY, "Gamma correction", "Gamma correction", kind=wx.ITEM_CHECK)
#             
#         optionMenu.Check(self.save.GetId(), False)
#         optionMenu.Check(self.wbg.GetId(), True)
#         optionMenu.Check(self.demo.GetId(), True)
#         optionMenu.Check(self.cc.GetId(), True)
#         optionMenu.Check(self.gc.GetId(), True)
        _save = False
        _whiteBalanceGain = True
        _demosaicing = True
        _colorCorrection = True
        _gammaCorrection = True
# 
#         self.Bind(wx.EVT_MENU, self.ToggleSave, self.save)
#         self.Bind(wx.EVT_MENU, self.ToggleSwitch, self.wbg)
#         self.Bind(wx.EVT_MENU, self.ToggleSwitch, self.demo)
#         self.Bind(wx.EVT_MENU, self.ToggleSwitch, self.cc)
#         self.Bind(wx.EVT_MENU, self.ToggleSwitch, self.gc)

        menubar.Append(fileMenu, "&File")
#         menubar.Append(optionMenu, "&Option")
        self.SetMenuBar(menubar)

        self.statusbar = self.CreateStatusBar()
        self.statusbar.SetStatusText("")

        self.SetSize(self.size)
        self.SetTitle(title)
        self.Centre()
        self.Show(True)
    
    def SwitchPanelToMetadata(self, e):
        self.panelMetadata.Show()
        self.panelFrameMetadata.Hide()
        self.panelPrivateMetadata.Hide()
        self.panelRawSensorImage.Hide()
        self.panelRefocusImage.Hide()
    def SwitchPanelToFrameMetadata(self, e):
        self.panelMetadata.Hide()
        self.panelFrameMetadata.Show()
        self.panelPrivateMetadata.Hide()
        self.panelRawSensorImage.Hide()
        self.panelRefocusImage.Hide()
    def SwitchPanelToPrivateMetadata(self, e):
        self.panelMetadata.Hide()
        self.panelFrameMetadata.Hide()
        self.panelPrivateMetadata.Show()
        self.panelRawSensorImage.Hide()
        self.panelRefocusImage.Hide()
    def SwitchPanelToRawSensorImage(self, e):
        self.panelMetadata.Hide()
        self.panelFrameMetadata.Hide()
        self.panelPrivateMetadata.Hide()
        self.panelRawSensorImage.Show()
        self.panelRefocusImage.Hide()
    def SwitchPanelToRefocusImage(self, e):
        self.panelMetadata.Hide()
        self.panelFrameMetadata.Hide()
        self.panelPrivateMetadata.Hide()
        self.panelRawSensorImage.Hide()
        self.panelRefocusImage.Show()
        
    def OnResize(self, e):
        self.panelMetadata.SetSize(self.GetClientSize() - (self.panelMenuSize[0], 0))
        self.panelFrameMetadata.SetSize(self.GetClientSize() - (self.panelMenuSize[0], 0))
        self.panelPrivateMetadata.SetSize(self.GetClientSize() - (self.panelMenuSize[0], 0))
        self.panelRawSensorImage.SetSize(self.GetClientSize() - (self.panelMenuSize[0], 0))
        self.panelRefocusImage.SetSize(self.GetClientSize() - (self.panelMenuSize[0], 0))
        
        self.ResizeRawSensorImage()
        self.ResizeRefocusImage()
        
    def ShowRawSensorImage(self, rgb):
        self.panelRawSensorImageImage = wx.EmptyImage(rgb.shape[1], rgb.shape[0])
        self.panelRawSensorImageImage .SetData(rgb.tostring())
        self.ResizeRawSensorImage()
    
    def ResizeRawSensorImage(self):    
        newWidth = 0
        newHeight = 0
        containerSize = self.panelRawSensorImage.GetSize()
        if containerSize.x >= containerSize.y:
            newHeight = containerSize.y
            newWidth = newHeight
        else:  
            newWidth = containerSize.x
            newHeight = newWidth
        if newWidth <= 0 or newHeight <= 0:
            return
        image = self.panelRawSensorImageImage.Scale(newWidth, newHeight)
        self.panelRawSensorImageBitmap.SetBitmap(wx.BitmapFromImage(image))
        self.panelRawSensorImage.Refresh()
        
    def ShowRefocusImage(self, rgb):
        self.panelRefocusImageImage = wx.EmptyImage(rgb.shape[1], rgb.shape[0])
        self.panelRefocusImageImage .SetData(rgb.tostring())
        self.ResizeRefocusImage()
    
    def ResizeRefocusImage(self):    
        newWidth = 0
        newHeight = 0
        containerSize = self.panelRefocusImage.GetSize() - (0, 30)
        if containerSize.x >= containerSize.y:
            newHeight = containerSize.y
            newWidth = newHeight
        else:  
            newWidth = containerSize.x
            newHeight = newWidth
        if newWidth <= 0 or newHeight <= 0:
            return
        image = self.panelRefocusImageImage.Scale(newWidth, newHeight)
        self.panelRefocusImageBitmap.SetPosition((0, 30))
        self.panelRefocusImageBitmap.SetBitmap(wx.BitmapFromImage(image))
        self.panelRefocusImage.Refresh()
        
            
    def OpenLfp(self, e):
        openFileDialog = wx.FileDialog(self, "Open LFP", "", "", "LFP files (*.lfp)|*.lfp", wx.FD_OPEN | wx.FD_FILE_MUST_EXIST)
        if openFileDialog.ShowModal() == wx.ID_CANCEL:
            return 
        
        self.filePath = openFileDialog.GetPath()
        
        self.SetTitle(self.title + " - " + self.filePath)
        processLfp(self.filePath)
        
    def SaveImage(self, e):
        self.Close()
        
    def OnQuit(self, e):
        self.Close()
        
        
    def OnSliderScroll(self, e):
        global _lambda
        
        obj = e.GetEventObject()
        _lambda = obj.GetValue() / 100.0
        
        self.panelRefocusImageText.SetLabel("lambda: " + str(_lambda)) 
        
        if self.filePath:
            doRefocus(_lambda)
        
    def ToggleSave(self, e):
        global _save
        
        if self.save.IsChecked():
            _save = True
        else:
            _save = False
        
    def ToggleSwitch(self, e):
        global  _whiteBalanceGain, _demosaicing, _colorCorrection, _gammaCorrection
        
        if self.wbg.IsChecked():
            _whiteBalanceGain = True
        else:
            _whiteBalanceGain = False
        
        if self.demo.IsChecked():
            _demosaicing = True
        else:
            _demosaicing = False
        
        if self.cc.IsChecked():
            _colorCorrection = True
        else:
            _colorCorrection = False
        
        if self.gc.IsChecked():
            _gammaCorrection = True
        else:
            _gammaCorrection = False
        
#         if self.filePath:
#             ProcessLfp(self.filePath)
        
def main():
    global gui
    
    app = wx.App()
    gui = ShowGui(None, title="lfp 0.93", size=(1200, 1200))
    app.MainLoop()
    

if __name__ == "__main__":
    main()

