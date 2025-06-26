# -*- coding: 'unicode' -*-
# Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.

import LJSwrap

import ctypes
import sys
import time

import numpy as np
import PIL
import matplotlib.pyplot as plt

##############################################################################
# sample_ImageAcquisition.py: LJ-S Image acquisition sample.
#
# -First half part: Describes how to acquire images via LJSwrap I/F.
# -Second half part: Describes how to display images using additional modules.
#
##############################################################################

TRG_READY = 0x0020           # Bitmask for the trigger ready status.

image_available = False      # Flag to confirm the image acquisition completed.
acquisition_timeout = False  # Flag to confirm the capturing timeout occurred.
processing_timeout = False   # Flag to confirm the processing timeout occurred.
isCommCloseComplete = False  # Flag to confirm ready to close high speed comm.
ysize_acquired = 0           # Number of Y lines of acquired image.
z_val = []                   # The buffer for height image.
lumi_val = []                # The buffer for luminance image.


def perform_scan(return_type="mm"):

    global image_available
    global acquisition_timeout
    global processing_timeout
    global isCommCloseComplete
    global ysize_acquired
    global z_val
    global lumi_val

    ##################################################################
    # CHANGE THIS BLOCK TO MATCH YOUR SENSOR SETTINGS (FROM HERE)
    ##################################################################

    deviceId = 0                  # Set "0" if you use only 1 head.
    interpolate_yline = 1         # Number of interpolated Y lines (up to 8). 4=3200x3200, 8 = 3200x6400
    timeout_sec = 30               # Timeout value for the acquiring image
    use_external_trigger = False  # 'True' to control trigger externally.
    use_image_filter = False      # 'True' to use the image filter (Windows).

    ethernetConfig = LJSwrap.LJS8IF_ETHERNET_CONFIG()
    ethernetConfig.abyIpAddress[0] = 192    # IP address
    ethernetConfig.abyIpAddress[1] = 168
    ethernetConfig.abyIpAddress[2] = 0
    ethernetConfig.abyIpAddress[3] = 1
    ethernetConfig.wPortNo = 24691          # Port No.
    HighSpeedPortNo = 24692                 # Port No. for high-speed

    ##################################################################
    # CHANGE THIS BLOCK TO MATCH YOUR SENSOR SETTINGS (TO HERE)
    ##################################################################

    # Check input parameters
    if (interpolate_yline < 1) or (interpolate_yline > 8):
        print("Invalid value of 'interpolate_yline'.")
        print("Set the value between 1 and 8.")
        print("Exit the program.")
        sys.exit()

    # Initialize DLL
    if 'LJS8IF_Initialize' in dir(LJSwrap):
        res = LJSwrap.LJS8IF_Initialize()
        print("LJSwrap.LJS8IF_Initialize:", hex(res))
        if res != 0:
            print("Failed to initialize the library.")
            LJSwrap.LJS8IF_Finalize()
            print("Exit the program.")
            sys.exit()

    # Ethernet open
    res = LJSwrap.LJS8IF_EthernetOpen(0, ethernetConfig)
    print("LJSwrap.LJS8IF_EthernetOpen:", hex(res))
    if res != 0:
        print("Failed to connect contoller.")
        res = LJSwrap.LJS8IF_CommunicationClose(deviceId)
        print("LJSwrap.LJS8IF_CommunicationClose:", hex(res))
        if 'LJS8IF_Initialize' in dir(LJSwrap):
            LJSwrap.LJS8IF_Finalize()
        print("Exit the program.")
        sys.exit()

    # Initialize Hi-Speed Communication
    my_callback_s_a = LJSwrap.LJS8IF_CALLBACK_SIMPLE_ARRAY(callback_s_a)

    res = LJSwrap.LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray(
        deviceId,
        ethernetConfig,
        HighSpeedPortNo,
        my_callback_s_a,
        0)
    print("LJSwrap.LJS8IF_InitializeHighSpeedDataCommunicationSimpleArray:",
          hex(res))
    if res != 0:
        res = LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication(deviceId)
        print("LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication:", hex(res))
        res = LJSwrap.LJS8IF_CommunicationClose(deviceId)
        print("LJSwrap.LJS8IF_CommunicationClose:", hex(res))
        if 'LJS8IF_Initialize' in dir(LJSwrap):
            LJSwrap.LJS8IF_Finalize()
        print("\nExit the program.")
        sys.exit()

    # PreStart Hi-Speed Communication
    req = LJSwrap.LJS8IF_HIGH_SPEED_PRE_START_REQ()
    req.bySendPosition = 2
    profinfo = LJSwrap.LJS8IF_HEIGHT_IMAGE_INFO()

    res = LJSwrap.LJS8IF_PreStartHighSpeedDataCommunication(
        deviceId,
        req,
        use_image_filter,
        profinfo)
    print("LJSwrap.LJS8IF_PreStartHighSpeedDataCommunication:", hex(res))
    if res != 0:
        res = LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication(deviceId)
        print("LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication:", hex(res))
        res = LJSwrap.LJS8IF_CommunicationClose(deviceId)
        print("LJSwrap.LJS8IF_CommunicationClose:", hex(res))
        if 'LJS8IF_Initialize' in dir(LJSwrap):
            LJSwrap.LJS8IF_Finalize()
        print("\nExit the program.")
        sys.exit()

    # allocate the memory
    xsize = profinfo.wXPointNum
    ysize = profinfo.wYLineNum
    z_val = [0] * xsize * ysize
    lumi_val = [0] * xsize * ysize

    # Start Hi-Speed Communication
    image_available = False
    acquisition_timeout = False
    processing_timeout = False
    res = LJSwrap.LJS8IF_StartHighSpeedDataCommunication(deviceId)
    print("LJSwrap.LJS8IF_StartHighSpeedDataCommunication:", hex(res))
    if res != 0:
        res = LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication(deviceId)
        print("LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication:", hex(res))
        res = LJSwrap.LJS8IF_CommunicationClose(deviceId)
        print("LJSwrap.LJS8IF_CommunicationClose:", hex(res))
        if 'LJS8IF_Initialize' in dir(LJSwrap):
            LJSwrap.LJS8IF_Finalize()
        print("\nExit the program.")
        sys.exit()

    # Start Measure
    waitTrigger_sec = 0
    if use_external_trigger is False:
        # Check the  status of the sensor head.
        status = ctypes.c_ushort()
        start_time = time.time()
        while True:
            LJSwrap.LJS8IF_GetAttentionStatus(deviceId, status)
            if status.value & TRG_READY:
                break
            waitTrigger_sec = time.time() - start_time
            if waitTrigger_sec > timeout_sec:
                break
        res = LJSwrap.LJS8IF_Trigger(deviceId)
        print("LJSwrap.LJS8IF_Trigger:", hex(res))
        if res != 0:
            res = LJSwrap.LJS8IF_StopHighSpeedDataCommunication(deviceId)
            print("LJSwrap.LJS8IF_StopHighSpeedDataCommunication:", hex(res))
            res = LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication(deviceId)
            print("LJSwrap.LJS8IF_FinalizeHighSpeedDataCommu.:", hex(res))
            res = LJSwrap.LJS8IF_CommunicationClose(deviceId)
            print("LJSwrap.LJS8IF_CommunicationClose:", hex(res))
            if 'LJS8IF_Initialize' in dir(LJSwrap):
                LJSwrap.LJS8IF_Finalize()
            print("\nExit the program.")
            sys.exit()

    # wait for the image acquisition complete
    start_time = time.time()
    while True:
        if image_available:
            break
        if time.time() - start_time > timeout_sec - waitTrigger_sec:
            acquisition_timeout = True
            break

    # Stop
    isCommCloseComplete = False
    res = LJSwrap.LJS8IF_StopHighSpeedDataCommunication(deviceId)
    print("LJSwrap.LJS8IF_StopHighSpeedDataCommunication:", hex(res))

    # Finalize
    # Wait for the high speed communication close completed.
    # Or wait until a timeout occurs.
    start_time = time.time()
    while True:
        if isCommCloseComplete:
            break
        if time.time() - start_time > timeout_sec:
            break
    res = LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication(deviceId)
    print("LJSwrap.LJS8IF_FinalizeHighSpeedDataCommunication:", hex(res))

    # Close
    res = LJSwrap.LJS8IF_CommunicationClose(deviceId)
    print("LJSwrap.LJS8IF_CommunicationClose:", hex(res))

    # Finalize DLL
    if 'LJS8IF_Finalize' in dir(LJSwrap):
        LJSwrap.LJS8IF_Finalize()

    if image_available is not True:
        print("\nFailed to acquire image (timeout)")
        print("\nTerminated normally.")
        sys.exit()


    if interpolate_yline >= 1:
        # Interpolation
        tmp = np.reshape(z_val, (ysize, xsize))
        tmp = np.tile(tmp, interpolate_yline)
        z_val = tmp.reshape(ysize*interpolate_yline*xsize)
        print(f"z_val: {z_val.shape}")

        tmp = np.reshape(lumi_val, (ysize, xsize))
        tmp = np.tile(tmp, interpolate_yline)
        lumi_val = tmp.reshape(ysize*interpolate_yline*xsize)

        # Update the profile information
        profinfo.wYLineNum *= interpolate_yline
        ysize = profinfo.wYLineNum
        profinfo.dwPitchY = int(profinfo.dwPitchY / interpolate_yline)

        # Generating Return Array
        tmp = PIL.Image.new('I', (xsize, ysize))
        tmp.putdata(list(map(int, z_val)))

    ##################################################################
    # Information of the acquired image
    ##################################################################

    print("----------------------------------------")
    print("Luminance output     : ", profinfo.byLuminanceOutput)
    print("Number of X points   : ", profinfo.wXPointNum)
    print("Number of Y lines    : ", profinfo.wYLineNum)
    print("X pitch in micrometer: ", profinfo.dwPitchX / 100.0)
    print("Y pitch in micrometer: ", profinfo.dwPitchY / 100.0)
    print("Z pitch in micrometer: ", profinfo.dwPitchZ / 100.0)
    print("Processing Timeout   : ", processing_timeout)
    print("----------------------------------------")

    if interpolate_yline >= 1:
        return_array = np.asarray(tmp)
        if return_type=="raw":
            return return_array
        elif return_type == "mm":
           # return_array[return_array == 0] = np.nan
           # return_array_mm[return_array != np.nan] = (return_array - 32768) * 0.0102
            return_array_mm = (return_array - 32768) * 0.0102
            min_value = np.min(return_array_mm)
            # Replace minimum values with NaN or another sentinel value
            return_array_mm[return_array_mm == min_value] = np.nan
            return return_array_mm
        elif return_type == 'png':
            heightmap = 0.0024*return_array - 39.32136785540743 
            heightmap[return_array==-39.32136785540743] = -99999.9999
            return heightmap
    
    return null


###############################################################################
# Callback function
# It is called when the specified number of profiles are received.
###############################################################################
def callback_s_a(p_header,
                 p_height,
                 p_lumi,
                 luminance_enable,
                 xpointnum,
                 profnum,
                 notify, user):

    global ysize_acquired
    global image_available
    global processing_timeout
    global z_val
    global lumi_val
    global isCommCloseComplete

    if (notify == 0) or (notify == 0x10000):
        if profnum != 0:
            if (image_available is False) and (acquisition_timeout is False):
                processing_timeout = p_header.contents.byProcTimeout > 0
                for i in range(xpointnum * profnum):
                    z_val[i] = p_height[i]
                    if luminance_enable == 1:
                        lumi_val[i] = p_lumi[i]

                ysize_acquired = profnum
                image_available = True

    if (notify == 1):
        isCommCloseComplete = True
    return


if __name__ == '__main__':
    perform_scan()
