# -*- coding: 'unicode' -*-
# Copyright (c) 2024 KEYENCE CORPORATION. All rights reserved.

import LJSwrap

import ctypes
import sys
import time


def main():

    deviceId = 0  # Set "0" if you use only 1 head.
    ethernetConfig = LJSwrap.LJS8IF_ETHERNET_CONFIG()
    ethernetConfig.abyIpAddress[0] = 192  # IP address
    ethernetConfig.abyIpAddress[1] = 168
    ethernetConfig.abyIpAddress[2] = 0
    ethernetConfig.abyIpAddress[3] = 1
    ethernetConfig.wPortNo = 24691        # Port No.

    if 'LJS8IF_Initialize' in dir(LJSwrap):
        LJSwrap.LJS8IF_Initialize()
    res = LJSwrap.LJS8IF_EthernetOpen(deviceId, ethernetConfig)
    print("LJSwrap.LJS8IF_EthernetOpen:", hex(res))
    if res != 0:
        print("Failed to connect contoller.")
        sys.exit()
    print("----")

    ##################################################################
    # sample_HowToCallFunctions.py:
    #  A sample collection of how to call LJSwrap I/F functions.
    #
    # Conditional branch of each sample is initially set to 'False'.
    # This is to prevent accidental execution. Set 'True' to execute.
    #
    # <NOTE> Head settings may change in some sample codes.
    #
    ##################################################################

    if False:
        print("LJSwrap.LJS8IF_Reboot:",
              hex(LJSwrap.LJS8IF_Reboot(deviceId)))
        print("Rebooting.")
        print("----")
        print("\nTerminated normally.")
        sys.exit()

    if False:
        print("LJSwrap.LJS8IF_ReturnToFactorySetting:",
              hex(LJSwrap.LJS8IF_ReturnToFactorySetting(deviceId)))
        print("----")

    if False:
        print("LJSwrap.LJS8IF_ControlLaser:",
              hex(LJSwrap.LJS8IF_ControlLaser(deviceId, 0)), "<Laser OFF>")
        time.sleep(1)
        print("LJSwrap.LJS8IF_ControlLaser:",
              hex(LJSwrap.LJS8IF_ControlLaser(deviceId, 1)), "<Laser ON>")
        print("----")

    if False:
        rcvMax = 10
        errCnt = ctypes.c_ubyte()
        errCode = (ctypes.c_ushort * rcvMax)()
        res = LJSwrap.LJS8IF_GetError(deviceId, rcvMax, errCnt, errCode)
        print("LJSwrap.LJS8IF_GetError:", hex(res),
              "<rcvMax, errCount, errCode>=",
              rcvMax, errCnt.value, hex(errCode[0]))

        res = LJSwrap.LJS8IF_ClearError(deviceId, errCode[0])
        print("LJSwrap.LJS8IF_ClearError:", hex(res),
              "<clear error code>=", errCode[0])
        print("----")

    if False:
        sensorT = ctypes.c_short()
        processorT1 = ctypes.c_short()
        processorT2 = ctypes.c_short()
        caseT = ctypes.c_short()
        driver_unitT = ctypes.c_short()
        res = LJSwrap.LJS8IF_GetHeadTemperature(deviceId, sensorT,
                                                processorT1, processorT2,
                                                caseT, driver_unitT)
        print("LJSwrap.LJS8IF_GetHeadTemperature:", hex(res),
              "<SensorT, ProcessorT1, ProcessorT2,",
              "CaseT, DriverUnitT [degree Celsius]>=",
              sensorT.value/100.0,
              processorT1.value/100.0, processorT2.value/100.0,
              caseT.value/100.0, driver_unitT.value/100.0)
        print("----")

    if False:
        headmodel = ctypes.create_string_buffer(32)
        res = LJSwrap.LJS8IF_GetHeadModel(deviceId, headmodel)
        print("LJSwrap.LJS8IF_GetHeadModel:", hex(res),
              "<headmodel>=", headmodel.value)
        print("----")

    if False:
        headSerial = ctypes.create_string_buffer(16)
        res = LJSwrap.LJS8IF_GetSerialNumber(deviceId, headSerial)
        print("LJSwrap.LJS8IF_GetSerialNumber:", hex(res),
              "<headSerial>=", headSerial.value)
        print("----")

    if False:
        attentionStatus = ctypes.c_ushort()
        res = LJSwrap.LJS8IF_GetAttentionStatus(deviceId, attentionStatus)
        print("LJSwrap.LJS8IF_GetAttentionStatus:", hex(res),
              "<AttentionStatus>=", bin(attentionStatus.value))
        print("----")

    if False:
        print("LJSwrap.LJS8IF_Trigger:",
              hex(LJSwrap.LJS8IF_Trigger(deviceId)))
        print("----")

    if False:
        print("LJSwrap.LJS8IF_ClearMemory:",
              hex(LJSwrap.LJS8IF_ClearMemory(deviceId)))
        print("----")

    if False:
        # Example of how to change some settings.
        # In this example, the "device name" is changed.

        depth = 1  # 0: Write, 1: Running, 2: Save

        targetSetting = LJSwrap.LJS8IF_TARGET_SETTING()
        targetSetting.byType = 0x01         # Environment setting
        targetSetting.byCategory = 0x00     # Trigger Category
        targetSetting.byItem = 0x00         # Device Name
        targetSetting.byTarget1 = 0x00      # reserved
        targetSetting.byTarget2 = 0x00      # reserved
        targetSetting.byTarget3 = 0x00      # reserved
        targetSetting.byTarget4 = 0x00      # reserved

        # Set the device name to 'abcd'
        err = ctypes.c_uint()
        pyArr = 'abcd'  # Sample setting value.
        dataSize = len(pyArr)

        # Convert each character in pyArr to its ASCII value
        asciiSettingArr = [ord(char) for char in pyArr]

        settingData_set = (ctypes.c_ubyte * dataSize)(*asciiSettingArr)

        res = LJSwrap.LJS8IF_SetSetting(deviceId, depth,
                                        targetSetting,
                                        settingData_set, dataSize, err)
        print("LJSwrap.LJS8IF_SetSetting:", hex(res),
              "<Set value>=", ''.join(pyArr),
              "<SettingError>=", hex(err.value))

        # Get setting. This is not mandatory. Just to confirm.
        settingData_get = (ctypes.c_ubyte * dataSize)()
        res = LJSwrap.LJS8IF_GetSetting(deviceId, depth,
                                        targetSetting,
                                        settingData_get, dataSize)
        SettingArr_get = [chr(char) for char in settingData_get]
        print("LJSwrap.LJS8IF_GetSetting:", hex(res),
              "<Get value>=", ''.join(SettingArr_get))
        print("----")

    if False:
        # Example of how to initialize settings.
        # In this example, whole settings of Program No.0 will be initialized.

        Depth = 1   # 0: Write, 1: Running, 2: Save
        Target = 0  # Program No.0
        res = LJSwrap.LJS8IF_InitializeSetting(deviceId, Depth, Target)
        print("LJSwrap.LJS8IF_InitializeSetting:", hex(res),
              "<Initialize Program No.>=", Target)
        print("----")

    if False:
        # Example of how to rewrite and reflect Settings.
        # In this example,
        # the currently running settings are overwritten
        # with the contents of the non-volatile memory.

        # (1)Copy settings from "Save area" to "Write area"
        Depth = 2  # 1: Running, 2: Save
        Error = ctypes.c_uint()
        res = LJSwrap.LJS8IF_RewriteTemporarySetting(deviceId, Depth)
        print("LJSwrap.LJS8IF_RewriteTemporarySetting:", hex(res))

        # (2)Reflect settings from "Write area" to "Running area"
        Depth = 1  # 1: Running, 2: Save
        res = LJSwrap.LJS8IF_ReflectSetting(deviceId, Depth, Error)
        print("LJSwrap.LJS8IF_ReflectSetting:", hex(res),
              "<SettingError>=", hex(Error.value))
        print("----")

    if False:
        Busy = ctypes.c_ubyte()
        res = LJSwrap.LJS8IF_CheckMemoryAccess(deviceId, Busy)
        print("LJSwrap.LJS8IF_CheckMemoryAccess:", hex(res),
              "<Busy>=", Busy.value)
        print("----")

    if False:
        # Example of how to change active Program No.

        # Set active program No. to '5'
        programNo_set = 5
        res = LJSwrap.LJS8IF_ChangeActiveProgram(deviceId, programNo_set)
        print("LJSwrap.LJS8IF_ChangeActiveProgram:", hex(res),
              "<ProgramNo_set>=", programNo_set)

        # Get active program No.
        programNo_get = ctypes.c_ubyte()
        res = LJSwrap.LJS8IF_GetActiveProgram(deviceId, programNo_get)
        print("LJSwrap.LJS8IF_GetActiveProgram:", hex(res),
              "<ProgramNo_get>=", programNo_get.value)

        time.sleep(1)

        # Set active program No. to '0'
        programNo_set = 0
        res = LJSwrap.LJS8IF_ChangeActiveProgram(deviceId, programNo_set)
        print("LJSwrap.LJS8IF_ChangeActiveProgram:", hex(res),
              "<ProgramNo_set>=", programNo_set)

        # Get active program No.
        programNo_get = ctypes.c_ubyte()
        res = LJSwrap.LJS8IF_GetActiveProgram(deviceId, programNo_get)
        print("LJSwrap.LJS8IF_GetActiveProgram:", hex(res),
              "<ProgramNo_get>=", programNo_get.value)
        print("----")

    if False:
        # Example of how to get profile data.
        #
        # <NOTE>
        # -This method is suitable for reading a few profile data.
        #
        # -Use high-speed communication method to acquire a large amount
        #  of profiles, such as height or luminance image data.
        #  For details, refer to another sample (sample_ImageAcquisition.py)

        # Change according to your head settings.
        xpointNum = 3200            # Number of X points per one profile.

        # Specifies the position, etc. of the profiles to get.
        req = LJSwrap.LJS8IF_GET_HEIGHT_IMAGE_PROFILE_REQUEST()
        req.byPositionMode = 0x0      # 0: from current position
        req.dwGetHeightImageNo = 0x0  # The height image number to get.
        req.dwGetProfileNo = 0x0  # The profile num to start getting profiles
        req.wGetProfileCount = 1      # the number of profiles to read.
        req.byErase = 0               # 0: Do not erase

        rsp = LJSwrap.LJS8IF_GET_HEIGHT_IMAGE_PROFILE_RESPONSE()

        profinfo = LJSwrap.LJS8IF_HEIGHT_IMAGE_INFO()

        use_image_filter = False    # set to TRUE to use image filter

        # Calculate the buffer size to store each data.
        datanum = int(xpointNum * req.wGetProfileCount)
        profheader = (LJSwrap.LJS8IF_PROFILE_HEADER * req.wGetProfileCount)()
        profheight = (ctypes.c_ushort * datanum)()
        profluminance = (ctypes.c_ubyte * datanum)()

        # Send command.
        res = LJSwrap.LJS8IF_GetHeightImageSimpleArray(deviceId,
                                                       req,
                                                       use_image_filter,
                                                       rsp,
                                                       profinfo,
                                                       profheader,
                                                       profheight,
                                                       profluminance)

        print("LJSwrap.LJS8IF_GetHeightImageSimpleArray:", hex(res))
        if res != 0:
            print("Failed to get profile.")
            sys.exit()

        print("----------------------------------------")
        print(" byLuminanceOutput     :", profinfo.byLuminanceOutput)
        print(" wXPointNum  :", profinfo.wXPointNum)
        print(" nXStart(in 0.01um)    :", profinfo.nXStart)
        print(" nYStart(in 0.01um)    :", profinfo.nYStart)
        print(" dwPitchX(in 0.01um)   :", profinfo.dwPitchX)
        print(" dwPitchY(in 0.01um)   :", profinfo.dwPitchY)
        print(" dwPitchZ(in 0.01um)   :", profinfo.dwPitchZ)
        print("-----")
        print(" dwCurrentHeightImageNo    :", rsp.dwCurrentHeightImageNo)
        print(" dwOldestHeightImageNo     :", rsp.dwOldestHeightImageNo)
        print(" dwGetHeightImageTopProfileNo  :",
              rsp.dwGetHeightImageTopProfileNo)
        print(" wGetProfileCount     :", rsp.wGetProfileCount)
        print("----------------------------------------")

        for i in range(profinfo.wXPointNum):
            # Conver X data to the actual length in millimeters
            x_val_mm = (profinfo.nXStart + profinfo.dwPitchX * i) / 100.0  # um
            x_val_mm /= 1000.0  # mm

            # Conver Z data to the actual length in millimeters
            z_val = profheight[i]

            if z_val <= -2147483645:  # invalid value
                z_val_mm = - 999.9999
            else:
                z_val_mm = (z_val - 32768) * profinfo.dwPitchZ / 100.0  # um
                z_val_mm /= 1000.0  # mm

            if profinfo.byLuminanceOutput == 1:
                # Luminance data
                lumi_val = profluminance[i]

                print('{:.04f}'.format(x_val_mm),
                      '{:.04f}'.format(z_val_mm),
                      lumi_val)
            else:
                print('{:.04f}'.format(x_val_mm),
                      '{:.04f}'.format(z_val_mm))

        print("----")

    res = LJSwrap.LJS8IF_CommunicationClose(deviceId)
    print("LJSwrap.LJS8IF_CommunicationClose:", hex(res))

    if 'LJS8IF_Finalize' in dir(LJSwrap):
        LJSwrap.LJS8IF_Finalize()

    return


if __name__ == '__main__':
    main()
