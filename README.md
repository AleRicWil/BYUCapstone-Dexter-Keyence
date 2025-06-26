# capstone-code

### Quick Links:
[Direct Sensor Control](#direct-sensor-control)  
[Dual Sensor Setup](#dual-sensor-setup)


### Direct Sensor Control
The presumed hardware setup is as follows: 
- Ethernet connection from computer to LJ-S 640 sensor
- Power connection from Keyence power supply to LJ-S 640 sensor

These instructions also assume that the target sensor is configured to the default IP address. To set the LJ-S 640 to the default address, use the following image:

![sensor-reset-default](https://github.com/user-attachments/assets/f10025b9-b064-4db2-8e12-05ccbbbca78b)

For direct sensor control to work, there is 1 tool that must be installed on your computer, linked here: 

[Keyence Software Download: LJ-S Observer and LJ-S Navigator](https://www.keyence.com/download/download/confirmation/?dlAssetId=AS_149536) 

Once you’ve downloaded that, unzip and run the installation scripts (setup.exe) for both the IP Setting Tool and the Installer 

![Screenshot 2025-03-20 164245](https://github.com/user-attachments/assets/e3a5fc43-9d56-43fc-9a55-e454a8ac6848)

Once installation is finished, restart your computer and run LJ-S Navigator with the sensor head connected and fully booted (takes a couple minutes after powering on. Solid green lights). It should look like this. Click ‘Start Monitoring’

![Screenshot 2025-03-20 164351](https://github.com/user-attachments/assets/aceef159-f584-43a5-9aa6-741e60d835dc)

![image](https://github.com/user-attachments/assets/7168df1f-fcd8-4ec8-8220-6846a668f4ae)

![image](https://github.com/user-attachments/assets/8af93e3d-8d96-464f-86cd-a4a4b0c65736)

If you get an error that looks like this, then it’s likely that your computer’s IP address isn’t set properly. 

Use the keyboard command Windows Key + R to open the application run dialog. Type ‘Control Panel’ and hit enter. 

![image](https://github.com/user-attachments/assets/2fa6ec47-1a39-489b-8032-9ca8f5519f15)

From there, select ‘Network and Internet’ > ‘Network and Sharing Center’ > ‘Ethernet’ > ‘Properties’ > Double click on ‘Internet Protocol Version 4 (TCP/IPV4)’ 

![image](https://github.com/user-attachments/assets/9731eba5-00ff-4bf6-8cbd-5cbc996ac647)

Then, enter in the IP address and subnet mask as shown: 

![image](https://github.com/user-attachments/assets/6e9e0f17-d218-44fe-ba4c-3ecd9a1ed98f)

Select ‘Ok’ on each of the pop-ups until you’re back at the Control Panel. Close the Control Panel. 

Now, try clicking the ‘Start Monitoring’ button again in LJ-S Navigator. You should now be able to trigger a scan using the ‘Trigger Input’ or ‘Continuous Trigger’ buttons. These scans will appear on screen. 

![image](https://github.com/user-attachments/assets/60f67f92-59c3-4668-b5a0-67487b283052)

That means that the direct link is working. Now, press the ‘Stop’ button. 

From there, the python code should work. 

### Dual Sensor Setup
After you’ve followed the setup and properly connected to one scanner [linked here](#direct-sensor-control), it’s possible to configure so that one computer can trigger two scans

Begin by setting the second scanner’s IP address to “Not set”, as follows:

![sensor-reset-not-set](https://github.com/user-attachments/assets/ad8e47f9-a1cf-45ec-ac81-3c311706d98c)

Then, connect the second scanner to the computer via ethernet and launch the Keyence ‘IP Setting Tool’:

![unnamed](https://github.com/user-attachments/assets/e09cc0f0-4cce-4dea-b3fb-cb638b90adfc)

If the scanner is properly connected and the IP address has been properly set to ‘Not set’, it should look like this:

![unnamed](https://github.com/user-attachments/assets/162ea519-cf89-49ea-8699-0549a1881a9d)

Select the scanner and then select ‘Setup IP addr’
As shown below, set the IP address to 192.168.0.3. The address needs to be in the same family as the first scanner and as your ethernet driver, which should be set to 192.168.0.1 and 192.168.0.2 respectively, so that the LJ-S Direct Link library can work properly. After the IP address is properly set, you should be able to use both scanners at once while connected to the same computer.

![setting-ip](https://github.com/user-attachments/assets/fb3b309e-fee0-4ccd-86c3-90304ad4aeb5)
