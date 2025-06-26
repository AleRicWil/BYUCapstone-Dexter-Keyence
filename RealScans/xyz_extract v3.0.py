'''Final use for the version is simply to demonstrate the algorithm used to select the hubface.
It does not return alignment values in the same sense as the final design package.'''

import numpy as np
from Torflex_Alignment import Axle_Cloud_LJS640
import time

if __name__ == "__main__":
    start = time.time()
    files = [r'RealScans\Hub_Scans\8-201_1.csv', r'RealScans\Hub_Scans\8-213_1.csv', r'RealScans\Hub_Scans\8-257_1.csv',
             r'RealScans\Hub_Scans\8-320_1.csv', r'RealScans\Hub_Scans\8-321_1.csv', r'RealScans\Hub_Scans\smallHD_1.csv',
             r'RealScans\Hub_Scans\smallHD_1.csv']

    filename2 = r'Simulated Scans\TorFlex_4000b_0.2TOE_0.5CAM.txt'

    # Load Scan
    # scan1 = Axle_Cloud_LJS640(filename=filename2, view_angle_horizontal=0.0, scanType='sim', cutOff=[-250, 250])
    '''Change number in box to cycle through hub types'''
    scan1 = Axle_Cloud_LJS640(filename=files[4], view_angle_horizontal=0.0, scanType='real', cutOff=[-122, 250])
    '''don't change anything below here'''
    scan1.show_cloud()
    # scan1.rotate('y', -20)

    # Calculate Angles
    # scan1.align_z(auto=True)
    scan1.sort_ledges()
    scan1.show_cloud(np.hstack(scan1.sorted_ledges))
    scan1.calc_ref_angle(plotNum=0)
    scan1.calc_hub_angle(auto=False, plotNum=1, deleteGround=False)
    scan1.calc_hub_relative_angle()

    # Display Results
    np.set_printoptions(precision=4, suppress=True)
    print('\n---Scan1---')
    print('Ref Angle: ', 90 - scan1.ref_angle)
    print('Hub Angle: ', 90 - scan1.hub_angle)
    print('Hub Rel Angle: ', scan1.hub_relative_angle)
    np.set_printoptions() 
    
    
    end = time.time()
    print(f'\nTotal Duration: {end-start:.3f}')
