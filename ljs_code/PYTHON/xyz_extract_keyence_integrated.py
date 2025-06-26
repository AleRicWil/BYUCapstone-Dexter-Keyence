import numpy as np
from Torflex_Alignment_Integrated import Axle_Cloud_LJS640
import time

if __name__ == "__main__":
    start = time.time()
    filename1 = r'.\..\png-conversion\the-one.csv'
    filename2 = r'.\simScan0c.txt'

    # Load Scan
    scan1 = Axle_Cloud_LJS640(filename=filename1, view_angle_horizontal=0.0, scanType='real', cutOff=[-500, 500])
    
    np.savetxt('scan1.txt', scan1.cloud, fmt='%f')

    # scan1.rotate('y', -20)

    # Calculate Angles
    # scan1.align_z(auto=True)
    scan1.sort_ledges()
    # scan1.show_cloud(np.hstack(scan1.sorted_ledges))
    scan1.calc_ref_angle(plotNum=1)
    scan1.calc_hub_angle(auto=False, plotNum=2, deleteGround=True)
    scan1.calc_hub_relative_angle()

    # Display Results
    np.set_printoptions(precision=4, suppress=True)
    print('\n---Scan1---')
    print('Ref Angle: ', scan1.ref_angle)
    print('Hub Angle: ', scan1.hub_angle)
    print('Hub Rel Angle: ', scan1.hub_relative_angle)
    np.set_printoptions() 
    
    
    end = time.time()
    print(f'\nTotal Duration: {end-start:.3f}')
