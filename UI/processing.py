import numpy as np
from Torflex_Alignment import Axle_Cloud_LJS640
import time


def main(Calibration):
    start = time.time()
    filename1 = r'Keyence Scans\Week Trial\FlatPlate\8201\8-201_12.csv'
    filename2 = r'GitHub\capstone-code\RealScans\simScan0c.txt'
    scan1 = Axle_Cloud_LJS640(filename=filename2, view_angle_horizontal=0.0, scanType='sim')
    scan1.rotate('y', Calibration['y'])
    scan1.rotate('x', Calibration['x'])

    scan1.align_z(auto=True)
    scan1.sort_ledges()
    # scan1.show_cloud(np.hstack(scan1.sorted_ledges))
    scan1.calc_ref_angle()
    scan1.calc_hub_angle(auto=True)
    scan1.calc_hub_relative_angle()

    np.set_printoptions(precision=4, suppress=True)
    print('\n---Scan1---')
    print('Ref Angle: ', scan1.ref_angle)
    print('Hub Angle: ', scan1.hub_angle)
    print('Hub Rel Angle: ', scan1.hub_relative_angle)
    np.set_printoptions() 
    
    
    end = time.time()
    print(f'\nTotal Duration: {end-start:.3f}')

    results = {"toe_angle": round(scan1.hub_relative_angle[0], 2), "camber_angle": round(scan1.hub_relative_angle[1], 2)}

    return results


if __name__ == "__main__":
    main()