import numpy as np
from TorFlex_Alignment import Axle_Hub_LJS640
import time

def main(Calibration={'x': 0, 'y':0}, filename=None, auto_flag=False, scan_type='live', ui=None):
    start = time.time()
    filename1 = r'Keyence Scans\Week Trial\FlatPlate\8201\8-201_12.csv'
    filename2 = r'TeamGitHub\capstone-code\capstone-code\RealScans\simScan0c.txt'
    scan1 = Axle_Hub_LJS640(filename, view_angle_horizontal=0.0, scanType=scan_type, cutOff=[-350, 500], ui=ui)
    scan1.show_cloud()

    # scan1.align_z(auto=auto_flag)
    scan1.sort_ledges()
    # scan1.show_cloud(np.hstack(scan1.sorted_ledges))
    scan1.calc_ref_angle()
    scan1.calc_hub_angle(auto=auto_flag, deleteGround=False)
    scan1.calc_hub_relative_angle()

    np.set_printoptions(precision=4, suppress=True)
    print('\n---Scan1---')
    print('Ref Angle: ', 90 - scan1.ref_angle)
    print('Hub Angle: ', 90 - scan1.hub_angle)
    print('Hub Rel Angle: ', scan1.hub_relative_angle)
    np.set_printoptions() 
    
    
    end = time.time()
    print(f'\nTotal Duration: {end-start:.3f}')

    results = {"camber_angle": round(90-scan1.hub_angle[0] - Calibration['x'], 4), "toe_angle": round(90-scan1.hub_angle[1] - Calibration['y'], 4)}

    return results


if __name__ == "__main__":
    main()
