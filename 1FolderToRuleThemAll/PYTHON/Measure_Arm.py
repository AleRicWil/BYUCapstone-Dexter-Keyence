import numpy as np
from TorFlex_Alignment import Torsion_Arm_LJS640
import time

def main(filename=None, auto_flag=False, scan_type='live', ui=None):
    start = time.time()
    if filename == None:
        filename = r'C:\Users\Public\CapstoneUI\temporary_scan.csv'

    # Load arm scan
    index = 3
    scan1 = Torsion_Arm_LJS640(filename, view_angle_horizontal=45, scanType='live')
    # scan1.show_cloud()q

    # Prepare data
    scan1.center_cloud()
    scan1.show_cloud()
    scan1.rotate_cloud(axis='z', angle=90)
    scan1.show_cloud()

    # Find bar faces
    scan1.fit_bar_faces(plotNum=0, show=True)
    # print(f'Bar Axis: {scan1.bar_axis}')

    # Find spindle
    scan1.fit_spindle(num_bins=100, show=True, plot=False)
    # print(f'Spindle Axis: {scan1.axis_dir}')

    scan1.calc_angles()
    scan1.print_angles()    

    end = time.time()

    # scan1.show_cloud(np.hstack((scan1.barPrimaryFace, scan1.barSecondaryFace, scan1.spindle_cloud)))

    print(f'\nTotal Duration: {end-start:.3f}')

    results = {"bar_x_angle": scan1.bar_angle[0], "bar_y_angle": scan1.bar_angle[1], "bar_z_angle": scan1.bar_angle[2], 
               "spindle_x_angle": scan1.spindle_angle[0], "spindle_y_angle": scan1.spindle_angle[1], "spindle_z_angle": scan1.spindle_angle[2],
               "rel_x_angle": scan1.relative_angle[0], "rel_y_angle": scan1.relative_angle[1], "rel_z_angle": scan1.relative_angle[2],
               "total_angle": scan1.total_angle}
    return results

if __name__ == "__main__":
    main(filename='RealScans\ArmTest1.csv')
    