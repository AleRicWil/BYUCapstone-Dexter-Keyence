import numpy as np
from Torflex_Coaxial import Torsion_Arm_LJS640
import time

if __name__ == "__main__":
    filename1 = r'Keyence Scans\Crank Arm\Round 1\1.csv'
    scan1 = Torsion_Arm_LJS640(filename1, -20, 'real')
    scan1.show_cloud()

    scan1.center_cloud()

    scan1.fit_bar_faces(axialCutOff=[-500, -120], show=True)
    # print(f'Bar Axis: {scan1.bar_axis}')


    box_start, box_width, box_length, box_thickness = [-20.0, -15.0, 35.0], 100.0, 140.0, 35.0
    bounds = [box_start, box_width, box_length, box_thickness]
    scan1.fit_spindle(axialCutOff=[-15, 130], num_bins=20, bounding_box=bounds, show=True, plot=True)
    # print(f'Spindle Axis: {scan1.axis_dir}')

    scan1.calc_angles()
    scan1.print_angles()    

    scan1.show_cloud(np.hstack((scan1.barPrimaryFace, scan1.barSecondaryFace, scan1.spindle_cloud)))
    