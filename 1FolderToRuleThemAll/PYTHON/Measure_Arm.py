import numpy as np
from TorFlex_Alignment import Torsion_Arm_LJS640
import time

def main(filename=None, auto_flag=False, scan_type='live', side='right', ui=None):
    start = time.time()
    if filename == None:
        filename = r'C:\Users\Public\CapstoneUI\temporary_scan.csv'

    # Load arm scan
    scan1 = Torsion_Arm_LJS640(filename, view_angle_horizontal=45, scan_type=scan_type, side = side,
                               cutOff=[-1000, 1000, -1000, 250, -1000, 1000], # x, y, z min & max
                               ui=None)
    
    # Prepare cloud so it is oriented as it would be on the axle on a trailer
    print('Showing raw scan'); scan1.show_cloud()
    scan1.center_cloud_xy()
    if scan_type == 'sim':
        if side == 'right':
            scan1.rotate_cloud(axis='x', angle=90)
        elif side == 'left':
            scan1.rotate_cloud(axis='x', angle=90); scan1.rotate_cloud(axis='z', angle=180)
    elif scan_type == 'live':
        if side == 'right':
            scan1.rotate_cloud(axis='z', angle=180); scan1.rotate_cloud(axis='x', angle=-90)
            # scan1.rotate_cloud(axis='z', angle=180); scan1.rotate_cloud(axis='x', angle=90)
        if side == 'left':
            pass
    print('Showing oriented scan'); scan1.show_cloud()

    # Setup cutoffs for type of scan and type of arm
    if scan_type == 'sim':
        if side == 'right':
            cutoff = [-1000, -25, -1000, 1000, -1000, 1000]  # x, y, z min & max
            axial_cutoff = 22   # cutoff along bar axis
        elif side == 'left':
            cutoff = [25, 1000, -1000, 1000, -1000, 1000]
            axial_cutoff = -22
    elif scan_type == 'live':
        if side == 'right':
            cutoff = [-5000, 0, -500, 500, -500, 500] 
            axial_cutoff = 57
        elif side == 'left':
            cutoff = [-10, 5000, -5000, 100, -5000, 5000]
            axial_cutoff = -50
    
    # Fit axes to bar and spindle
    scan1.fit_bar_faces(plotNum=0, cutoff=cutoff, show=True, num_points=2000) #; print(f'Bar Axis: {scan1.bar_axis}')
    scan1.fit_spindle_3D(axial_cutoff=axial_cutoff, show_flag=True, box_size=8.0) #; print(f'Spindle Axis: {scan1.spindle_axis}')
    # scan1.visualize_axes(length=100)

    # Process axes direction vectors into toe and camber
    scan1.calc_toe_camber()
    scan1.print_angles()    
    scan1.save_angles_to_csv()
    # scan1.plot_vectors()

    end = time.time()

    # scan1.show_cloud(np.hstack((scan1.barPrimaryFace, scan1.barSecondaryFace, scan1.spindle_cloud)))

    print(f'\nTotal Duration: {end-start:.3f}')

    results = {"bar_toe": scan1.bar_align[0], "bar_camber": scan1.bar_align[1], 
               "spindle_toe": scan1.spindle_align[0], "spindle_camber": scan1.spindle_align[1],
               "toe": scan1.toe, "camber": scan1.camber, 
               "total_misalign": scan1.total_misalign}
    return results

if __name__ == "__main__":
    main(filename=r'RealScans\2D\2DA01.csv', side='right', scan_type='live')
 