from TorFlex_Alignment import Crank_Arm_ASSY_LJS640
import time

# global variables
NO_LIMIT = 2000 # units: mm. bounding box face default value to not trim. max possible scanner value is 1136

def main(filename=None, scan_type='real', side='right', ui=None, debug_flag=False):
    if filename is None:
        filename = r'C:/Users/Public/CapstoneUI/temporary_scan.csv'
    if scan_type not in ['real', 'sim']: raise ValueError("Invalid scan_type. Options: 'real', 'sim'")
    if side not in ['right', 'left']: raise ValueError("Invalid side. Options: 'right', 'left'")
    
    # setup bounding boxes for type of scan and type of arm

    bboxes = {} # units: mm. bounding box faces. x_min, x_max, y_min, y_max, z_min, z_max 

    bboxes['raw_trim'] = [-NO_LIMIT, NO_LIMIT, -NO_LIMIT, 250, -NO_LIMIT, NO_LIMIT]
    
    if scan_type == 'real':
        if side == 'right':             #    x_min,    x_max,     y_min,    y_max,     z_min,    z_max
            bboxes['inner_bar'] =       [-NO_LIMIT,       50, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT] 
            bboxes['spindle_coarse'] =  [       96, NO_LIMIT, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT]
            bboxes['spindle_fine'] =    [       96, NO_LIMIT,      -115,      -45, -NO_LIMIT,      -70]
        elif side == 'left':
            bboxes['inner_bar'] =       [      -50, NO_LIMIT, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT]
            bboxes['spindle_coarse'] =  [-NO_LIMIT,      -96, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT]
            bboxes['spindle_fine'] =    [-NO_LIMIT,      -96,      -115,      -70,      -110,      -40]
    
    elif scan_type == 'sim':
        if side == 'right':
            bboxes['inner_bar'] =       [-NO_LIMIT,      -26, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT]  
            bboxes['spindle_coarse'] =  [       22, NO_LIMIT, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT] 
        elif side == 'left':
            bboxes['inner_bar'] =       [       25, NO_LIMIT, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT]
            bboxes['spindle_coarse'] =  [-NO_LIMIT,     -150, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT]
    
    # load scan data into Crank_Arm_ASSY_LJS640 object

    start = time.time()
    scan1 = Crank_Arm_ASSY_LJS640(filename, inner_bar_view_angle_x=45, scan_type=scan_type, side=side,
                                  bbox=bboxes['raw_trim'], ui=ui, debug_flag=debug_flag) 

    # oriented point cloud as it would be on a trailer

    scan1.center_cloud_xy() # shift trimmed point cloud to XY center for consistency
    if scan_type == 'real':
        if side == 'right':
            scan1.rotate_cloud(axis='z', angle=180)
            scan1.rotate_cloud(axis='x', angle=180)
        if side == 'left':
            scan1.rotate_cloud(axis='z', angle=0); scan1.rotate_cloud(axis='x', angle=90)
    
    elif scan_type == 'sim':
        if side == 'right':
            scan1.rotate_cloud(axis='x', angle=90)
            scan1.rotate_cloud(axis='y', angle=0.3); scan1.rotate_cloud(axis='x', angle=0.3); scan1.rotate_cloud(axis='z', angle=0.3)
        elif side == 'left':
            scan1.rotate_cloud(axis='x', angle=90); scan1.rotate_cloud(axis='z', angle=180)
            scan1.rotate_cloud(axis='y', angle=1.2); scan1.rotate_cloud(axis='x', angle=0.5); scan1.rotate_cloud(axis='z', angle=0.1)

    scan1.center_cloud() # shift trimmed and oriented scan's centroid to the origin
    if debug_flag:
        print('Showing oriented scan. User verify orientation with trailer axes...'); scan1.show_cloud()

    # Fit axes to bar and spindle
    scan1.fit_bar_faces(plotNum=0, cutoff=bboxes['inner_bar'], show_flag=debug_flag, num_points=1000)
    scan1.fit_spindle_3D(bbox_coarse=bboxes['spindle_coarse'], bbox_fine=bboxes['spindle_fine'], show_flag=debug_flag, 
                         plot_flag=debug_flag, box_size=8.0)

    # Process axes direction vectors into toe and camber
    scan1.calc_toe_camber()
    scan1.print_angles()    
    scan1.save_angles_to_csv()
    if debug_flag: scan1.plot_unit_vectors()

    end = time.time()
    print(f'\nTotal Duration: {end-start:.3f}')

    results = {"bar_toe": scan1.bar_align[0], "bar_camber": scan1.bar_align[1], 
               "spindle_toe": scan1.spindle_align[0], "spindle_camber": scan1.spindle_align[1],
               "toe": scan1.toe, "camber": scan1.camber, 
               "total_misalign": scan1.total_misalign}
    return results

if __name__ == "__main__":
    main(filename=r'Scan_Data/cold_rolled_arm_01.csv', side='right', scan_type='real', debug_flag=False)
 