from TorFlex_Alignment import Crank_Arm_ASSY_LJS640
import time

# global variables
NO_LIMIT = 2000 # units: mm. bounding box face default value to not trim. max possible scanner value is 1136

# Updates to Measure_Arm.py

def main(filename=None, scan_type='real', side='right', ui=None, debug_flag=False, bboxes_dict=None, rotations_dict=None):
    """
    Main processing function for crank arm alignment.
    If bboxes_dict is provided, uses those bounding boxes for trimming regions.
    Otherwise, defaults to unlimited bounds (NO_LIMIT) for all regions to avoid trimming,
    and prints a message to console if no custom bboxes are specified.
    This ensures computational efficiency by only applying necessary trims, but falls back
    gracefully if no custom bboxes are provided.
    """
    if filename is None:
        filename = r'C:/Users/Public/CapstoneUI/temporary_scan.csv'
    if scan_type not in ['real', 'sim']: raise ValueError("Invalid scan_type. Options: 'real', 'sim'")
    if side not in ['right', 'left']: raise ValueError("Invalid side. Options: 'right', 'left'")
    
    # Define NO_LIMIT and full unbounded bbox
    NO_LIMIT = 2000  # units: mm. Max possible scanner value is 1136, but 2000 ensures no trim
    full_bbox = [-NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT, -NO_LIMIT, NO_LIMIT]
    
    # Use provided bboxes or default to unlimited
    if bboxes_dict is None:
        print("No bounding boxes provided. Using unlimited bounds for all regions to process the full scan.")
        bboxes = {
            'raw_trim': full_bbox,
            'inner_bar': full_bbox,
            'spindle_coarse': full_bbox,
            'spindle_fine': full_bbox  # Optional, but included for consistency
        }
    else:
        bboxes = bboxes_dict
        # Ensure all keys exist, fall back to full if missing (for robustness)
        for key in ['raw_trim', 'inner_bar', 'spindle_coarse', 'spindle_fine']:
            if key not in bboxes:
                bboxes[key] = full_bbox
                print(f"Missing '{key}' in provided bboxes. Using unlimited bounds for this bbox.")
    
    # Load scan data into Crank_Arm_ASSY_LJS640 object
    start = time.time()
    scan1 = Crank_Arm_ASSY_LJS640(filename, inner_bar_view_angle_x=45, scan_type=scan_type, side=side,
                                  bbox=bboxes['raw_trim'], ui=ui, debug_flag=debug_flag) 

    # Oriented point cloud as it would be on a trailer
    scan1.center_cloud_xy()  # Shift trimmed point cloud to XY center for consistency
    if rotations_dict:
        for rot_set in rotations_dict:
            x, y, z = rotations_dict[rot_set]
            scan1.rotate_cloud(axis='x', angle=x)
            scan1.rotate_cloud(axis='y', angle=y)
            scan1.rotate_cloud(axis='z', angle=z)
    else:
        print('No rotations from JSON. Using defaults')
        if scan_type == 'real':
            if side == 'right':
                scan1.rotate_cloud(axis='z', angle=180)
                scan1.rotate_cloud(axis='x', angle=180)
            if side == 'left':
                scan1.rotate_cloud(axis='z', angle=0)
                scan1.rotate_cloud(axis='x', angle=90+180)
        
        elif scan_type == 'sim':
            if side == 'right':
                scan1.rotate_cloud(axis='x', angle=90)
                scan1.rotate_cloud(axis='y', angle=0.3)
                scan1.rotate_cloud(axis='x', angle=0.3)
                scan1.rotate_cloud(axis='z', angle=0.3)
            elif side == 'left':
                scan1.rotate_cloud(axis='x', angle=90)
                scan1.rotate_cloud(axis='z', angle=180)
                scan1.rotate_cloud(axis='y', angle=1.2)
                scan1.rotate_cloud(axis='x', angle=0.5)
                scan1.rotate_cloud(axis='z', angle=0.1)

    scan1.center_cloud()  # Shift trimmed and oriented scan's centroid to the origin
    if debug_flag:
        print('Showing oriented scan. User verify orientation with trailer axes...')
        scan1.show_cloud()

    # Fit axes to bar and spindle
    scan1.fit_bar_faces(plotNum=0, cutoff=bboxes['inner_bar'], show_flag=debug_flag, num_points=10000)
    scan1.rot_bar_to_zero(debug_flag=debug_flag)
    scan1.fit_spindle_3D(bbox_coarse=bboxes['spindle_coarse'], bbox_fine=bboxes['spindle_fine'], show_flag=debug_flag, 
                         plot_flag=debug_flag, box_size=8.0)

    # Process axes direction vectors into toe and camber
    scan1.calc_toe_camber2()
    scan1.print_angles()    
    scan1.save_angles_to_csv()
    # if debug_flag: scan1.plot_unit_vectors()

    end = time.time()
    print(f'\nTotal Duration: {end-start:.3f}')

    results = {"bar_toe": scan1.bar_align[0], "bar_camber": scan1.bar_align[1], 
               "spindle_toe": scan1.spindle_align[0], "spindle_camber": scan1.spindle_align[1],
               "toe": scan1.toe, "camber": scan1.camber, 
               "total_misalign": scan1.total_misalign}
    return results

if __name__ == "__main__":
    main(filename=r"3D Simulation\SimScans\arm_r_22.5_d_0_0_A.txt", side='right', scan_type='sim', debug_flag=True)
 