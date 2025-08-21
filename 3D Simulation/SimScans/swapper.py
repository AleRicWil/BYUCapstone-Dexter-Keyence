with open(r'3D Simulation\SimScans\CrankArm22.5-right_0toe_-5cam.txt', 'r') as f:
    lines = f.readlines()

with open('swapped.txt', 'w') as out:
    for line in lines:
        if line.strip() == 'X Y Z':
            out.write('X Z Y\n')
        else:
            parts = line.split()
            if len(parts) == 3:
                x, y, z = parts
                out.write(f'{x} {z} {y}\n')