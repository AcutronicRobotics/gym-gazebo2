def getModelFileType(path):
    if path.endswith('.sdf'):
        return "sdf"
    elif path.endswith('.urdf'):
        return "urdf"
    else:
        raise TypeError('the file must be .sdf or .urdf')

def getArgsMARA():
    import argparse
    parser = argparse.ArgumentParser(description='MARA environment argument provider.')
    parser.add_argument('-g', '--gzclient', action='store_true', help='Run user interface.')
    parser.add_argument('-r', '--real_speed', action='store_true', help='Execute the simulation in real speed. RTF=1.')
    parser.add_argument('-v', '--velocity', type=float, default=1.0, help='Set servo motor velocity. Keep < 1.41 for real speed.')
    args = parser.parse_args()

    return args