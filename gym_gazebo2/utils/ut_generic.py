def getModelFileType(path):
    if path.endswith('.sdf'):
        return "sdf"
    elif path.endswith('.urdf'):
        return "urdf"
    else:
        raise TypeError('the file must be .sdf or .urdf')

def getArgsParserMARA():
    import argparse
    parser = argparse.ArgumentParser(description='MARA environment argument provider.')
    parser.add_argument('-g', '--gzclient', action='store_true', help='Run user interface.')
    parser.add_argument('-r', '--real_speed', action='store_true', help='Execute the simulation in real speed and using the running specific driver.')
    parser.add_argument('-v', '--velocity', type=float, default=0.5, help='Set servo motor velocity. Keep < 1.41 for real speed. Applies only with -r --real_speed option.')

    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('-m', '--multi_instance', action='store_true', help='Provide network segmentation to allow multiple instances.')
    group.add_argument('-p', '--port', type=int, default=11345, help='Provide exact port to the network segmentation to allow multiple instances.')

    return parser

def clean_old_files(temp_path, file_ending, days):
    import os, time
    folder_path = temp_path
    file_ends_with = file_ending

    now = time.time()
    only_files = []

    for file in os.listdir(folder_path):
        file_full_path = os.path.join(folder_path,file)
        if os.path.isfile(file_full_path) and file.endswith(file_ends_with):
            #Delete files older than x days
            if os.stat(file_full_path).st_mtime < now - days * 86400:
                 os.remove(file_full_path)
