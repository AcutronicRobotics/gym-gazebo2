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
    parser.add_argument('-r', '--realSpeed', action='store_true', help='Execute the simulation in \
        real speed and using the running specific driver.')
    parser.add_argument('-v', '--velocity', type=float, default=0.5, help='Set servo motor \
        velocity. Keep < 1.57 for real speed. Applies only with -r --realSpeed option.')

    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument('-m', '--multiInstance', action='store_true', help='Provide network \
        segmentation to allow multiple instances.')
    group.add_argument('-p', '--port', type=int, default=11345, help='Provide exact port to the \
        network segmentation to allow multiple instances.')

    return parser

def cleanOldFiles(tempPath, fileEnding, days):
    import os
    import time
    folderPath = tempPath
    fileEndsWith = fileEnding

    now = time.time()

    for file in os.listdir(folderPath):
        fileFullPath = os.path.join(folderPath, file)
        if os.path.isfile(fileFullPath) and file.endswith(fileEndsWith):
            #Delete files older than x days
            if os.stat(fileFullPath).st_mtime < now - days * 86400:
                os.remove(fileFullPath)
