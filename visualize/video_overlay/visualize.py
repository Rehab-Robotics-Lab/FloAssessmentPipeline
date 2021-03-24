#!/usr/bin/env python3
# Expect one argument of form: <file stub> which will be used to access
# <file stub>.hdf5 and <filestub>-novid.hdf5 and create <file stub>-wrists.avi

import sys
from overlay_wrists import overlay_wrists
from skeleton_overlay2d import overlay_2dSkeleton

if __name__ == '__main__':
    file_stub = sys.argv[1]
    cam = sys.argv[2]
    func = sys.argv[3]

    if func == 'wrists':
        print('overlaying wrists')
        overlay_wrists(file_stub, cam)

    elif func == '2dSkeleton':
        print('overlaying 2D Skeleton')
        overlay_2dSkeleton(file_stub, cam)
