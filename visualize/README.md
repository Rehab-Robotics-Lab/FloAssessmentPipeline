# Visualize Results

## Video with wrist annotation

### RUN Locally

1.  `docker build -t video-overlay -f "dockerfiles/video-overlay" .`
2.  Run script: './run-locally.sh -s <subj> -c \<cond: augmented-telepresence, classical-telepresence, in-person> -a \<activity: simon-says, target-touch> -p <camera>, -d \<data_directory(looks for /data/condition/activity.hdf5)>, -o \<overlay: wrists, 2dSkeleton, 3dSkelton>'

notes:
python -m visualize.video_overlay.visualize
