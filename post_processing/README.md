# Post Processing

These are tools for post processing the pose estimation data
to generate insights on subject's upper extremity function.

## Steps to run:

1.  Run the pose processing with depth extraction (see pose/README.md)
2.  Download the pose HDF5 files:
    `oci os object sync -bn rrl-flo-hdf5 --include '*-poses-depth.hdf5' --dest-dir <target_dir>`
3.  Run filtering and smoothing: `python3 -m post_processing.src.generate_state -t <target_dir>`
4.  Generate data labels using R [export_labels.Rmd](https://github.com/Rehab-Robotics-Lab/FloProspectiveTrialAnalysis/blob/8116be17cecbd31d8580893b7e7176a1f40245cf/aim1/notebooks/export_labels.Rmd) and copy the resulting file from the output folder from the R project to `<target_dir>`
5.  Generate test/train split: `python3 -m post_processing.src.test_train_split -t <target_dir>`
6.  Run simon says processing: `python3 -m post_processing.src.simon_says -t <target_dir>`
7.  Run target touch processing: `python3 -m post_processing.src.target_touch -t <target_dir>`
