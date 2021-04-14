# openpose-docker

A docker build file for CMU openpose with Python API support

https://hub.docker.com/r/cwaffles/openpose

### Requirements

*   Nvidia Docker runtime: https://github.com/NVIDIA/nvidia-docker#quickstart
*   CUDA 10.0 or higher on your host, check with `nvidia-smi`

### Example

`docker run -it --rm --runtime=nvidia -e NVIDIA_VISIBLE_DEVICES=0 cwaffles/openpose`

The Openpose repo is in `/openpose`

## on AWS

1.  provision instance with Ubuntu 20
2.  type g4dn.2xlarge 20GB of disk space
3.  Push over code, ex: `scp -rp -i ~/.aws/keys/flo-exp-aim1-openpose-key.pem ~/Documents/git/LilFloAssessmentPipeline/openpose ubuntu@10.128.253.74:/home/ubuntu`
4.  run setup script
5.  Edit `/etc/docker/daemon.json` to have `"default-runtime": "nvidia",` as the second line, [like this](http://reader.epubee.com/books/mobile/40/40b7b4f104fa62b2b8fdda2b7c4b0503/text00340.html)
6.  build dockerfile: `docker build . --tag openpose`
7.  attach ssd: `./mount_instance_store.sh`
8.  Run script: ` ./scripts/process-in-ec2.sh -s <subj> -c <cond: augmented-telepresence, classical-telepresence, in-person> -a <activity: simon-says, target-touch>
     `

### permission denied error

you can get permission denied errors if you haven't properly run the mount_instance_store.sh file


### Running Locally

1. Run script: ./setup_local.sh
2. Build openpose using: build dockerfile: `docker build . --tag openpose`
3. Run script: ./scripts/run_local.sh -d <"Location to data directory">

You might get a common GPU architecture not supported error. In that case, make sure to restart docker with : 

sudo systemctl restart docker

Make sure you are able to run:
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
