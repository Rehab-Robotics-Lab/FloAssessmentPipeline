# Inspect data to generate meta data

## In EC2

### Setup

1.  Provision an EC2 insance:
    *   ubuntu 20 arm
    *   c6gd.4xlarge
    *   spot pricing, persistant, $0.6, stop
    *   deploy the instance to one of the private subnets in pennnet-research-vpc-1 (i.e. subnet-0c0e1f2ce3ad0ca21 or subnet-03a7ce9eee249898f)
    *   iam: flo-data-coding-instance-profile
    *   give the root drive 12 GiB
    *   add Name tag set to: flo-exp-aim1-inspect
    *   use the already made ssh security group, which opens port 22 to the penn private group (pl-03aa016d1005ef401)
    *   use the key: [`flo-exp-aim1-inspect-key.pem`](https://upenn.app.box.com/file/771977315034)
    *   12GB of EBS
2.  setup an alarm to turn off if inactive for more than 30 minutes
3.  connect to Penn VPN
4.  Send over the code we need: `scp -rp -i ~/.aws/keys/flo-exp-aim1-inspect-key.pem ~/Documents/git/LilFloAssessmentPipeline/inspect/* ubuntu@10.128.253.100:/home/ubuntu`
5.  ssh in: `ssh -i ~/.aws/keys/flo-exp-aim1-inspect-key.pem ubuntu@10.128.253.81` <- or wherever you put your key
6.  run `./setup_ec2.sh`

### Operation

1.  Connect to the Penn VPN
2.  Turn on the flo-exp-aim1-inspect EC2 instance
3.  Connect to remote
    *   Connect to the EC2 instance using ssh (ip addr may be different): `ssh -i ~/.aws/keys/flo-exp-aim1-inspect-key.pem ubuntu@10.128.253.100 -X`
    *   Or connect to the EC2 instance using X2Go
        1.  Install X2Go client: `apt-get install x2goclient`
        2.  Run X2Go, select the IP address of the target EC2 instance, XFCE for the desktop manager, and the SSH Private key for the private key
        3.  Select the connection you want off of the right pain of the session control window
    *   Connect VLC directly. On your local machine: `vlc sftp://ubuntu@<ip-addr>:/data/gopro/...`
4.  Mount the ephemeral storage with `./mount_instance_store.sh`
5.  Bring in the subject's data you are interested in: `./load_subj_data.sh -s <subj no, ex: 8>`
6.  You can now see the subject's data in `/data`
7.  You can edit `/data/meta.yaml` using whichever text editor you like. vim, nano, and gedit should all be available
8.  View the gopro videos with VLC
9.  View the ROS files by running `roscore & rqt_bag`, a new window should pop up, select the bag files you want to loook at (if you load too many at once it might lag, one session seems to be a good amount (look at file sizes to determine session start and end)). You can see the timestamp you wish to record on the bottom of the screen as you move the read head. Note, it may take a while for all of the bags to load in. For one of the video streams, right click and select view->image to see what is going on. Then drag the little red vertical line around. You can also pause and play. Use your mouse wheel to zoom in and out. Click on your mouse wheel and drag to go left and right.
10. Once the meta file is filled in, save it.
11. Push the meta file back to S3 by running `./save_meta.sh`
12. If you want to do another subject, delete the contents of /data and run `load_subj_data.sh` with the new subject number
13. Shutdown the EC2 instance from the aws ec2 console, cleanup will be automatic

## Use docker compose to inspect bags

This provides the ability to use docker to view bag files.

### Setup

1.  install docker
2.  install docker-compose

### Running

run `rqt_bag_docker.bash`
