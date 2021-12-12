# Kubernetes on OCI

OCI Kubernetes Engine (OKE) can be used to run jobs.

## Setup

1.  Setup cluster on OCI (this is already done)
    1.  Click Create Cluster
    2.  Select Quick Create
    3.  Fill in the name and compartment
    4.  Select the latest version of kubernetes (should be default)
    5.  Select PRIVATE endpoint and workers
    6.  Select whatever type of nodes you want (will likely change this later)
    7.  Select 1 node to get started
    8.  Open advanced options
    9.  Select a custom boot volume size, 2TB seems good
    10. Select No SSH key
    11. Next; Create Cluster
2.  Create a new bastion (already done)
    1.  Select the network created by OKE
    2.  Select the API Enpoint subnet
    3.  If you are able to come up with a restricted IP address list, then you can enter
        that into the CIDR list. Otherwise, enter `0.0.0.0/0`
3.  Install the client on your machine: `setup-kubectl.sh`
4.  [Authenticate using tokens](https://github.com/Rehab-Robotics-Lab/oci-cli-helpers/blob/main/auth/token-alive.sh)
5.  Setup a connection with an ssh tunnel to the cluster using `connect.sh` (leave this running)
6.  Test kubectl with `export KUBECONFIG=$HOME/.kube/config && kubectl get nodes`
7.  You can now run additional commands with kubectl
