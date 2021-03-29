import docker

if __name__ == "__main__":
    client = docker.from_env(version='1.40')
    ret = client.containers.run(
        'nvidia/cuda:9.0-base',
        'nvidia-smi',
        device_requests=[
            docker.types.DeviceRequest(count=-1, capabilities=[['gpu']])
        ]
    )
    print(ret)
