# FloAssessmentPipeline

This is the pipeline for assessing patient function based on data from the FloSystem

## Setting up:

- [install docker](https://docs.docker.com/get-docker/)
  - be sure to follow the post-install steps (https://docs.docker.com/engine/install/linux-postinstall/)
- [install pipenv](https://pipenv-fork.readthedocs.io/en/latest/index.html)
- install python dependencies with pipenv: `pipenv install`
- [install cuda drivers](https://docs.nvidia.com/cuda/cuda-quick-start-guide/index.html#linux)
                        ( https://developer.nvidia.com/cuda-downloads)
- [install nvidia-container-toolkit](https://github.com/NVIDIA/nvidia-docker#ubuntu-160418042004-debian-jessiestretchbuster)


## Running:

- `mkdir data` (The data folder should be at the root of the LilFloAssessment Folder)
- Put bag files in data directory
- `pipenv run python top_level_runner.py`

## Architecture

The idea is to have modules which represent a single type of processing
on the bag file. Processing is done by reading from the bag file and writing
the results back into it.

NOTE: Always backup your bag files in advance.

The modules are called in sequence by `top_level_runner.py`.

### Adding new modules

The system is designed to make it easy to add new modules.

1. Create a new new ros package in its own folder under the root.
   Be sure that your new ros package package file is up to date
2. Create a dockerfile to run your new module
3. Create a new python file with a class that extends the `ProcessingStep` class from the
   top_level_runner
4. Create a new `__init__.py` file to expose your new class
5. Import your new class into the `top_level_runner` and add
   it to the OPERATIONS array.

## Troubleshooting

The first thing to do is to see if you are hooking up to your
gpu well. Run: `pipenv run python test_gpu.py`

## TODO:

- [ ] Add more modules
- [ ] Currently using a PR branch for docker, when merged need to switch
