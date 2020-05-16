# FloAssessmentPipeline

This is the pipeline for assessing patient function based on data from the FloSystem

## Setting up:

- install docker
- [install pipenv](https://pipenv-fork.readthedocs.io/en/latest/index.html)
- install python dependencies with pipenv: `pipenv install`

## Running:

- `mkdir data`
- Put bag files in data directory
- `pipenv run python top_level_runner.py`
