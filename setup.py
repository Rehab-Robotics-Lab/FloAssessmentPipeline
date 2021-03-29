import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name="LilFloAssessmentPipeline",
    version="0.0.1",
    author="Michael Sobrepera",
    author_email="mjsobrep@seas.upenn.edu",
    description="For running the assesment of the LilFlo experiments",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Rehab-Robotics-Lab/LilFloAssessmentPipeline",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 2.7",
        "Operating System :: OS Independent",
    ],
    python_requires='>=2.7',
)
