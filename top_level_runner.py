"""Control code for the entire post processing and analysis
pipeline for the Lil'Flo project experiments"""

from __future__ import print_function
import os
import logging
import sys
import docker
import emotion_recognition


def get_file_path():
    """Get the path of the calling file.

    Note: This doesn't work quite right when using inheritance
    and imports, so be sure to test.
    """
    return os.path.dirname(os.path.realpath(__file__))


class ProcessingStep(object):
    def __init__(self, bag_file_nm, topic_name):
        self.bag_file = bag_file_nm
        self.topic = topic_name
        self.client = docker.from_env(version='1.40')
        self.type = 'parent'
        self.path = self.get_file_path()
        logging.info('Building Dockerfile at: %s', self.path)
        ret = self.client.images.build(path=self.path)
        self.image = ret[0]
        logging.debug(list(ret[1]))
        logging.info('Done building: %s', self.image.id)

    def _run(self, args):
        """Run the internals of setting up and passing data to the
        docker container for this analysis step.

        The intention is that this is used in the run method of a child
        class.

        Args:
            args: The args to pass to the docker run command
        """
        target_mount = '/home/data/'
        source_mount = os.path.join(get_file_path(), 'data')
        mounts = [
            docker.types.Mount(
                target=target_mount,
                source=source_mount,
                type='bind'
            )
        ]
        logging.info('connection host file: %s to docker file: %s',
                     source_mount, target_mount)
        logging.info('running with args: %s', args)

        ret = self.client.containers.run(
            self.image.id,
            args,
            mounts=mounts,
            stdout=True,
            stderr=True,
            device_requests=[
                docker.types.DeviceRequest(
                    count=-1,
                    capabilities=[['gpu']])
            ]
        )
        logging.info(ret)

    def run(self):
        """Run this anlysis step.

        It is expected that any inheriting classes will override
        this function and make use of _run.
        """

    def get_file_path(self):
        """Get the file path of the actual calling file.

        This is useful for classes that inherit from this one to
        get the actual path of the file that the code is in."""
        return os.path.dirname(os.path.realpath(
            sys.modules[self.__class__.__module__].__file__))


if __name__ == "__main__":
    OPERATIONS = [emotion_recognition.runner.Process]
    TOPICS = ["/upper_realsense/color/image_raw",
              "/lower_realsense/color/image_raw"
              ]
    logging.basicConfig(level=logging.INFO)
    for file in os.listdir(os.path.join(get_file_path(), 'data')):
        bag_file = os.path.join('/home/data', file)
        for topic in TOPICS:  # TODO: this migh be better handled in a config file
                             #      or in the sub class runner
            for op in OPERATIONS:
                process_manager = op(file, topic)
                logging.info('Running: %s\n\tbag file: %s\n\ttopic: %s',
                             process_manager.type, bag_file, topic)
                process_manager.run(bag_file, topic)
                logging.info('Done processing')
