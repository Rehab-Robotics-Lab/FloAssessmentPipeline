from __future__ import print_function
import os
import logging
import sys
import docker
import emotion_recognition


def get_file_path():
    return os.path.dirname(os.path.realpath(__file__))


class ProcessingStep(object):
    def __init__(self, bag_file, topic_name):
        self.bag_file = bag_file
        self.topic = topic_name
        self.client = docker.from_env()
        self.type = 'parent'
        self.path = self.get_file_path()
        logging.info('Building Dockerfile at: %s', self.path)
        self.image = self.client.images.build(path=self.path)[0]
        logging.info('Done building')

    def _run(self,  args):
        target_mount = '/home/data/'
        source_mount = 'data'
        mounts = [
            docker.types.Mount(
                target=target_mount,
                source=source_mount
            )
        ]
        logging.info('connection host file: %s to docker file: %s',
                     source_mount, target_mount)

        ret = self.client.containers.run(
            self.image.id,
            args, mounts=mounts)
        logging.debug(ret)

    def run(self):
        pass

    def get_file_path(self):
        return os.path.dirname(os.path.realpath(
            sys.modules[self.__class__.__module__].__file__))


if __name__ == "__main__":
    OPERATIONS = [emotion_recognition.runner.Process]
    TOPICS = ["/upper_realsense/color/image_raw",
              "/lower_realsense/color/image_raw"
              ]
    logging.basicConfig(level=logging.DEBUG)
    for file in os.listdir(os.path.join(get_file_path(), 'data')):
        for topic in TOPICS:
            for op in OPERATIONS:
                process_manager = op(file, topic)
                logging.info('Running: %s\n\tbag file: %s\n\ttopic: %s',
                             process_manager.type, file, topic)
                process_manager.run(file, topic)
                logging.info('Done processing')
