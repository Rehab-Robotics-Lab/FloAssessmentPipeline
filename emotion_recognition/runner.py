import os
from top_level_runner import ProcessingStep


class Process(ProcessingStep):
    def __init__(self, *args):
        super(Process, self).__init__(*args)
        self.type = 'Emotion Recognition'

    def run(self, file, topic):
        args = ['bag_file:={}'.format(file),
                'topic_name:={}'.format(topic),
                'model_weights:=/home/catkin_ws/src/emotion_recognition/src/model.h5'
		'cascade_file:= /home/catkin_ws/src/emotion_recognition/src/haarcascade_frontalface_default.xml']
        super(Process, self)._run(args)
