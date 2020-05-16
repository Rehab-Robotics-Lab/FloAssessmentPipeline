import os
from top_level_runner import ProcessingStep


class Process(ProcessingStep):
    def __init__(self, *args):
        super(Process, self).__init__(*args)
        self.type = 'Emotion Recognition'

    def run(self, file, topic):
        args = ['bag_file:="{}"'.format(file),
                'topic_name:="{}"'.format(topic),
                'model_weights:="model.h5"']
        super(Process, self)._run(args)
