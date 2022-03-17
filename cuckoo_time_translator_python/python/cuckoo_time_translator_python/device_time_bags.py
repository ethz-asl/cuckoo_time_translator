import os

import pickle
import rosbag
import numpy as np

from .timestamp_series import TimestampSeries
from .timestamp_series import TimestampSeries
from .tools import *

DefaultTopic = '/device_time'


def guessTopics(bagFile):
  with rosbag.Bag(bagFile) as bag:
    found = []
    for topic, value in bag.get_type_and_topic_info().topics.items() :
      if value.msg_type == 'cuckoo_time_translator/DeviceTimestamp':
        found.append(topic)

  return found


class DeviceTimeStream():

  def __init__(self, bagFile, topic, invalidate=False):
    eventsFile = os.path.realpath(bagFile) + topic.replace("/", "_") + ".p"
    if os.path.exists(eventsFile):
        if invalidate:
            info("Removing " + eventsFile)
        else:
            verbose("Loading from " + eventsFile)
            self.__dict__ = pickle.load(open(eventsFile, "rb"))
            return

    bag = rosbag.Bag(bagFile)

    self.translated_hw_times = TimestampSeries()
    self.translated_hw_times_without_offset = TimestampSeries()
    self.receive_times = TimestampSeries()
    self.raw_hw_times = TimestampSeries()
    for topic, msg, t in bag.read_messages(topics=[topic]):
      self.translated_hw_times.append(msg.header.stamp.to_sec())
      self.translated_hw_times_without_offset.append(msg.header.stamp.to_sec() - msg.offset_secs)
      self.receive_times.append(msg.receive_time.to_sec())
      self.raw_hw_times.append(float(msg.event_stamp))

    self.zeroOffsetAllTheTime = np.linalg.norm(np.array(self.translated_hw_times_without_offset) - np.array(self.translated_hw_times)) < 1e-9

    bag.close()
    # Save data to file
    pickle.dump(self.__dict__, open(eventsFile, "wb"))
