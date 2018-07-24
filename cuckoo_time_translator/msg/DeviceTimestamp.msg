####
# This message's purpose is to support the user when tuning device time translation.
# It collects all the relevant input and output such that a mutual comparison of various configuration (algorithms / parametes) and the current configuration in the driver is possible given a single bag file.
# It can also be used to inspect the performance of the current configuration.
####

# The standard header's timestamp should be the device time translated into local / system time, potentially plus an artificial offset (see below)
Header header

# the device clock stamp of the event in the device (e.g. LIDAR beam fire)
uint64 event_stamp
# the device clock stamp of the transmission event (e.g. shortly before the transmission (sending) starts) if available separately. Otherwise equal to event_stamp.
uint64 transmit_stamp

# the local / system clock time of when the device's message was received.
time receive_time

# the artifical offeset in seconds that was added to the header's timestamp after translating the hardware time (e.g. to correct an known transport delay (would be negative then))
float32 offset_secs

# correction_secs := receiveTime - translatedTime in seconds (without the offset above)
float32 correction_secs

# the algorithm employed to compute the timestamp in the header
uint8 filter_algorithm
