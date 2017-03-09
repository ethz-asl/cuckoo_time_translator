# foreign time translation
### Algorithms for translating times emitted by foreign clocks into local time

The dominant application is translating hardware timestamps of a sensor into the system time of a receiving computer system.

Currently it covers on way communication based translation only.
I.e. the foreign clock does not answer to specific to any delay estimation / synchronization messages.

### Why is called **translation** rather than **synchronization**? 
Because **synchronization** refers to what one does to clocks to make them run more in sync (as for instance NTP and PTP do to computer clocks), whereas these algorithms are supposed to only **translate** times of one clock into times another clock would have emitted (ideally) at the same physical time and not touch the clocks at all.
