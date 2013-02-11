#!/usr/bin/env python
import hubo_ach
import ach

ha = hubo_ach



c = ach.Channel(ha.HUBO_CHAN_REF_NAME)
c.flush()
b = ha.HUBO_REF()

while(1):
    [status, framesize] = c.get(b, wait=True, last=False)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME:
        print b.ref[0]
    else:
        raise ach.AchException( c.result_string(status) )
c.close()

