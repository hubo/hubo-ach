#ifndef _CAN_BUFFER_H_
#define _CAN_BUFFER_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/

enum {

    /* Maximum size of ring buffer, should be power of 2 */
    CAN_BUFFER_MAX_SIZE = 256,

    /* Bitwise and with this equals number mod max size */
    CAN_BUFFER_MOD_MASK = CAN_BUFFER_MAX_SIZE - 1

};


/******************************************************************************/

/* tag a can frame with whether it expects a reply */
typedef struct can_tagged_frame {

    /* the actual frame */
    struct can_frame frame;

    /* for debugging */
    int              sequence_no;

} can_tagged_frame_t;

/******************************************************************************/

/* circular buffer of can_tagged_frame */
typedef struct can_buf {

    /* index of head in data */
    size_t head;

    /* count */
    size_t size;

    /* actual can frames */
    can_tagged_frame_t data[CAN_BUFFER_MAX_SIZE];

} can_buf_t;

/******************************************************************************/

/* clear buffer */
void can_buf_clear(can_buf_t* buf);

/* return true if empty */
int can_buf_isempty(const can_buf_t* buf);

/* return true if full */
int can_buf_isfull(const can_buf_t* buf);

/* push a message to the tail of the buffer and return 1 on success. */
int can_buf_push(can_buf_t* buf, 
                 const struct can_frame* frame, 
                 int sequence_no);

/* get the message at the head (least recently inserted) return NULL if empty */
can_tagged_frame_t* can_buf_head(can_buf_t* buf);

/* get the message at the tail (most recently inserted) return NULL if empty */
can_tagged_frame_t* can_buf_tail(can_buf_t* buf);

/* pop a can buf from the head and return 1 on success */
int can_buf_pop(can_buf_t* buf);

/******************************************************************************/


#ifdef __cplusplus
}
#endif

#endif

/* Local Variables:                          */
/* mode: c                                   */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
