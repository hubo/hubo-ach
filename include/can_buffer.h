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

/* two different priorities for messages */
typedef enum can_chan_buf_priority {
  CAN_PRIORITY_HI = 0,
  CAN_PRIORITY_LO = 1,
  CAN_NUM_PRIORITY = 2,
} can_chan_buf_priority_t;

/******************************************************************************/

/* tag a can frame with whether it expects a reply */
typedef struct can_tagged_frame {

    /* the actual frame */
    struct can_frame frame;

    /* non-zero means we expect a reply. */
    int              expect_reply;

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

/* all the information we need to manage a single can channel */

typedef struct can_chan_buf {

    /* file descriptor */
    int fd;

    /* two buffers */
    can_buf_t buffers[CAN_NUM_PRIORITY];
  
} can_chan_buf_t;


/******************************************************************************/

/* clear buffer */
void can_buf_clear(can_buf_t* buf);

/* return true if empty */
int can_buf_isempty(const can_buf_t* buf);

/* return true if full */
int can_buf_isfull(const can_buf_t* buf);

/* push a message into the buffer and return 1 on success. */
int can_buf_push(can_buf_t* buf, 
                 const struct can_frame* frame, 
                 int expect_reply,
                 int sequence_no);

/* get the message at the head (least recently inserted) return NULL if empty */
can_tagged_frame_t* can_buf_head(can_buf_t* buf);

/* get the message at the tail (most recently inserted) return NULL if empty */
can_tagged_frame_t* can_buf_tail(can_buf_t* buf);

/* pop a can buf and return 1 on success */
int can_buf_pop(can_buf_t* buf);

/******************************************************************************/

/*
void can_chan_buf_clear(can_chan_buf_t* cbuf);

int can_chan_buf_isempty(const can_chan_buf_t* cbuf);

int can_chan_buf_isfull(const can_chan_buf_t* cbuf,
                        can_chan_buf_priority_t priority);

int can_chan_buf_push(can_chan_buf_t* cbuf,
                      can_chan_buf_priority_t priority,
                      const struct can_frame* frame,
                      int expect_reply);

can_tagged_frame_t* can_chan_buf_head(can_chan_buf_t* buf);

can_tagged_frame_t* can_chan_buf_tail(can_chan_buf_t* buf,
                                      can_chan_buf_priority_t priority);
*/

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
