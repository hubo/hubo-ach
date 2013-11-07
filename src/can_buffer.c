#include "can_buffer.h"

/* clear buffer */
void can_buf_clear(can_buf_t* buf) {
    buf->head = 0;
    buf->size = 0;
}

/* return true if empty */
int can_buf_isempty(const can_buf_t* buf) {
    return (buf->size == 0);
}

/* return true if full */
int can_buf_isfull(const can_buf_t* buf) {
    return (buf->size == CAN_BUFFER_MAX_SIZE);
}

/* push a message into the buffer and return 1 on success */
int can_buf_push(can_buf_t* buf, 
                 const struct can_frame* frame, 
                 int sequence_no) {

    if (buf->size >= CAN_BUFFER_MAX_SIZE) { 

        return 0;

    } else {

        ++buf->size;

        can_tagged_frame_t* tf = can_buf_tail(buf);

        tf->frame = *frame;
        tf->sequence_no = sequence_no;

        return 1;
    
    }

}

/* get the message at the head (least recently inserted) return NULL if empty */
can_tagged_frame_t* can_buf_head(can_buf_t* buf) {

    if (!buf->size) { 

        return NULL;

    } else {

        return buf->data + buf->head;

    }
    
}

/* get the message at the tail (most recently inserted) return NULL if empty */
can_tagged_frame_t* can_buf_tail(can_buf_t* buf) {

    if (!buf->size) {

        return NULL;

    } else {

        // presumably some of these casts are unnecessary
        return buf->data + ( (buf->head + buf->size - (size_t)1) & (size_t)CAN_BUFFER_MOD_MASK );

    }

}

/* pop a can buf and return 1 on success */
int can_buf_pop(can_buf_t* buf) {

    if (!buf->size) {
    
        return 0;

    } else {

        // presumably some of these casts are unnecessary
        buf->head = ( (buf->head + (size_t)1) & (size_t)CAN_BUFFER_MOD_MASK );
        --buf->size;

        return 1;

    }
      

}


/* Local Variables:                          */
/* mode: c                                   */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
