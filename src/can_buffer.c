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
                 int expect_reply) {

  if (buf->size >= CAN_BUFFER_MAX_SIZE) { 

    return 0;

  } else {

    ++buf->size;

    can_buf_head(buf)->frame = *frame;
    can_buf_head(buf)->expect_reply = expect_reply;

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

    return buf->data + ( (buf->head + buf->size) & CAN_BUFFER_MOD_MASK );

  }

}

/* pop a can buf and return 1 on success */
int can_buf_pop(can_buf_t* buf) {

  if (!buf->size) {
    
    return 0;

  } else {

    buf->head = ( (buf->head + 1) & CAN_BUFFER_MOD_MASK );
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
