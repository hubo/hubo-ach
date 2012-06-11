/* Can_debug
 *
 * LINUX CAN device driver source
 */
#include "defs.h"


/* default debugging level */

#if DEBUG
unsigned int   dbgMask  = 7;
#else
unsigned int   dbgMask  = 0;
#endif


