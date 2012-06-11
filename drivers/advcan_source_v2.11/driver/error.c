/* Can_error
 *
 * advcan -- LINUX CAN device driver source
 *
 */
#include "defs.h"

int Can_errno = 0;


int Error(int err)
{
    Can_errno = err;
    return 0;
}
