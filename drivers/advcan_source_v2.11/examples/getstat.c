/*
 * getstat.c implements showCANStat(fd) 
 */

void		showCANStat(int fd);

void showCANStat(int fd)
{
#if CAN4LINUXVERSION > 0x0300
	CanStatusPar_t status;
#else
	CanSja1000Status_par_t status;
#endif
	char *m;
   ioctl(fd, CAN_IOCTL_STATUS, &status);
   switch(status.type) 
   {
      case  CAN_TYPE_SJA1000:
         m = "sja1000";
         break;
      case  CAN_TYPE_FlexCAN:
         m = "FlexCan";
         break;
      case  CAN_TYPE_TouCAN:
         m = "TouCAN";
         break;
      case  CAN_TYPE_82527:
         m = "I82527";
         break;
      case  CAN_TYPE_TwinCAN:
         m = "TwinCAN";
         break;
    	case CAN_TYPE_UNSPEC:
      default:
         m = "unknown";
         break;
   }

   printf(":: %s %4d %2d %2d %2d %2d %2d tx:%3d/%3d: rx:%3d/%3d:\n",
      m,
      status.baud,
      status.status,
      status.error_warning_limit,
      status.rx_errors,
      status.tx_errors,
      status.error_code,
      status.tx_buffer_size,
      status.tx_buffer_used,
      status.rx_buffer_size,
      status.rx_buffer_used);
}
