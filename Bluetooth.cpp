#include "Bluetooth.h"
#include "ev3api.h"

#ifdef EnableBluetooth
FILE *bt = ev3_serial_open_file(EV3_SERIAL_BT);
#endif