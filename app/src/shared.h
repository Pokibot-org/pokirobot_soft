#ifndef SHARED_H
#define SHARED_H

#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include "utils.h"

// declare here every global device that is not defined by its own task

extern uart_hdb_t steppers_uart_hdb;

int shared_init(void);

#endif // SHARED_H
