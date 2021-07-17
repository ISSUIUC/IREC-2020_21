#ifndef DATALOG_H
#define DATALOG_H

#include <SD.h>

typedef struct {
    uint32_t usec;
    uint16_t value1;
    uint16_t value2;
    uint16_t value3;
    uint16_t errors;
} FifoItem_t;

void init_dataLog(File* dataFile);

void logData(File* dataFile, FifoItem_t* data);

#endif
