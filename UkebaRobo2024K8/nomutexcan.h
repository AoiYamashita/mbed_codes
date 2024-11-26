#ifndef NOMUTEXCAN_H
#define NOMUTEXCAN_H

#include "mbed.h"


class NoMutexCAN: public CAN {
  public:
    NoMutexCAN(PinName rd, PinName td): CAN(rd, td) {}
    NoMutexCAN(PinName rd, PinName td, int hz): CAN(rd, td, hz) {}
    virtual ~NoMutexCAN() {};
    void lock() override {};
    void unlock() override {};
};

#endif  // NOMUTEXCAN_H