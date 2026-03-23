#pragma once
#include "SinglePortModule.h"
#include "concurrency/OSThread.h"

/**
 * Automated mesh packet sender.
 */
class AutoNeighborMessage : public SinglePortModule, public concurrency::OSThread
{
  public:
    AutoNeighborMessage();

  protected:
    virtual int32_t runOnce() override;
};

extern AutoNeighborMessage *autoNeighborMessage;