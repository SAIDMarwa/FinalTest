#ifndef LORAWANNETWORK_H_
#define LORAWANNETWORK_H_

#include <omnetpp.h>

using namespace omnetpp;

class LoRaWANNetwork : public cSimpleModule {
  protected:

    int main();
    virtual void initialize() override;
};

#endif /* LORAWANNETWORK_H_ */
