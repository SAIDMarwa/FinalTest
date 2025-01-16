#ifndef LORAWANNETWORK_H_
#define LORAWANNETWORK_H_

#include <omnetpp.h>

using namespace omnetpp;

namespace LoRaWANNetwork {

class LoRaWANNetwork : public cSimpleModule
{
public:
    virtual ~LoRaWANNetwork() {}

protected:

    virtual void initialize() override;
     int main();

};

}

#endif /* LORAWANNETWORK_H_ */
