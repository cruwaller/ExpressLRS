#ifndef CRSF_RX_H_
#define CRSF_RX_H_

#include "CRSF.h"

#define CRSF_v3_PORT_ID 0xCF


class CRSF_RX : public CRSF
{
public:
    CRSF_RX(HwSerial &dev) : CRSF(&dev) {}

    void Begin(void);

    void handleUartIn(void);

    void sendRCFrameToFC(rc_channels_rx_t * channels) const;
    void LinkStatisticsSend(LinkStatsLink_t & stats) const;
    void sendMSPFrameToFC(mspPacket_t & msp) const;

    void negotiate_baud(uint32_t baudrate) const;
    uint8_t negotiate_accepted(void) const {
        return new_baud_ok;
    }

private:
    void sendFrameToFC(uint8_t *buff, uint8_t size, uint8_t poly = 0xD5) const;
    void processPacket(uint8_t const *data);

    uint8_t new_baud_ok;
};

#endif /* CRSF_RX_H_ */
