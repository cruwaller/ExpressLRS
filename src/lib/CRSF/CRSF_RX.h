#ifndef CRSF_RX_H_
#define CRSF_RX_H_

#include "CRSF.h"

#define ELRS_BOOT_CMD_DEST 0x62
#define ELRS_BOOT_CMD_ORIG 0x6c

#define CRSF_v3_PORT_ID 0x5 // 3bits = 0 ... 7


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
    void sendFrameToFC(uint8_t *buff, uint8_t size) const;
    void processPacket(uint8_t const *data);
    uint8_t TypeIsValid(uint8_t const type) const {
        switch (type) {
            /* RX */
            case CRSF_FRAMETYPE_COMMAND:
            case CRSF_FRAMETYPE_DEVICE_INFO:
            case CRSF_FRAMETYPE_BATTERY_SENSOR:
            case CRSF_FRAMETYPE_GPS:
            case CRSF_FRAMETYPE_MSP_RESP:
                return 1;
        }
        return 0;
    }

    uint8_t new_baud_ok;
};

#endif /* CRSF_RX_H_ */
