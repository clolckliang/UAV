#ifndef __ANOSERIAL_HPP__
#define __ANOSERIAL_HPP__
#include "serial/serial.h"
#include <thread>

class AnoSerial : public serial::Serial {
#define SERIAL_TXBUFSIZE 2000

public:
    AnoSerial(const std::string name, const uint32_t baud, const uint8_t linktype, void(*fp)(uint16_t, uint8_t));
    ~AnoSerial();

    void addTxData(const uint8_t* buf, const uint16_t len);
    static bool ttyExists(const std::string& ttyPath);

private:
    uint8_t m_linkType;
    uint8_t m_txBuf[SERIAL_TXBUFSIZE];
    uint16_t m_txBufInAddr, m_txBufOutAddr;

    std::thread* m_runThread;
    void (*rxFuncPtr)(uint16_t, uint8_t);
    void runThread(void);
};

#endif