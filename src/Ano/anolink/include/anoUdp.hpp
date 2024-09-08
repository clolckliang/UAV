#ifndef __ANOUDP_HPP__
#define __ANOUDP_HPP__
#include <arpa/inet.h>

#include <iostream>
#include <thread>
class AnoUdp {
#define ANOUDP_RXBUFSIZE 20000
#define ANOUDP_TXBUFSIZE 10000
public:
    AnoUdp(const uint16_t port, const uint8_t linktype, void(*fp)(uint16_t, uint8_t));
    void addTxData(const uint8_t* buf, const uint16_t len);
    bool sendBuf(const uint8_t* data, const uint16_t len);
    bool sendBuf(const uint32_t ip, const uint16_t port, const uint8_t* data, const uint16_t len);

private:
    uint8_t m_linkType;
    int m_sockfd;
    struct sockaddr_storage m_remoteAddr;
    uint8_t m_rxBuf[ANOUDP_RXBUFSIZE];
    uint8_t m_txBuf[ANOUDP_TXBUFSIZE];
    uint16_t m_txBufInAddr, m_txBufOutAddr;

    std::thread* m_runThread;
    void (*rxFuncPtr)(uint16_t, uint8_t);
    void runThread(void);
};
#endif