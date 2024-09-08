#include "anoUdp.hpp"

#include "anoSys.hpp"
#include "anolink.hpp"

AnoUdp::AnoUdp(const uint16_t port, const uint8_t linktype, void(*fp)(uint16_t, uint8_t)) {
    m_runThread = nullptr;
    m_linkType = linktype;
    rxFuncPtr = fp;
    m_sockfd = -1;
    m_txBufInAddr = 0;
    m_txBufOutAddr = 0;

    struct sockaddr_in _addr;
    _addr.sin_addr.s_addr = htonl(INADDR_ANY);
    _addr.sin_family = AF_INET;
    _addr.sin_port = htons(port);

    m_sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (m_sockfd < 0) {
        RCLCPP_ERROR(AnoLink::m_self->get_logger(), "create udp socket err!");
        return;
    }

    int ret = bind(m_sockfd, (struct sockaddr*)&_addr, sizeof(_addr));
    if (ret < 0) {
        RCLCPP_ERROR(AnoLink::m_self->get_logger(), "bind udp socket err!");
        return;
    }
    m_runThread = new std::thread(&AnoUdp::runThread, this);
    RCLCPP_INFO(AnoLink::m_self->get_logger(), "udp init ok! port:%d", port);
}

void AnoUdp::addTxData(const uint8_t* buf, const uint16_t len) {
    for (int i = 0; i < len; i++) {
        m_txBuf[m_txBufInAddr++] = buf[i];
        if (m_txBufInAddr >= ANOUDP_TXBUFSIZE) m_txBufInAddr = 0;
    }
}

bool AnoUdp::sendBuf(const uint8_t* data, const uint16_t len) {
    return sendBuf(((struct sockaddr_in*)&m_remoteAddr)->sin_addr.s_addr, ((struct sockaddr_in*)&m_remoteAddr)->sin_port, data, len);
}

bool AnoUdp::sendBuf(const uint32_t ip, const uint16_t port, const uint8_t* data, const uint16_t len) {
    struct sockaddr_in _addr;
    _addr.sin_addr.s_addr = ip;
    _addr.sin_port = port;
    _addr.sin_family = AF_INET;
    if (m_sockfd < 0) return false;
    int ret = sendto(m_sockfd, data, len, 0, (struct sockaddr*)&_addr, sizeof(_addr));
    if (ret < 0)
        return false;
    else
        return true;
}

void AnoUdp::runThread(void) {
    int rx_len = 0;
    socklen_t socklen = sizeof(m_remoteAddr);
    uint8_t txbuf[ANOUDP_TXBUFSIZE];

    while (true) {
        rx_len = recvfrom(m_sockfd, m_rxBuf, ANOUDP_RXBUFSIZE, MSG_DONTWAIT, (struct sockaddr*)&m_remoteAddr, &socklen);
        if (rx_len > 0) {
            // RCLCPP_INFO(AnoLink::m_self->get_logger(), "udp rx: %s", toHex(m_rxBuf, rx_len).c_str());
            for (int i = 0; i < rx_len; i++) {
                rxFuncPtr(m_linkType, m_rxBuf[i]);
                //AnoPTv8HwRecvByte(LT_UDP_UPPC, m_rxBuf[i]);
            }
        }
        if (m_txBufInAddr != m_txBufOutAddr) {
            int _len = 0;
            if (m_txBufInAddr > m_txBufOutAddr)
                _len = m_txBufInAddr - m_txBufOutAddr;
            else
                _len = ANOUDP_TXBUFSIZE + m_txBufInAddr - m_txBufOutAddr;
            if (_len < ANOUDP_TXBUFSIZE) {
                for (int i = 0; i < _len; i++) {
                    txbuf[i] = m_txBuf[m_txBufOutAddr++];
                    if (m_txBufOutAddr >= ANOUDP_TXBUFSIZE) m_txBufOutAddr = 0;
                }
                sendBuf(txbuf, _len);
            }
            // RCLCPP_INFO(AnoLink::m_self->get_logger(), "udp tx: %s", toHex(_t, _len).c_str());
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}
