#include "anoSerial.hpp"

#include <sys/stat.h>

#include <chrono>

#include "AnoPTv8ExAPI.hpp"
#include "anoSys.hpp"
#include "anolink.hpp"

AnoSerial::AnoSerial(const std::string name, const uint32_t baud, const uint8_t linktype, void(*fp)(uint16_t, uint8_t)) : serial::Serial() {
    m_runThread = nullptr;
    m_txBufInAddr = 0;
    m_txBufOutAddr = 0;

    if (!ttyExists(name)) {
        RCLCPP_ERROR(AnoLink::m_self->get_logger(), "serial %s not exist!", name.c_str());
        return;
    }
    try {
        m_linkType = linktype;
        rxFuncPtr = fp;
        setPort(name);
        setBaudrate(baud);
        setBytesize(serial::eightbits);
        setStopbits(serial::stopbits_one);
        setParity(serial::parity_none);
        open();
        m_runThread = new std::thread(&AnoSerial::runThread, this);
        RCLCPP_INFO(AnoLink::m_self->get_logger(), "serial %s open ok!", name.c_str());
        return;
    }
    catch (serial::IOException& e) {
        RCLCPP_ERROR(AnoLink::m_self->get_logger(), "serial %s open err!%s", name.c_str(), e.what());
        return;
    }
}

AnoSerial::~AnoSerial() {}

void AnoSerial::addTxData(const uint8_t* buf, const uint16_t len) {
    for (int i = 0; i < len; i++) {
        m_txBuf[m_txBufInAddr++] = buf[i];
        if (m_txBufInAddr >= SERIAL_TXBUFSIZE) m_txBufInAddr = 0;
    }
}

bool AnoSerial::ttyExists(const std::string& ttyPath) {
    struct stat buffer;
    int result = stat(ttyPath.c_str(), &buffer);
    return result == 0;
}

void AnoSerial::runThread(void) {
    while (true) {
        try {
            int _rxlen = available();
            if (_rxlen > 0) {
                uint8_t* _buf = new uint8_t[_rxlen];
                read(_buf, _rxlen);
                for (int i = 0; i < _rxlen; i++) {
                    rxFuncPtr(m_linkType, _buf[i]);
                    //AnoPTv8HwRecvByte(LT_U1, _buf[i]);
                }
                //RCLCPP_INFO(AnoLink::m_self->get_logger(), "com rx: %s", toHex(_buf, _rxlen).c_str());
                delete(_buf);
            }
            if (m_txBufInAddr != m_txBufOutAddr) {
                int _len = 0;
                if (m_txBufInAddr > m_txBufOutAddr)
                    _len = m_txBufInAddr - m_txBufOutAddr;
                else
                    _len = SERIAL_TXBUFSIZE + m_txBufInAddr - m_txBufOutAddr;
                uint8_t* _tbuf = new uint8_t[_len];
                for (int i = 0; i < _len; i++) {
                    _tbuf[i] = m_txBuf[m_txBufOutAddr++];
                    if (m_txBufOutAddr >= SERIAL_TXBUFSIZE) m_txBufOutAddr = 0;
                }
                write(_tbuf, _len);
                delete(_tbuf);
                // RCLCPP_INFO(AnoLink::m_self->get_logger(), "udp tx: %s", toHex(_t, _len).c_str());
            }
        }
        catch (serial::IOException& e) {
            RCLCPP_ERROR(AnoLink::m_self->get_logger(), "serial err!%s", e.what());
            return;
        }
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
}
