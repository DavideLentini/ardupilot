#include <AP_gtest.h>
#include <AP_HAL/UARTDriver.h>
#include <AP_Common/NMEA.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class DummyUart: public AP_HAL::UARTDriver {
public:
    void begin(uint32_t baud) override {  };
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {  };
    void end() override {  };
    void flush() override {  };
    bool is_initialized() override { return true; };
    bool tx_pending() override { return false; };
    uint32_t available() override { return 1; };
    uint32_t txspace() override { return _txspace; };
    int16_t read() override { return 1; };

    bool discard_input() override { return true; };
    size_t write(uint8_t c) override { return 1; };
    size_t write(const uint8_t *buffer, size_t size) override { return 1; };
    void set_txspace(uint32_t space) {
        _txspace = space;
    }
    uint32_t _txspace;
};

static DummyUart test_uart;


TEST(NMEA, Printf)
{
    // test not enought space
    test_uart.set_txspace(2);
    EXPECT_FALSE(nmea_printf(&test_uart, "TEST"));
    // normal test
    test_uart.set_txspace(9);
    EXPECT_TRUE(nmea_printf(&test_uart, "TEST"));
}

AP_GTEST_MAIN()
