#include <modm/board.hpp>

#include <etl/string.h>
#include <etl/string_stream.h>

#include <UbloxDriver.hpp>

using namespace Board;

// in main
int main() {
    Board::initialize();
    Leds::setOutput();

    Usart2::connect<GpioD5::Tx, GpioD6::Rx>();
    // Usart2::initialize<Board::SystemClock, 9600_Bd>();
    Usart2::initialize<Board::SystemClock, 38400_Bd>();
    MODM_LOG_INFO << "Initialized" << modm::endl;

    ublox::UbloxDriver ublox;

    ublox.registerNavStatusCallback([](ublox::ubx_nav_status_t status) {
         MODM_LOG_INFO << "Nav Status Received: " << modm::endl << " - gpsFix: " << status.gpsFix << modm::endl;
     });
     ublox.registerNavPvtCallback([](ublox::ubx_nav_pvt_t pvt) {
         etl::string<40> latlon;
         etl::string_stream ss(latlon);
         ss << etl::setprecision(7) << " - lat: " << static_cast<float>(pvt.lat * 1e-7) << "\n"
            << " - lon: " << static_cast<float>(pvt.lon * 1e-7);
         MODM_LOG_INFO << "Nav Pvt Received: " << modm::endl << ss.str().c_str() << modm::endl;
     });
     ublox.registerNavClockCallback([](ublox::ubx_nav_clock_t clock) {
         MODM_LOG_INFO << "Nav Clock Received: " << modm::endl
                       << " - iTOW: " << clock.iTOW << modm::endl
                       << " - clock bias: " << clock.clkB << modm::endl;
     });
    ublox.registerNavCovCallback([](ublox::ubx_nav_cov_t cov) {
        MODM_LOG_INFO << "Nav Cov Received: " << modm::endl
                      << " - [valid=" << cov.posCovValid << "]"
                      << "CovN²: " << cov.posCovNN << ", covE²: " << cov.posCovEE << ", covD²: " << cov.posCovDD
                      << modm::endl
                      << " - [valid=" << cov.velCovValid << "]"
                      << "covVn²: " << cov.velCovNN << ", covVe²: " << cov.velCovEE << ", covVd²: " << cov.velCovDD
                      << modm::endl;
    });

    uint8_t data;
    while (true) {
        if (Usart2::read(&data, 1)) {
            ublox.next(data);
        }
    }

    return 0;
}
