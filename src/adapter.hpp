//
// Created by think on 10/29/24.
//

#ifndef SRI_FTSENSOR_SDK_ADAPTER_HPP
#define SRI_FTSENSOR_SDK_ADAPTER_HPP

#include <stddef.h>

namespace SRI {

    class Adapter {
    public:
        Adapter() = default;
        virtual ~Adapter() = 0; // Pure virtual deconstructor for polymorphism

        /// \brief Connect the sensor
        /// \return bool true if connected successfully
        virtual bool connect() = 0;
        /// \brief Disconnect the sensor
        virtual void disconnect() = 0;

        /// Write Data Buffer to Sensor
        /// \param buf The data need to send
        /// \return       The number of chars have been sent
        virtual size_t write(char* buf, size_t n) = 0;

        /// Read Data Buffer from Sensor
        /// \param[out] buf The data received
        /// \return            The number of chars have been received
        virtual size_t read(char* buf, size_t n) = 0;



    };

} // SRI

#endif //SRI_FTSENSOR_SDK_ADAPTER_HPP
