/*
Copyright 2021, Yang Luo"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author
Yang Luo, PHD
Shenyang Institute of Automation, Chinese Academy of Sciences.
 email: luoyang@sia.cn

@Created on: 2021.04.06
@Last modified: 2021.04.09
*/

#ifndef SRI_FTSENSOR_SDK_SRI_SENSOR_H
#define SRI_FTSENSOR_SDK_SRI_SENSOR_H

#include <sri/sensorcomm.hpp>
#include <sri/types.hpp>

#include <memory>
#include <regex>

#include <iostream>
#include <algorithm>
#include <numeric> // std::accumulate
#include <thread>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/format.hpp>
#include <boost/crc.hpp> //crc32
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <chrono>

#include "sri/Logger.h"

// #define BOOST_THREAD_VERSION 5 //using the v5 version of boost::thread
#define DELAY_US 500 // tcp delay in us
#define WAIT_TIME 20 // Max_waiting_time = WAIT_TIME * DELAY_US

namespace SRI
{
    class DllExport FTSensor
    {
    public:
        // explicit FTSensor(SensorComm *pcomm) : commPtr(pcomm)
        // {
        //     if (!commPtr->initialize())
        //     {
        //         std::cout << "Sensor initializing failed" << std::endl;
        //     }
        // }


        explicit FTSensor(SensorComm* pcomm) : commPtr(pcomm), Fx(0), Fy(0), Fz(0), Tx(0), Ty(0), Tz(0), ZeroFx(0),
                                               ZeroFy(0), ZeroFz(0), ZeroTx(0), ZeroTy(0), ZeroTz(0)
        {
            logPtr = Logger::getInstance("FTSensor");

            if (!commPtr->initialize())
            {
                logPtr->error("Sensor initializing failed");
                // std::cout << "Sensor initializing failed" << std::endl;
            }
        }

        boost::posix_time::ptime recorded_time_; // 存储记录的时间

        IpAddr getIpAddress()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return IpAddr();
            }

            commPtr->write(generateCommandBuffer(EIP, "?"));

            int waitTime = 0;
            while (commPtr->available() == 0 && waitTime < WAIT_TIME)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
                waitTime++;
            }

            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, EIP, "?");

            return response;
        }

        bool setIpAddress(const IpAddr& ip)
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(EIP, ip));

            int waitTime = 0;
            while (commPtr->available() == 0 && waitTime < WAIT_TIME)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
                waitTime++;
            }

            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, EIP, ip);

            if (response == "OK")
                return true;
            else
                return false;
        }

        MacAddr getMacAddress()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return MacAddr();
            }

            commPtr->write(generateCommandBuffer(EMAC, "?"));

            int waitTime = 0;
            while (commPtr->available() == 0 && waitTime < WAIT_TIME)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
                waitTime++;
            }

            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, EMAC, "?");

            return response;
        }

        bool setMacAddress(const MacAddr& mac)
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(EMAC, mac));

            int waitTime = 0;
            while (commPtr->available() == 0 && waitTime < WAIT_TIME)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
                waitTime++;
            }

            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, EIP, mac);

            if (response == "OK")
                return true;
            else
                return false;
        }

        GateAddr getGateWay()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return GateAddr();
            }

            commPtr->write(generateCommandBuffer(EGW, "?"));

            int waitTime = 0;
            while (commPtr->available() == 0 && waitTime < WAIT_TIME)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
                waitTime++;
            }

            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, EGW, "?");

            return response;
        }

        bool setGateWay(const GateAddr& gate)
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(EMAC, gate));

            while (commPtr->available() == 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, EMAC, gate);

            if (response == "OK")
                return true;
            else
                return false;
        }

        NetMask getNetMask()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return NetMask();
            }

            commPtr->write(generateCommandBuffer(ENM, "?"));
            while (commPtr->available() == 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, ENM, "?");

            return response;
        }

        bool setNetMask(const NetMask& mask)
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(ENM, mask));

            while (commPtr->available() == 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, ENM, mask);

            if (response == "OK")
                return true;
            else
                return false;
        }

        Gains getChannelGains()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return Gains();
            }

            commPtr->write(generateCommandBuffer(CHNAPG, "?"));

            while (commPtr->available() == 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, CHNAPG, "?");

            std::vector<std::string> resInString;
            try
            {
                boost::split(resInString, response, boost::is_any_of(";"), boost::algorithm::token_compress_on);
            }
            catch (boost::bad_lexical_cast& e)
            {
                logPtr->error("Error in boost::split()");
                // std::cout << "ERROR::FTSensor::getChannelGains():" << e.what() << std::endl;
            }
            Gains gains;

            for (auto& res : resInString)
            {
                gains.push_back(std::stof(res));
            }

            return gains;
        }

        SampleRate getSamplingRate()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return SampleRate();
            }

            commPtr->write(generateCommandBuffer(SMPR, "?"));

            while (commPtr->available() == 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, SMPR, "?");

            return std::stoi(response);
        }

        bool setSamplingRate(SampleRate rate)
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(SMPR, boost::lexical_cast<std::string>(rate)));

            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, SMPR, boost::lexical_cast<std::string>(rate));

            if (response == "OK")
                return true;
            else
                return false;
        }

        Voltages getExcitationVoltages()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return Voltages();
            }

            commPtr->write(generateCommandBuffer(EXMV, "?"));

            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, EXMV, "?");

            std::vector<std::string> resInString;
            try
            {
                boost::split(resInString, response, boost::is_any_of(";"), boost::algorithm::token_compress_on);
            }
            catch (boost::bad_lexical_cast& e)
            {
                logPtr->error("Error in boost::split()");
                // std::cout << "ERROR::FTSensor::getExcitationVoltages():" << e.what() << std::endl;
            }
            Voltages voltages;

            for (auto& res : resInString)
            {
                voltages.push_back(std::stof(res));
            }

            return voltages;
        }

        Sensitivities getSensorSensitivities()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return Sensitivities();
            }

            commPtr->write(generateCommandBuffer(SENS, "?"));

            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, SENS, "?");

            std::vector<std::string> resInString;
            try
            {
                boost::split(resInString, response, boost::is_any_of(";"), boost::algorithm::token_compress_on);
            }
            catch (boost::bad_lexical_cast& e)
            {
                logPtr->error("Error in boost::split()");
                // std::cout << "ERROR::FTSensor::getSensorSensitivities():" << e.what() << std::endl;
            }
            Sensitivities sens;

            for (auto& res : resInString)
            {
                sens.push_back(std::stof(res));
            }

            return sens;
        }

        bool setSensorSensitivities(const Sensitivities& sens)
        {
            std::string parameters;
            for (auto& s : sens)
            {
                parameters += boost::lexical_cast<std::string>(s) + ";";
            }
            parameters = parameters.substr(0, parameters.find_last_of(';'));

            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(SENS, parameters));

            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, SENS, parameters);

            if (response == "OK")
                return true;
            else
                return false;
        }

        Offsets getAmplifierZeroOffsets()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return Offsets();
            }

            commPtr->write(generateCommandBuffer(AMPZ, "?"));

            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, AMPZ, "?");

            std::vector<std::string> resInString;
            try
            {
                boost::split(resInString, response, boost::is_any_of(";"),
                             boost::algorithm::token_compress_on);
            }
            catch (boost::bad_lexical_cast& e)
            {
                logPtr->error("Error in boost::split()");
                // std::cout << "ERROR::FTSensor::getAmplifierZeroOffsets():" << e.what() << std::endl;
            }
            Offsets offsets;

            for (auto& res : resInString)
            {
                offsets.push_back(std::stof(res));
            }

            return offsets;
        }

        bool setAmplifierZeroOffsets(const Offsets& offsets)
        {
            logPtr->error("SRI::FTSensor::setAmplifierZeroOffsets::Has not been implemented.");
            // std::cout << "SRI::FTSensor::setAmplifierZeroOffsets::Has not been implemented." << std::endl;
            return false;
        }

        RTDataMode getRealTimeDataMode()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return RTDataMode();
            }

            commPtr->write(generateCommandBuffer(SGDM, "?"));

            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, SGDM, "?");

            std::vector<std::string> resInString;
            try
            {
                boost::split(resInString, response, boost::is_any_of(";"),
                             boost::algorithm::token_compress_on);
            }
            catch (boost::bad_lexical_cast& e)
            {
                logPtr->error("Error in FTSensor getRealTimeDataMode()" + std::string(e.what()));
                // std::cout << "ERROR::FTSensor::getRealTimeDataMode():" << e.what() << std::endl;
            }

            RTDataMode rtDataMode;

            if (resInString.size() != 4)
            {
                logPtr->error("ERROR::FTSensor::getRealTimeDataMode():Parse Data False");
                // std::cout << "ERROR::FTSensor::getRealTimeDataMode():Parse Data False" << std::endl;
                return rtDataMode;
            }


            // 1. Get the relevant analog channels. format: (A01,A02,A03,A04,A05,A06)
            std::vector<std::string> channelsInString;
            boost::trim_if(resInString[0], boost::is_any_of("()"));
            boost::split(channelsInString, resInString[0], boost::is_any_of("(),"),
                         boost::algorithm::token_compress_on);
            rtDataMode.channelOrder.clear(); // rtDataMode has default value 1,2,3,4,5,6
            for (auto& cs : channelsInString)
            {
                auto c = std::stoi(cs.substr(cs.find('A') + 1));
                rtDataMode.channelOrder.push_back(c);
            }
            // 2. The unit of data uploaded from M8128.
            rtDataMode.DataUnit = resInString[1][0];
            // 3. Number of data which are desired.
            rtDataMode.PNpCH = std::stoi(resInString[2]);
            // 4. Filter model. Set to WMA. format: (WMA:1,1,2,3,4)
            std::vector<std::string> fmInString;
            boost::trim_if(resInString[3], boost::is_any_of("()"));
            boost::split(fmInString, resInString[3], boost::is_any_of("():"),
                         boost::algorithm::token_compress_on);

            rtDataMode.FM = fmInString[0];
            // 5. WMA's relevant parameters, default 1.
            std::vector<std::string> weightsInString;
            boost::split(weightsInString, fmInString[1], boost::is_any_of(","),
                         boost::algorithm::token_compress_on);
            rtDataMode.filterWeights.clear();
            for (auto& ws : weightsInString)
            {
                auto w = std::stoi(ws);
                rtDataMode.filterWeights.push_back(w);
            }

            return rtDataMode;
        }

        bool setRealTimeDataMode(const RTDataMode& rtDataMode)
        {
            // format (A02,A03,A04,A01,A05,A06);C;1;(WMA:1,1,2,3)
            // 1.
            std::string parameters;
            parameters += "(";
            for (auto& c : rtDataMode.channelOrder)
            {
                parameters += boost::str(boost::format("A%02d,") % c);
            }
            boost::trim_right_if(parameters, boost::is_any_of(","));
            parameters += ");";
            // 2.
            parameters += rtDataMode.DataUnit;
            parameters += ";";
            // 3.
            parameters += std::to_string(rtDataMode.PNpCH);
            parameters += ";";
            // 4.
            std::string weights;
            for (auto& w : rtDataMode.filterWeights)
            {
                weights += boost::str(boost::format("%d,") % w);
            }
            boost::trim_right_if(weights, boost::is_any_of(","));
            parameters += boost::str(boost::format("(%s:%s)") % rtDataMode.FM % weights);

            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(SGDM, parameters));
            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, SGDM, parameters);

            if (response == "OK")
                return true;
            else
                return false;
        }

        RTDataValid getRealTimeDataValid()
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return RTDataValid();
            }

            commPtr->write(generateCommandBuffer(DCKMD, "?"));

            while (commPtr->available() == 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, DCKMD, "?");

            return response;
        }

        bool setRealTimeDataValid(const RTDataValid& rtDataValid)
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return false;
            }

            commPtr->write(generateCommandBuffer(DCKMD, rtDataValid));

            while (commPtr->available() == 0 && commPtr->isValid() == true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);
            std::string response = extractResponseBuffer(recvbuf, DCKMD, rtDataValid);

            if (response == "OK")
                return true;
            else
                return false;
        }

        template <typename T>
        std::vector<RTData<T>>
        getRealTimeDataOnce(const RTDataMode& rtMode = RTDataMode(), const RTDataValid& rtValid = "SUM")
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return std::vector<RTData<T>>();
            }

            commPtr->write("AT+GOD\r\n");

            while (commPtr->available() == 0)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));
            }
            std::vector<int8_t> recvbuf;
            commPtr->read(recvbuf);

            // parse the received buffer
            uint32_t paritybit = 1;
            if (rtValid == "SUM")
            {
                paritybit = 1;
            }
            else if (rtValid == "CRC32")
            {
                paritybit = 4;
            }

            if (((uint8_t)recvbuf[0] != 0xAA) || ((uint8_t)recvbuf[1] != 0x55))
            {
                // FRAME HEADER FAULT
                logPtr->error("SRI::REAL-TIME-ONCE-ERROR::Frame header is fault.");
                // std::cout << "SRI::REAL-TIME-ONCE-ERROR::Frame header is fault. " << std::endl;
                return std::vector<RTData<T>>();
            }

            uint32_t PackageLength = recvbuf[2] * 256 + recvbuf[3];
            if (PackageLength != recvbuf.size() - 4)
            {
                logPtr->error("SRI::REAL-TIME-ONCE-ERROR::Package Length is fault.");
                // std::cout << "SRI::REAL-TIME-ERROR::Package Length is fault. " << std::endl;
                return std::vector<RTData<T>>();
            }

            uint32_t dataLen = PackageLength - paritybit - 2;
            if (dataLen != rtMode.channelOrder.size() * sizeof(T) * rtMode.PNpCH)
            {
                logPtr->error("SRI::REAL-TIME-ONCE-ERROR::Data Length is fault.Maybe Data Mode need update");
                // std::cout << "SRI::REAL-TIME-ERROR::Expected Data Length is fault. Maybe Data Mode need update "
                //           << std::endl;
                return std::vector<RTData<T>>();
            }

            // TODO:
            if (rtValid == "SUM")
            {
                if ((uint8_t)recvbuf.back() != getChecksum(&(recvbuf[6]), dataLen))
                {
                    logPtr->error("SRI::REAL-TIME-ONCE-ERROR::Checksum is incorrect.");
                    // std::cout << "SRI::REAL-TIME-ERROR::Checksum is incorrect. " << std::endl;
                    return std::vector<RTData<T>>();
                }
            }
            else if (rtValid == "CRC32")
            {
                uint32_t crc32 = getCRC32(&(recvbuf[6]), dataLen);
                int8_t* pCRC = &(recvbuf[recvbuf.size() - 4]);
                for (int i = 0; i < 4; i++)
                {
                    if (pCRC[i] != ((int8_t*)&crc32)[i])
                    {
                        logPtr->error("SRI::REAL-TIME-ONCE-ERROR::CRC32 is incorrect.");
                        // std::cout << "SRI::REAL-TIME-ERROR::CRC32 is incorrect. " << std::endl;
                        return std::vector<RTData<T>>();
                    }
                }
            }

            std::vector<RTData<T>> rtData(rtMode.PNpCH);
            // i*nChannel*sizeof(T) + sizeof(T)*j
            size_t nChannel = rtMode.channelOrder.size();
            for (int i = 0; i < rtMode.PNpCH; i++)
            {
                for (int j = 0; j < nChannel; j++)
                {
                    T val = *((T*)(&(recvbuf[6]) + i * nChannel * sizeof(T) + sizeof(T) * j));
                    rtData[i][j] = val;
                }
            }

            return rtData;
        };

        /// // this function need a callback function
        /// \tparam T The template parameters that defines the real time data format
        /// \param rtDataHandler The callback function defined as: void rtDataHandler(std::vector<RTData<T>>&)
        /// \param rtMode
        /// \param rtValid
        template <typename T>
        void startRealTimeDataRepeatedly(
            const RTDataMode& rtMode = RTDataMode(),
            const RTDataValid& rtValid = "SUM")
        {
            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return;
            }

            commPtr->write("AT+GSD\r\n");

            isRepeatedly = true;
            boost::thread(&FTSensor::realTimeDataCyclingHandler<T>, this, rtMode, rtValid).detach();
            logPtr->info("Getting real time data repeatedly.");
            // std::cout << "Getting real time data repeatedly." << std::endl;
        }

        void stopRealTimeDataRepeatedly()
        {
            isRepeatedly = false;

            std::this_thread::sleep_for(std::chrono::milliseconds(250));


            if (!commPtr->isValid())
            {
                logPtr->error("Communication is not valid");
                // std::cout << "ERROR::Communication is not valid" << std::endl;
                return;
            }
            logPtr->info("Stop FT!!!");
            // std::cout << "Stop FT!!!" << std::endl;
            commPtr->write("AT+GSD=STOP\r\n");

            logPtr->info("Stop real time data repeatedly");
            // std::cout << "Stop real time data repeatedly" << std::endl;
        }

        void getForceAndTorque(double& fx, double& fy, double& fz, double& tx, double& ty, double& tz)
        {
            std::lock_guard<std::mutex> lock(dataMutex); // 确保数据读取是线程安全的
            fx = Fx - ZeroFx;
            fy = Fy - ZeroFy;
            fz = Fz - ZeroFz;
            tx = Tx - ZeroTx;
            ty = Ty - ZeroTy;
            tz = Tz - ZeroTz;
        }

        void setZeroForceAndTorque(double& zerofx, double& zerofy, double& zerofz, double& zerotx, double& zeroty,
                                   double& zerotz)
        {
            ZeroFx = zerofx;
            ZeroFy = zerofy;
            ZeroFz = zerofz;
            ZeroTx = zerotx;
            ZeroTy = zeroty;
            ZeroTz = zerotz;
        }

    private:
        Logger::logger_ptr logPtr; ///< Logger pointer


        std::shared_ptr<SensorComm> commPtr; // store the polymorphic pointer of communication
        bool isRepeatedly = false; // if start RT Data Repeatedly, set true
        double Fx, Fy, Fz, Tx, Ty, Tz; // store the force and torque data
        double ZeroFx, ZeroFy, ZeroFz, ZeroTx, ZeroTy, ZeroTz; // store the zero force and torque data
        std::mutex dataMutex;

        void rtDataHandler(std::vector<RTData<float>>& rtData)
        {
            // static int i = 0;

            std::lock_guard<std::mutex> lock(dataMutex); // 确保数据更新是线程安全的
            for (int i = 0; i < rtData.size(); i++)
            {
                Fx = rtData[i][0];
                Fy = rtData[i][1];
                Fz = rtData[i][2];
                Tx = rtData[i][3];
                Ty = rtData[i][4];
                Tz = rtData[i][5];
            }
            recorded_time_ = boost::posix_time::microsec_clock::local_time();
            // std::cout << "Time recorded: " << recorded_time_ << std::endl;
            // i++;
        }

        /// Generate Command Buffer
        /// \param[in] Command      The CMD such as UARTCFG.
        /// \param[in] parameter    The parameters of the command.
        /// \return                 command buffer(string format).
        std::string generateCommandBuffer(const std::string& command,
                                          const std::string& parameter)
        {
            return AT + command + "=" + parameter + "\r\n";
        }

        /// Extract Response Buffer
        /// \param[in] recvbuf          The reference of received buffer.
        /// \param[in] expect_command   The expected command.
        /// \return                     The response from sensor(string format)
        std::string extractResponseBuffer(std::vector<int8_t>& buf,
                                          const std::string& command,
                                          const std::string& parameter)
        {
            std::string s((char*)buf.data(), buf.size());

            if (s.find(ACK) != 0)
                return "";
            if (s.find(command) == s.npos)
                return "";

            size_t nStart, nEnd;
            if (parameter == "?")
            {
                nStart = s.find("=") + 1;
                nEnd = s.find("$");
            }
            else
            {
                nStart = s.find("$") + 1;
                nEnd = s.find("\r\n");
            }

            return s.substr(nStart, nEnd - nStart);
        }

        uint8_t getChecksum(int8_t* pData, size_t len)
        {
            uint8_t sum = 0;
            sum = std::accumulate(pData, pData + len, 0);
            return sum;
        }

        uint32_t getCRC32(int8_t* pData, size_t len)
        {
            boost::crc_32_type crc32;
            crc32.process_bytes(pData, len);

            return crc32.checksum();
        }

        template <typename T>
        void realTimeDataCyclingHandler(
            const RTDataMode& rtMode,
            const RTDataValid& rtValid)
        {
            while (isRepeatedly)
            {
                if (!commPtr->isValid())
                {
                    logPtr->error("Communication is not valid");
                    // std::cout << "ERROR::Communication is not valid" << std::endl;
                    continue;
                }

                while (commPtr->available() == 0)
                {
                    int waitCount=0;
                    if (isRepeatedly == false)
                    {
                        return;
                    }
                    if (waitCount >= WAIT_TIME) {
                        logPtr->warn("Maximum wait count reached. Exiting wait loop.");

                    }
                    std::this_thread::sleep_for(std::chrono::microseconds(DELAY_US));

                }

                std::vector<int8_t> recvbuf;
                commPtr->read(recvbuf);
                std::cout << "recvbuf size: " << recvbuf.size() << std::endl;


                // parse the received buffer
                uint32_t paritybit = 1;
                if (rtValid == "SUM")
                {
                    paritybit = 1;
                }
                else if (rtValid == "CRC32")
                {
                    paritybit = 4;
                }
                if (recvbuf.empty())
                {
                    logPtr->warn("Received buffer is empty.");
                }
                while (!recvbuf.empty())
                {
                    if (((uint8_t)recvbuf[0] != 0xAA) || ((uint8_t)recvbuf[1] != 0x55))
                    {
                        // FRAME HEADER FAULT
                        logPtr->warn("Frame header is fault.");
                        // std::cout << "SRI::REAL-TIME-ERROR::Frame header is fault. " << std::endl;
                        break;
                    }

                    uint32_t PackageLength = recvbuf[2] * 256 + recvbuf[3];
                    std::cout << "PackageLength: " << PackageLength << std::endl;

                    if (PackageLength != recvbuf.size() - 4)
                    {
                        logPtr->warn(
                            "SRI::REAL-TIME-WARNING::Package Length not equal to received buf. Package Length is: {}",
                            recvbuf.size());
                        // std::cout << "SRI::REAL-TIME-WARNING::Package Length not equal to received buf. Package Length is: "
                        //           << PackageLength + 4 << " .  buf size is: " << recvbuf.size() << std::endl;
                    }

                    if (PackageLength > recvbuf.size() - 4)
                    {
                        logPtr->warn("Package Length is fault.");
                        // std::cout << "SRI::REAL-TIME-ERROR::Package Length is fault. " << std::endl;
                        break;
                    }

                    uint32_t dataLen = PackageLength - paritybit - 2;
                    if (dataLen != rtMode.channelOrder.size() * sizeof(T) * rtMode.PNpCH)
                    {
                        logPtr->warn("Data Length is fault.");
                        // std::cout << "SRI::REAL-TIME-ERROR::Expected Data Length is fault. Maybe Data Mode need update "
                        //           << std::endl;
                        break;
                    }

                    if (rtValid == "SUM")
                    {
                        if ((uint8_t)recvbuf[PackageLength + 3] != getChecksum(&(recvbuf[6]), dataLen))
                        {
                            logPtr->warn("Checksum is incorrect.");
                            // std::cout << "SRI::REAL-TIME-ERROR::Checksum is incorrect. " << std::endl;
                            break;
                        }
                    }
                    else if (rtValid == "CRC32")
                    {
                        uint32_t crc32 = getCRC32(&(recvbuf[6]), dataLen);
                        int8_t* pCRC = &(recvbuf[recvbuf.size() - 4]);
                        for (int i = 0; i < 4; i++)
                        {
                            if (pCRC[i] != ((int8_t*)&crc32)[i])
                            {
                                logPtr->warn("CRC32 is incorrect.");
                                // std::cout << "SRI::REAL-TIME-ERROR::CRC32 is incorrect. " << std::endl;
                                break;
                            }
                        }
                    }

                    std::vector<RTData<T>> rtData(rtMode.PNpCH);
                    // i*nChannel*sizeof(T) + sizeof(T)*j
                    size_t nChannel = rtMode.channelOrder.size();
                    std::cout << "rtMode.PNpCH: " << rtMode.PNpCH << "," << "nChannel: " << nChannel << std::endl;
                    for (int i = 0; i < rtMode.PNpCH; i++)
                    {
                        for (int j = 0; j < nChannel; j++)
                        {
                            T val = *((T*)(&(recvbuf[6]) + i * nChannel * sizeof(T) + sizeof(T) * j));
                            rtData[i][j] = val;
                        }
                    }


                    rtDataHandler(rtData); // Callback function

                    if (PackageLength != recvbuf.size() - 4)
                    {
                        std::vector<int8_t>(recvbuf.begin() + PackageLength + 4, recvbuf.end()).swap(recvbuf);
                        logPtr->error("    recvbuf length: {}", recvbuf.size());
                    }
                    else
                        break;
                }
            }
            logPtr->error("stop real time data repeatedly");
        }
    }; // class FTSensor
} // namespace SRI

#endif // SRI_FTSENSOR_SDK_SRI_SENSOR_H
