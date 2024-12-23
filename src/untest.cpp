

#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp> // connection to the tcp-type FTSensor

#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp> // Boost 时间库
using namespace SRI;

bool isRunning = true;
void signalHandler(int signo)
{
    if (signo == SIGINT)
    {
        std::cout << "\033[1;31m"
                  << "[!!SIGNAL!!]"
                  << "INTERRUPT by CTRL-C"
                  << "\033[0m" << std::endl;

        isRunning = false;



    }
}

int main(int argc, char* argv[])
{
    std::cout<<"Data collection is starting..."<<std::endl;
    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        std::cout << "\033[1;31m"
                  << "Can not catch SIGINT"
                  << "\033[0m" << std::endl;
    }

    // 打开文件以追加模式写入，并确保数据在写入后实时刷新
    std::ofstream ofs("data.csv", std::ios::out | std::ios::trunc);

    if (!ofs.is_open()) {
        std::cerr << "Failed to open file!" << std::endl;
        return -1;
    }
    // 写入表头
    ofs << "Time, Fx, Fy, Fz, Tx, Ty, Tz" << std::endl;

    SRI::CommEthernet* ce = new SRI::CommEthernet("192.168.0.108", 4008);
    SRI::FTSensor sensor(ce);
    sensor.startRealTimeDataRepeatedly<float>();
    usleep(1000000);
    double Fx, Fy, Fz, Tx, Ty, Tz;
    // 数据采集
    while (isRunning) {
        sensor.getForceAndTorque(Fx, Fy, Fz, Tx, Ty, Tz);
        // 格式化时间为字符串
        std::string time_str = boost::posix_time::to_simple_string(sensor.recorded_time_);

        ofs << time_str << ", " << Fx << ", " << Fy << ", " << Fz << ", " << Tx << ", " << Ty << ", " << Tz << std::endl;

        // 实时刷新文件
        ofs.flush();

        // 模拟延时，每隔 60 秒采集一次数据
        usleep(60000000);
        // usleep(3000);
    }

    // 关闭文件
    ofs.close();
    std::cout<<"Data collection is done!"<<std::endl;
    return 0;


}