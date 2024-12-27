#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sri/ftsensor.hpp>
#include <sri/commethernet.hpp> // connection to the tcp-type FTSensor

#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp> // Boost 时间库
using namespace SRI;
#define DIM 6
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

typedef struct {
    double F[DIM][DIM]; // State transition model
    double H[DIM][DIM]; // Measurement model
    double Q[DIM][DIM]; // Process noise covariance
    double R[DIM][DIM]; // Measurement noise covariance
    double P[DIM][DIM]; // Estimate error covariance
    double x[DIM];      // State estimate
} KalmanFilter6D;

void matrix_identity(double mat[DIM][DIM]) {
    memset(mat, 0, sizeof(double) * DIM * DIM);
    for (int i = 0; i < DIM; i++) {
        mat[i][i] = 1.0;
    }
}

void matrix_add(double result[DIM][DIM], double a[DIM][DIM], double b[DIM][DIM]) {
    for (int i = 0; i < DIM; i++) {
        for (int j = 0; j < DIM; j++) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

void matrix_sub(double result[DIM][DIM], double a[DIM][DIM], double b[DIM][DIM]) {
    for (int i = 0; i < DIM; i++) {
        for (int j = 0; j < DIM; j++) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
}

void matrix_mult(double result[DIM][DIM], double a[DIM][DIM], double b[DIM][DIM]) {
    memset(result, 0, sizeof(double) * DIM * DIM);
    for (int i = 0; i < DIM; i++) {
        for (int j = 0; j < DIM; j++) {
            for (int k = 0; k < DIM; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

void vector_mult(double result[DIM], double mat[DIM][DIM], double vec[DIM]) {
    memset(result, 0, sizeof(double) * DIM);
    for (int i = 0; i < DIM; i++) {
        for (int j = 0; j < DIM; j++) {
            result[i] += mat[i][j] * vec[j];
        }
    }
}

void vector_add(double result[DIM], double a[DIM], double b[DIM]) {
    for (int i = 0; i < DIM; i++) {
        result[i] = a[i] + b[i];
    }
}
void initialize_kalman_filter(KalmanFilter6D *kf) {
    // 设置状态转移矩阵 F
    matrix_identity(kf->F);
    // memset(kf->F, 0, sizeof(double) * DIM * DIM);
    // for (int i = 0; i < DIM; i++) {
    //     kf->F[i][i] = 1.0; // 自身状态维持
    // }

    // 设置测量矩阵 H
    memset(kf->H, 0, sizeof(double) * DIM * DIM);
    for (int i = 0; i < DIM; i++) {
        kf->H[i][i] = 1.0; // 直接观测对应的状态
    }

    // 设置过程噪声协方差 Q
    memset(kf->Q, 0, sizeof(double) * DIM * DIM);
    for (int i = 0; i < DIM; i++) {
        kf->Q[i][i] = 0.01; // 假设过程噪声较小
    }

    // 设置测量噪声协方差 R
    memset(kf->R, 0, sizeof(double) * DIM * DIM);
    for (int i = 0; i < DIM; i++) {
        kf->R[i][i] = 0.005; // 假设测量噪声较大
    }

    // 设置初始协方差矩阵 P
    matrix_identity(kf->P);
    // memset(kf->P, 0, sizeof(double) * DIM * DIM);
    // for (int i = 0; i < DIM; i++) {
    //     kf->P[i][i] = 1.0; // 初始状态的较高不确定性
    // }

    // 初始化状态 x
    memset(kf->x, 0, sizeof(double) * DIM);
}


void kalman_filter_update(KalmanFilter6D *kf, const double z[DIM]) {
    double x_pred[DIM], P_pred[DIM][DIM];
    double S[DIM][DIM], K[DIM][DIM], temp[DIM][DIM], temp_vec[DIM];

    // Predict step
    vector_mult(x_pred, kf->F, kf->x);
    matrix_mult(temp, kf->F, kf->P);
    matrix_mult(P_pred, temp, kf->F);
    matrix_add(P_pred, P_pred, kf->Q);

    // Update step
    matrix_mult(temp, kf->H, P_pred);
    matrix_mult(S, temp, kf->H);
    matrix_add(S, S, kf->R);

    // Calculate Kalman Gain
    matrix_mult(temp, P_pred, kf->H);
    for (int i = 0; i < DIM; i++) {
        for (int j = 0; j < DIM; j++) {
            K[i][j] = temp[i][j] / S[j][j]; // Assume S is diagonal for simplicity
        }
    }

    // Update state
    vector_mult(temp_vec, kf->H, x_pred);
    for (int i = 0; i < DIM; i++) {
        temp_vec[i] = z[i] - temp_vec[i];
    }
    vector_mult(kf->x, K, temp_vec);
    vector_add(kf->x, x_pred, kf->x);

    // Update covariance
    memset(temp, 0, sizeof(double) * DIM * DIM);
    for (int i = 0; i < DIM; i++) {
        temp[i][i] = 1.0;
        for (int j = 0; j < DIM; j++) {
            temp[i][j] -= K[i][j] * kf->H[j][i];
        }
    }
    matrix_mult(kf->P, temp, P_pred);
}

int main() {
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
    ofs << "Time, Fx, Fy, Fz, Tx, Ty, Tz,fiterFx,fiterFy,fiterFz,fiterTx,fiterTy,fiterTz" << std::endl;
    KalmanFilter6D kf;
    initialize_kalman_filter(&kf);
    SRI::CommEthernet* ce = new SRI::CommEthernet("192.168.0.108", 4008);
    SRI::FTSensor sensor(ce);
    sensor.startRealTimeDataRepeatedly<float>();
    usleep(1000000);
    double Fx, Fy, Fz, Tx, Ty, Tz;
    while(isRunning){
        sensor.getForceAndTorque(Fx, Fy, Fz, Tx, Ty, Tz);
        double noisy_measurements[DIM] = {Fx, Fy, Fz, Tx, Ty, Tz};
        kalman_filter_update(&kf, noisy_measurements);
        std::cout<<"Fx: "<<Fx<<", Fy: "<<Fy<<", Fz: "<<Fz<<", Tx: "<<Tx<<", Ty: "<<Ty<<", Tz: "<<Tz<<std::endl;
        std::cout<<"fiterFx: "<<kf.x[0]<<", fiterFy: "<<kf.x[1]<<", fiterFz: "<<kf.x[2]<<", fiterTx: "<<kf.x[3]<<", fiterTy: "<<kf.x[4]<<", fiterTz: "<<kf.x[5]<<std::endl;
        ofs << boost::posix_time::to_simple_string(sensor.recorded_time_) << ", " << Fx << ", " << Fy << ", " << Fz << ", " << Tx << ", " << Ty << ", " << Tz << ", " << kf.x[0] << ", " << kf.x[1] << ", " << kf.x[2] << ", " << kf.x[3] << ", " << kf.x[4] << ", " << kf.x[5] << std::endl;
        ofs.flush();
        usleep(100000);
    }

     // 关闭文件
    ofs.close();
    std::cout<<"Data collection is done!"<<std::endl;
    return 0;
    
}
