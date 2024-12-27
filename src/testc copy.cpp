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
    matrix_identity(kf->F);
    matrix_identity(kf->H);
    matrix_identity(kf->Q);
    matrix_identity(kf->R);
    matrix_identity(kf->P);
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
    KalmanFilter6D kf;
    initialize_kalman_filter(&kf);

    // Simulated noisy 6D force data
    double noisy_measurements[][DIM] = {
        {1.0, 2.0, 3.0, 0.1, 0.2, 0.3},
        {1.1, 2.1, 3.1, 0.15, 0.25, 0.35},
        {0.9, 1.9, 2.9, 0.05, 0.15, 0.25}
    };

    for (int i = 0; i < 3; i++) {
        kalman_filter_update(&kf, noisy_measurements[i]);
        printf("Filtered output: ");
        for (int j = 0; j < DIM; j++) {
            printf("%.3f ", kf.x[j]);
        }
        printf("\n");
    }

    return 0;
}
