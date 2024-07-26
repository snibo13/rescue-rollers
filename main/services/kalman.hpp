#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

Matrix5f F;
Matrix5f Q;
Matrix<float, 5, 2> B = Matrix<float, 5, 2>::Zero();
Matrix5f P;

Matrix5f I = Matrix5f::Identity();

// Vector2f z_encoder;
Matrix<float, 2, 5> H_encoder;
Matrix2f R_encoder;
// Vector4f z_gyro;
Matrix<float, 4, 5> H_gyro;
Matrix4f R_gyro;
// Vector2f z_optical;
Matrix<float, 2, 5> H_optical;
Matrix2f R_optical;

Vector6f q_hat; // x, y, vx, vy, theta, omega
Vector2f u;

const double DT = 0.01;

void initialise_ekf()
{
    // TODO: Set Q and R values
    q_hat << 0, 0, 0, 0, 0, 0;
    P = Matrix5f::Identity();
    Q = Matrix5f::Identity();
    R_encoder = Matrix2f::Identity();
    R_gyro = Matrix4f::Identity();
    R_optical = Matrix2f::Identity();
}

void setF(double dt = DT)
{
    x = q_hat(0);
    y = q_hat(1);
    v = q_hat(2); // NOTE: Be wary of local verson global coordinates for velocity and sensors
    theta = q_hat(3);
    omega = q_hat(4);

    F << 1, 0, dt * cos(theta), 0, 0 0, 1, dt * sin(theta), 0, 0 0, 0, 1, 0, 0,
        0, 0, 0, 1, dt,
        0, 0, 0, 0, 1;
}

void predict()
{
    setF();
    q_hat = F * q_hat;
    P = F * P * F.transpose() + Q;
}

void update_encoder(Vector2f z_encoder, double dt = DT)
{
    H_encoder << 0, 0, CONFIG_WHEELRADIUS * 0.5, 0, CONFIG_WHEELBASE / 2,
        0, 0, CONFIG_WHEELRADIUS * 0.5, 0, CONFIG_WHEELBASE / 2;
    z_encoder = z_encoder / dt;
    y_encoder = z_encoder - H_encoder * q_hat;
    S_encoder = H_encoder * P * H_encoder.transpose() + R_encoder;
    K_encoder = P * H_encoder.transpose() * S_encoder.inverse();
    q_hat = q_hat + K_encoder * y_encoder;
    P = (I - K_encoder * H_encoder) * P;
}

void update_gyro(Vector4f z_gyro, double dt = DT)
{
    H_gyro << 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0, 1;
    y_gyro = z_gyro - H_gyro * q_hat;
    S_gyro = H_gyro * P * H_gyro.transpose() + R_gyro;
    K_gyro = P * H_gyro.transpose() * S_gyro.inverse();
    q_hat = q_hat + K_gyro * y_gyro;
    P = (I - K_gyro * H_gyro) * P;
}

void update_optical(Vector2f z_optical, double dt = DT)
{
    H_optical << 0, 0, dt * cos(q_hat(3)), 0, 0,
        0, 0, dt * sin(q_hat(3)), 0, 0;
    y_optical = z_optical - H_optical * q_hat;
    S_optical = H_optical * P * H_optical.transpose() + R_optical;
    K_optical = P * H_optical.transpose() * S_optical.inverse();
    q_hat = q_hat + K_optical * y_optical;
    P = (I - K_optical * H_optical) * P;
}

void ekf(Vector2f z_encoder, Vector4f z_gyro, Vector2f z_optical, int num_sensors)
{
    predict();
    if (num_sensors == 3)
    {
        update_encoder(z_encoder);
        update_gyro(z_gyro);
        update_optical(z_optical);
    }
    else if (num_sensors == 2)
    {
        update_encoder(z_encoder);
        update_gyro(z_gyro);
    }
    else if (num_sensors == 1)
    {
        update_encoder(z_encoder);
    }
    else
    {
        ESP_LOGE(TAG, "Invalid number of sensors");
    }
}

Vector6f get_state()
{
    return q_hat;
}
