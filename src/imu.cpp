//
// Created by hyj on 18-1-19.
//

#include <random>
#include "imu.h"
#include "utilities.h"

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d RIb;
    RIb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return RIb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,// 第二节课件公式(31)
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}


IMU::IMU(Param p): param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

void IMU::addIMUnoise(MotionData& data)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_; // todo: 这里为什么要除以采样时间的平方根

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;    // bias是随机游走噪声, 它的导数是高斯白噪声 todo: 这里为什么要乘以采样时间的平方根
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;       // bias是随机游走噪声
    data.imu_acc_bias = acc_bias_;

}

/*
 * @input: time point 
 * @output: 根据时间和imu参数生成的imu数据
 * @method： 根据参数设计位置的路径和角度的变化，对路径和角度进行微分可以得到线加速度和角速度
*/
MotionData IMU::MotionModel(double t)
{

    MotionData data;
    // param
    float ellipse_x = 15;
    float ellipse_y = 20;
    float z = 1;           // z轴做sin运动
    float K1 = 10;          // z轴的正弦频率是x，y的k1倍
    float K = M_PI/ 10;    // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周

    // translation
    // twb:  body frame in world frame
    Eigen::Vector3d position( ellipse_x * cos( K * t) + 5, ellipse_y * sin( K * t) + 5,  z * sin( K1 * K * t ) + 5);
    Eigen::Vector3d dp(- K * ellipse_x * sin(K*t),  K * ellipse_y * cos(K*t), z*K1*K * cos(K1 * K * t));              // position导数　in world frame
    double K2 = K*K;
    Eigen::Vector3d ddp( -K2 * ellipse_x * cos(K*t),  -K2 * ellipse_y * sin(K*t), -z*K1*K1*K2 * sin(K1 * K * t));     // position二阶导数

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    Eigen::Vector3d eulerAngles(k_roll * cos(t) , k_pitch * sin(t) , K*t );   // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数

//    Eigen::Vector3d eulerAngles(0.0,0.0, K*t );   // roll ~ 0, pitch ~ 0, yaw ~ [0,2pi]
//    Eigen::Vector3d eulerAnglesRates(0.,0. , K);      // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates;   //  euler rates trans to body gyro

    Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs  

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t;
    return data;

}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist)
{
    std::vector<MotionData>imudata;
    LoadPose(src,imudata);

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];
        MotionData imupose_kk = imudata[i-1];


        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]


        /// imu 动力学模型 欧拉积分
        // Eigen::Vector3d gyro_w = imupose.imu_gyro;
        /// 中值积分
        Eigen::Vector3d gyro_w = 0.5 *(imupose.imu_gyro + imupose_kk.imu_gyro);

        //
        // // Eigen::Vector3d dtheta_half_tmp =  imupose_kk.imu_gyro * dt /2/.0;  //
        Eigen::Vector3d dtheta_half_tmp =  gyro_w * dt /2.0;  //
        Eigen::Quaterniond Qwb_next, dp_delta;
        dp_delta.w() = 1;
        dp_delta.x() = dtheta_half_tmp.x();
        dp_delta.y() = dtheta_half_tmp.y();
        dp_delta.z() = dtheta_half_tmp.z();
        dp_delta.normalize();
        Qwb_next = Qwb*dp_delta;                     // Qwb是k-1(kk)时刻的姿态，dp_delta是k-1到k时刻的姿态变化值 k 时刻 quaterniond的预计值

        /// imu 动力学模型 欧拉积分
        // Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
        /// 中值积分
        Eigen::Vector3d acc_w = 0.5*(Qwb * (imupose_kk.imu_acc) + gw + Qwb_next * (imupose.imu_acc) + gw) ;  // aw = 0.5*(Rwb_kk * ( acc_body_kk - acc_bias_kk ) + gw + Rwb * ( acc_body - acc_bias ) + gw)

        Qwb = Qwb_next;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw = Vw + acc_w * dt;
        
        
        

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points<<imupose.timestamp<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<Qwb.w()<<" "
                   <<Qwb.x()<<" "
                   <<Qwb.y()<<" "
                   <<Qwb.z()<<" "
                   <<Pwb(0)<<" "
                   <<Pwb(1)<<" "
                   <<Pwb(2)<<" "
                   <<std::endl;

    }

    std::cout<<"test　end"<<std::endl;

}
