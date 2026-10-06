
#ifndef KALMAN_FILTER_IMU_KALMAN_FILTER_C_H
#define KALMAN_FILTER_IMU_KALMAN_FILTER_C_H

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif
//涓嶈鍒狅紝鍙嶆鍒犱簡浼氭湁闂
typedef struct Attitude_3D_t
{
    float yaw;
    float pitch;
    float roll;
    float unbiased_gyro_x;
    float unbiased_gyro_y;
    float unbiased_gyro_z;
} Attitude_3D_t;

typedef struct {
    float X_last; //涓婁竴鏃跺埢鐨勬渶浼樼粨鏋?  X(k-|k-1)
    float X_mid;  //褰撳墠鏃跺埢鐨勯娴嬬粨鏋?  X(k|k-1)
    float X_now;  //褰撳墠鏃跺埢鐨勬渶浼樼粨鏋?  X(k|k)
    float P_mid;  //褰撳墠鏃跺埢棰勬祴缁撴灉鐨勫崗鏂瑰樊  P(k|k-1)
    float P_now;  //褰撳墠鏃跺埢鏈�浼樼粨鏋滅殑鍗忔柟宸?  P(k|k)
    float P_last; //涓婁竴鏃跺埢鏈�浼樼粨鏋滅殑鍗忔柟宸?  P(k-1|k-1)
    float kg;     //kalman澧炵泭
    float A;      //绯荤粺鍙傛暟
    float B;
    float Q;
    float R;
    float H;
}extKalman_t;

typedef struct
{
  extKalman_t Angle_KF;
  extKalman_t Out_KF;
  float Angle;                      //瑙掑害  锛堝潗鏍囩郴鐨勮搴﹀叾瀹炲氨鏄宸級                     //瑙掑姞閫熷害
  float Out;//鎬昏緭鍑?	
}KF_t;

extern KF_t yaw_auto_kf;
extern KF_t pitch_auto_kf;
extern KF_t mouse_x_kf_fliter;
extern KF_t mouse_y_kf_fliter;

void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float Kalman_Filter(extKalman_t* p,float dat);

void Kalman_Init(void);
float AutoAim_pitch_Algorithm(KF_t *str);//pitch
float AutoAim_Algorithm(KF_t *str,float input);//yaw


#endif //KALMAN_FILTER_IMU_KALMAN_FILTER_C_H

#ifdef __cplusplus
}
#endif

