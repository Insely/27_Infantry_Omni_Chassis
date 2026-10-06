#!/usr/bin/env python3
"""Replay identical inputs through original/new C chassis + actual PID and ramps."""
import ctypes as C,pathlib,subprocess,tempfile,re,os,shlex,random,math
ROOT=pathlib.Path(__file__).resolve().parents[4]
TESTS=pathlib.Path(__file__).resolve().parent
APP=ROOT/'27_Infantry_Chassis/User'
def strip(s):return re.sub(r'^\s*#include[^\n]*','',s,flags=re.M).replace('pid_t','control_pid_t')
STUB='''
#include <stdint.h>
#include <math.h>
#include <string.h>
#define BOARD_CHASSIS 1
#define BOARD_GIMBAL 0
#define CAN_1_1 0
#define CAN_1_2 1
#define CAN_1_3 2
#define CAN_1_4 3
#define DM_CAN_3_2 1
#define MAX_CURRENT 16384
#define DJIMOTOR_T_A 0.3
#define SQRT1_2 0.7071067811865475244f
#define SQRT2 1.4142135623730950488f
#define RPM_TO_RAD_S 0.10471975511965977f
#define RAD_S_TO_RPM 9.549296585513720f
#define RAD_TO_DEG 57.29577951308232f
static unsigned now;
static float speed[4],current[4];
typedef struct {float speed_rpm;} DJI_motor_data_s;
typedef struct {struct {struct {float pos;} para;} motor_data;} DM_motor_data_s;
static DM_motor_data_s yaw_motor;
static struct {struct {float pitch;} AHRS;} IMU_data;
static struct {float cache_energy,remain_vol,Chassis_power;} cap;
static struct {float Buffer_Energy,Chassis_Power_Limit;} Referee_data;
void DJIMotor_Init(int type,int id) {(void)type;(void)id;}
void DJIMotor_Set(float val,int id) {current[id]=val;}
DJI_motor_data_s DJIMotor_GetData(int id) {DJI_motor_data_s v={speed[id]};return v;}
DM_motor_data_s DMMotor_GetData(int id) {(void)id;return yaw_motor;}
void Supercap_SetPower(float p) {(void)p;}
unsigned Get_SysTime_ms(void) {return now;}
float normalize_angle(float a) {while(a>3.141592653589793f)a-=6.283185307179586f;while(a< -3.141592653589793f)a+=6.283185307179586f;return a;}
'''
TAIL='''
void Test_Init(void) {Chassis_Init();Global.Control.mode=RC;}
void Test_Step(int mode,int lock,unsigned tick,float *input,float *output)
{
    now=tick;Global.Chassis.mode=mode;Global.Control.mode=lock ? LOCK : RC;
    Chassis_SetX(input[0]);Chassis_SetY(input[1]);Chassis_SetR(input[2]);
    yaw_motor.motor_data.para.pos=input[3];
    for(int i=0;i<4;++i)speed[i]=input[4+i];
    Chassis_Tasks();
    for(int i=0;i<4;++i)output[i]=current[i];
    output[4]=Chassis.speed_set_FL;output[5]=Chassis.speed_set_FR;
    output[6]=Chassis.speed_set_BL;output[7]=Chassis.speed_set_BR;
    output[8]=Chassis.Vx_now;output[9]=Chassis.Vy_now;output[10]=Chassis.W_now;
}
void Test_Roundtrip(float x,float y,float w,float *out)
{
    float fl=IK_WHEEL_FL(x,y,w),fr=IK_WHEEL_FR(x,y,w),bl=IK_WHEEL_BL(x,y,w),br=IK_WHEEL_BR(x,y,w);
    out[0]=FK_VX(fl,fr,bl,br);out[1]=FK_VY(fl,fr,bl,br);out[2]=FK_OMEGA(fl,fr,bl,br);
}
'''
def load(tmp,new):
    source=APP/'Software/Infantry' if new else TESTS/'reference'
    config=APP/'System/robot_param.h' if new else source/'robot_param.h'
    text=STUB+strip(config.read_text())
    for name in ['Algorithm/pid.h','Algorithm/ramp_generator.h','Software/Infantry/Global_status.h']:
        text+='\n'+strip((APP/name).read_text())
    text+='\nGlobalStatus_t Global;\n'+strip((source/'Chassis_omni.h').read_text())
    for name in ['Algorithm/pid.c','Algorithm/ramp_generator.c']:
        text+='\n'+strip((APP/name).read_text())
    text+='\n'+strip((source/'Chassis_omni.c').read_text())+TAIL
    p=pathlib.Path(tmp)/f'chassis_{new}.c';p.write_text(text)
    lib=p.with_suffix('.so')
    subprocess.run([os.environ.get('CC','cc'),*shlex.split(os.environ.get('HOST_CFLAGS','')),'-shared','-fPIC','-std=c11',str(p),'-lm','-o',str(lib)],check=True)
    h=C.CDLL(str(lib));h.Test_Init();h.Test_Roundtrip.argtypes=[C.c_float]*3+[C.POINTER(C.c_float)];return h
if __name__=='__main__':
 with tempfile.TemporaryDirectory() as tmp:
    old,new=load(tmp,False),load(tmp,True);rng=random.Random(27)
    for tick in range(1,2001):
        mode=rng.randrange(4);lock=tick%97==0
        inputs=(C.c_float*8)(*[rng.uniform(-2.5,2.5),rng.uniform(-2.5,2.5),rng.uniform(-6,6),rng.uniform(-math.pi,math.pi),*[rng.uniform(-6000,6000) for _ in range(4)]])
        a=(C.c_float*11)();b=(C.c_float*11)()
        old.Test_Step(mode,lock,tick,inputs,a);new.Test_Step(mode,lock,tick,inputs,b)
        for x,y in zip(a,b):assert math.isclose(x,y,rel_tol=1e-6,abs_tol=1e-4),(tick,x,y)
        if lock:assert list(b[:4])==[0]*4
        v=[rng.uniform(-5,5) for _ in range(3)];out=(C.c_float*3)();new.Test_Roundtrip(*v,out)
        for x,y in zip(v,out):assert math.isclose(x,y,rel_tol=1e-5,abs_tol=1e-5)
    print('PASS 2000 C replay steps: original/new wheel targets, PID/feedforward currents, four modes, LOCK, ramps; 2000 FK/IK roundtrips')
