#!/usr/bin/env python3
"""Compare original chassis Yaw control with its migrated Gimbal implementation."""
import ctypes as C,pathlib,subprocess,tempfile,re,random,os,shlex,math
from test_chassis import strip
ROOT=pathlib.Path(__file__).resolve().parents[4]
TESTS=pathlib.Path(__file__).resolve().parent
APP=ROOT/'27_Infantry_Gimbal/User'
def function(s,name):
 m=re.search(r'void '+name+r'\([^)]*\)\s*\{',s);end=m.end();depth=1
 while depth:
  if s[end]=='{':depth+=1
  if s[end]=='}':depth-=1
  end+=1
 return s[m.start():end]
def load(tmp,new):
 t='''#include <stdint.h>
#include <math.h>
#define MAX_CURRENT 16384
#define RAD_TO_DEG 57.29577951308232f
#define DEG_TO_RAD 0.017453292519943295f
#define YAWMotor 1
#define PITCHMotor 0
#define GIMBAL_YAW_MOTOR_TYPE 0
#define GIMBAL_PITCH_MOTOR_TYPE 0
#define BOARD_LINK_YAW_SPEED_LIMIT_RAD_S 20.0f
static struct {float body_gyro_z;} BoardLink;
static int fresh=1;
int BoardLink_BodyGyroIsFresh(void){return fresh;}
int BoardLink_RelativeAngleIsFresh(void){return fresh;}
static struct {float gyro[3];} IMU_data;
static struct {float pitch,yaw,gyro[3];} dm_imu_gimbal;
void DMMotor_Init(int t,int id){(void)t;(void)id;}
'''
 for name in ['Algorithm/pid.h','Algorithm/ramp_generator.h','Software/Infantry/Global_status.h','Software/Infantry/Gimbal.h','Algorithm/pid.c']:
  t+='\n'+strip((APP/name).read_text())
 t+='\nGimbal_t Gimbal; GlobalStatus_t Global;\n'
 if new:
  t+=strip((APP/'Software/Infantry/YawControl.c').read_text())
  names=['YawControl_Init','YawControl_Updater','YawControl_Calculater']
 else:
  src=(TESTS/'reference/Gimbal_yaw.c').read_text()
  names=['Gimbal_Init','Gimbal_Updater','Gimbal_Calculater']
  t+='\n#define CHASSIS_DECOUPLE_FF_GAIN (1.07f)\n'+'\n'.join(function(src,n) for n in names)
 t+=f'\nvoid Test_Init(void){{{names[0]}();}}\n'
 t+='''void Test_Step(int mode,int lock,int auto_mode,float *in,float *out)
{
 Global.Chassis.mode=mode;Global.Control.mode=lock ? LOCK : RC;
 Global.Auto.mode=auto_mode ? CAR : NONE;Global.Auto.input.control_mode=auto_mode;
 Global.Auto.input.Auto_control_online=20;Global.Gimbal.mode=NORMAL;
 Global.Gimbal.input.yaw=in[0];dm_imu_gimbal.yaw=in[1];dm_imu_gimbal.pitch=in[2];
 dm_imu_gimbal.gyro[0]=in[3];dm_imu_gimbal.gyro[2]=in[4];
 BoardLink.body_gyro_z=IMU_data.gyro[2]=in[5];Global.Auto.input.yaw_ff=in[6];
'''+names[1]+'();'+names[2]+'''();
 out[0]=Gimbal.yaw_speed_set;out[1]=Gimbal.yaw_location_now;out[2]=Gimbal.yaw_speed_now;out[3]=Global.Gimbal.input.yaw;
}
'''
 if new:t+='\nfloat Test_Stale(void){fresh=0;YawControl_Step();return Gimbal.yaw_speed_set;}\n'
 p=pathlib.Path(tmp)/f'yaw_{new}.c';p.write_text(t);lib=p.with_suffix('.so')
 subprocess.run([os.environ.get('CC','cc'),*shlex.split(os.environ.get('HOST_CFLAGS','')),'-shared','-fPIC','-std=c11',str(p),'-lm','-o',str(lib)],check=True)
 h=C.CDLL(str(lib));h.Test_Init();return h
if __name__=='__main__':
 with tempfile.TemporaryDirectory() as tmp:
  old,new=load(tmp,False),load(tmp,True);rng=random.Random(27)
  for tick in range(2000):
   args=(rng.randrange(4),tick%97==0,rng.randrange(3))
   inputs=(C.c_float*7)(rng.uniform(-360,360),rng.uniform(-180,180),rng.uniform(-24,30),*[rng.uniform(-5,5) for _ in range(4)])
   a=(C.c_float*4)();b=(C.c_float*4)();old.Test_Step(*args,inputs,a);new.Test_Step(*args,inputs,b)
   for x,y in zip(a,b):assert math.isclose(x,y,rel_tol=1e-6,abs_tol=1e-5),(tick,x,y)
  new.Test_Stale.restype=C.c_float;assert new.Test_Stale()==0
  print('PASS 2000 C Yaw replay steps: original/migrated PID, angle wrap, manual/auto transitions, chassis gyro decoupling; stale feedback inhibits command')
