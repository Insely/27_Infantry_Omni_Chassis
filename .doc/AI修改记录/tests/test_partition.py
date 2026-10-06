#!/usr/bin/env python3
"""Inspect linked symbols: reserved Sentry firmware must not reach motor/Infantry control."""
import pathlib,subprocess,os
ROOT=pathlib.Path(__file__).resolve().parents[4]
TESTS=pathlib.Path(__file__).resolve().parent
NM=os.environ.get('ARM_NM','arm-none-eabi-nm')
for board in ['Gimbal','Chassis']:
 for role in [0,1]:
  for transport in [1,2,3]:
   elf=ROOT/f'27_Infantry_{board}/build/matrix/r{role}_t{transport}/27_Infantry_{board}.elf'
   symbols={line.split()[-1] for line in subprocess.check_output([NM,str(elf)],text=True).splitlines() if line.split()}
   if role==1:
    for name in ['DMMotor_Init','DMMotor_SendCtrl','DJIMotor_SendCurrent','Chassis_Calculater','Remote_Tasks','Shoot_Tasks']:
     assert name not in symbols,(board,role,transport,name)
   elif board=='Gimbal':
    assert 'YawControl_Step' in symbols and 'Chassis_Calculater' not in symbols
   else:
    assert 'Chassis_Calculater' in symbols and 'YawControl_Step' not in symbols and 'Remote_Tasks' not in symbols
print('PASS ELF partition: Gimbal Yaw/remote owner, Chassis wheel owner, all six Sentry configurations exclude actuator execution')
