#!/usr/bin/env python3
import pathlib,subprocess,json
root=pathlib.Path(__file__).resolve().parents[4]
records=pathlib.Path(__file__).resolve().parents[1]
rows=[]
for role in [0,1]:
 for transport in [1,2,3]:
  for board in ['Gimbal','Chassis']:
   directory=root/f'27_Infantry_{board}'
   target=f'build/matrix/r{role}_t{transport}'
   log=records/'validation'/f'{board}_r{role}_t{transport}.log'
   log.parent.mkdir(parents=True,exist_ok=True)
   with log.open('w') as stream:
    result=subprocess.run(['make','-j8',f'BUILD_DIR={target}',f'EXTRA_DEFS=-DROBOT_TYPE={role} -DBOARD_LINK_TRANSPORT={transport}'],cwd=directory,stdout=stream,stderr=subprocess.STDOUT)
   print(board,role,transport,'PASS' if result.returncode==0 else 'FAIL',flush=True)
   rows.append(dict(board=board,role=role,transport=transport,success=result.returncode==0))
   if result.returncode:print(log.read_text()[-5000:]);raise SystemExit(1)
(records/'validation/build_matrix.json').write_text(json.dumps(rows,indent=2)+'\n')
