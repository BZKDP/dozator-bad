from pathlib import Path
import json
from two_tier_physics import trial,HERE,pymunk
cases=[]
cases.append(trial(47,.1,0.,.0005,25,2.7))
cases.append(trial(47,.3,0.,.00025,25,2.7))
cases.append(trial(47,.3,0.,.0005,25,2.7,rear_friction=0.))
cases.append(trial(49,.3,0.,.0005,25,2.7,rear_friction=0.))
(HERE/'physics-sensitivity.json').write_text(json.dumps({'pymunk':pymunk.version,'cases':cases},ensure_ascii=False,indent=2),encoding='utf-8')
print('SENSITIVITY_FINISHED',flush=True)
