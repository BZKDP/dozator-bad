from two_tier_physics import trial,HERE,pymunk
import json
results=[]
for D,mu,rr,back in [(47,.3,.02,None),(49,.3,.02,None),(47,.3,.02,0.),(47,.15,.05,None)]:
    results.append(trial(D,mu,rr,.0005,25,2.7,rear_friction=back,full_rocker=True))
(HERE/'physics-full-rocker.json').write_text(json.dumps({'pymunk':pymunk.version,'cases':results},ensure_ascii=False,indent=2),encoding='utf-8')
print('FULL_ROCKER_FINISHED',flush=True)
