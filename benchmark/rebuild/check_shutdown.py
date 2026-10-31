import hashlib,json,os,pathlib,signal,subprocess,time
base=pathlib.Path('/workspace/bench')
results=[]
for version in ['fixed','baseline']:
 binary=f'/workspace/install-{version}/adaptive_lio/lib/adaptive_lio/adaptive_lio'
 for repeat in range(2):
  logfile=base/f'shutdown_idle_{version}_{repeat}.log'
  env=dict(os.environ,ROS_DOMAIN_ID=str(90+repeat),ADAPTIVE_LIO_OUTPUT_TRACE=str(base/f'shutdown_idle_{version}_{repeat}.trace'))
  with logfile.open('w') as f:
   p=subprocess.Popen([binary,'--config_file','/workspace/Adaptive-LIO/benchmark/rebuild/flatwall.yaml'],env=env,stdout=f,stderr=subprocess.STDOUT)
   time.sleep(3)
   p.send_signal(signal.SIGINT)
   try:code=p.wait(timeout=15)
   except subprocess.TimeoutExpired:p.kill();code=p.wait()
  results.append(dict(version=version,repeat=repeat,exit_code=code,sha256=hashlib.sha256(pathlib.Path(binary).read_bytes()).hexdigest(),log=str(logfile)))
(base/'shutdown_validation.json').write_text(json.dumps(results,indent=2)+'\n')
print(json.dumps(results,indent=2))
assert all(x['exit_code']==0 for x in results)
