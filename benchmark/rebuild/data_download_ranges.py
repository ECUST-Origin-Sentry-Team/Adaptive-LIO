import concurrent.futures,subprocess,pathlib,sys,os,json
url,dest,size=sys.argv[1],pathlib.Path(sys.argv[2]),int(sys.argv[3])
start=dest.stat().st_size if dest.exists() else 0
step=16*1024**2
parts=[(a,min(a+step,size)-1) for a in range(start,size,step)]
def download(r):
 a,b=r;p=dest.with_name(dest.name+f'.part{a}')
 subprocess.run(['curl','-fsSL','--retry','4','--range',f'{a}-{b}',url,'-o',str(p)],check=True)
 assert p.stat().st_size==b-a+1,(p,p.stat().st_size,b-a+1)
 print('part complete',a,b,flush=True)
 return p
with concurrent.futures.ThreadPoolExecutor(max_workers=8) as ex: out=list(ex.map(download,parts))
with dest.open('ab') as f:
 for p in out:
  with p.open('rb') as g:
   while block:=g.read(1024**2):f.write(block)
  p.unlink()
assert dest.stat().st_size==size
print('complete',dest,size)
