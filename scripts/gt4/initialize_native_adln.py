"""Unmodified GT4 ADLN initialization and sampler execution on upstream Play!.

Owner includes the additional ADLN pointer at +9c required by native469f20.
No game callees are replaced. Every target instruction is original CORE code.
"""
import argparse,contextlib,hashlib,io,json,re,struct,subprocess,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
AUDIT=ROOT/'.audit/gt4-tracks'
OUT=ROOT/'.audit/gt4-native-adln'
RUNNER=ROOT/'.audit/gt4-tracks/native-oracle-build/gt4-native-ee-runner'
ELF=ROOT/'.audit/gt4-handling/extracted/CORE.ps2.elf'
CYCLE_BUDGET=100000000
RW,AD,TR,SELF,OWNER,COURSE=0x1800000,0x1b00000,0x1d00000,0x1d20000,0x1d24000,0x1d28000
STOP=0x1fff000
u32=lambda x:struct.pack('<I',x)


def li(reg,value):
    return [0x3c000000|(reg<<16)|(value>>16),0x34000000|(reg<<21)|(reg<<16)|(value&65535)]


def packet(entry,gprs,fprs,patches,reads):
    b=u32(entry)+u32(STOP)+u32(len(gprs))
    b+=b''.join(u32(r)+struct.pack('<Q',v)for r,v in gprs.items())
    b+=u32(len(fprs))+b''.join(u32(r)+u32(v)for r,v in fprs.items())
    b+=u32(len(patches))+b''.join(u32(a)+u32(len(v))+v for a,v in patches)
    return b+u32(len(reads))+b''.join(u32(a)for a in reads)


def initialize(name,extended=False,allow_legacy=False):
    rw=(AUDIT/f'decoded/runways/{name}.rnw').read_bytes()
    outer=(AUDIT/f'extracted/crs/{name}.ad').read_bytes()
    ad=outer[struct.unpack_from('<I',outer,4)[0]:]
    assert ad[:4]==b'ADLN' or allow_legacy
    owner=bytearray(0xa0)
    struct.pack_into('<I',owner,4,COURSE)
    struct.pack_into('<I',owner,0x9c,AD)
    course=bytearray(0xe0);struct.pack_into('<I',course,0xdc,RW)
    code=li(4,RW)+[0x0c000000|(0x3e5818>>2),0]+li(4,SELF)+li(5,AD)+[0x0c000000|(0x46a510>>2),0]
    code+=li(4,SELF)+li(5,6)+[0x0c000000|(0x46bb48>>2),0]
    code+=li(8,SELF+12)+[0xad020000,0x02000008,0]
    code=struct.pack('<'+'I'*len(code),*code)
    patches=[(RW,rw),(AD,ad),(TR,code),(SELF,u32(OWNER)+bytes(12)),(OWNER,owner),(COURSE,course)]
    reads=[SELF+4,SELF+8]+[AD+i for i in range(0,len(ad),4)]+[SELF+12]
    data=b'GT4RUN1\0'+u32(1)+packet(TR,{16:STOP},{},patches,reads)
    runner=RUNNER
    prefix=OUT/(name+'-native-init')
    inp=Path(str(prefix)+'-inputs.bin');inp.write_bytes(data)
    elf=ELF
    start=time.monotonic();r=subprocess.run([str(runner),str(elf),str(inp),f'--cycle-budget={CYCLE_BUDGET}'],capture_output=True,text=True,timeout=120)
    Path(str(prefix)+'-stdout.txt').write_text(r.stdout);Path(str(prefix)+'-stderr.txt').write_text(r.stderr)
    meta=dict(course=name,owner_adln_pointer_offset='0x9c',runner=str(runner),returncode=r.returncode,
              source_format='ADLN' if ad[:4]==b'ADLN' else 'original-legacy-header',
              cycle_budget=CYCLE_BUDGET,original_ad_sha256=hashlib.sha256(outer).hexdigest(),original_rnw_sha256=hashlib.sha256(rw).hexdigest(),elapsed_seconds=time.monotonic()-start,sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [runner,elf,inp]})
    if r.returncode==0:
        bits=r.stdout.split();assert int(bits[1],16)==STOP
        vals=bits[5:];assert len(vals)==len(reads)
        initialized=b''.join(u32(int(v,16))for v in vals[2:-1])
        suffix='-initialized-adln.bin' if ad[:4]==b'ADLN' else '-diagnostic-legacy-after46a510.bin'
        dest=OUT/(name+suffix);dest.write_bytes(initialized)
        meta['output_file']=dest.name
        meta['initialized_adln_sha256']=hashlib.sha256(initialized).hexdigest();meta['cycles']=int(bits[4]);meta['self_count']=int(vals[0],16);meta['self_table']=vals[1]
        chosen=int(vals[-1],16);meta['native_default_route_type']=chosen if chosen<0x80000000 else chosen-0x100000000
    Path(str(prefix)+'-metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(meta,indent=2));print(r.stderr[-500:]);return meta



def main():
    global AUDIT,OUT,RUNNER,ELF,CYCLE_BUDGET
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-root',type=Path,default=AUDIT)
    parser.add_argument('--out',type=Path,default=OUT)
    parser.add_argument('--runner',type=Path,default=RUNNER)
    parser.add_argument('--elf',type=Path,default=ELF)
    parser.add_argument('--cycle-budget',type=int,default=100000000)
    parser.add_argument('--course',action='append',default=[])
    parser.add_argument('--all-available',action='store_true')
    args=parser.parse_args()
    AUDIT=args.audit_root.resolve();OUT=args.out.resolve();RUNNER=args.runner.resolve();ELF=args.elf.resolve();CYCLE_BUDGET=args.cycle_budget
    if not 1000<=CYCLE_BUDGET<=100000000:parser.error('cycle budget must be1000..100000000')
    if hashlib.sha256(ELF.read_bytes()).hexdigest()!='e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804':parser.error('CORE identity mismatch')
    names=args.course
    if args.all_available:
        catalog=ROOT/'crates/neural_racing_core/src/circuit_data/gt4_catalog.rs'
        names=[re.search(r'id: "([^"]+)"',line).group(1)for line in catalog.read_text().splitlines()if 'available: true' in line]
    if not names:parser.error('Select --course ID or --all-available')
    OUT.mkdir(parents=True,exist_ok=True);results=[]
    for name in names:
        try:
            source=AUDIT/f'extracted/crs/{name}.ad';raw=source.read_bytes();base=struct.unpack_from('<I',raw,4)[0]
            if raw[base:base+4]!=b'ADLN':
                result=dict(course=name,status='explicit-legacy-unsupported',source_sha256=hashlib.sha256(raw).hexdigest())
            else:
                with contextlib.redirect_stdout(io.StringIO()):result=initialize(name,True)
                result['status']='native-initialized'if result['returncode']==0 else'native-execution-failed'
        except Exception as error:result=dict(course=name,status='explicit-error',error=str(error))
        results.append(result);print(name,result['status'],result.get('cycles'),flush=True)
    report=dict(authority='unchanged original3e5818-46a510-46bb48onPlayEE',requested=len(names),initialized=sum(r['status']=='native-initialized'for r in results),script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),courses=results)
    (OUT/'native-adln-initialization.json').write_text(json.dumps(report,indent=2)+'\n')


if __name__=='__main__':main()
