"""Original MDLS static textured-ground prototype, with explicit omissions.

VIF position deltas, UVs, color bytes, VM texture bindings and literal model
matrices are retained. No dynamic callbacks/instances are fabricated. RNW
height matching is a display-layer filter, not a redefinition of road physics.
"""
import argparse,collections,concurrent.futures,gzip,hashlib,json,math,re,struct,sys,time
from pathlib import Path
from visual_vif import decode_packet
from visual_tex1 import decode_set
ROOT=Path(__file__).resolve().parents[2]
AUDIT=ROOT/'.audit/gt4-tracks'
OUT=ROOT/'.audit/gt4-visual-export'


def identity():return [[float(i==j)for j in range(4)]for i in range(4)]
def mul(a,b):return [[sum(a[i][k]*b[k][j]for k in range(4))for j in range(4)]for i in range(4)]
def transform(m,p):return [sum(m[i][j]*p[j]for j in range(3))+m[i][3]for i in range(3)]


def vm_draws(b,index):
    tab=struct.unpack_from('<I',b,56)[0];p=struct.unpack_from('<I',b,tab+index*40+24)[0]
    if not p:return []
    matrix=identity();stack=[];slot=None;draws=[]
    one={1,0xb,0x17,0x18,0x1a,0x1b,0x1d,0x1e,0x22,0x23,0x25,0x26,0x2d,0x2e,0x30,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,0x40,0x47,0x4e,0x4f,0x56,0x57,0x58}
    sizes={0x19:2,0x1c:3,0x1f:2,0x20:3,0x21:5,0x24:5,0x27:5,0x28:17,0x29:2,0x2a:2,0x2f:2,0x31:13,0x32:5,0x33:17,0x34:5,0x35:5,0x36:5,0x37:17,0x41:4,0x42:4,0x52:17,0x53:17}
    for _ in range(10000):
        op=b[p]
        if op==0:
            assert not stack
            return draws
        if op in (4,5):
            shape=b[p+1]if op==4 else struct.unpack_from('<H',b,p+1)[0]
            draws.append(dict(model=index,shape=shape,texture_slot=slot,matrix=[r[:]for r in matrix],command_offset=p))
            p+=2 if op==4 else 3
        elif op in (2,3,6,0x43,0x44,0x45,0x46,0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x50,0x51,0x54,0x55,0x59,0x5a):
            raise ValueError(f'dynamic/nested/instance opcode {op:02x}')
        elif op==7:
            assert b[p+17]>0
            q=p+18;p=q+struct.unpack_from('<h',b,q)[0]
        elif op==8:p=p+1+struct.unpack_from('<b',b,p+1)[0]
        elif op==9:p=p+1+struct.unpack_from('<h',b,p+1)[0]
        elif op==10:p+=2+b[p+1]*12+2
        elif op==12:stack.append([r[:]for r in matrix]);p+=1
        elif op==13:matrix=stack.pop();p+=1
        elif op in (15,16):
            data=struct.unpack_from('<16f',b,p+1);other=[[data[j*4+i]for j in range(4)]for i in range(4)]
            matrix=other if op==15 else mul(matrix,other);p+=65
        elif op in (17,18):
            v=struct.unpack_from('<3f',b,p+1);other=identity()
            for i in range(3):
                if op==17:other[i][3]=v[i]
                else:other[i][i]=v[i]
            matrix=mul(matrix,other);p+=13
        elif op==19:
            angle,x,y,z=struct.unpack_from('<4f',b,p+1);length=math.sqrt(x*x+y*y+z*z);assert length>0
            x,y,z=x/length,y/length,z/length;a=math.radians(angle);c=math.cos(a);s=math.sin(a);t=1-c
            other=[[t*x*x+c,t*x*y-s*z,t*x*z+s*y,0],[t*x*y+s*z,t*y*y+c,t*y*z-s*x,0],[t*x*z-s*y,t*y*z+s*x,t*z*z+c,0],[0,0,0,1]]
            matrix=mul(matrix,other);p+=17
        elif op in (0x2b,0x2c):
            slot=b[p+1]if op==0x2b else struct.unpack_from('<H',b,p+1)[0];p+=2 if op==0x2b else 3
        elif op in one:p+=1
        elif op in sizes:p+=sizes[op]
        else:raise ValueError(f'unhandled opcode {op:02x}')
    raise ValueError('VM step budget')


def mesh_heights(name):
    raw=(AUDIT/f'decoded/meshes/{name}.bin').read_bytes();nv,nt=struct.unpack_from('<II',raw)
    verts=[struct.unpack_from('<3f',raw,8+i*16)for i in range(nv)];cells=collections.defaultdict(list)
    for i in range(nt):
        ix=struct.unpack_from('<3H',raw,8+nv*16+i*12);p=[verts[k]for k in ix]
        ax,az=p[1][0]-p[0][0],p[1][2]-p[0][2];bx,bz=p[2][0]-p[0][0],p[2][2]-p[0][2]
        den=ax*bz-az*bx
        # RNW positive-up frontface has negative XZ determinant.
        if den>=-1e-10:continue
        for x in range(math.floor(min(v[0]for v in p)/25),math.floor(max(v[0]for v in p)/25)+1):
            for z in range(math.floor(min(v[2]for v in p)/25),math.floor(max(v[2]for v in p)/25)+1):cells[x,z].append((p,ax,az,bx,bz,den))
    def nearest(q):
        best=math.inf
        for p,ax,az,bx,bz,den in cells.get((math.floor(q[0]/25),math.floor(q[2]/25)),[]):
            dx,dz=q[0]-p[0][0],q[2]-p[0][2];u=(dx*bz-dz*bx)/den;v=(ax*dz-az*dx)/den
            if u<-.00001 or v<-.00001 or u+v>1.00001:continue
            y=p[0][1]+u*(p[1][1]-p[0][1])+v*(p[2][1]-p[0][1]);best=min(best,abs(y-q[1]))
        return best
    return nearest


def export(name):
    path=AUDIT/f'extracted/crs/{name}';raw=path.read_bytes();b=raw[256:]
    model_count,shape_count,material_count,slots,groups=struct.unpack_from('<5H',b,22)
    draws=[];omitted=[]
    for i in range(model_count):
        try:draws.extend(vm_draws(b,i))
        except Exception as e:omitted.append(dict(model=i,reason=str(e)))
    cache={};shape_table=struct.unpack_from('<I',b,60)[0]
    def shape_packets(index):
        if index in cache:return cache[index]
        assert index<shape_count
        off=struct.unpack_from('<I',b,shape_table+4*index)[0];s=b[off:];scale=struct.unpack_from('<f',s,4)[0];count=struct.unpack_from('<H',s,10)[0];packets=[]
        for i in range(count):
            po,d=struct.unpack_from('<II',s,16+8*i);p=decode_packet(s[po:po+(d&65535)*16],bool(s[8]),scale);p['texture_index']=(d>>16)&511;p['source_offset']=256+off+po;packets.append(p)
        cache[index]=packets;return packets
    query=mesh_heights(name);triangles=[];decode_failures=[];rejected=collections.Counter();texture_keys=set()
    for d in draws:
        try:packets=shape_packets(d['shape'])
        except Exception as e:decode_failures.append(dict(model=d['model'],shape=d['shape'],reason=str(e)));continue
        for p in packets:
            if not p['uvs'] or not p['texture_index'] or p['texture_index']>=511 or d['texture_slot'] is None:rejected['no-explicit-texture-or-uv']+=1;continue
            points=[transform(d['matrix'],v)for v in p['vertices']];start=0
            for length in p['strip_lengths']:
                for k in range(length-2):
                    ids=[start+k,start+k+1,start+k+2]
                    if k&1:ids[0],ids[1]=ids[1],ids[0]
                    vs=[points[i]for i in ids];a=[vs[1][i]-vs[0][i]for i in range(3)];c=[vs[2][i]-vs[0][i]for i in range(3)]
                    n=[a[1]*c[2]-a[2]*c[1],a[2]*c[0]-a[0]*c[2],a[0]*c[1]-a[1]*c[0]];norm=math.sqrt(sum(v*v for v in n))
                    if norm<1e-10 or abs(n[1])/norm<.7:rejected['vertical-or-degenerate']+=1;continue
                    q=[sum(v[i]for v in vs)/3 for i in range(3)]
                    if query(q)>.4:rejected['not-near-original-RNW-ground']+=1;continue
                    key=(d['texture_slot'],p['texture_index']-1);texture_keys.add(key)
                    triangles.append(dict(p=vs,uv=[p['uvs'][i]for i in ids],colors_rgba8=[p['colors_rgba8'][i]for i in ids] if p['colors_rgba8'] else None,texture=list(key),model=d['model'],shape=d['shape'],source_offset=p['source_offset']))
                start+=length
    texture_table=struct.unpack_from('<I',b,68)[0];group0=struct.unpack_from('<I',b,texture_table)[0];textures=[];texture_failures=[]
    for slot in sorted({key[0]for key in texture_keys}):
        assert slot<slots;pos=struct.unpack_from('<I',b,group0+4*slot)[0]
        try:
            infos=decode_set(b[pos:])
            for key in sorted(k for k in texture_keys if k[0]==slot):
                t=dict(infos[key[1]]);t.update(key=list(key),source_tex1_offset=256+pos);textures.append(t)
        except Exception as e:texture_failures.append(dict(slot=slot,reason=str(e)))
    valid={tuple(t['key'])for t in textures};triangles=[t for t in triangles if tuple(t['texture'])in valid]
    report=dict(format='original-GT4-static-textured-ground-prototype',course=name,source_sha256=hashlib.sha256(raw).hexdigest(),source_coordinates='RNW XYZ metres (Y up), convertto core[X,-Z,Y] without fitting',model_count=model_count,supported_models=model_count-len(omitted),omitted_models=omitted,decode_failures=decode_failures,rejected=dict(rejected),texture_failures=texture_failures,textures=textures,triangles=triangles,display_filter=dict(min_abs_normal_up=.7,max_centroid_RNW_height_difference_metres=.4),limitations=['Display-only RNW proximity filter; not physics or painted-edge classification.','Unknown/dynamic/nested instance models omitted explicitly.','HighestLOD0 and texturegroup0; static literal model matrices preserved.','GS shader/reflection/lighting state not emulated; vertexcolors retained separately only when decoded.'])
    return report


def encode(report):
    """Indexed GTV1, little endian. Texture payload offsets are absolute."""
    vertices=[];vertex_ids={};triangles=[]
    textures=report['textures'];texture_ids={tuple(t['key']):i for i,t in enumerate(textures)}
    for triangle in report['triangles']:
        indices=[]
        colors=triangle['colors_rgba8'] or [[128]*4]*3
        for p,uv,color in zip(triangle['p'],triangle['uv'],colors):
            # Exact native-to-core axis transform. No route fitting or scale fudge.
            values=[p[0],-p[2],p[1],*uv]
            if not all(math.isfinite(v)for v in values):raise ValueError('Nonfinite visual vertex')
            packed=struct.pack('<5f4B',*values,*color)
            if packed not in vertex_ids:
                vertex_ids[packed]=len(vertices);vertices.append(packed)
            indices.append(vertex_ids[packed])
        triangles.append(struct.pack('<7I',*indices,texture_ids[tuple(triangle['texture'])],triangle['model'],triangle['shape'],triangle['source_offset']))
    header=b'GTV1'+struct.pack('<7I',len(vertices),len(triangles),len(textures),24,28,40,0)
    payload_offset=32+24*len(vertices)+28*len(triangles)+40*len(textures)
    records=[];payload=[]
    for texture in textures:
        rgba=texture['rgba'];assert len(rgba)==texture['width']*texture['height']*4
        records.append(struct.pack('<IIQQIIII',texture['width'],texture['height'],int(texture['clamp'],16),int(texture['tex0'],16),payload_offset,len(rgba),texture['source_tex1_offset'],texture['index']))
        payload.append(rgba);payload_offset+=len(rgba)
    raw=header+b''.join(vertices)+b''.join(triangles)+b''.join(records)+b''.join(payload)
    assert len(raw)==payload_offset
    return gzip.compress(raw,compresslevel=9,mtime=0),len(vertices),len(triangles),len(textures),len(raw)


def export_one(task):
    global AUDIT,OUT
    name,audit,destination,receipt_dir=task
    AUDIT=Path(audit);OUT=Path(receipt_dir)
    start=time.monotonic()
    try:
        report=export(name)
        data,nv,nt,nx,raw_bytes=encode(report)
        if nt==0:raise ValueError('No supported source ground triangles; explicit fallback required')
        dest=Path(destination)/(name+'.gtv.gz');dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(data)
        summary={k:v for k,v in report.items()if k not in ['triangles','textures']}
        summary.update(status='exported-supported-static-ground',vertex_count=nv,triangle_count=nt,texture_count=nx,
            uncompressed_bytes=raw_bytes,compressed_bytes=len(data),output_file=dest.name,output_sha256=hashlib.sha256(data).hexdigest(),
            elapsed_seconds=time.monotonic()-start,texture_modes=dict(collections.Counter(str(((int(t['tex0'],16)>>35)&3,(int(t['tex0'],16)>>34)&1))for t in report['textures'])))
    except Exception as error:
        summary=dict(course=name,status='explicit-unsupported-fallback',error=str(error),elapsed_seconds=time.monotonic()-start)
    OUT.mkdir(parents=True,exist_ok=True);(OUT/(name+'-visual.json')).write_text(json.dumps(summary,indent=2)+'\n')
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-root',type=Path,default=AUDIT,help='Contains extracted/crs and decoded/meshes source inputs')
    parser.add_argument('--out',type=Path,default=ROOT/'crates/neural_racing_core/src/circuit_data/gt4_visual')
    parser.add_argument('--receipts',type=Path,default=OUT)
    parser.add_argument('--binding',type=Path,default=ROOT/'crates/neural_racing_core/src/circuit_data/gt4_visual_files.rs')
    parser.add_argument('--course',action='append',default=[])
    parser.add_argument('--all-available',action='store_true')
    parser.add_argument('--jobs',type=int,default=1,choices=range(1,5))
    args=parser.parse_args()
    names=args.course
    catalog=ROOT/'crates/neural_racing_core/src/circuit_data/gt4_catalog.rs'
    if args.all_available:names=[re.search(r'id: "([^"]+)"',line).group(1)for line in catalog.read_text().splitlines()if 'available: true' in line]
    if not names:parser.error('Select --course ID or --all-available')
    args.out.mkdir(parents=True,exist_ok=True);args.receipts.mkdir(parents=True,exist_ok=True)
    tasks=[(name,str(args.audit_root.resolve()),str(args.out.resolve()),str(args.receipts.resolve()))for name in names]
    summaries=[];start=time.monotonic()
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs)as pool:
        for summary in pool.map(export_one,tasks):
            summaries.append(summary);print(summary['course'],summary['status'],summary.get('triangle_count'),summary.get('compressed_bytes'),flush=True)
    manifest=dict(schema='indexed-GTV1-gzip',source='original supplied GT4 CRS MDLS/Tex1/VIF, no route scaling',requested=len(names),
        supported=sum(s['status']=='exported-supported-static-ground'for s in summaries),elapsed_seconds=time.monotonic()-start,
        decoder_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in [Path(__file__),Path(__file__).with_name('visual_vif.py'),Path(__file__).with_name('visual_tex1.py')]},courses=summaries)
    (args.receipts/'visual-export-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if args.all_available:
        # Missing resources are omitted explicitly, making runtime fallback honest.
        lines=['// Generated original GT4 static visuals; see scripts/gt4/export_visual_ground.py.','&[']
        for summary in summaries:
            if summary['status']!='exported-supported-static-ground':continue
            path=args.out.resolve()/summary['output_file'];relative=__import__('os').path.relpath(path,args.binding.resolve().parent)
            lines.append(f'    ("{summary["course"]}", include_bytes!("{relative}")),')
        lines.append(']');args.binding.write_text('\n'.join(lines)+'\n')
    print('supported',manifest['supported'],'requested',manifest['requested'],'seconds',manifest['elapsed_seconds'])


if __name__=='__main__':main()
