"""Original GT4 MDLS VIF packet decoding; preserves native source attributes."""
import struct

def decode_packet(data,packed,scale):
    pos=0;mode=0;row=[0,0,0,0];vertices={};uvs={};colors={};strips=None;commands=[]
    while pos+4<=len(data):
        word=struct.unpack_from('<I',data,pos)[0];cmd=(word>>24)&127;n=(word>>16)&255;imm=word&65535
        pos+=4;nb=0;commands.append(hex(cmd))
        if cmd==5:mode=imm
        elif cmd==0x30:
            row=list(struct.unpack_from('<4i',data,pos));nb=16
        elif cmd==0x20:nb=4
        elif cmd==1:
            assert (imm&255)==(imm>>8),'Nontrivial STCYCL requires explicit VU addressing'
        elif cmd&0x60==0x60:
            count=n or 256;vn=((cmd>>2)&3)+1;vl=cmd&3;assert vl!=3
            width=[4,2,1][vl];nb=count*vn*width;raw=data[pos:pos+nb];assert len(raw)==nb
            addr=imm&1023;unsigned=bool(imm&0x4000)
            fmt=('<'+('I' if unsigned else 'i')) if width==4 else ('<'+('H' if unsigned else 'h')) if width==2 else ('<B' if unsigned else '<b')
            values=[v[0]for v in struct.iter_unpack(fmt,raw)]
            if vn==3 and addr<64:
                for i in range(count):
                    v=values[i*3:i*3+3]
                    if packed:
                        v=[x+row[k] if mode in (1,2) else x for k,x in enumerate(v)]
                        if mode==2:row[:3]=v
                        point=[x*scale for x in v]
                    else:
                        assert width==4 and mode==0
                        point=list(struct.unpack_from('<3f',raw,i*12))
                    vertices[addr+i]=point
            elif vn==2 and 64<=addr<128:
                for i in range(count):
                    v=values[i*2:i*2+2]
                    if packed or width!=4:
                        v=[x+row[k] if mode in (1,2) else x for k,x in enumerate(v)]
                        if mode==2:row[:2]=v
                        uv=[x/4096 for x in v]
                    else:
                        assert mode==0
                        uv=list(struct.unpack_from('<2f',raw,i*8))
                    uvs[addr-64+i]=uv
            elif vn==4 and width==1 and addr==128:
                for i in range(count):colors[i]=list(raw[i*4:i*4+4])
            elif cmd==0x72 and addr==64:
                assert mode==0;strips=list(raw)
        elif cmd not in (0,):raise ValueError(('Unknown command',hex(cmd)))
        pos+=(nb+3)//4*4
    assert strips and strips[0]+1==len(strips)
    lengths=[x//3+2 for x in strips[1:]];assert all(x%3==0 for x in strips[1:])
    count=sum(lengths);assert set(vertices)==set(range(count)),(len(vertices),count)
    assert not uvs or set(uvs)==set(range(count)),('UV address coverage',len(uvs),count)
    assert not colors or len(colors)==count
    return dict(vertices=[vertices[i]for i in range(count)],uvs=[uvs[i]for i in range(count)] if uvs else None,colors_rgba8=[colors[i]for i in range(count)] if colors else None,strip_lengths=lengths,commands=commands)
