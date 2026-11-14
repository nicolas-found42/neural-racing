"""Tex1 diagnostic decoder for observed CAR4 PSMCT32 transfers/PSMT4 textures.

GS address permutations follow Play! 1f27232772e484bfb79364b751b84f41f4c87481,
Source/gs/GsPixelFormats.{h,cpp}; its BSD license is retained alongside this file.
Container offsets are independently recovered from original Tex1 bytes.
"""
import json
import struct
from pathlib import Path
from PIL import Image, ImageDraw

B32 = [[0,1,4,5,16,17,20,21],[2,3,6,7,18,19,22,23],
       [8,9,12,13,24,25,28,29],[10,11,14,15,26,27,30,31]]
B4 = [[0,2,8,10],[1,3,9,11],[4,6,12,14],[5,7,13,15],
      [16,18,24,26],[17,19,25,27],[20,22,28,30],[21,23,29,31]]
C32 = [[0,1,4,5,8,9,12,13],[2,3,6,7,10,11,14,15]]
W4 = [C32, [[8,9,12,13,0,1,4,5],[10,11,14,15,2,3,6,7]]]


def address32(x,y,base,width):
    page = x//64 + y//32*width
    x,y = x%64,y%32
    return (base*256+page*8192+B32[y//8][x//8]*256+(y%8//2)*64+C32[y%2][x%8]*4)&0x3fffff


def pixel4(mem,x,y,base,width):
    page = x//128 + y//128*(width*64//128)
    x,y = x%128,y%128
    block = B4[y//16][x//32]
    x,y = x%32,y%16
    column = y//4
    x,y = x,y%4
    shift = (x&24)+((y&2)<<1)
    sub = ((y&2)>>1)^(column&1)
    addr = base*256+page*8192+block*256+column*64+W4[sub][y&1][x&7]*4
    return (struct.unpack_from('<I',mem,addr&0x3fffff)[0]>>shift)&15


def pixel8(mem,x,y,base,width):
    page=x//128+y//64*(width*64//128)
    x,y=x%128,y%64
    block=B32[y//16][x//16]
    x,y=x%16,y%16
    column=y//4
    x,y=x,y%4
    byte=((x&8)>>2)+((y&2)>>1)
    sub=((y&2)>>1)^(column&1)
    addr=base*256+page*8192+block*256+column*64+W4[sub][y&1][x&7]*4+byte
    return mem[addr&0x3fffff]


def decode_set(b,out=None):
    assert b[:4]==b'Tex1'
    n,transfers=struct.unpack_from('<HH',b,0x14)
    infos,tx=struct.unpack_from('<II',b,0x18)
    mem=bytearray(4*1024*1024)
    for i in range(transfers):
        pos,dest,width,psm,w,h=struct.unpack_from('<IHBBHH',b,tx+12*i)
        assert psm==0 and width>0
        assert pos+w*h*4<=len(b)
        for y in range(h):
            for x in range(w):
                a=address32(x,y,dest,width)
                start=pos+(y*w+x)*4
                mem[a:a+4]=b[start:start+4]
    if out is not None: out.mkdir(parents=True,exist_ok=True)
    textures=[]
    for i in range(n):
        t=struct.unpack_from('<Q',b,infos+40*i)[0]
        base,width,psm=t&16383,t>>14&63,t>>20&63
        w,h=1<<(t>>26&15),1<<(t>>30&15)
        cbp,cpsm,csm,csa=t>>37&16383,t>>51&15,t>>55&1,t>>56&31
        assert psm in (19,20) and cpsm==0 and csm==0,(i,psm,cpsm,csm)
        palette=[None]*(256 if psm==19 else 16)
        for j in range(len(palette)):
            ix=j if psm==19 else j+csa*16
            source=(ix&~0x18)|((ix&8)<<1)|((ix&16)>>1)
            a=address32(j%16,j//16,cbp,1) if psm==19 else address32(source%16,source//16,cbp,1)
            r,g,blue,alpha=mem[a:a+4]
            ix=((j&~0x18)|((j&8)<<1)|((j&16)>>1)) if psm==19 else j
            palette[ix]=(r,g,blue,min(255,alpha*2))
        read=pixel8 if psm==19 else pixel4
        pixels=[palette[read(mem,x,y,base,width)]for y in range(h)for x in range(w)]
        image=Image.new('RGBA',(w,h));image.putdata(pixels)
        if out is not None: image.save(out/f'{i:03}.png')
        clamp=struct.unpack_from('<Q',b,infos+40*i+32)[0]
        textures.append(dict(index=i,width=w,height=h,tex0=hex(t),clamp=hex(clamp),csa=csa,rgba=image.tobytes()))
    if out is not None:
        sheet=Image.new('RGBA',(8*180,((n+7)//8)*180),(50,50,50,255));draw=ImageDraw.Draw(sheet)
        for i,t in enumerate(textures):
            image=Image.open(out/f'{i:03}.png');image.thumbnail((170,150))
            x,y=(i%8)*180,(i//8)*180
            sheet.alpha_composite(image,(x,y+20));draw.text((x+2,y+2),str(i),fill='white')
        sheet.save(out/'contact-sheet.png')
        (out/'metadata.json').write_text(json.dumps([{k:v for k,v in t.items()if k!='rgba'}for t in textures],indent=2)+'\n')
    return textures
