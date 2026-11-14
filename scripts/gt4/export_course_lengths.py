#!/usr/bin/env python3
"""Freeze original race distances and published non-lap navigation extents for tests."""
import argparse
import hashlib
import json
import struct
from pathlib import Path


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--metadata",type=Path,required=True)
    ap.add_argument("--decoded",type=Path,required=True)
    ap.add_argument("--stations",type=Path,required=True)
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args()
    result={"source":"Original RNW TrackV/selected RACE sections; Area extents are native-V navigation only, never race distances", "courses":{}}
    for course in json.loads(args.metadata.read_text())["courses"]:
        resource=json.loads((args.decoded/(course["id"]+".json")).read_text())
        period=resource["runway"]["track_v_m"]
        distance=period
        row={"source_sha256":course["source_sha256"],"closed":course["topology"]=="Closed"}
        if course["topology"]=="Open":
            distance=(course["goal_v"]-course["start_v"])%period
        elif course["topology"]=="Area":
            path=args.stations/(course["id"]+".stations")
            if path.is_file():
                b=path.read_bytes();count=struct.unpack_from("<I",b)[0]
                if len(b)!=4+count*32 or count<2:raise ValueError("invalid navigation stations")
                values=[struct.unpack_from("<f",b,4+i*32)[0]for i in range(count)]
                distance=(max(values)-min(values))%period
                row["navigation_stations_sha256"]=hashlib.sha256(b).hexdigest()
            row["navigation_only"]=True
        row["length_metres"]=distance
        result["courses"][course["id"]]=row
    args.output.write_text(json.dumps(result,indent=2)+"\n")

if __name__=="__main__":main()
