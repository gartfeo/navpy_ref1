#!/usr/bin/env python3
"""Analyze raw JSONL from measure_zr10_latency.py without hiding spread."""
import argparse, json, math, statistics
from pathlib import Path


def percentile(xs, p):
    if not xs: return None
    ys = sorted(xs); k = (len(ys)-1)*p/100; lo, hi = math.floor(k), math.ceil(k)
    return ys[lo] if lo == hi else ys[lo] + (ys[hi]-ys[lo])*(k-lo)


def stats_ms(ns_values):
    xs = [x/1e6 for x in ns_values if x is not None]
    if not xs: return {"n": 0}
    return {"n": len(xs), "min_ms": min(xs), "median_ms": statistics.median(xs),
            "p95_ms": percentile(xs,95), "p99_ms": percentile(xs,99),
            "max_ms": max(xs), "spread_ms": max(xs)-min(xs),
            "stdev_ms": statistics.stdev(xs) if len(xs)>1 else 0.0}


def main(path, report):
    rows=[json.loads(x) for x in path.read_text().splitlines() if x.strip()]
    starts=[r for r in rows if r["event"]=="gpio" and r["value"]==1]
    frames=[r for r in rows if r["event"]=="frame"]
    offs={r["event_id"]:r for r in rows if r["event"]=="gpio" and r["value"]==0}
    matched=[]; j=0
    for g in starts:
        off=offs.get(g["event_id"])
        while j < len(frames) and frames[j]["publication_mono_ns"] < g["post_write_mono_ns"]: j+=1
        k=j
        while off and k < len(frames) and frames[k]["publication_mono_ns"] <= off["post_write_mono_ns"] + 100_000_000:
            if frames[k]["led_on"]:
                matched.append((g,frames[k])); break
            k+=1
    latency=[t["publication_mono_ns"]-g["post_write_mono_ns"] for g,t in matched]
    packet_to_decode=[]; decode_to_publish=[]
    timestamp_offsets={"pts":[],"rtp":[],"packet_first":[],"packet_last":[],"decoder_output":[]}
    for g,t in matched:
        src=t.get("source") or {}; dec=src.get("decoder_output_mono_ns"); au=src.get("au") or {}; rtp=au.get("rtp") or {}
        pub=t["publication_mono_ns"]
        if dec is not None: decode_to_publish.append(pub-dec); timestamp_offsets["decoder_output"].append(dec-g["post_write_mono_ns"])
        if dec is not None and rtp.get("last_packet_mono_ns") is not None: packet_to_decode.append(dec-rtp["last_packet_mono_ns"])
        if t.get("pts_ns") is not None: timestamp_offsets["pts"].append(t["pts_ns"]-g["post_write_mono_ns"])
        if rtp.get("rtp_timestamp") is not None: timestamp_offsets["rtp"].append(rtp["rtp_timestamp"]*1e9/90000-g["post_write_mono_ns"])
        for k, name in [("first_packet_mono_ns","packet_first"),("last_packet_mono_ns","packet_last")]:
            if rtp.get(k) is not None: timestamp_offsets[name].append(rtp[k]-g["post_write_mono_ns"])
    pubs=[f["publication_mono_ns"] for f in frames]
    cadence=[b-a for a,b in zip(pubs,pubs[1:])]
    rtpframes=[r for r in rows if r["event"]=="rtp_frame"]
    seq_missing=sum(max(0, ((b["first_seq"]-a["last_seq"])&65535)-1) for a,b in zip(rtpframes,rtpframes[1:]))
    rtp_dups=sum(1 for a,b in zip(rtpframes,rtpframes[1:]) if a["rtp_timestamp"]==b["rtp_timestamp"])
    rtp_steps=[(b["rtp_timestamp"]-a["rtp_timestamp"]) & 0xffffffff for a,b in zip(rtpframes,rtpframes[1:])]
    pts=[f["pts_ns"] for f in frames if f.get("pts_ns") is not None]
    pts_dups=sum(a==b for a,b in zip(pts,pts[1:]))
    wall_span=(pubs[-1]-pubs[0]) if len(pubs)>1 else 0
    pts_span=(pts[-1]-pts[0]) if len(pts)>1 else 0
    result={"input":str(path),"stimuli":len(starts),"matched_led_on":len(matched),
            "led_on_to_first_visible":stats_ms(latency),
            "last_packet_to_decoder_output":stats_ms(packet_to_decode),
            "decoder_output_to_publication":stats_ms(decode_to_publish),
            "frame_cadence":stats_ms(cadence),"rtp_sequence_packets_missing":seq_missing,
            "consecutive_duplicate_rtp_timestamps":rtp_dups,
            "consecutive_duplicate_pts":pts_dups,
            "rtp_timestamp_step":stats_ms([x*1e9/90000 for x in rtp_steps]),
            "stream_wall_span_s":wall_span/1e9,"stream_pts_span_s":pts_span/1e9,
            "pts_minus_wall_span_s":(pts_span-wall_span)/1e9,
            "timestamp_offset_spread":{k:stats_ms(v) for k,v in timestamp_offsets.items()},
            "pipeline": next((r["pipeline"] for r in rows if r["event"]=="run_start"),None),
            "versions": next((r["versions"] for r in rows if r["event"]=="run_start"),None)}
    report.write_text(json.dumps(result,indent=2)+"\n")
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    p=argparse.ArgumentParser(); p.add_argument("log",type=Path); p.add_argument("--report",type=Path,required=True); a=p.parse_args(); main(a.log,a.report)
