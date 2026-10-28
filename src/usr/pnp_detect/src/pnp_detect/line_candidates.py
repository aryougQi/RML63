"""Bounded Hough quadrilateral proposals for interrupted/rounded rims."""
from itertools import combinations
import numpy as np
from .geometry import order_corners, validate_quad, line_intersection


def propose_quads(lines, width, height, cfg, roi=None):
    opts=cfg.get('line_proposals',{})
    if not opts.get('enabled',False): return [], {'enabled':False}
    max_lines=int(opts.get('max_lines',24))
    if not 4 <= max_lines <= 40: raise ValueError('line_proposals.max_lines must be in [4,40]')
    selected=[]
    # Merge nearly identical Hough segments before combinatorial enumeration.
    for raw in sorted(lines,key=lambda x:(-np.linalg.norm(np.array(x[2:])-x[:2]),x)):
        a,b=np.asarray(raw,dtype=float).reshape(2,2);v=b-a;length=np.linalg.norm(v)
        if length < cfg['hough_min_line_length']:continue
        u=v/length
        if any(abs(u@w)>np.cos(np.deg2rad(4)) and abs(w[0]*(a-c)[1]-w[1]*(a-c)[0])<opts.get('merge_distance_px',3) for c,w in selected):continue
        selected.append((a,u))
        if len(selected)>=max_lines:break
    pairs=[]
    opposite_cos=np.cos(np.radians(opts.get('max_opposite_angle_deg',28)))
    for i,j in combinations(range(len(selected)),2):
        a,u=selected[i];b,v=selected[j]
        if abs(u@v)<opposite_cos:continue
        if abs(u[0]*(b-a)[1]-u[1]*(b-a)[0])<cfg['min_side_px']:continue
        pairs.append((i,j))
    proposals=[];tested=0;seen=set()
    for first,second in combinations(pairs,2):
        if len(set(first+second))<4:continue
        if abs(selected[first[0]][1]@selected[second[0]][1])>np.cos(np.deg2rad(40)):continue
        key=tuple(sorted(first+second))
        if key in seen:continue
        seen.add(key);tested+=1
        try:q=order_corners([line_intersection(selected[i],selected[j]) for i in first for j in second])
        except ValueError:continue
        if not validate_quad(q,width,height,**cfg)[0]:continue
        if roi is not None:
            x,y,w,h=roi
            if ((q[:,0]<x)|(q[:,0]>=x+w)|(q[:,1]<y)|(q[:,1]>=y+h)).any():continue
        vectors=np.roll(q,-1,axis=0)-q
        vectors=vectors/np.linalg.norm(vectors,axis=1)[:,None]
        opposite=[np.degrees(np.arccos(np.clip(abs(vectors[i]@vectors[i+2]),0,1))) for i in (0,1)]
        if min(opposite)>cfg.get('max_near_parallel_pair_angle_deg',90)+3:continue
        # Dedup only almost identical vertices; retain inner/outer rim alternatives.
        if proposals:
            pool=np.asarray(proposals)
            if any(np.any(np.max(np.linalg.norm(q-np.roll(pool,k,axis=1),axis=2),axis=1)<2) for k in range(4)):continue
        proposals.append(q)
        if len(proposals)>cfg['max_candidates']:
            return proposals,{'enabled':True,'proposal_limit_exceeded':True,'selected_line_count':len(selected),'proposal_count':len(proposals)}
    return proposals,{'enabled':True,'input_line_count':len(lines),'line_pool_limit_reached':len(selected)>=max_lines,'selected_line_count':len(selected),'tested_line_combinations':tested,'proposal_count':len(proposals)}


def rim_polarity(gray, q, opts):
    """Outer dark rim should be darker immediately inside each straight side.

    Image-space clockwise polygon has its inward normal on the right of each edge.
    This is a configurable basket appearance prior, not a universal object test.
    """
    offset=float(opts.get('sample_offset_px',4))
    contrasts=[]
    for a,b in zip(q,np.roll(q,-1,axis=0)):
        v=b-a;u=v/np.linalg.norm(v);normal=np.array([-u[1],u[0]])
        p=np.linspace(a+v*0.18,b-v*0.18,max(8,int(np.linalg.norm(v)*0.64)))
        # A thin/rounded far rim can move by a few pixels around the fitted line.
        # Search only the configured edge tolerance, retaining the signed contrast.
        deltas=[]
        radius=float(opts.get('normal_search_px',0))
        for shift in np.arange(-radius,radius+0.01,1):
            inside=np.rint(p+(shift+offset)*normal).astype(int)
            outside=np.rint(p+(shift-offset)*normal).astype(int)
            valid=((inside[:,0]>=0)&(inside[:,0]<gray.shape[1])&(inside[:,1]>=0)&(inside[:,1]<gray.shape[0])&
                   (outside[:,0]>=0)&(outside[:,0]<gray.shape[1])&(outside[:,1]>=0)&(outside[:,1]<gray.shape[0]))
            delta=np.full(len(p),-255.,dtype=float)
            i,o=inside[valid],outside[valid]
            delta[valid]=gray[o[:,1],o[:,0]].astype(float)-gray[i[:,1],i[:,0]].astype(float)
            deltas.append(delta)
        contrasts.append(float(np.median(np.max(deltas,axis=0))))
    return contrasts
