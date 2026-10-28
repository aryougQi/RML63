"""Conservative geometry-based hypotheses; no learned semantic basket detector."""
import cv2
import numpy as np
from .geometry import order_corners, validate_quad, line_intersection
from .preprocessing import preprocess, scale_detection_parameters
from .line_candidates import propose_quads, rim_polarity
from .region_candidates import region_proposals
from .opencv_quads import opencv_quad_proposals
from .docdetect_adapter import docdetect_proposals


def _samples(a, b):
    return np.linspace(a, b, max(2, int(np.linalg.norm(b-a))+1))


def _longest_gap(supported):
    longest = run = 0
    for x in supported:
        run = 0 if x else run+1
        longest = max(longest, run)
    return longest / len(supported)


def _refine(q, edge_xy, cfg):
    fitted = []
    for a,b in zip(q, np.roll(q,-1,axis=0)):
        v = b-a; length = np.linalg.norm(v); u = v/length
        rel = edge_xy-a; along = rel@u
        distance = np.abs(rel[:,0]*u[1]-rel[:,1]*u[0])
        near = edge_xy[(distance <= cfg["refine_distance_px"]) & (along >= 0.08*length) & (along <= 0.92*length)]
        if len(near) < 12: return q, False
        vx,vy,x,y = cv2.fitLine(near.astype(np.float32), cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
        fitted.append(((x,y),(vx,vy)))
    try:
        refined = np.array([line_intersection(fitted[(i-1)%4], fitted[i]) for i in range(4)])
        if np.max(np.linalg.norm(refined-q, axis=1)) > cfg["max_refine_shift_px"]: return q, False
        return order_corners(refined), True
    except ValueError:
        return q, False


def _line_support(a, b, lines, cfg):
    v = b-a; length = np.linalg.norm(v); u = v/length
    intervals = []
    for line in lines:
        c,e = np.asarray(line,dtype=float).reshape(2,2)
        w = e-c; n = np.linalg.norm(w)
        if n < 1: continue
        if abs(w@u/n) < np.cos(np.radians(cfg["line_angle_tolerance_deg"])): continue
        if max(abs(u[0]*(c-a)[1]-u[1]*(c-a)[0]), abs(u[0]*(e-a)[1]-u[1]*(e-a)[0])) > cfg["line_distance_px"]: continue
        lo,hi = sorted(((c-a)@u,(e-a)@u)); lo=max(0.,lo); hi=min(length,hi)
        if hi > lo: intervals.append((lo,hi))
    total=0.; end=0.
    for lo,hi in sorted(intervals):
        total += max(0.,hi-max(lo,end)); end=max(end,hi)
    return total/length


def detect(image, config):
    cv2.setNumThreads(1); cv2.setRNGSeed(0)
    config,parameter_scale=scale_detection_parameters(config,image.shape[1],image.shape[0])
    debug = preprocess(image, config); d = config["detection"]
    edges = debug["edges"]; height,width = edges.shape
    contours,_ = cv2.findContours(debug["closed_edges"], cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda c: (-abs(cv2.contourArea(c)), cv2.boundingRect(c)))
    contour_view = image.copy(); cv2.drawContours(contour_view, contours, -1, (110,150,80), 1)
    debug["contours"] = contour_view
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, d["hough_threshold"],
                            minLineLength=d["hough_min_line_length"], maxLineGap=d["hough_max_line_gap"])
    lines = [] if lines is None else lines.reshape(-1,4).tolist()
    line_view = image.copy()
    for x1,y1,x2,y2 in lines: cv2.line(line_view,(x1,y1),(x2,y2),(0,200,255),1)
    debug["hough_lines"] = line_view
    distance = cv2.distanceTransform(255-edges, cv2.DIST_L2, 3)
    ys,xs = np.nonzero(edges); edge_xy = np.column_stack((xs,ys)).astype(float)
    candidates=[]; rejected=[]; evaluated=0; truncated=False
    proposals=[]
    for contour in contours:
        area = abs(cv2.contourArea(contour))/(height*width)
        if not d["min_area_ratio"] <= area <= d["max_area_ratio"]: continue
        for epsilon in d["approx_epsilon_ratios"]:
            approx = cv2.approxPolyDP(contour, epsilon*cv2.arcLength(contour,True), True)
            if len(approx) == 4:
                try: proposals.append((order_corners(approx.reshape(4,2)),contour,"closed_contour"))
                except ValueError: pass
    external_quads,external_stats=docdetect_proposals(image,d)
    proposals.extend((q,None,"docdetect_upstream") for q in external_quads)
    if external_stats.get("truncated",False):truncated=True
    sample_quads,sample_stats=opencv_quad_proposals(image,d,debug["search_mask"])
    proposals.extend(sample_quads)
    if sample_stats.get("truncated",False):truncated=True
    region_quads,region_debug=region_proposals(image,d,debug["search_mask"])
    proposals.extend(region_quads);debug.update(region_debug)
    line_quads,line_stats=propose_quads(lines,width,height,d,config["preprocessing"]["roi"])
    proposals.extend((q,None,"hough_line_combination") for q in line_quads)
    for q,contour,source in proposals:
        if evaluated >= d["max_candidates"]:
            truncated=True; break
        evaluated += 1
        ok,metrics=validate_quad(q,width,height,**d)
        if not ok:
            rejected.append(metrics); continue
        q,refined=_refine(q,edge_xy,d)
        ok,metrics=validate_quad(q,width,height,**d)
        if not ok:
            rejected.append(metrics); continue
        supports=[]; gaps=[]; line_support=[]; endpoint_gaps=[]
        for a,b in zip(q,np.roll(q,-1,axis=0)):
            pts=np.rint(_samples(a,b)).astype(int)
            supported=distance[pts[:,1],pts[:,0]] <= d["edge_distance_px"]
            supports.append(float(supported.mean())); gaps.append(_longest_gap(supported))
            visible=np.flatnonzero(supported)
            endpoint_gaps.append(float(max(visible[0],len(supported)-1-visible[-1])) if visible.size else float(len(supported)))
            line_support.append(float(_line_support(a,b,lines,d)))
        coverage=min(abs(cv2.contourArea(contour))/abs(cv2.contourArea(q.astype(np.float32))),1.) if contour is not None else None
        # Line hypotheses do not have a closed source contour: do not invent one.
        score=0.70*np.mean(supports)+0.30*np.mean(line_support)
        polarity_opts=d.get("rim_polarity",{})
        polarity=rim_polarity(debug["gray"],q,polarity_opts) if polarity_opts.get("enabled",False) else None
        metrics.update(edge_support=float(np.mean(supports)), side_edge_support=supports,
                       side_line_support=line_support, max_unsupported_run_ratio=max(gaps),
                       side_endpoint_gap_px=endpoint_gaps,
                       side_longest_gap_px=[float(gap*(int(length)+1)) for gap,length in zip(gaps,metrics["side_lengths_px"])],
                       contour_coverage=float(coverage) if coverage is not None else None,
                       score=float(score), line_refined=refined,proposal_source=source,
                       side_outer_minus_inner_gray=polarity)
        vectors=np.roll(q,-1,axis=0)-q
        vectors=vectors/np.linalg.norm(vectors,axis=1)[:,None]
        opposite_angles=[float(np.degrees(np.arccos(np.clip(abs(vectors[i]@vectors[i+2]),0,1)))) for i in (0,1)]
        metrics["opposite_edge_angles_deg"]=opposite_angles
        reason=None
        if min(opposite_angles)>d.get("max_near_parallel_pair_angle_deg",90): reason="inconsistent_near_overhead_perspective"
        elif min(supports)<d["min_side_support"] or np.mean(supports)<d["min_edge_support"]: reason="insufficient_real_edge_support"
        elif max(endpoint_gaps)>d.get("max_endpoint_gap_px",float("inf")): reason="unsupported_corner_extrapolation"
        elif max(metrics["side_longest_gap_px"])>d.get("max_unsupported_gap_px",float("inf")): reason="excessive_absolute_edge_gap"
        elif max(gaps)>d["max_unsupported_run_ratio"]: reason="possible_occlusion_or_broken_rim"
        elif coverage is not None and coverage<d["min_contour_coverage"]: reason="poor_closed_contour_coverage"
        elif min(line_support)<d["min_line_support"]: reason="insufficient_hough_line_support"
        elif polarity is not None and min(polarity)<polarity_opts.get("min_side_contrast",8): reason="inconsistent_outer_dark_rim_polarity"
        if reason:
            rejected.append(dict(metrics,reason=reason,corners=q.tolist())); continue
        candidates.append({"corners":q.tolist(),"metrics":metrics})
    # 多次近似及轮廓内外双边缘只形成一个候选；明显不同的上沿内外圈保留为歧义。
    unique=[]
    for candidate in sorted(candidates,key=lambda c:(-c["metrics"]["score"],c["corners"])):
        q=np.array(candidate["corners"])
        if not any(min(np.max(np.linalg.norm(q-np.roll(np.array(c["corners"]),k,axis=0),axis=1)) for k in range(4)) <= d["duplicate_distance_px"] for c in unique):
            unique.append(candidate)
    debug["candidates"] = image.copy()
    for i,c in enumerate(unique):
        pts=np.rint(c["corners"]).astype(np.int32)
        cv2.polylines(debug["candidates"],[pts],True,(0,180,255),2)
        cv2.putText(debug["candidates"],f'{i}: {c["metrics"]["score"]:.3f}',tuple(pts[0]),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,180,255),1)
    # Rejected hypotheses are explicitly labelled, never emitted as successful corners.
    ranked_rejected=sorted((c for c in rejected if "corners" in c),key=lambda c:-c.get("score",0))
    for i,c in enumerate(ranked_rejected[:6]):
        view=image.copy();pts=np.rint(c["corners"]).astype(np.int32)
        cv2.polylines(view,[pts],True,(0,165,255),1)
        cv2.putText(view,"REJECTED: "+c["reason"],(5,20),cv2.FONT_HERSHEY_SIMPLEX,0.36,(0,80,255),1)
        debug[f"rejected_{i:02d}"]=view
    reason=None
    if external_stats.get("failure_reason"): reason=external_stats["failure_reason"]
    elif truncated: reason="candidate_limit_exceeded: narrow ROI or tune configuration"
    elif not unique: reason="no_reliable_rim: no quadrilateral passed geometry, edge, gap, polarity and line checks"
    elif len(unique)>1 and unique[0]["metrics"]["score"]-unique[1]["metrics"]["score"] < d["ambiguity_score_margin"]:
        reason="ambiguous_rims: multiple similarly supported quadrilaterals; use ROI/color prior or manual annotation"
    result={"success":reason is None,"corners":unique[0]["corners"] if reason is None else [],
            "parameter_scale":parameter_scale,"success_scope":"passed_configured_image_checks_only",
            "corner_definition":"virtual_intersections_of_fitted_straight_rim_edges; not rounded contour extrema",
            "failure_reason":reason,"method":"contour_region_or_hough_quad_robust_line_intersections",
            "quality_metrics":unique[0]["metrics"] if unique else {},
            "diagnostics":{"docdetect":external_stats,"opencv_quads":sample_stats,"line_proposals":line_stats,"contour_count":len(contours),"evaluated_count":evaluated,"candidate_limit_exceeded":truncated,
                           "accepted_candidates":unique,"rejected_candidates":rejected}}
    return result,debug
