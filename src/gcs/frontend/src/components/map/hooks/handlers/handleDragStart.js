import { pickCartographic, pickEntity } from '../../utils/entityPicking';

/**
 * LEFT_DOWN: identify drag target and set dragState.
 * Returns { dragState, skipNextClick } or null if no drag started.
 */
export function handleDragStart(Cesium, viewer, click, callbacksRef, refs) {
  const cb = callbacksRef.current;
  const { setCorridorPointsRef, activeSetIndex, lastCorridorDown } = refs;

  const hit = pickEntity(Cesium, viewer, click.position);
  const latlon = pickCartographic(Cesium, viewer, click.position);

  // Corridor midpoint click -> insert new corridor point into the correct set and start drag
  if (hit && hit.type === 'corridorMidpoint' && cb.onCorridorMidpointInsert && cb.editable) {
    const midSetIdx = hit.setIdx ?? (activeSetIndex ?? 0);
    if (midSetIdx !== (activeSetIndex ?? 0) && cb.onSwitchSet) cb.onSwitchSet(midSetIdx);
    const midIdx = hit.index;
    if (latlon) cb.onCorridorMidpointInsert(midIdx, latlon, midSetIdx);
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    return {
      dragState: { type: 'setCorridorPoint', setIdx: midSetIdx, corridorIndex: midIdx, isLastPoint: false },
      skipNextClick: true,
    };
  }

  // Corridor point drag (all sets, corridorIndex -1 = launch point)
  if (hit && hit.type === 'setCorridorPoint' && cb.onSetCorridorPointDrag && cb.editable) {
    // Double-click to delete corridor waypoints (not launch points)
    if (hit.corridorIndex >= 0) {
      const now = Date.now();
      if (lastCorridorDown.index === hit.corridorIndex && lastCorridorDown.setIdx === hit.setIdx && now - lastCorridorDown.time < 400) {
        refs.lastCorridorDown = { index: -1, setIdx: -1, time: 0 };
        if (hit.setIdx !== (activeSetIndex ?? 0) && cb.onSwitchSet) cb.onSwitchSet(hit.setIdx);
        if (cb.onCorridorPointDelete) cb.onCorridorPointDelete(hit.corridorIndex, hit.setIdx);
        return { dragState: null, skipNextClick: true };
      }
      refs.lastCorridorDown = { index: hit.corridorIndex, setIdx: hit.setIdx, time: now };
    }
    // Record pre-drag position for undo
    if (cb.onCorridorDragRecord) {
      const oldPos = hit.corridorIndex === -1
        ? refs.setLaunchPointsRef.current[hit.setIdx]
        : (refs.setCorridorPointsRef.current[hit.setIdx] || [])[hit.corridorIndex];
      if (oldPos) cb.onCorridorDragRecord(hit.setIdx, hit.corridorIndex, oldPos);
    }
    // Determine if this is the last approach-determining point
    const setCps = setCorridorPointsRef.current[hit.setIdx] || [];
    const isLastPoint = hit.corridorIndex === -1
      ? setCps.length === 0
      : hit.corridorIndex === setCps.length - 1;
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    if (isLastPoint && cb.onDragStart) cb.onDragStart();
    return {
      dragState: { type: 'setCorridorPoint', setIdx: hit.setIdx, corridorIndex: hit.corridorIndex, isLastPoint },
      skipNextClick: true,
    };
  }

  // Partition handle drag
  if (hit && hit.type === 'partitionHandle' && cb.onPartitionAngleDrag && cb.editable) {
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    return {
      dragState: { type: 'partitionHandle' },
      skipNextClick: true,
    };
  }

  // delivery hub drag — only when delivery hub edit mode is active
  if (hit && hit.type === 'deliveryHub' && cb.placingDeliveryHub && cb.onMoveDeliveryHub && cb.editable) {
    let deliveryHubEntity = null;
    const picks = viewer.scene.drillPick(click.position, 5);
    for (const p of picks) {
      if (Cesium.defined(p.id?.properties?.deliveryHubIndex)) { deliveryHubEntity = p.id; break; }
    }
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    return {
      dragState: { type: 'deliveryHub', index: hit.index, entity: deliveryHubEntity, lastLatLon: null },
      skipNextClick: true,
    };
  }

  // Fence handle drag — reshape the inclusion fence like the zone polygon.
  // Placed before the placement-mode guard so a fence vertex stays grabbable
  // in every mode (the LEFT_DOWN sets skipNextClick, so no stray point lands).
  if (hit && hit.type === 'fenceVertex' && cb.onFenceVertexDrag && cb.editable) {
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    return {
      dragState: { type: 'fenceVertex', index: hit.index },
      skipNextClick: true,
    };
  }
  if (hit && hit.type === 'fenceMidpoint' && cb.onFenceMidpointInsert && cb.editable) {
    // No terrain pick → no vertex was inserted; starting the drag anyway would
    // grab the ring's ORIGINAL next vertex (index+1) instead of a new one.
    if (!latlon) return null;
    cb.onFenceMidpointInsert(hit.index, latlon);
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    return {
      dragState: { type: 'fenceVertex', index: hit.index + 1 },
      skipNextClick: true,
    };
  }

  // Keep-out (exclusion) ring handles — same zone-style gestures, per ring.
  if (hit && hit.type === 'exclusionVertex' && cb.onExclusionVertexDrag && cb.editable) {
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    return {
      dragState: { type: 'exclusionVertex', ring: hit.ring, index: hit.index },
      skipNextClick: true,
    };
  }
  if (hit && hit.type === 'exclusionMidpoint' && cb.onExclusionMidpointInsert && cb.editable) {
    if (!latlon) return null;
    cb.onExclusionMidpointInsert(hit.ring, hit.index, latlon);
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    return {
      dragState: { type: 'exclusionVertex', ring: hit.ring, index: hit.index + 1 },
      skipNextClick: true,
    };
  }

  // Skip polygon drag handling during placement modes — unless clicking a vertex/midpoint
  if (cb.placingLaunchPoint || cb.placingCorridor || cb.placingDeliveryHub) {
    if (!hit || (hit.type !== 'vertex' && hit.type !== 'midpoint' && hit.type !== 'polygon')) return null;
  }

  if (!cb.editable) return null;
  if (!cb.polygon || cb.polygon.length < 1) return null;
  if (!latlon) return null;

  if (hit && hit.type === 'vertex') {
    if (cb.onPolygonDragRecord && cb.polygon) {
      cb.onPolygonDragRecord(cb.polygon.map(p => ({ lat: p.lat, lon: p.lon })));
    }
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    if (cb.onDragStart) cb.onDragStart();
    return {
      dragState: { type: 'vertex', index: hit.index },
      skipNextClick: true,
    };
  } else if (hit && hit.type === 'midpoint') {
    if (cb.onPolygonDragRecord && cb.polygon) {
      cb.onPolygonDragRecord(cb.polygon.map(p => ({ lat: p.lat, lon: p.lon })));
    }
    const midIdx = hit.index;
    if (cb.onMidpointInsert) {
      cb.onMidpointInsert(midIdx, latlon);
    }
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    if (cb.onDragStart) cb.onDragStart();
    return {
      dragState: { type: 'vertex', index: midIdx + 1 },
      skipNextClick: true,
    };
  } else if (!cb.isDrawing && hit && hit.type === 'polygon') {
    if (cb.onPolygonDragRecord && cb.polygon) {
      cb.onPolygonDragRecord(cb.polygon.map(p => ({ lat: p.lat, lon: p.lon })));
    }
    viewer.scene.screenSpaceCameraController.enableRotate = false;
    if (cb.onDragStart) cb.onDragStart();
    return {
      dragState: { type: 'polygon', startLatLon: latlon },
      skipNextClick: false,
    };
  }

  return null;
}
