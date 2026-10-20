/**
 * Camera-mark history for the whole fleet, keyed by sys_id. `order` lists
 * the sys_id of every stored mark, oldest first, so the cap evicts the
 * fleet's oldest mark whichever UAV made it.
 */
export function newCoverageHistory() {
  return { bySid: {}, order: [] };
}

/** The UAV's mark list and sampling state, created on first use. */
export function coverageFor(hist, sid) {
  if (!hist.bySid[sid]) {
    hist.bySid[sid] = { marks: [], lastPos: null, lastAttemptedGimbalSignatures: null, colorIdx: -1 };
  }
  return hist.bySid[sid];
}

/** Store one mark ({ entity, alpha }) for sid. */
export function recordMark(hist, sid, mark) {
  coverageFor(hist, sid).marks.push(mark);
  hist.order.push(sid);
}

/** Total stored marks over the fleet. */
export function markCount(hist) {
  return hist.order.length;
}

/** Evict the fleet's oldest marks until at most max remain. */
export function evictOverCap(hist, max, remove) {
  let total = hist.order.length;
  while (total > max) {
    const sid = hist.order.shift();
    const mark = hist.bySid[sid]?.marks.shift();
    if (mark) remove(mark.entity);
    total--;
  }
}

/** Remove every mark of one UAV (it left the fleet). */
export function dropSid(hist, sid, remove) {
  const cov = hist.bySid[sid];
  if (!cov) return;
  for (const mark of cov.marks) remove(mark.entity);
  delete hist.bySid[sid];
  hist.order = hist.order.filter((s) => s !== sid);
}

/** Remove every mark and reset sampling anchors. */
export function clearCoverage(hist, remove) {
  for (const cov of Object.values(hist.bySid)) {
    for (const mark of cov.marks) remove(mark.entity);
  }
  hist.bySid = {};
  hist.order = [];
}
