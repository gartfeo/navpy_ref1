/**
 * Vehicles whose rally table does not match their mission after an upload.
 *
 * The backend sets one rally point at each mission's home hub, so every RTL
 * flies there and circles it, or clears the table when the mission has no
 * home hub. A failure here means an RTL may go to HOME or to an
 * older hub. It is reported separately from the mission result: the mission
 * itself is on the vehicle.
 *
 * @param {object|null} result - the /upload response
 * @returns {Array<{sysId:number, action:string, error:string|null, failedParams:string[]}>}
 */
export function rallyUploadFailures(result) {
  if (!result) return [];
  return (result.results || [])
    .filter((r) => r.rally && r.rally.applied === false)
    .map((r) => ({
      sysId: r.sys_id,
      action: r.rally.action,
      error: r.rally.error || null,
      failedParams: r.rally.failed_params || [],
    }));
}

/**
 * One alert line per failure, saying where an RTL would go instead. A failed
 * set leaves whatever the vehicle held before: no rally point (RTL goes to the
 * launch point) or a previous mission's hub, so the line names both.
 */
export function rallyFailureLines(failures) {
  return failures.map((f) => {
    const what = f.action === 'clear'
      ? 'old rally point not cleared, RTL may fly to an older hub'
      : "RTL may go to the launch point or to a previous mission's hub, not this hub";
    const params = f.failedParams.length ? ` (${f.failedParams.join(', ')})` : '';
    return `  Vehicle ${f.sysId}: ${f.error || `${f.action} failed`}${params}; ${what}`;
  });
}
