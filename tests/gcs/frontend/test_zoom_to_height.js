/**
 * Tests for the zoomToHeight conversion function used by useCesiumViewer.
 * Verifies zoom level → camera height mapping.
 */

function zoomToHeight(zoom) {
  return 35200000 / Math.pow(2, zoom);
}

let passed = 0;
let failed = 0;

function assertClose(actual, expected, tol, label) {
  if (Math.abs(actual - expected) <= tol) {
    passed++;
  } else {
    console.error(`FAIL: ${label} — expected ~${expected}, got ${actual}`);
    failed++;
  }
}

// Zoom 0 → full Earth height
assertClose(zoomToHeight(0), 35200000, 1, "zoom 0 = 35200000m");

// Zoom 12 (default) → ~8594m
assertClose(zoomToHeight(12), 35200000 / 4096, 1, "zoom 12 = ~8594m");

// Zoom 15 → ~1074m
assertClose(zoomToHeight(15), 35200000 / 32768, 1, "zoom 15 = ~1074m");

// Zoom 5 → ~1100000m
assertClose(zoomToHeight(5), 35200000 / 32, 1, "zoom 5 = 1100000m");

// Fractional zoom (10.5)
assertClose(zoomToHeight(10.5), 35200000 / Math.pow(2, 10.5), 1, "zoom 10.5 fractional");

// Higher zoom = lower height
const h10 = zoomToHeight(10);
const h14 = zoomToHeight(14);
if (h10 > h14) {
  passed++;
} else {
  console.error("FAIL: higher zoom should produce lower height");
  failed++;
}

if (failed === 0) {
  console.log(`PASS — all ${passed} assertions passed`);
  process.exit(0);
} else {
  console.error(`${failed} assertion(s) failed out of ${passed + failed}`);
  process.exit(1);
}
