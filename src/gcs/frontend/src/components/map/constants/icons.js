// SVG circle icons for vertex/midpoint billboards (consistent rendering on iPad)
export const VERTEX_ICON = `data:image/svg+xml,${encodeURIComponent(
  '<svg xmlns="http://www.w3.org/2000/svg" width="20" height="20">' +
  '<circle cx="10" cy="10" r="8" fill="red" stroke="white" stroke-width="2.5"/>' +
  '</svg>'
)}`;

export const MIDPOINT_ICON = `data:image/svg+xml,${encodeURIComponent(
  '<svg xmlns="http://www.w3.org/2000/svg" width="14" height="14">' +
  '<circle cx="7" cy="7" r="5" fill="#00d2ff" fill-opacity="0.5" stroke="white" stroke-width="1.5"/>' +
  '</svg>'
)}`;

// Launch location, shared with the per-set symbol.
export const LAUNCH_POINT_ICON = makeLaunchIcon('#ff9800', 24);

export const CAMERA_KEY = 'gcs-camera';

// Generate colored assembly point icon per set — X-cross with circle
export function makeLaunchIcon(color, size) {
  const c = size / 2;
  const r = c - 3;
  return `data:image/svg+xml,${encodeURIComponent(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}">` +
    `<circle cx="${c}" cy="${c}" r="${r}" fill="none" stroke="${color}" stroke-width="2.5"/>` +
    `<path d="M${c} ${size - 6}V6m-5 5 5-5 5 5" fill="none" stroke="${color}" stroke-width="2"/>` +
    `</svg>`
  )}`;
}

// Generate colored control point (КТ) icon — filled circle with inner dot
export function makeCorridorIcon(color, size) {
  const c = size / 2;
  const outerR = c - 2;
  const innerR = Math.max(2, c / 4);
  return `data:image/svg+xml,${encodeURIComponent(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}">` +
    `<circle cx="${c}" cy="${c}" r="${outerR}" fill="${color}" stroke="white" stroke-width="1.5"/>` +
    `<circle cx="${c}" cy="${c}" r="${innerR}" fill="white"/>` +
    `</svg>`
  )}`;
}

// Configured delivery location: pad symbol.
export function makeDockIcon(size) { return makeAssignmentIcon(size, '#00d2ff'); }

// Available task — dashed orange circle with center dot (detected, pending assignment)
export function makeAvailableTaskIcon(size) {
  const c = size / 2;
  const r = c * 0.6;
  return `data:image/svg+xml,${encodeURIComponent(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}">` +
    `<circle cx="${c}" cy="${c}" r="${r}" fill="none" stroke="#ff9800" stroke-width="2" stroke-dasharray="4,3"/>` +
    `<circle cx="${c}" cy="${c}" r="2.5" fill="#ff9800"/>` +
    `</svg>`
  )}`;
}

// Track waypoint diamond — clickable marker for sim target selection in planning mode
export function makeTrackWpIcon(color, size, selected) {
  const c = size / 2;
  const d = c - 2;
  const pts = `${c},${c - d} ${c + d},${c} ${c},${c + d} ${c - d},${c}`;
  if (selected) {
    return `data:image/svg+xml,${encodeURIComponent(
      `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}">` +
      `<polygon points="${pts}" fill="${color}" stroke="white" stroke-width="2"/>` +
      `</svg>`
    )}`;
  }
  return `data:image/svg+xml,${encodeURIComponent(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}">` +
    `<polygon points="${pts}" fill="${color}" fill-opacity="0.4" stroke="rgba(255,255,255,0.3)" stroke-width="1"/>` +
    `</svg>`
  )}`;
}

// Assigned task location, retaining the task's state color.
export function makeAssignmentIcon(size, color) {
  return `data:image/svg+xml,${encodeURIComponent(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${size}" height="${size}" viewBox="0 0 24 24">` +
    `<rect x="3" y="3" width="18" height="18" rx="3" fill="#16364a" stroke="${color}" stroke-width="2"/>` +
    `<path d="M9 7h3a5 5 0 0 1 0 10H9z" fill="none" stroke="${color}" stroke-width="2"/>` +
    '</svg>'
  )}`;
}

