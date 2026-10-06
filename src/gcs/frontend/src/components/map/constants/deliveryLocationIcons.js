/** Neutral host symbols for configured delivery locations. Type IDs stay unchanged. */
const svgIcon = (body) => `data:image/svg+xml,${encodeURIComponent(
  `<svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="white" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round">${body}</svg>`
)}`;
export const DOCK_ICON = svgIcon('<rect x="3" y="3" width="18" height="18" rx="3" fill="#16364a"/><path d="M9 7h3a5 5 0 0 1 0 10H9z"/>');
const ICONS = {
  building: svgIcon('<path d="M5 21V3h14v18zM8 7h2m4 0h2M8 11h2m4 0h2M10 21v-5h4v5"/>'),
  vehicle: svgIcon('<path d="m3 14 2-7h14l2 7v5H3zM4 12h16"/><circle cx="6" cy="19" r="2"/><circle cx="18" cy="19" r="2"/>'),
  antenna: svgIcon('<path d="m7 21 5-13 5 13M9 16h6M5 4a10 10 0 0 0 0 10M19 4a10 10 0 0 1 0 10"/><circle cx="12" cy="7" r="2"/>'),
  operations_site: svgIcon('<rect x="3" y="7" width="18" height="14" rx="2"/><path d="M8 7V3h8v4M3 12h18M10 12v3h4v-3"/>'),
  bridge: svgIcon('<path d="M2 16h20M5 21V5m14 16V5M5 6q7 12 14 0M8 12v4m4-2v2m4-4v4"/>'),
  fuel: svgIcon('<rect x="3" y="3" width="11" height="18" rx="1"/><path d="M5 6h7v5H5zM14 10h3v7a2 2 0 0 0 4 0V8l-3-3"/>'),
  other: DOCK_ICON,
};
export const DELIVERY_LOCATION_TYPES = ['building', 'vehicle', 'antenna', 'operations_site', 'bridge', 'fuel', 'other'];
export function getDeliveryLocationIcon(type) { return ICONS[type] || DOCK_ICON; }
