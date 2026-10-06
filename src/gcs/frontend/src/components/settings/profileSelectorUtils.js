import { computeMaxDetectDist } from '../../utils/planner';

export function buildCatalogDeviceInfos(catalog, profileName, deviceName, zoomLevel, rangeKey) {
  const profileData = catalog?.profiles?.[profileName];
  if (!profileData) return [];

  const devices = profileData.devices || [];
  const selectedDevice = devices.find((d) => d.name === deviceName) || devices[0];
  if (!selectedDevice) return [];

  const selectedZoom = selectedDevice.zooms[zoomLevel] ? zoomLevel : Object.keys(selectedDevice.zooms)[0] || '1';
  return devices.map((device) => {
    const isPrimary = device.name === selectedDevice.name;
    const zoom = isPrimary ? (device.zooms[selectedZoom] || device.zooms['1']) : device.zooms['1'];
    if (!zoom) return null;
    return {
      name: device.name,
      gimbal_device_id: device.gimbal_device_id,
      footprintSource: device.publishes_gimbal_telemetry === false ? 'none' : 'live-gimbal',
      imageWidth: device.image_width,
      imageHeight: device.image_height,
      fovH: 2 * Math.atan(device.image_width / (2 * zoom.fx)),
      fovV: 2 * Math.atan(device.image_height / (2 * zoom.fy)),
      pitchDeg: device.pitch_deg,
      primary: isPrimary,
      maxDetectDist: zoom[rangeKey],
      setup_att: device.setup_att,
      setup_seq: device.setup_seq,
    };
  }).filter(Boolean);
}

/**
 * Build the per-device data the detection diagram renders.
 *
 * Each camera uses its OWN effective zoom for FOV/range: an explicit entry in
 * `deviceZooms` (per-camera selection) wins; otherwise the primary uses the
 * selected `zoomLevel` and the rest fall back to '1' (the prior behavior, so
 * callers that pass no `deviceZooms` get an unchanged diagram). `pitchByDevice`
 * lets a multi-camera caller supply each camera's resolved pitch directly; it
 * falls back to local overrides, then the primary's `cameraPitchDeg`, then the
 * catalog pitch.
 */
export function buildDiagramDevices(
  catalog, profileName, deviceName, zoomLevel, cameraPitchDeg, localOverrides,
  deviceZooms = {}, pitchByDevice = {},
) {
  const profileData = catalog?.profiles?.[profileName];
  if (!profileData) return [];

  const catalogDevices = profileData.devices || [];
  return buildCatalogDeviceInfos(
    catalog,
    profileName,
    deviceName,
    zoomLevel,
    'confirm_range_m',
  ).map((device) => {
    const catalogDevice = catalogDevices.find((entry) => entry.name === device.name);
    const zoomsAvail = catalogDevice?.zooms || {};

    // Effective per-camera zoom: explicit selection wins, else primary→zoomLevel,
    // others→'1'. Fall back to the first available level if the pick is unknown.
    let effZoom = deviceZooms[device.name];
    if (!effZoom || !zoomsAvail[effZoom]) effZoom = device.primary ? zoomLevel : '1';
    if (!zoomsAvail[effZoom]) effZoom = Object.keys(zoomsAvail)[0] || '1';

    const deviceOverrides = localOverrides.devices?.[device.name] || {};
    const zoomOverrides = localOverrides.zooms?.[`${device.name}/${effZoom}`] || {};
    const pitchDeg = pitchByDevice[device.name]
      ?? deviceOverrides.pitch_deg
      ?? (device.primary ? cameraPitchDeg : undefined)
      ?? device.pitchDeg;

    const baseZoom = zoomsAvail[effZoom] || zoomsAvail['1'];
    const fx = zoomOverrides.camera_fx ?? baseZoom?.fx ?? 2000;
    const fy = zoomOverrides.camera_fy ?? baseZoom?.fy ?? 2000;
    const width = deviceOverrides.image_width ?? catalogDevice?.image_width ?? 1920;
    const height = deviceOverrides.image_height ?? catalogDevice?.image_height ?? 1080;
    return {
      ...device,
      pitchDeg,
      fovH: 2 * Math.atan(width / (2 * fx)),
      fovV: 2 * Math.atan(height / (2 * fy)),
      maxDetectDist: computeMaxDetectDist(
        fy,
        width,
        height,
        profileData.reference_height_m,
        profileData.imgsz,
      ),
    };
  });
}
