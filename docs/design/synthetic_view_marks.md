# Synthetic view: what to show (research, 2026-11-09)

Owner request (2026-11-08): improve the vehicle window's Synthetic view with
the best known practice for views like ours. The Synthetic view is a small
(about 320x180 CSS px on the SIYI UniRC 10 Pro; larger maximized) first-person
render of the UAV's pose on the map's terrain and imagery, labelled
"SYNTHETIC · not camera", for an operator supervising an autonomous
fixed-wing fleet. Design: `gcs_video_streams.md`, section 5 "Synthetic view".

## Sources

- FAA AC 23-26, Synthetic Vision and Pathway Depictions on the PFD (2005),
  https://www.govinfo.gov/content/pkg/GOVPUB-TD4-PURL-LPS105470/pdf/GOVPUB-TD4-PURL-LPS105470.pdf
  (superseded by AC 20-167/20-185A, which could not be fetched).
- FAA AM-19/20, vision systems literature review,
  https://libraryonline.erau.edu/online-full-text/faa-aviation-medicine-reports/AM19-20.pdf
- Garmin SVT pilot guides (flight path marker, highlighted active runway).
- Calhoun, Draper et al., AFRL synthetic vision for UAV operators (SPIE 2006,
  https://doi.org/10.1117/12.665633): landmark overlays improved situation
  awareness, workload and time to designate a point.
- Tadema and Theunissen, conformal traffic on synthetic vision for UAVs;
  ForeFlight SV traffic; DJI Pilot 2 AR home point; SmartCam3D declutter.
- Cummings (MIT), supervision of multiple UAVs,
  https://dspace.mit.edu/bitstream/1721.1/46734/1/HAL2007_08.pdf

## Ranked recommendations and what was done

1. A held (frozen) picture must say so and degrade (AC 23-26 l(1)). The
   window's "View held · reason · n s" row existed; now the picture also
   greys out and the flight path marker hides. **Done.**
2. Say where the view looks and never draw a stale gimbal angle as current.
   The tag says gimbal or nose, and only live gimbal telemetry turns the
   camera. **Already so.**
3. Highlight the POI, the anchor of the approach (AC 23-26 i(b); Garmin's
   active runway): the UAV's dock with its distance. **Done.**
4. A damped flight path marker, where the aircraft is going rather than where
   the nose points (AC 23-26 2d, o(1)): large wind crab on a fixed-wing.
   Measured ground velocity, display only (it never reaches navigation; the
   Pure Vision Approach Constraint is about the command path). **Done.**
5. Other fleet vehicles in the scene (ForeFlight SV; Tadema), in neutral
   zone colours, never threat colours. **Done.**
6. The home point, for us the default delivery hub (DDH), in the scene (DJI
   Pilot 2 AR home point). **Done.**
7. The view's field of view on the plan display (AC 23-26 f(2)): the main
   map's vision cones show the camera footprint. Later.
8. Terrain caution/warning shading by height above terrain (AC 23-26 b, g(2);
   Garmin): advisory only. Later.
9. A full-width, high-contrast zero-pitch line above the terrain (AC 23-26 e,
   g(3)). Later.
10. Declutter levels per size (ForeFlight, SmartCam3D, AC 23-26 i(2)); today
    `utils/hudLayout.js` drops parts by height. Later.

Keep: the SYNTHETIC label, real video in its own view, the route with the next
waypoint numbered, the status line, photo imagery, pose updates at the 5 Hz
telemetry rate (benefits held above 2 to 4 Hz).

Do not: make it pass for camera video; use it to judge terrain clearance or
range (a narrow view on a small screen distorts both, AC 23-26 b(2), AM-19/20);
let labels crowd out the dock, path or alerts; call the dock "target".
