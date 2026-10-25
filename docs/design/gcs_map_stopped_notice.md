# Map stopped notice (design v2.1)

Approved by the owner 2026-10-25 (v2.1, after two Codex rounds).

## Problem
Cesium stops the main map for good when its WebGL context is lost or a frame
throws (`useDefaultRenderLoop` false). The operator sees a blank or frozen map
under Cesium's English "Rendering has stopped" panel with a stack trace, and
nothing says what to do. Todo 87(a) reproduced this by losing the context by
hand and through Chrome's 16-context limit
(`.logs/map-load-errors-87a-118/87a/context-loss`).

## Change
- `useCesiumViewer` returns `mapStopped`, set once (never cleared) when:
  - the map canvas fires `webglcontextlost` (the canvas is blank at once,
    before the next frame), or
  - Cesium's default render loop stops on an error after the Viewer is
    built. Cesium reports each such stop through
    `cesiumWidget.showErrorPanel` (`CesiumWidget.js`: the render loop's
    catch, `:73`, and `_onRenderError`, `:414`). Only errors inside
    `scene.render` raise `scene.renderError`; errors in resize, frame setup
    and the clock tick, where entities update, reach only the loop's catch.
    So the Viewer keeps `showRenderLoopErrors: true` (now explicit), and the
    hook replaces `showErrorPanel` on the map's widget right after building
    it: it logs the title and error with `console.error`, as Cesium's panel
    does, and sets `mapStopped`. Cesium's panel is no longer built.
- `CesiumMap` shows `MapStoppedNotice` (new file) over the map box when
  `mapStopped`. The Cesium container becomes an inner div filling the
  outer one, since the hook clears it on start; the notice is its sibling.
  A stopped map also dismisses the loading screen at once.
- The notice (mock: `gcs_map_stopped_notice_mock.html`):
  - a dim layer over the map box only (z-index 5); the top bar, the sidebar
    and the map toolbar (z-index 10) stay usable. The layer lets clicks
    through (`pointer-events: none`) and only Reload takes them, so a click
    on the small map still enlarges it (App's "Show the map large" button);
  - a card: warning icon and "Map stopped"; the line "Check UAV status in
    the sidebar. To restore the map, try reloading the page; unsaved plan
    edits are lost."; a "Reload" button (`window.location.reload()`);
    `role="alert"`; English and Armenian;
  - the card shows only when all of it fits the box (it is always laid out
    and measured; no size threshold). It is hidden with `visibility`, so it
    keeps its size and cannot take focus or clicks; a ResizeObserver on the
    layer and on the card's content re-measures on resize and text change. Otherwise only the icon and title
    show, cut short if needed, and there is no Reload: the loss warning is
    never left out of a Reload. This covers the small map beside a
    maximized vehicle window (Restore gives the full notice) and the 55 px
    map strip of a 375 px viewport.
- Nothing restarts the map by itself (owner, 2026-10-25).

## Not covered
- A Viewer that cannot be built (no WebGL) still fails under the loading
  screen, as today.
- A loop stopped without an error (code setting `useDefaultRenderLoop`
  false; none in the app) and a hidden tab (no frames, not a stop).
- What lost v11's context (87(a), not reproduced).
- The Viewer is never destroyed on unmount (todo 135); the canvas listener
  goes with it.

## Tests
- `useCesiumViewer.test.jsx`, fake Cesium: `webglcontextlost` sets
  `mapStopped`; a `showErrorPanel` call sets it and logs, and Cesium's own
  panel is not built; both signals together set it once; a normal start
  leaves it false. They fail on origin/main (no `mapStopped`).
- `MapStoppedNotice.test.jsx`: `role="alert"`, the text keys, Reload calls
  its handler (`window.location.reload` by default, which jsdom cannot
  stub; checked live); with measured sizes stubbed: vertical and
  horizontal overflow show the title only, with no Reload; an exact fit and
  a box that grows again show the card; the observer disconnects on unmount.
- `CesiumMap.test.jsx`, hooks mocked: a stopped map shows the notice and
  dismisses the loading screen before tiles are ready.
- Live, built frontend, through real Cesium: lose the context
  (`WEBGL_lose_context`), throw inside `scene.render` (a `preRender`
  listener) and throw in the clock tick (an `onTick` listener). Each: the
  notice shows, no Cesium panel, `useDefaultRenderLoop` false, the toolbar
  still clicks. Also English and Armenian, the maximized vehicle window
  (title only; a click on the small map enlarges it) and Restore (full
  card), and Reload brings the map back.

## Review
Codex, 2026-10-25, on v1 (7 points). All taken:
1. Keep the `showErrorPanel` replacement and `showRenderLoopErrors: true`;
   state the coverage as error-driven stops after construction. Done.
2. Inner container fills the outer one; notice as a sibling; no z-index on
   the wrapper. Done.
3. The small-box layout offered Reload without the loss warning. Now no
   Reload unless the whole card shows.
4. The 240 x 150 rule did not match the real card size. Replaced by
   measuring whether the card fits (both languages, any text size).
5. "The UAVs and the rest of the screen still work" was not established by
   the signals. New line as above. Armenian "Վերբեռնել" means Upload in
   this app; Reload is "Վերաբեռնել".
6. A stop before the first tiles stayed under the loading screen up to 8 s.
   A stopped map now dismisses it.
7. Tests did not prove the real Cesium routes. Added the live checks above
   and the duplicate-signal and loading-screen tests.

Codex, round 2 on v2: points 1-7 resolved. New, all taken: (10, medium)
the layer blocked the small map's click-to-enlarge; it now lets clicks
through. (8, 9, 11, low) hide with `visibility`, also observe the card's
content, and test both overflow directions, exact fit, regrowth and
observer cleanup.
