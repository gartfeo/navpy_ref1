/**
 * Browser fullscreen for the handheld GCS.
 *
 * Chrome on Android hides the status and navigation bars only for element
 * fullscreen, and only after a user tap. A manifest "display": "fullscreen"
 * does not reach an app installed as a home-screen shortcut, so the operator
 * gets a top-bar button instead.
 */
export function isFullscreenSupported(doc) {
  return !!doc?.fullscreenEnabled;
}

export function isFullscreenActive(doc) {
  return !!doc?.fullscreenElement;
}

// Launched from the home screen (not a browser tab). Chrome drops fullscreen
// on every launch and return, so the app re-enters it on the next tap. Only
// "standalone" (the manifest's display) counts: display-mode "fullscreen"
// also matches a desktop browser in F11.
export function isInstalledApp(win) {
  return !!win?.matchMedia?.('(display-mode: standalone)')?.matches;
}

// Call from a tap/click handler: requestFullscreen needs a user gesture.
export function toggleFullscreen(doc) {
  return isFullscreenActive(doc)
    ? doc.exitFullscreen()
    : doc.documentElement.requestFullscreen();
}
