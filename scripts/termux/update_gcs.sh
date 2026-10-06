#!/data/data/com.termux/files/usr/bin/bash
# Replace the handheld's GCS copy with a new one unpacked next to it, then
# restart the backend if it was running. Run the script from the NEW copy:
#   mkdir ~/navpy.manual && tar -xzf navpy-update.tgz -C ~/navpy.manual &&
#       bash ~/navpy.manual/scripts/termux/update_gcs.sh
# (or pass the new copy's folder as the only argument). scripts/deploy_handheld.py
# does these steps over adb. ~/navpy.prev is the backup; it changes only to a
# copy whose backend was seen healthy (VERIFIED). A new copy whose backend
# does not come up is put aside as ~/navpy.failed and the previous one
# restored.
# Exit: 0 updated; 1 failed, the previous copy kept or restored; 2 updated,
# but the app build failed; 3 updated, but the copy could not be marked
# VERIFIED. 0, 2 and 3 end with an "update result:" line.
set -euo pipefail

NEW="$(cd "${1:-$(dirname "$0")/../..}" && pwd)"
LIVE="${GCS_REPO_DIR:-$HOME/navpy}"
PREV="$LIVE.prev"
FAILED="$LIVE.failed"
DATA="${GCS_DATA_DIR:-$HOME/.gcs}"
PORT="${GCS_PORT:-8000}"
LOG="$DATA/logs/start_gcs.log"
ANDROID_JAR="${ANDROID_JAR:-$DATA/android-sdk/android.jar}"
# Sources of the last successful app build (build_apk.sh writes the app there).
APK_STAMP="$DATA/apk/sources.sha256"
# The backend needs about 10 s to start on the UniRC 10 Pro.
START_TIMEOUT_S="${GCS_START_TIMEOUT_S:-90}"
# One /health request; a backend that accepts but never replies must not hold
# the wait past START_TIMEOUT_S.
HEALTH_REQUEST_S=5
STOP_TIMEOUT_S="${GCS_STOP_TIMEOUT_S:-30}"
KILL_WAIT_S=5
APP_BUILD_FAILED=2
NOT_MARKED=3
DEPLOY_LOCK="$HOME/.navpy-deploy.lock"
# Termux has no flock; Android's toybox does.
FLOCK="${GCS_FLOCK:-/system/bin/flock}"

if [ "$NEW" = "$(cd "$LIVE" 2>/dev/null && pwd)" ]; then
    echo "Run this script from the new copy, not from $LIVE." >&2
    exit 1
fi
if [ ! -f "$NEW/DEPLOYED" ]; then
    echo "$NEW has no DEPLOYED file; it is not a deploy bundle." >&2
    exit 1
fi

# One update at a time, from the PC or by hand. scripts/deploy_handheld.py
# takes the lock on fd 9 before it unpacks the new copy and passes the
# descriptor down; a flock belongs to the open file, so taking it again here
# succeeds. Run by hand, the updater opens the lock file itself.
if ! { : >&9; } 2>/dev/null; then exec 9> "$DEPLOY_LOCK"; fi
if ! "$FLOCK" -n 9; then
    echo "another update is running on the handheld; nothing was changed" >&2
    exit 1
fi

version() {
    if [ -f "$1/DEPLOYED" ]; then head -n 2 "$1/DEPLOYED" | tr '\n' ' '; else echo "unknown (copied by hand)"; fi
}

# Every file the app build reads from the repository, by content.
app_sources_hash() {
    (cd "$1" && { find src/gcs/android scripts/termux/build_apk.sh \
        src/gcs/frontend/public/aas-icon-maskable-512.png -type f -print0 2>/dev/null || true; } \
        | LC_ALL=C sort -z | xargs -0 -r sha256sum | sha256sum | cut -c1-64)
}

backend_pids() {
    pgrep -f 'uvicorn gcs\.backend\.main' || true
}

# Only the GCS backend; start_gcs.sh exits with it and releases its wake lock.
stop_backend() {
    local pid
    for pid in $(backend_pids); do
        echo "stopping backend (pid $pid)"
        kill "$pid" 2>/dev/null || true
        if timeout "$STOP_TIMEOUT_S" tail --pid="$pid" -f /dev/null; then continue; fi
        echo "backend pid $pid ignored TERM for ${STOP_TIMEOUT_S}s; sending KILL"
        kill -KILL "$pid" 2>/dev/null || true
        if ! timeout "$KILL_WAIT_S" tail --pid="$pid" -f /dev/null; then
            echo "backend pid $pid is still running after KILL" >&2
            return 1
        fi
    done
}

start_backend() {
    mkdir -p "$(dirname "$LOG")"
    # 9>&-: scripts/deploy_handheld.py holds its deploy lock on fd 9; a
    # backend that kept it would refuse every later deploy.
    setsid nohup bash "$LIVE/scripts/termux/start_gcs.sh" >> "$LOG" 2>&1 < /dev/null 9>&- &
    echo "starting backend (log: $LOG)"
}

# One /health request. Healthy = it answers and every API route module
# loaded. Returns 0 healthy, 1 no answer, 2 answered with missing routes;
# HEALTH_BODY holds the reply.
HEALTH_BODY=""
MARK_FAILED=no
check_health() {
    HEALTH_BODY="$(curl -fs --max-time "$HEALTH_REQUEST_S" "http://127.0.0.1:$PORT/health")" || return 1
    python -c 'import json, sys; sys.exit(json.load(sys.stdin).get("unavailable_routes") != [])' \
        <<<"$HEALTH_BODY" || return 2
}

# A copy whose backend was seen healthy may become the backup.
mark_verified() {
    date -u +%Y-%m-%dT%H:%M:%SZ > "$LIVE/VERIFIED"
}

wait_healthy() {
    local result deadline=$((SECONDS + START_TIMEOUT_S))
    while :; do
        result=0
        check_health || result=$?
        if [ "$result" = 0 ]; then
            echo "backend up: $HEALTH_BODY"
            if ! mark_verified; then
                echo "could not mark $LIVE as checked (VERIFIED); the next deploy treats it as unchecked" >&2
                MARK_FAILED=yes
            fi
            return 0
        fi
        if [ "$result" = 2 ]; then
            echo "backend is up but not all API routes loaded (unavailable_routes): $HEALTH_BODY" >&2
            return 1
        fi
        if [ "$SECONDS" -ge "$deadline" ]; then
            echo "backend did not answer /health within ${START_TIMEOUT_S}s; see $LOG" >&2
            return 1
        fi
        sleep 1
    done
}

# The last line tells scripts/deploy_handheld.py the update completed; an
# exit code alone could come from tar or bash failing first.
finish() {
    if [ "$MARK_FAILED" = yes ] && [ "$status" = 0 ]; then status=$NOT_MARKED; fi
    case "$status" in
        "$APP_BUILD_FAILED") echo "update result: app build failed" ;;
        "$NOT_MARKED") echo "update result: updated, but not marked checked" ;;
        *) echo "update result: updated" ;;
    esac
    exit "$status"
}

echo "installed: $(version "$LIVE")"
echo "new:       $(version "$NEW")"
if ! cmp -s "$LIVE/src/gcs/requirements-termux.txt" "$NEW/src/gcs/requirements-termux.txt"; then
    echo "WARNING: src/gcs/requirements-termux.txt changed; re-run the pip step:"
    echo "  cd $LIVE && pip install -r src/gcs/requirements-termux.txt"
fi

# Build the app first, while the running backend still serves the operator.
# The stamp is dropped before a build (build_apk.sh may overwrite the APK and
# then fail) and written only after one succeeds, so a failed build is retried.
status=0
sources="$(app_sources_hash "$NEW")"
if [ "$sources" = "$(cat "$APK_STAMP" 2>/dev/null || true)" ]; then
    echo "app sources unchanged since the last build; no app build"
elif [ ! -f "$ANDROID_JAR" ] || ! command -v aapt2 >/dev/null; then
    echo "The app's sources changed, but its build tools are not installed; see the guide."
else
    rm -f "$APK_STAMP"
    if bash "$NEW/scripts/termux/build_apk.sh"; then
        mkdir -p "$(dirname "$APK_STAMP")"
        echo "$sources" > "$APK_STAMP"
    else
        echo "The app build failed; the next deploy builds it again." >&2
        status=$APP_BUILD_FAILED
    fi
fi

was_running=no
if [ -n "$(backend_pids)" ]; then was_running=yes; fi
# A copy started by hand was never checked here; if its backend is healthy
# now, it is a working copy and must not be dropped for an older backup,
# even when its VERIFIED file cannot be written.
live_healthy=no
if [ "$was_running" = yes ] && [ -d "$LIVE" ] && check_health; then
    live_healthy=yes
    if mark_verified; then
        echo "the running backend is healthy; $LIVE counts as checked"
    else
        echo "the running backend is healthy, but $LIVE could not be marked checked" >&2
    fi
fi
if ! stop_backend; then
    echo "nothing was replaced." >&2
    exit 1
fi

# The backup changes only to a copy that was seen working, or fills an empty
# slot. An unchecked copy is deleted instead, so a copy deployed while the
# backend was stopped never replaces the backup.
if [ -d "$LIVE" ]; then
    if [ "$live_healthy" = yes ] || [ -f "$LIVE/VERIFIED" ] || [ ! -d "$PREV" ]; then
        rm -rf "$PREV"
        mv "$LIVE" "$PREV"
    else
        echo "keeping $PREV as the backup; dropping the unchecked $LIVE"
        rm -rf "$LIVE"
    fi
fi
if ! mv "$NEW" "$LIVE"; then
    [ -d "$PREV" ] && mv "$PREV" "$LIVE"
    echo "could not move the new copy into place; the previous one is restored." >&2
    exit 1
fi
echo "replaced $LIVE (backup: $PREV)"

if [ "$was_running" = no ]; then
    echo "backend was not running; start it with: bash $LIVE/scripts/termux/start_gcs.sh"
    finish
fi
start_backend
if wait_healthy; then finish; fi

# Put the previous copy back, so the handheld keeps a working GCS.
stop_failed=no
stop_backend || stop_failed=yes
rm -rf "$FAILED"
mv "$LIVE" "$FAILED"
if [ ! -d "$PREV" ]; then
    echo "there is no previous copy to roll back to; the failed copy is $FAILED" >&2
    exit 1
fi
mv "$PREV" "$LIVE"
echo "rolled back to $(version "$LIVE"); the failed copy is $FAILED"
if [ "$stop_failed" = yes ]; then
    echo "the new backend could not be stopped, so the previous copy was not started" >&2
    exit 1
fi
start_backend
wait_healthy || echo "the previous copy did not come up either" >&2
exit 1
