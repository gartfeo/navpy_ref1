#!/data/data/com.termux/files/usr/bin/bash
# Build the full-screen Android shell (src/gcs/android) on the handheld.
#   pkg install openjdk-21 aapt2 d8 apksigner zip     (once)
#   ANDROID_JAR=/path/to/android.jar bash scripts/termux/build_apk.sh
# ANDROID_JAR is the Android 13 (API 33) platform android.jar from Google's
# SDK (platforms/android-33/android.jar); Termux ships none that aapt2 can use.
# Output: $GCS_DATA_DIR/apk/aas-gcs.apk (default ~/.gcs/apk).
set -euo pipefail

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
SRC="$REPO/src/gcs/android"
DATA="${GCS_DATA_DIR:-$HOME/.gcs}"
OUT="$DATA/apk"
BUILD="$OUT/build"
ANDROID_JAR="${ANDROID_JAR:-$DATA/android-sdk/android.jar}"
# Signing key for this handheld's installs; kept outside the repo.
KEYSTORE="$DATA/apk-signing.keystore"
KEYPASS="$DATA/apk-signing.pass"
ICON="$REPO/src/gcs/frontend/public/aas-icon-maskable-512.png"

if [ ! -f "$ANDROID_JAR" ]; then
    echo "No android.jar at $ANDROID_JAR; set ANDROID_JAR (see the header)." >&2
    exit 1
fi

rm -rf "$BUILD"
mkdir -p "$BUILD/gen" "$BUILD/classes" "$BUILD/dex"
cp -r "$SRC/res" "$BUILD/res"
mkdir -p "$BUILD/res/drawable-nodpi"
cp "$ICON" "$BUILD/res/drawable-nodpi/ic_launcher_foreground.png"

aapt2 compile --dir "$BUILD/res" -o "$BUILD/res.zip"
aapt2 link -I "$ANDROID_JAR" --manifest "$SRC/AndroidManifest.xml" \
    --java "$BUILD/gen" -o "$BUILD/unsigned.apk" "$BUILD/res.zip"

find "$SRC/java" "$BUILD/gen" -name '*.java' > "$BUILD/sources.txt"
javac --release 8 -nowarn -classpath "$ANDROID_JAR" -d "$BUILD/classes" @"$BUILD/sources.txt"
find "$BUILD/classes" -name '*.class' > "$BUILD/classes.txt"
d8 --release --min-api 30 --lib "$ANDROID_JAR" --output "$BUILD/dex" @"$BUILD/classes.txt"
(cd "$BUILD/dex" && zip -q -j "$BUILD/unsigned.apk" classes.dex)

if [ ! -f "$KEYSTORE" ]; then
    (umask 077; head -c 24 /dev/urandom | base64 | tr -d '/+=' > "$KEYPASS")
    keytool -genkeypair -keystore "$KEYSTORE" -storepass:file "$KEYPASS" \
        -alias gcs -keyalg RSA -keysize 3072 -validity 10000 \
        -dname "CN=AAS GCS handheld" >/dev/null
fi
apksigner sign --ks "$KEYSTORE" --ks-pass "file:$KEYPASS" \
    --out "$OUT/aas-gcs.apk" "$BUILD/unsigned.apk"
apksigner verify "$OUT/aas-gcs.apk"
echo "Built $OUT/aas-gcs.apk"
