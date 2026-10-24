<#
.SYNOPSIS
  Start an Android emulator shaped like the SIYI UniRC 10 Pro screen and open
  this checkout's GCS in it.

.DESCRIPTION
  The UniRC 10 Pro has a 10.1 inch 1920x1200 screen and runs Android 13
  (SIYI user manual v1.2, p.13, p.23-24). SIYI does not publish the display
  density, so -Density defaults to 240 (dpr 1.5, a 1280x800 CSS viewport),
  the nearest Android bucket to the panel's 224 ppi; try 320 (dpr 2) too.
  An emulator is not the device: touch size and gloves need the real one.

  Creates or updates the AVD "unirc10pro_api<Api>" (suffix "_default" on the
  plain image), boots it on console port 5580 (adb serial emulator-5580) and
  opens the GCS URL: in Chrome on a Google APIs image, else in the system
  WebView's test browser, whose WebView is the image's own version. The default URL is
  this checkout's running frontend (from ~\.gcs\instances.json, read only),
  reached from the emulator as 10.0.2.2. Start the GCS first.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File C:\path\to\repo\scripts\unirc_emulator.ps1
.EXAMPLE
  powershell -ExecutionPolicy Bypass -File C:\path\to\repo\scripts\unirc_emulator.ps1 -Density 320 -Portrait
.EXAMPLE
  powershell -ExecutionPolicy Bypass -File C:\path\to\repo\scripts\unirc_emulator.ps1 -Stop
#>
param(
    [int]$Api = 33,
    [int]$Density = 240,
    [string]$Url = "",
    [switch]$Portrait,
    [switch]$Stop
)
$ErrorActionPreference = "Stop"

$Port = 5580
$Serial = "emulator-$Port"
$Sdk = $env:ANDROID_HOME
if (-not $Sdk) { $Sdk = Join-Path $env:LOCALAPPDATA "Android\Sdk" }
$Adb = Join-Path $Sdk "platform-tools\adb.exe"
$Emulator = Join-Path $Sdk "emulator\emulator.exe"
$AvdManager = Join-Path $Sdk "cmdline-tools\latest\bin\avdmanager.bat"
# Prefer the Google APIs image (it has Chrome); fall back to the plain one.
$Tag = "google_apis"
if (-not (Test-Path (Join-Path $Sdk "system-images\android-$Api\$Tag\x86_64"))) { $Tag = "default" }
$Image = "system-images;android-$Api;$Tag;x86_64"
$ImageDir = Join-Path $Sdk "system-images\android-$Api\$Tag\x86_64"
$AvdName = "unirc10pro_api$Api"
if ($Tag -ne "google_apis") { $AvdName += "_$Tag" }
$RepoRoot = Split-Path -Parent $PSScriptRoot

if ($Stop) {
    & $Adb -s $Serial emu kill
    exit 0
}

foreach ($tool in @($Adb, $Emulator, $AvdManager)) {
    if (-not (Test-Path $tool)) { throw "Missing $tool. Install it with Android Studio > Tools > SDK Manager." }
}
if (-not (Test-Path $ImageDir)) {
    throw ("Missing $Image. Install it with Android Studio > Tools > SDK Manager > SDK Platforms > " +
           "Show Package Details > Android API $Api > 'Google APIs Intel x86_64 Atom System Image'.")
}

function Set-AvdConfig([string]$path, [hashtable]$values) {
    $lines = @(Get-Content $path | Where-Object { $values.Keys -notcontains ($_ -split "=", 2)[0].Trim() })
    $lines += $values.GetEnumerator() | ForEach-Object { "$($_.Key)=$($_.Value)" }
    Set-Content -Path $path -Value $lines -Encoding ascii
}

$AvdDir = Join-Path $env:USERPROFILE ".android\avd\$AvdName.avd"
if (-not (Test-Path $AvdDir)) {
    "no" | & $AvdManager create avd -n $AvdName -k $Image -d pixel_tablet | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "avdmanager could not create $AvdName" }
}
# The screen of the UniRC 10 Pro. The emulator draws it as a 1920x1200
# landscape panel; hw.initialOrientation picks how Android starts.
$orientation = "landscape"
if ($Portrait) { $orientation = "portrait" }
Set-AvdConfig (Join-Path $AvdDir "config.ini") @{
    "hw.lcd.width" = 1920; "hw.lcd.height" = 1200; "hw.lcd.density" = $Density
    "hw.initialOrientation" = $orientation; "hw.keyboard" = "yes"
    "showDeviceFrame" = "no"; "skin.name" = "1920x1200"; "skin.path" = "_no_skin"
    "hw.ramSize" = 4096; "hw.gpu.enabled" = "yes"; "hw.gpu.mode" = "auto"
}

$running = (& $Adb devices) -match "^$Serial\s+device"
if (-not $running) {
    Start-Process -FilePath $Emulator -ArgumentList @("-avd", $AvdName, "-port", $Port, "-no-snapshot-save", "-no-boot-anim")
    & $Adb -s $Serial wait-for-device
    while ((& $Adb -s $Serial shell getprop sys.boot_completed 2>$null) -ne "1") { Start-Sleep -Seconds 2 }
}
# A running AVD keeps its boot density; apply this run's value and rotation.
& $Adb -s $Serial shell wm density $Density | Out-Null
& $Adb -s $Serial shell settings put system accelerometer_rotation 0
# The 1920x1200 panel is naturally landscape (rotation 0); portrait is 90.
$rotation = 0
if ($Portrait) { $rotation = 1 }
& $Adb -s $Serial shell settings put system user_rotation $rotation

if (-not $Url) {
    $registry = Join-Path $env:USERPROFILE ".gcs\instances.json"
    $entry = $null
    if (Test-Path $registry) {
        $instances = (Get-Content $registry -Raw | ConvertFrom-Json).instances
        foreach ($p in $instances.PSObject.Properties) {
            if ($p.Value.clone -and ((Resolve-Path $p.Value.clone -ErrorAction SilentlyContinue).Path -eq $RepoRoot)) { $entry = $p.Value }
        }
    }
    if (-not $entry) { throw "No running GCS for $RepoRoot. Start it first, or pass -Url." }
    $Url = "http://10.0.2.2:$($entry.frontend)"
}
$browser = "Chrome"
if ((& $Adb -s $Serial shell pm path com.android.chrome) -match "package:") {
    # Skip Chrome's first-run screens: Chrome reads this flags file when it is
    # the debug app at start. Then open the GCS in a new Chrome tab.
    & $Adb -s $Serial shell "echo '_ --disable-fre --no-default-browser-check --no-first-run' > /data/local/tmp/chrome-command-line"
    & $Adb -s $Serial shell am set-debug-app --persistent com.android.chrome | Out-Null
    & $Adb -s $Serial shell am force-stop com.android.chrome
    & $Adb -s $Serial shell am start -a android.intent.action.VIEW -d $Url -p com.android.chrome | Out-Null
} else {
    # The plain image has no Chrome: use the system WebView's test browser.
    $browser = "WebView Browser Tester"
    & $Adb -s $Serial shell am start -a android.intent.action.VIEW -d $Url -n org.chromium.webview_shell/.WebViewBrowserActivity | Out-Null
}
Write-Host "UniRC 10 Pro emulator ($AvdName, density $Density, $orientation) on $Serial opened $Url in $browser"
Write-Host "Stop it with: powershell -ExecutionPolicy Bypass -File $PSCommandPath -Stop"
