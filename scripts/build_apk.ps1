<#
.SYNOPSIS
  Build the full-screen Android shell (src/gcs/android) on this PC.

.DESCRIPTION
  The Windows twin of scripts/termux/build_apk.sh: aapt2, javac, d8, zipalign
  and apksigner, no Gradle. Needs the Android SDK (ANDROID_HOME, default
  %LOCALAPPDATA%\Android\Sdk) with a build-tools version and the Android 13
  (API 33) platform, both from Android Studio's SDK Manager, and JDK 17
  (JAVA_HOME, else javac and keytool on PATH).

  Output: <GCS_DATA_DIR>\apk\aas-gcs.apk (default %USERPROFILE%\.gcs\apk).
  The signing key is made on the first build and kept beside it, outside the
  repo; an install over an APK signed with another key needs an uninstall.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File C:\path\to\repo\scripts\build_apk.ps1
#>
$ErrorActionPreference = "Stop"

$Repo = Split-Path -Parent $PSScriptRoot
$Src = Join-Path $Repo "src\gcs\android"
$Data = $env:GCS_DATA_DIR
if (-not $Data) { $Data = Join-Path $env:USERPROFILE ".gcs" }
$Data = [System.IO.Path]::GetFullPath($Data)
# The signing key and its password must never land in the checkout.
$repoPrefix = [System.IO.Path]::GetFullPath($Repo).TrimEnd('\') + '\'
if (($Data.TrimEnd('\') + '\').StartsWith($repoPrefix, [StringComparison]::OrdinalIgnoreCase)) {
    throw "GCS_DATA_DIR $Data is inside the repository; the signing key must be kept outside it."
}
$Out = Join-Path $Data "apk"
$Build = Join-Path $Out "build"
$Keystore = Join-Path $Data "apk-signing.keystore"
$KeyPass = Join-Path $Data "apk-signing.pass"
$Icon = Join-Path $Repo "src\gcs\frontend\public\aas-icon-maskable-512.png"

$Sdk = $env:ANDROID_HOME
if (-not $Sdk) { $Sdk = Join-Path $env:LOCALAPPDATA "Android\Sdk" }
$AndroidJar = Join-Path $Sdk "platforms\android-33\android.jar"
if (-not (Test-Path $AndroidJar)) {
    throw "No $AndroidJar. Install 'Android 13 (API 33)' with Android Studio > Tools > SDK Manager > SDK Platforms."
}
# The newest installed stable build-tools (previews such as 36.0.0-rc1 are skipped).
$Tools = Get-ChildItem (Join-Path $Sdk "build-tools") -Directory -ErrorAction SilentlyContinue |
    Where-Object { $_.Name -match '^\d+(\.\d+)+$' -and (Test-Path (Join-Path $_.FullName "aapt2.exe")) } |
    Sort-Object { [version]$_.Name } | Select-Object -Last 1
if (-not $Tools) { throw "No build-tools in $Sdk. Install them with Android Studio > Tools > SDK Manager > SDK Tools." }
$Aapt2 = Join-Path $Tools.FullName "aapt2.exe"
$D8 = Join-Path $Tools.FullName "d8.bat"
$Zipalign = Join-Path $Tools.FullName "zipalign.exe"
$Apksigner = Join-Path $Tools.FullName "apksigner.bat"

$Javac = "javac"; $Keytool = "keytool"
if ($env:JAVA_HOME) {
    $Javac = Join-Path $env:JAVA_HOME "bin\javac.exe"
    $Keytool = Join-Path $env:JAVA_HOME "bin\keytool.exe"
}
$javaVersion = (& $Javac -version | Out-String).Trim()
if ($javaVersion -notmatch "javac (\d+)" -or [int]$Matches[1] -lt 17) { throw "Need JDK 17 or later; $Javac says: $javaVersion" }

function Invoke-Tool([string]$tool, [string[]]$arguments) {
    & $tool @arguments
    if ($LASTEXITCODE -ne 0) { throw "$(Split-Path -Leaf $tool) failed (exit $LASTEXITCODE)" }
}

Add-Type -AssemblyName System.IO.Compression.FileSystem

if (Test-Path $Build) { Remove-Item -Recurse -Force $Build }
foreach ($dir in @("gen", "classes", "dex")) { New-Item -ItemType Directory -Force (Join-Path $Build $dir) | Out-Null }
Copy-Item -Recurse (Join-Path $Src "res") (Join-Path $Build "res")
New-Item -ItemType Directory -Force (Join-Path $Build "res\drawable-nodpi") | Out-Null
Copy-Item $Icon (Join-Path $Build "res\drawable-nodpi\ic_launcher_foreground.png")

$Unsigned = Join-Path $Build "unsigned.apk"
Invoke-Tool $Aapt2 @("compile", "--dir", (Join-Path $Build "res"), "-o", (Join-Path $Build "res.zip"))
Invoke-Tool $Aapt2 @("link", "-I", $AndroidJar, "--manifest", (Join-Path $Src "AndroidManifest.xml"),
    "--java", (Join-Path $Build "gen"), "-o", $Unsigned, (Join-Path $Build "res.zip"))

# javac argument files take quoted paths with forward slashes.
$sources = Get-ChildItem -Recurse -Filter *.java (Join-Path $Src "java"), (Join-Path $Build "gen") |
    ForEach-Object { '"' + ($_.FullName -replace '\\', '/') + '"' }
$sourceList = Join-Path $Build "sources.txt"
[System.IO.File]::WriteAllLines($sourceList, [string[]]$sources)
Invoke-Tool $Javac @("--release", "8", "-nowarn", "-Xlint:-options", "-classpath", $AndroidJar,
    "-d", (Join-Path $Build "classes"), "@$sourceList")
$classesJar = Join-Path $Build "classes.jar"
[System.IO.Compression.ZipFile]::CreateFromDirectory((Join-Path $Build "classes"), $classesJar)
Invoke-Tool $D8 @("--release", "--min-api", "30", "--lib", $AndroidJar, "--output", (Join-Path $Build "dex"), $classesJar)

$apk = [System.IO.Compression.ZipFile]::Open($Unsigned, "Update")
try {
    [System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile($apk, (Join-Path $Build "dex\classes.dex"), "classes.dex") | Out-Null
} finally {
    $apk.Dispose()
}
# Android 11+ installs only APKs whose resources.arsc is stored aligned.
$Aligned = Join-Path $Build "aligned.apk"
Invoke-Tool $Zipalign @("-p", "-f", "4", $Unsigned, $Aligned)

if (-not (Test-Path $Keystore)) {
    $bytes = New-Object byte[] 24
    [System.Security.Cryptography.RandomNumberGenerator]::Create().GetBytes($bytes)
    $pass = [Convert]::ToBase64String($bytes) -replace '[/+=]', ''
    [System.IO.File]::WriteAllText($KeyPass, $pass)
    Invoke-Tool $Keytool @("-genkeypair", "-keystore", $Keystore, "-storepass:file", $KeyPass,
        "-alias", "gcs", "-keyalg", "RSA", "-keysize", "3072", "-validity", "10000",
        "-dname", "CN=AAS GCS handheld") | Out-Null
}
$Apk = Join-Path $Out "aas-gcs.apk"
Invoke-Tool $Apksigner @("sign", "--ks", $Keystore, "--ks-pass", "file:$KeyPass", "--out", $Apk, $Aligned)
Invoke-Tool $Apksigner @("verify", $Apk)
Write-Host "Built $Apk"
