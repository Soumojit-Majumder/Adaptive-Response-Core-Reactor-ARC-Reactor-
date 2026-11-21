$ErrorActionPreference = 'Stop'
$repoRoot = Split-Path -Parent $PSScriptRoot
$firmwarePath = Join-Path $repoRoot 'Firmware\ARC_Reactor_Sensor___Dashboard\ARC_Reactor_Sensor___Dashboard.ino'
if (-not (Test-Path -LiteralPath $firmwarePath)) { throw "Firmware not found: $firmwarePath" }
$source = [IO.File]::ReadAllText($firmwarePath)
$start = $source.IndexOf('// ---------------- small vector helpers ----------------', [StringComparison]::Ordinal)
$end = $source.IndexOf('// ---------------- ECG sampling + heart-rate task ----------------', [StringComparison]::Ordinal)
if ($start -lt 0 -or $end -le $start) { throw 'Could not locate vector, pose, and notch algorithms in the firmware.' }
$algorithms = $source.Substring($start, $end - $start)
$medianStart = $source.IndexOf('static inline float med5(', [StringComparison]::Ordinal)
$samplerStart = $source.IndexOf('void sampler(void*)', $medianStart, [StringComparison]::Ordinal)
if ($medianStart -lt 0 -or $samplerStart -le $medianStart) { throw 'Could not locate ECG median helpers in the firmware.' }
$algorithms += $source.Substring($medianStart, $samplerStart - $medianStart)
$prefix = @"
#include <cstddef>
#include <cstdint>
#include <cmath>
#include <iostream>
#define PI 3.14159265358979323846f
constexpr float FS = 250.0f;
constexpr float POSE_AXIS_MIN = 0.70f;
enum Pose : uint8_t { P_UNKNOWN, P_UPRIGHT, P_BACK, P_STOMACH, P_UPSIDE, P_SIDE, P_TILT };
struct PreferencesMock { void begin(const char*, bool) {} size_t getBytesLength(const char*) { return 0; } size_t getBytes(const char*, void*, size_t) { return 0; } void end() {} size_t putBytes(const char*, const void*, size_t) { return 0; } } prefs;
"@
$tests = @"
int failures = 0;
void check(bool condition, const char* name) { std::cout << (condition ? "PASS " : "FAIL ") << name << "\n"; if (!condition) ++failures; }
int main() {
  check(med5(1, 2, 100, 3, 4) == 3, "median-of-five suppresses high impulse");
  check(med5(1, -100, 2, 3, 4) == 2, "median-of-five suppresses low impulse");
  uint32_t rr[] = {100, 200, 300, 400, 500, 600, 9000};
  check(medianU(rr, 7) == 400, "RR median rejects high outlier");
  Notch notch50; notch50.init(50, 10); notch50.prime(0); double energy50 = 0;
  for (int i = 0; i < 5000; ++i) { float y = notch50.run(sinf(2 * PI * 50 * i / FS)); if (i >= 2500) energy50 += y * y; }
  energy50 = sqrt(energy50 / 2500);
  Notch notch10; notch10.init(50, 10); notch10.prime(0); double energy10 = 0;
  for (int i = 0; i < 5000; ++i) { float y = notch10.run(sinf(2 * PI * 10 * i / FS)); if (i >= 2500) energy10 += y * y; }
  energy10 = sqrt(energy10 / 2500);
  check(energy50 < 0.05, "50 Hz notch attenuates interference");
  check(energy10 > 0.65, "notch preserves 10 Hz signal");
  rebuildBasis();
  check(classify({0, 1, 0}) == P_UPRIGHT, "upright pose");
  check(classify({0, -1, 0}) == P_UPSIDE, "upside-down pose");
  check(classify({0, 0, 1}) == P_BACK, "back pose");
  check(classify({0, 0, -1}) == P_STOMACH, "stomach pose");
  check(classify({1, 0, 0}) == P_SIDE, "side pose");
  check(classify({0.4f, 0.4f, 0.4f}) == P_TILT, "tilted pose");
  cal = {{0, 1, 0}, {0, 1, 0}, 3}; rebuildBasis();
  check(std::fabs(v_dot(bU, bF)) < 0.001f, "parallel calibration vectors use orthogonal fallback");
  return failures ? 1 : 0;
}
"@
$compiler = Get-Command g++ -ErrorAction SilentlyContinue
if (-not $compiler) { throw 'g++ is required for this host simulation.' }
$tempPath = Join-Path $env:TEMP 'arc-reactor-host-simulation'
New-Item -ItemType Directory -Force -Path $tempPath | Out-Null
$cppPath = Join-Path $tempPath 'simulation.cpp'; $exePath = Join-Path $tempPath 'simulation.exe'
[IO.File]::WriteAllText($cppPath, $prefix + $algorithms + $tests)
& $compiler.Source -std=c++17 -Wall -Wextra -Werror $cppPath -o $exePath
if ($LASTEXITCODE -ne 0) { throw 'Host simulation compile failed.' }
Write-Output 'Host simulation compile: PASS'
& $exePath
if ($LASTEXITCODE -ne 0) { throw 'One or more host simulation checks failed.' }
$jsMatch = [regex]::Match($source, '<script>([\s\S]*?)</script>')
if (-not $jsMatch.Success) { throw 'Dashboard script block was not found.' }
$jsPath = Join-Path $tempPath 'dashboard.js'; [IO.File]::WriteAllText($jsPath, $jsMatch.Groups[1].Value)
Get-Content -Raw -LiteralPath $jsPath | node --check -
if ($LASTEXITCODE -ne 0) { throw 'Dashboard JavaScript syntax check failed.' }
Write-Output 'Dashboard JavaScript syntax: PASS'