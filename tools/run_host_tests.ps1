[CmdletBinding()]
param()

$vcvarsCandidates = @(
  'C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat',
  'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat'
)
$vcvarsPath = $vcvarsCandidates | Where-Object {
  Test-Path -LiteralPath $_
} | Select-Object -First 1
if($null -eq $vcvarsPath) {
  throw 'Visual Studio C compiler environment was not found'
}

$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$sourceIncludePaths = @(
  (Join-Path $repoRoot 'src\app'),
  (Join-Path $repoRoot 'src\services'),
  (Join-Path $repoRoot 'src\drivers'),
  (Join-Path $repoRoot 'src\config')
)
$sourceIncludeArguments = ($sourceIncludePaths | ForEach-Object {
  '/I "{0}"' -f $_
}) -join ' '
$testIncludePath = Join-Path $repoRoot 'tests'
$boardIncludePath = Join-Path $repoRoot 'at32f45x_board'

function Invoke-CHostTest {
  param(
    [Parameter(Mandatory = $true)]
    [string]$Name,
    [Parameter(Mandatory = $true)]
    [string]$TestRelativePath,
    [string[]]$ModuleRelativePath = @(),
    [string[]]$IncludeRelativePath = @()
  )

  $testSource = Join-Path $repoRoot $TestRelativePath
  $moduleSources = $ModuleRelativePath | ForEach-Object {
    '"{0}"' -f (Join-Path $repoRoot $_)
  }
  $moduleArguments = $moduleSources -join ' '
  $extraIncludeArguments = ($IncludeRelativePath | ForEach-Object {
    '/I "{0}"' -f (Join-Path $repoRoot $_)
  }) -join ' '
  $testOutput = Join-Path ([IO.Path]::GetTempPath()) ("agv_{0}_{1}" -f $Name, [guid]::NewGuid().ToString('N'))
  New-Item -ItemType Directory -Path $testOutput | Out-Null
  $testExecutable = Join-Path $testOutput "$Name.exe"
  $command = 'call "{0}" >nul && cl /nologo /W4 /WX /utf-8 /TC "{1}" {2} {3} {7} /I "{4}" /I "{5}" /Fe:"{6}" && "{6}"' -f `
    $vcvarsPath, $testSource, $moduleArguments, $sourceIncludeArguments, `
    $testIncludePath, $boardIncludePath, $testExecutable, $extraIncludeArguments

  Push-Location $testOutput
  try { & $env:ComSpec /d /c $command } finally { Pop-Location }
  if($LASTEXITCODE -ne 0) {
    throw "Host test failed: $Name"
  }
}

Invoke-CHostTest `
  -Name 'odometry_test' `
  -TestRelativePath 'tests\odometry_test.c' `
  -ModuleRelativePath 'src\services\odometry.c'
Invoke-CHostTest `
  -Name 'drive_control_test' `
  -TestRelativePath 'tests\drive_control_test.c' `
  -ModuleRelativePath 'src\services\drive_control.c'
Invoke-CHostTest `
  -Name 'power_telemetry_test' `
  -TestRelativePath 'tests\power_telemetry_test.c' `
  -ModuleRelativePath 'src\services\power_telemetry.c'
Invoke-CHostTest `
  -Name 'chassis_protocol_test' `
  -TestRelativePath 'tests\chassis_protocol_test.c' `
  -ModuleRelativePath 'src\services\chassis_protocol.c'
Invoke-CHostTest `
  -Name 'chassis_app_test' `
  -TestRelativePath 'tests\chassis_app_test.c' `
  -ModuleRelativePath 'src\services\power_telemetry.c'
Invoke-CHostTest `
  -Name 'app_freertos_test' `
  -TestRelativePath 'tests\app_freertos_test.c' `
  -ModuleRelativePath 'src\app\app_freertos.c' `
  -IncludeRelativePath 'tests\rtos_stubs', 'inc', 'libraries\cmsis\cm4\device_support'
