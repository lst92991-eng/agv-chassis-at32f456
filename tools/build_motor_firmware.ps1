[CmdletBinding()]
param([switch]$Rebuild)

$repoRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$projectPath = Join-Path $repoRoot 'mdk_v5\agv_motor.uvprojx'
$objectDirectory = Join-Path $repoRoot 'mdk_v5\objects'
$axfPath = Join-Path $objectDirectory 'agv_motor.axf'
$binPath = Join-Path $objectDirectory 'agv_motor.bin'
$uv4Path = 'D:\App\Keil\UV4\UV4.exe'
$fromelfPath = 'D:\App\Keil\ARM\ARMCC\bin\fromelf.exe'
$buildLog = Join-Path ([IO.Path]::GetTempPath()) 'agv_motor_build.log'

foreach($requiredPath in @($projectPath, $uv4Path, $fromelfPath)) {
  if(-not (Test-Path -LiteralPath $requiredPath)) {
    throw "Required build input not found: $requiredPath"
  }
}

$buildMode = if($Rebuild) { '-r' } else { '-b' }
$arguments = @($buildMode, $projectPath, '-j0', '-o', $buildLog)
$process = Start-Process -FilePath $uv4Path `
  -ArgumentList $arguments `
  -Wait `
  -WindowStyle Hidden `
  -PassThru

if(-not (Test-Path -LiteralPath $buildLog)) {
  throw 'Keil did not create a build log'
}
$logText = Get-Content -LiteralPath $buildLog -Raw
Write-Output $logText.TrimEnd()

if(($process.ExitCode -ne 0) -or
   ($logText -notmatch '0 Error\(s\), 0 Warning\(s\)') -or
   (-not (Test-Path -LiteralPath $axfPath))) {
  throw "Motor firmware build failed; log: $buildLog"
}

& $fromelfPath --bin --output $binPath $axfPath
if(($LASTEXITCODE -ne 0) -or (-not (Test-Path -LiteralPath $binPath))) {
  throw 'fromelf failed to generate agv_motor.bin'
}

$firmware = Get-Item -LiteralPath $binPath
$hash = Get-FileHash -LiteralPath $binPath -Algorithm SHA256
Write-Output ("FIRMWARE={0}" -f $firmware.FullName)
Write-Output ("SIZE_BYTES={0}" -f $firmware.Length)
Write-Output ("SHA256={0}" -f $hash.Hash)
