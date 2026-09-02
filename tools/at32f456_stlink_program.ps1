[CmdletBinding()]
param(
  [Parameter(Mandatory = $true)]
  [string]$Firmware,

  [switch]$RunAfterProgram
)

$openocd = 'D:\App\OpenOCD\bin\openocd.exe'
$scripts = 'D:\App\OpenOCD\openocd\scripts'
$flm = Join-Path $env:TEMP 'AT32F456_256.combined.bin'
$flmSource = 'D:\App\Keil\ARM\PACK\ArteryTek\AT32F45x_DFP\2.0.1\Flash\AT32F456_256.FLM'
$fromelf = 'D:\App\Keil\ARM\ARMCC\bin\fromelf.exe'
$flashBase = 0x08000000
$algorithmBase = 0x20000000
$bufferBase = 0x20004000
$stackTop = 0x2001A000
$returnBreakpoint = 0x20001300
$sectorSize = 0x800
$pageSize = 0x400

$firmwarePath = (Resolve-Path -LiteralPath $Firmware).Path
if (-not (Test-Path -LiteralPath $openocd)) {
  throw "OpenOCD not found: $openocd"
}
if (-not (Test-Path -LiteralPath $flm -PathType Leaf)) {
  if (-not (Test-Path -LiteralPath $flmSource)) {
    throw "Official AT32F456 FLM not found: $flmSource"
  }
  if (-not (Test-Path -LiteralPath $fromelf)) {
    throw "FromElf not found: $fromelf"
  }
  # FLM含PrgCode与DevDscr两个加载区；--bin会创建目录，需合并为单一RAM镜像。
  & $fromelf --bincombined --output $flm $flmSource
  if (($LASTEXITCODE -ne 0) -or
      (-not (Test-Path -LiteralPath $flm -PathType Leaf)) -or
      ((Get-Item -LiteralPath $flm).Length -ne 4848)) {
    throw 'Failed to extract the official AT32F456 FLM binary'
  }
}

$firmwareLength = (Get-Item -LiteralPath $firmwarePath).Length
if (($firmwareLength -le 0) -or ($firmwareLength -gt 0x40000)) {
  throw "Firmware size is invalid for AT32F456RCT7: $firmwareLength"
}

$firmwareOcd = $firmwarePath.Replace('\', '/')
$flmOcd = $flm.Replace('\', '/')
$commands = [System.Collections.Generic.List[string]]::new()

@(
  'adapter speed 100',
  'init',
  'halt',
  'wait_halt 3000',
  # NRST未连接，使用目标配置中的SYSRESETREQ回到复位态后再装载FLM。
  'reset halt',
  'wait_halt 3000',
  'reg primask 1',
  'mww 0xE000E010 0',
  # TMR8 主输出与四路比较值清零，避免停核时保留旧 PWM。
  'mww 0x40010444 0',
  'mww 0x40010434 0',
  'mww 0x40010438 0',
  'mww 0x4001043C 0',
  'mww 0x40010440 0'
) | ForEach-Object { $commands.Add($_) }

$commands.Add("load_image $flmOcd 0x20000000 bin")
$commands.Add("load_image $firmwareOcd 0x20004000 bin")
$commands.Add('mwh 0x20001300 0xBE00')

function Add-FlmCall {
  param(
    [string]$Name,
    [string]$ProgramCounter,
    [string]$R0,
    [string]$R1,
    [string]$R2,
    [int]$TimeoutMs
  )

  $commands.Add("echo FLM_BEGIN_$Name")
  # 官方FLM采用RWPI；PrgData位于+0x248，r9不能指向算法代码起点。
  $commands.Add(('reg r9 0x{0:X8}' -f ($algorithmBase + 0x248)))
  $commands.Add(('reg msp 0x{0:X8}' -f $stackTop))
  $commands.Add(('reg sp 0x{0:X8}' -f $stackTop))
  $commands.Add(('reg lr 0x{0:X8}' -f ($returnBreakpoint + 1)))
  $commands.Add("reg r0 $R0")
  $commands.Add("reg r1 $R1")
  $commands.Add("reg r2 $R2")
  $commands.Add("resume $ProgramCounter")
  $commands.Add("wait_halt $TimeoutMs")
  $commands.Add("echo FLM_RETURN_$Name=[reg r0]")
  $commands.Add("echo FLM_END_$Name")
}

Add-FlmCall 'INIT_ERASE' '0x200000A3' '0x08000000' '12000000' '1' 5000
$sectorCount = [int][Math]::Ceiling($firmwareLength / [double]$sectorSize)
for ($sector = 0; $sector -lt $sectorCount; $sector++) {
  $address = '0x{0:X8}' -f ($flashBase + ($sector * $sectorSize))
  Add-FlmCall "ERASE_$sector" '0x2000012B' $address '0' '0' 15000
}
Add-FlmCall 'UNINIT_ERASE' '0x200000DB' '1' '0' '0' 5000

Add-FlmCall 'INIT_PROGRAM' '0x200000A3' '0x08000000' '12000000' '2' 5000
$pageCount = [int][Math]::Ceiling($firmwareLength / [double]$pageSize)
for ($page = 0; $page -lt $pageCount; $page++) {
  $offset = $page * $pageSize
  $size = [Math]::Min($pageSize, $firmwareLength - $offset)
  $flashAddress = '0x{0:X8}' -f ($flashBase + $offset)
  $bufferAddress = '0x{0:X8}' -f ($bufferBase + $offset)
  Add-FlmCall "PROGRAM_$page" '0x2000019D' $flashAddress ([string]$size) $bufferAddress 10000
}
Add-FlmCall 'UNINIT_PROGRAM' '0x200000DB' '2' '0' '0' 5000

$commands.Add("verify_image $firmwareOcd 0x08000000 bin")
$commands.Add('echo FLASH_VERIFY_COMMAND_OK')
$commands.Add('echo FLASH_WORDS=[read_memory 0x08000000 32 8]')
$commands.Add('shutdown')

$logPath = Join-Path $env:TEMP 'at32f456_stlink_program.log'
$commandPath = Join-Path $env:TEMP 'at32f456_stlink_program.cfg'
$configLines = [System.Collections.Generic.List[string]]::new()
$configLines.Add('source [find interface/stlink.cfg]')
$configLines.Add('transport select swd')
# 该批次AT32F456实测SWD DPIDR为0x2BA01477。
$configLines.Add('set CPUTAPID 0x2ba01477')
$configLines.Add('source [find target/artery/at32f4x.cfg]')
foreach ($command in $commands) {
  $configLines.Add($command)
}
[System.IO.File]::WriteAllLines(
  $commandPath,
  $configLines,
  [System.Text.UTF8Encoding]::new($false))

& $openocd -s $scripts -f $commandPath 2>&1 |
  Tee-Object -FilePath $logPath |
  Where-Object { $_ -match 'FLM_BEGIN|FLM_RETURN|FLM_END|FLASH_VERIFY|FLASH_WORDS|Error:' }
$flashExitCode = $LASTEXITCODE
if ($flashExitCode -ne 0) {
  throw "Programming or verification failed; target remains halted. OpenOCD exit: $flashExitCode"
}

$logText = [System.IO.File]::ReadAllText($logPath)
$badCalls = @()
$callPattern = '(?s)FLM_BEGIN_(?<name>\S+)(?<body>.*?)FLM_END_\k<name>'
foreach ($call in [regex]::Matches($logText, $callPattern)) {
  $returns = [regex]::Matches($call.Groups['body'].Value,
                              'FLM_RETURN_\S+=.*0x(?<value>[0-9A-Fa-f]{8})')
  if (($returns.Count -eq 0) -or
      ($returns[$returns.Count - 1].Groups['value'].Value -ne '00000000')) {
    $badCalls += $call.Groups['name'].Value
  }
}
if ($badCalls.Count -ne 0) {
  throw "FLM returned an error; target remains halted: $($badCalls -join ', ')"
}

Write-Output "FLASH_VERIFY_OK bytes=$firmwareLength"
if ($RunAfterProgram) {
  # 使用 Cortex-M SYSRESETREQ，不依赖未连接的 NRST 线。
  $resetArguments = @(
    '-s', $scripts,
    '-f', 'interface/stlink.cfg',
    '-c', 'transport select swd',
    '-c', 'set CPUTAPID 0x2ba01477',
    '-f', 'target/artery/at32f4x.cfg',
    '-c', 'adapter speed 100',
    '-c', 'init',
    '-c', 'halt',
    '-c', 'reset run',
    '-c', 'shutdown'
  )
  & $openocd @resetArguments | Out-Null
  $resetExitCode = $LASTEXITCODE
  if ($resetExitCode -ne 0) {
    throw "Flash verified, but reset run failed: $resetExitCode"
  }
  Write-Output 'RESET_RUN_OK'
}
