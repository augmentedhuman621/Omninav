# =============================================================================
#  OmniNav STM32 —— 一键「编译 + 烧录 + 串口回读」
#
#  这个脚本证明了：这套 Keil 工程可以被完全无人化地构建与烧录，
#  不需要任何人手点 µVision 的 F7 / F8。Lead（AI）自己就能跑。
#
#  用法（在 D:\dsh\工程设计\stm32_fw\tools 下用 pwsh 运行）：
#     .\build_flash.ps1                 # 全量重编 + 下载 + 读 4 秒串口
#     .\build_flash.ps1 -BuildOnly      # 只编译
#     .\build_flash.ps1 -FlashOnly      # 只下载（要求已经编译过）
#     .\build_flash.ps1 -NoSerial       # 编译+下载，但不读串口
#
#  依赖：Keil MDK 装在 D:\Keil_v5（UV4.exe 在 D:\Keil_v5\UV4\UV4.exe）
#  UV4 退出码：0=无错无警 1=有警告 2=有错误 3=致命 11=打不开工程 20=命令行错
# =============================================================================

[CmdletBinding()]
param(
    [switch]$BuildOnly,
    [switch]$FlashOnly,
    [switch]$NoSerial,
    [string]$ComPort = 'COM9',
    [int]$SerialSeconds = 4
)

$ErrorActionPreference = 'Continue'

$Uv4   = 'D:\Keil_v5\UV4\UV4.exe'
$Root  = Split-Path -Parent $PSScriptRoot          # ...\stm32_fw
$Proj  = Join-Path $Root 'MDK-ARM\OmniNav_STM32.uvprojx'
$ObjD  = Join-Path $Root 'MDK-ARM\Objects'
$AxF   = Join-Path $ObjD 'OmniNav_STM32.axf'
$Tmp   = Join-Path $Root '_probe'
$BLog  = Join-Path $Tmp 'build.log'
$FLog  = Join-Path $Tmp 'flash.log'

if (-not (Test-Path $Uv4))  { Write-Host "[X] 找不到 $Uv4" -ForegroundColor Red; exit 11 }
if (-not (Test-Path $Proj)) { Write-Host "[X] 找不到 $Proj" -ForegroundColor Red; exit 11 }
if (-not (Test-Path $Tmp))  { New-Item -ItemType Directory -Force -Path $Tmp | Out-Null }

# ---------------------------------------------------------------- 编译
if (-not $FlashOnly) {
    Write-Host '== 1. µVision 全量重编（UV4 -r） ==' -ForegroundColor Cyan
    Remove-Item $BLog -Force -ErrorAction SilentlyContinue
    $sw = [System.Diagnostics.Stopwatch]::StartNew()
    $p  = Start-Process -FilePath $Uv4 -ArgumentList @('-j0', '-r', $Proj, '-o', $BLog) -Wait -PassThru
    $sw.Stop()
    $code = $p.ExitCode
    Write-Host ("   UV4 -r EXIT={0}   耗时 {1:N1} s" -f $code, $sw.Elapsed.TotalSeconds)

    # 只把结论行打出来 —— 743 条警告全在 ST 的 HAL 头文件里，淹没有用信息
    if (Test-Path $BLog) {
        Get-Content $BLog | Select-String -Pattern 'Program Size|Error\(s\)|Build Time Elapsed' |
            ForEach-Object { "   $_" }
    }
    if ($code -ge 2) { Write-Host '[X] 编译失败，看 _probe\build.log' -ForegroundColor Red; exit $code }
    if (-not (Test-Path $AxF)) { Write-Host "[X] 没有产出 $AxF" -ForegroundColor Red; exit 2 }
    Write-Host ("   [OK] axf = {0} B" -f (Get-Item $AxF).Length) -ForegroundColor Green
}

if ($BuildOnly) { exit 0 }

# ---------------------------------------------------------------- 烧录
Write-Host '== 2. 下载到 Flash（UV4 -f，走 ST-Link SWD） ==' -ForegroundColor Cyan
Remove-Item $FLog -Force -ErrorAction SilentlyContinue
$sw2 = [System.Diagnostics.Stopwatch]::StartNew()
$p2  = Start-Process -FilePath $Uv4 -ArgumentList @('-j0', '-f', $Proj, '-o', $FLog) -Wait -PassThru
$sw2.Stop()
Write-Host ("   UV4 -f EXIT={0}   耗时 {1:N1} s" -f $p2.ExitCode, $sw2.Elapsed.TotalSeconds)
if (Test-Path $FLog) { Get-Content $FLog | ForEach-Object { "   $_" } }
if ($p2.ExitCode -ne 0) {
    Write-Host '[!] 下载失败。常见原因：' -ForegroundColor Yellow
    Write-Host '      - Cannot access target  -> 板子没独立供电（只靠 ST-Link 的 3.3V 针带不动）'
    Write-Host '      - No Cortex-M SW Device -> SWD 四线没接好（1=3.3V 7=SWDIO 9=SWCLK GND 任选）'
    Write-Host '      - 山寨 ST-Link         -> Debug->Settings 里把 Reset 改成 Connect under Reset'
    exit $p2.ExitCode
}

# ---------------------------------------------------------------- 串口回读
if (-not $NoSerial) {
    Write-Host ("== 3. 从 {0} 读 {1} 秒串口回显 ==" -f $ComPort, $SerialSeconds) -ForegroundColor Cyan
    try {
        $sp = New-Object System.IO.Ports.SerialPort $ComPort, 115200, 'None', 8, 'One'
        # ★★ 必须先置 false 再 Open：CH340 的 DTR/RTS 接着一键下载电路（管 BOOT0/NRST），
        #    打开串口时如果它们被拉高，STM32 会被拽进 bootloader 或反复复位。
        $sp.DtrEnable  = $false
        $sp.RtsEnable  = $false
        $sp.ReadTimeout = 500
        $sp.Open()
        Start-Sleep -Milliseconds 300
        $sb = [System.Text.StringBuilder]::new()
        $sw3 = [System.Diagnostics.Stopwatch]::StartNew()
        while ($sw3.Elapsed.TotalSeconds -lt $SerialSeconds) {
            try { $null = $sb.Append($sp.ReadExisting()) } catch { }
            Start-Sleep -Milliseconds 80
        }
        $sp.Close()
        $text = $sb.ToString()
        Write-Host ("   读到 {0} 个字符" -f $text.Length)
        $text -split "`r`n" | ForEach-Object { "   | $_" }
        if ($text.Length -gt 0) { Write-Host '   ★★ 串口有输出，固件在跑' -ForegroundColor Green }
        else { Write-Host '   [!] 没有字节。板子跑起来了吗？串口口对吗？（选 -ComPort COMx）' -ForegroundColor Yellow }
    }
    catch { Write-Host ("   [X] 打不开 {0}：{1}" -f $ComPort, $_.Exception.Message) -ForegroundColor Red; exit 3 }
}

exit 0
