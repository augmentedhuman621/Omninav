# =============================================================================
#  dismiss_stlink_popup.ps1
#  用途：无人值守地关掉 Keil 每次连 ST-Link 时弹的
#        「ST-LINK Firmware Upgrade / Old ST-LINK firmware detected.」模态窗。
#
#  行为：只点 No（= 拒绝升级），绝不去碰 Yes。
#        ★ 千万不要把这里改成点 Yes —— 克隆版 ST-Link 升级失败会变砖。
#
#  用法：powershell.exe -NoProfile -ExecutionPolicy Bypass -File dismiss_stlink_popup.ps1 -Seconds 180
#
#  背景：这个弹窗会挡住 UV4 -f（烧录）直到有人点 No（实测卡 85 秒）。
#        UV4 -r（编译）不碰仿真器，所以不受影响。
#
#  ★★ 2026-10-07 修正（核心-link-dev 实测发现的真 bug）：
#     旧版用  Get-Process | Where-Object { $_.MainWindowHandle -ne 0 }
#     去枚举窗口 —— 但**这个弹窗属于 headless 的 UV4.exe 自己**，而 headless
#     UV4 的 MainWindowHandle 是 0，于是永远枚举不到它，脚本形同虚设
#     （5 次烧录里 4 次被弹窗卡死）。
#     现在改成 EnumWindows 枚举**所有**顶级窗口，完全不依赖进程信息。
#     另：同一个 UV4 进程还有第二个隐藏的 #32770 窗（"Find Trace"，
#     里面没有 No 按钮），所以必须按标题含 ST-LINK + 真有 No 按钮 双重筛选。
# =============================================================================
param(
    [int]$Seconds = 180,          # 监视多久
    [int]$PollMs   = 250,         # 轮询间隔
    [switch]$Quiet
)

Add-Type @'
using System;
using System.Text;
using System.Runtime.InteropServices;
public class W32Popup {
    public delegate bool EnumProc(IntPtr hWnd, IntPtr lParam);
    [DllImport("user32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    public static extern bool EnumWindows(EnumProc lpEnumFunc, IntPtr lParam);
    [DllImport("user32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    public static extern bool EnumChildWindows(IntPtr hWndParent, EnumProc lpEnumFunc, IntPtr lParam);
    [DllImport("user32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    public static extern int GetWindowText(IntPtr hWnd, StringBuilder lpString, int nMaxCount);
    [DllImport("user32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    public static extern int GetClassName(IntPtr hWnd, StringBuilder lpClassName, int nMaxCount);
    [DllImport("user32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    public static extern bool IsWindowVisible(IntPtr hWnd);
    [DllImport("user32.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    public static extern IntPtr SendMessage(IntPtr hWnd, uint Msg, IntPtr wParam, IntPtr lParam);

    public static string Text(IntPtr h) {
        StringBuilder sb = new StringBuilder(512);
        GetWindowText(h, sb, sb.Capacity);
        return sb.ToString();
    }
    public static string Cls(IntPtr h) {
        StringBuilder sb = new StringBuilder(256);
        GetClassName(h, sb, sb.Capacity);
        return sb.ToString();
    }
}
'@

$BM_CLICK = 0x00F5

# --- 只点 No 的核心逻辑 -------------------------------------------------
function Invoke-DismissOnce {
    param([switch]$Verbose2)
    $script:killed = 0
    $tops = New-Object System.Collections.ArrayList

    # ★ 关键修正：EnumWindows 枚举**所有**顶级窗口，不看进程
    $cbTop = [W32Popup+EnumProc]{
        param($h, $lp)
        if ([W32Popup]::Cls($h) -eq '#32770') { $null = $script:tops.Add($h) }
        return $true
    }
    $script:tops = $tops
    $null = [W32Popup]::EnumWindows($cbTop, [IntPtr]::Zero)

    foreach ($h in $script:tops) {
        $title = [W32Popup]::Text($h)
        if ($title -notmatch 'ST-?LINK') { continue }

        # 在子窗口里找文本为 No / &No 的按钮
        $script:btnNo = [IntPtr]::Zero
        $cbChild = [W32Popup+EnumProc]{
            param($child, $lp)
            $t = [W32Popup]::Text($child)
            if ($t -eq 'No' -or $t -eq '&No') { $script:btnNo = $child }
            return $true
        }
        $null = [W32Popup]::EnumChildWindows($h, $cbChild, [IntPtr]::Zero)

        if ($script:btnNo -ne [IntPtr]::Zero) {
            $null = [W32Popup]::SendMessage($script:btnNo, $BM_CLICK, [IntPtr]::Zero, [IntPtr]::Zero)
            $script:killed++
            if (-not $Quiet) { "   [dismiss] 已自动点 No：`"$title`"  hwnd=$h  (第 $script:killed 次)" }
        }
    }
    return $script:killed
}

if (-not $Quiet) { "   [dismiss] 开始监视 $Seconds 秒（弹窗出现即点 No）" }

$total = 0
$deadline = (Get-Date).AddSeconds($Seconds)
while ((Get-Date) -lt $deadline) {
    $total += (Invoke-DismissOnce)
    Start-Sleep -Milliseconds $PollMs
}

if (-not $Quiet) { "   [dismiss] 监视结束，共关掉 $total 个弹窗" }
exit 0
