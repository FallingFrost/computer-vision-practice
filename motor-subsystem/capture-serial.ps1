#Requires -Version 7
<#
.SYNOPSIS
    抓取 StampC3 串口输出，存成能被 Colab notebook 直接读取的 CSV。

.DESCRIPTION
    电机练习用。上传好代码、插上 USB-C 后运行本脚本，按 StampC3 上的按钮开始，
    跑够就 Ctrl+C。练习 2 是跑一轮 10 秒 / 200 行；练习 3 是按一次按钮跑完
    整条死区扫描（约 2 分 15 秒），所以练习 3 建议直接给 -Seconds 自动停。

    练习 2 的输出文件名默认值就是 notebook 里 CSV_FILENAME 的默认值。
    练习 3 换练习了，表头不同，必须显式给 -ExpectedHeader 和 -OutFile，见 .EXAMPLE。

    实现说明：用 .NET SerialPort 而不是 `arduino-cli monitor`。
    后者会从 stdin 读数据往串口发，在非交互环境（stdin 接空设备）里
    一读到 EOF 就自己退出，抓不到任何东西。

.PARAMETER Port
    串口号，如 COM3。不给就自动找。

.PARAMETER OutFile
    输出文件名。

.PARAMETER Seconds
    跑多少秒后自动停止。不指定则一直跑到 Ctrl+C。

.PARAMETER ExpectedHeader
    这一轮固件会打印的表头，整行照抄。默认是练习 2 的 9 列。
    只用它的第一列来判「这行是不是 CSV」，以及事后检查表头有没有抓全。
    换练习必须改：练习 3 的第一列是 wheel，不是 trial_num。

.EXAMPLE
    .\capture-serial.ps1
    .\capture-serial.ps1 -Port COM3 -Seconds 60
    .\capture-serial.ps1 -OutFile trial1.csv

.EXAMPLE
    # 练习 3 死区扫描：整条 ladder 约 2 分 15 秒，给 150 秒收尾
    cd "D:\work space\robotics\StampC3projects"
    .\capture-serial.ps1 -Port COM3 -Seconds 150 `
      -OutFile 数据\motor_exercise03_left_fwd.csv `
      -ExpectedHeader "wheel,direction,PWM,trial_num,sample_num,speed_cps,speed_mm_s,dt_ms,time_ms,encoder_count,other_encoder_count,rest_residual_counts,rest_settled"

.EXAMPLE
    # 练习 4 饱和扫描：整条 ladder 约 3 分 15 秒，给 210 秒收尾
    .\capture-serial.ps1 -Port COM3 -Seconds 210 `
      -OutFile 数据\motor_exercise04_saturation.csv `
      -ExpectedHeader "wheel,direction,PWM,trial_num,sample_num,settled_speed_cps,speed_mm_s,dt_ms,time_ms,encoder_count,other_encoder_count"
#>
[CmdletBinding()]
param(
    [string]$Port,
    [string]$OutFile = "数据\motor_exercise02_encoder_snapshots.csv",
    [int]$Seconds = 0,
    # 这一轮预期抓到的表头。行过滤和「缺表头补写」都由它推导，
    # 所以换练习时只改这一个参数，不用动脚本逻辑。
    [string]$ExpectedHeader = "trial_num,sample_num,timestamp_ms,requested_left_pwm,requested_right_pwm,left_encoder_count,right_encoder_count,left_speed_mm_s,right_speed_mm_s"
)

$ErrorActionPreference = "Stop"

# --- 什么算一行 CSV ---
# 判据是「行型」而不是具体练习：第一个字段是裸标识符（列名或 left/right 这类标签）
# 或数字，且紧跟一个逗号。
#
# 为什么不用「表头第一列的名字」来匹配：那个名字在数据行里根本不出现。
# 练习 3 的表头是 wheel,...，但数据行是 left,... / right,...——
# 拿 wheel 去匹配会把整份数据当启动横幅丢掉。
#
# 为什么这个行型能挡掉启动横幅：ESP-ROM 那几行的第一个字段都带冒号
# （ESP-ROM:esp32c3-…、rst:0x1 (POWERON)、mode:DIO, clock div:1、
# load:0x3fcd5810,len:0x16d4），冒号不是标识符字符，匹配不上。
$CsvLinePattern = '^(?:[A-Za-z_][A-Za-z0-9_.\-]*|-?\d+),'

# 表头检测仍然由 -ExpectedHeader 推导：表头行的第一个字段就是列名本身。
$FirstColumn = ($ExpectedHeader -split ',')[0].Trim()
if (-not $FirstColumn) {
    throw "-ExpectedHeader 至少要有一列，例如 ""wheel,direction,..."""
}
$HeaderPattern = '^' + [regex]::Escape($FirstColumn) + ','

# ---------- 找串口 ----------
if (-not $Port) {
    Write-Host "未指定 -Port，正在自动查找串口..."
    $found = Get-PnpDevice -Class Ports -Status OK -ErrorAction SilentlyContinue |
        Where-Object { $_.FriendlyName -match '\((COM\d+)\)' } |
        ForEach-Object {
            [pscustomobject]@{
                Port = [regex]::Match($_.FriendlyName, '\((COM\d+)\)').Groups[1].Value
                Name = $_.FriendlyName
            }
        }

    if (-not $found) {
        Write-Error "没找到任何串口。检查：1) USB-C 线插好了吗 2) 板子开机了吗 3) 换根线试试（有些线只供电不传数据）"
    }

    if ($found.Count -eq 1) {
        $Port = $found[0].Port
        Write-Host "找到：$($found[0].Name)"
    } else {
        Write-Host "找到多个串口，请用 -Port 指定其中之一："
        $found | Format-Table -AutoSize
        exit 1
    }
}

# 相对路径按"脚本所在目录"解析，而不是"当前目录"。
# 否则在 C:\Users\hbx 这种默认目录里跑脚本，文件会落在那边找不着。
# 脚本放在项目根目录，所以相对路径 = 相对项目根目录，例如 -OutFile 数据\xxx.csv
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$full = if ([System.IO.Path]::IsPathRooted($OutFile)) {
    $OutFile
} else {
    Join-Path $scriptDir $OutFile
}

# ---------- 打开串口 ----------
# 不要碰 DTR/RTS：那两根线接在板子的复位和 BOOT 电路上，乱动可能把板子
# 按进下载模式。只听不发，保持它们为低即可。
$sp = [System.IO.Ports.SerialPort]::new(
    $Port,
    115200,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$sp.DtrEnable = $false
$sp.RtsEnable = $false
$sp.ReadTimeout = 500
$sp.NewLine = "`n"

$writer = [System.IO.StreamWriter]::new($full, $false, [System.Text.UTF8Encoding]::new($false))
$writer.AutoFlush = $true

Write-Host ""
Write-Host "端口    : $Port"
Write-Host "波特率  : 115200"
Write-Host "输出到  : $full"
if ($Seconds -gt 0) { Write-Host "时长    : $Seconds 秒后自动停止" }
Write-Host ""
Write-Host "现在按 StampC3 上的按钮开始一轮 trial。" -ForegroundColor Cyan
Write-Host ""

$deadline = if ($Seconds -gt 0) { (Get-Date).AddSeconds($Seconds) } else { [datetime]::MaxValue }
$count = 0
$dropped = 0

try {
    $sp.Open()

    while ((Get-Date) -lt $deadline) {
        try {
            $line = $sp.ReadLine()
        }
        catch [System.TimeoutException] {
            # 500ms 内没收到一整行，正常，继续等
            continue
        }

        if ($null -eq $line) { continue }
        $line = $line.TrimEnd("`r")
        if ($line -eq "") { continue }

        # 只留 CSV：以 $FirstColumn 开头的表头行，或形如 "数字,..." 的数据行。
        # 板子复位/上电时会先吐一段 ESP-ROM 启动横幅（9 行左右），
        # 混进文件里 pandas 会把第一行当成列名，整个表就废了。
        # 固件打的 "[db] ..." 这类状态行也会走到这里被丢掉，不会进文件——
        # 所以凡是分析要用的量，都必须在固件里做成 CSV 列，不能靠调试行。
        if ($line -notmatch $CsvLinePattern) {
            Write-Host "  [丢弃] $line" -ForegroundColor DarkGray
            $dropped++
            continue
        }

        Write-Host $line
        $writer.WriteLine($line)
        $count++
    }
}
finally {
    if ($sp.IsOpen) { $sp.Close() }
    $sp.Dispose()
    $writer.Dispose()

    # 如果板子在我们打开串口之前就打印过表头（从半路接上的），文件里就没有
    # 表头——pandas 会把第一行数据当成列名。补一行。
    $lines = Get-Content $full -ErrorAction SilentlyContinue
    $hasHeader = $lines | Where-Object { $_ -match $HeaderPattern } | Select-Object -First 1
    if ($count -gt 0 -and $lines -and -not $hasHeader) {
        Write-Host ""
        Write-Host "文件里没有表头（从半路接上的），已按 -ExpectedHeader 补上：" -ForegroundColor Yellow
        Write-Host "  $ExpectedHeader" -ForegroundColor Yellow
        Write-Host "  如果这串不是这一轮固件真正打印的表头，补进去的列名就是错的——" -ForegroundColor Yellow
        Write-Host "  保险做法：先复位板子，再起抓取，最后按按钮。" -ForegroundColor Yellow
        @($ExpectedHeader) + $lines | Set-Content -Path $full -Encoding utf8
    }

    Write-Host ""
    if ($dropped -gt 0) {
        Write-Host "已丢弃 $dropped 行非 CSV 输出（板子启动横幅之类）。" -ForegroundColor DarkGray
    }
    Write-Host "共 $count 行，已保存到：$full" -ForegroundColor Green
}
