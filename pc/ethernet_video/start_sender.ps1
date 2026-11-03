param(
    [string]$Bind = '192.168.1.10',
    [string]$Python = 'python',
    [ValidateSet('desktop', 'synthetic', 'pattern')]
    [string]$Source = 'desktop',
    [ValidateRange(0.1, 30)]
    [double]$Fps = 30,
    [ValidateRange(0, 2147483647)]
    [int]$Frames = 0,
    [switch]$ConfigureNeighbor,
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
$target = '192.168.1.100'
$targetMac = '02-00-00-00-00-01'
$addresses = @(Get-NetIPAddress -AddressFamily IPv4 -IPAddress $Bind -ErrorAction SilentlyContinue |
    Where-Object { $_.AddressState -eq 'Preferred' })
if ($addresses.Count -ne 1) {
    throw "Bind address $Bind must identify exactly one active local IPv4 interface."
}
$interfaceIndex = $addresses[0].InterfaceIndex
$adapter = Get-NetAdapter -InterfaceIndex $interfaceIndex
if ($adapter.Status -ne 'Up' -or $addresses[0].PrefixLength -ne 24 -or
    $Bind -notmatch '^192\.168\.1\.' -or $Bind -eq $target) {
    throw 'Use an active FPGA Ethernet interface with an existing 192.168.1.x/24 address; this script does not change adapter IPs.'
}
Write-Output "Interface: $($adapter.Name), IPv4: $Bind/24, link: $($adapter.LinkSpeed)"
if ($adapter.ReceiveLinkSpeed -lt 1000000000 -or $adapter.TransmitLinkSpeed -lt 1000000000) {
    throw 'The current RGB444 stream needs a gigabit Ethernet link.'
}

$neighbor = Get-NetNeighbor -InterfaceIndex $interfaceIndex -IPAddress $target -ErrorAction SilentlyContinue
if ($neighbor -and $neighbor.LinkLayerAddress -notin @($targetMac, '00-00-00-00-00-00', '')) {
    throw "FPGA neighbor has a different MAC ($($neighbor.LinkLayerAddress)); verify the actual hardware before changing it."
}
if ($ConfigureNeighbor -and (!$neighbor -or $neighbor.State -ne 'Permanent')) {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = [Security.Principal.WindowsPrincipal]::new($identity)
    if (!$principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
        throw 'ConfigureNeighbor needs an administrator PowerShell. It sets only the FPGA neighbor in ActiveStore, without changing adapter IPs or persistent settings.'
    }
    & netsh.exe interface ipv4 set neighbors "interface=$interfaceIndex" "address=$target" "neighbor=$targetMac" store=active
    if ($LASTEXITCODE -ne 0) { throw "Setting the FPGA neighbor failed (exit $LASTEXITCODE)." }
    $neighbor = Get-NetNeighbor -InterfaceIndex $interfaceIndex -IPAddress $target -ErrorAction Stop
}
if (!$neighbor -or $neighbor.LinkLayerAddress -ne $targetMac -or $neighbor.State -ne 'Permanent') {
    throw "The supported stop/resume setup needs an active fixed FPGA neighbor. Run this script once in administrator PowerShell with -ConfigureNeighbor -CheckOnly -Bind $Bind; then start normally. This is the documented ARP workaround, not an FPGA ARP fix."
}
Write-Output "FPGA neighbor: $target -> $($neighbor.LinkLayerAddress), $($neighbor.State) (ARP workaround)"
if ($CheckOnly) { return }

$senderPath = Join-Path $PSScriptRoot 'sender.py'
$pythonCommand = @(Get-Command $Python -CommandType Application -ErrorAction Stop)[0]
Write-Output "Python: $($pythonCommand.Source)"
Write-Output "Starting $Source -> ${target}:5000, target $Fps fps; Ctrl+C stops sending."
& $pythonCommand.Source $senderPath --target $target --port 5000 --bind $Bind --source $Source --fps $Fps --frames $Frames
if ($LASTEXITCODE -ne 0) { throw "Sender exited with code $LASTEXITCODE." }
