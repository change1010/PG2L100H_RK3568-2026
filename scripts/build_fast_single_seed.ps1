param(
    [Parameter(Mandatory=$true)][string]$PdsShell,
    [Parameter(Mandatory=$true)][string]$OutputDirectory,
    [ValidateRange(1,4)][int]$RoutingThreads = 4,
    [ValidateSet('fast','normal','performance')][string]$RouteMode = 'fast',
    [switch]$OptimizeRecovery,
    [switch]$DisableControlChains,
    [switch]$KeepIpBuildArtifacts
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$output = [IO.Path]::GetFullPath($OutputDirectory)
$shellPath = [IO.Path]::GetFullPath($PdsShell)
if (-not (Test-Path -LiteralPath $shellPath -PathType Leaf)) {
    throw "PDS shell is missing: $shellPath"
}
if (Test-Path -LiteralPath $output) {
    throw "Use a new output directory: $output"
}
$canonical = [IO.File]::ReadAllText((Join-Path $projectRoot 'impl.tcl'))
$pnrStart = $canonical.IndexOf('set pnr_rc [catch {pnr')
if ($pnrStart -lt 0) {
    throw 'Missing canonical flow boundary'
}

New-Item -ItemType Directory -Path $output | Out-Null
$frozen = Join-Path $output 'inputs'
New-Item -ItemType Directory -Path $frozen | Out-Null
$statePath = Join-Path $output 'status.json'
$state = [ordered]@{
    started_at = (Get-Date).ToString('o')
    finished_at = $null
    status = 'preparing'
    seed = 1
    routing_threads = $RoutingThreads
    route_mode = $RouteMode
    optimize_recovery = [bool]$OptimizeRecovery
    disable_control_chains = [bool]$DisableControlChains
    tool_sha256 = (Get-FileHash -LiteralPath $shellPath -Algorithm SHA256).Hash
    process_started_at = $null
    process_finished_at = $null
    process_elapsed_seconds = $null
    bitstream_sha256 = $null
    bitstream_written_at = $null
    flow_sha256 = $null
    wrapper_sha256 = (Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash
    clean_top_level_build = $true
    clean_ip_databases = -not [bool]$KeepIpBuildArtifacts
    incremental_build = $false
    ip_policy = 'Freeze generated IP source; exclude prior IP build databases by default'
    os = [Environment]::OSVersion.VersionString
    logical_processors = [Environment]::ProcessorCount
    machine = [Environment]::MachineName
    cache_policy = 'New output and frozen IP inputs; OS filesystem cache is not flushed'
    load_policy = 'Run one PDS build at a time; background OS load is not controlled'
    output_directory = $output
    wrapper_pid = $PID
    exit_code = $null
    bitstream = $null
    error = $null
}
$state | ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding utf8
try {
    foreach ($name in @('rtl', 'constraints')) {
        Copy-Item -LiteralPath (Join-Path $projectRoot $name) -Destination $frozen -Recurse
    }
    $ipSource = Join-Path $projectRoot 'ip'
    $ipDestination = Join-Path $frozen 'ip'
    $excludedInputs = @()
    foreach ($file in Get-ChildItem -LiteralPath $ipSource -Recurse -File) {
        $relative = $file.FullName.Substring($ipSource.Length + 1)
        if (-not $KeepIpBuildArtifacts -and $relative -match
            '(^|[\\/])(rev_\d+|compile|synthesize|device_map|place_route|generate_bitstream|logs|bak)([\\/]|$)') {
            $excludedInputs += "ip/$($relative.Replace('\', '/'))"
            continue
        }
        $destination = Join-Path $ipDestination $relative
        New-Item -ItemType Directory -Path (Split-Path -Parent $destination) -Force | Out-Null
        Copy-Item -LiteralPath $file.FullName -Destination $destination
    }
    ConvertTo-Json -InputObject @($excludedInputs) |
        Set-Content -LiteralPath (Join-Path $output 'excluded_ip_build_artifacts.json') -Encoding utf8
    foreach ($name in @('video_dma_hdmi.pds', 'impl.tcl')) {
        Copy-Item -LiteralPath (Join-Path $projectRoot $name) -Destination $frozen
    }
    Get-ChildItem -LiteralPath $frozen -Recurse -File | ForEach-Object {
        [PSCustomObject]@{
            path = $_.FullName.Substring($frozen.Length + 1).Replace('\', '/')
            sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
        }
    } | ConvertTo-Json -Depth 3 |
        Set-Content -LiteralPath (Join-Path $output 'inputs.json') -Encoding utf8

    $flow = $canonical.Substring(0, $pnrStart).Replace(
        'set project_root [file normalize [file dirname [info script]]]',
        'set project_root [file normalize [file join [file dirname [info script]] "inputs"]]')
    $recoveryOptions = ''
    if ($OptimizeRecovery) {
        $recoveryOptions = '-optimize_gp_recovery -optimize_dp_recovery -optimize_rp_recovery -optimize_routing_recovery TRUE'
    }
    $chainOptions = ''
    if ($DisableControlChains) {
        $chainOptions = '-share_router_control_signal FALSE'
    }
    $flow += (@'

pnr -optimize_multi_corner_timing -fix_hold_violation \
    -gplace_seed 1 -gplace_times 1 -seed_step 1 -saved_outcome 1 -parallel 1 \
    -number_of_router_thread @ROUTER_THREADS@ -route_mode @ROUTE_MODE@ \
    @RECOVERY_OPTIONS@ @CHAIN_OPTIONS@
report_timing -report_exception TRUE -max_path 500
gen_bit_stream
puts "FAST_SINGLE_SEED_BUILD_COMPLETE"
'@).Replace('@ROUTER_THREADS@', $RoutingThreads.ToString()).
        Replace('@ROUTE_MODE@', $RouteMode).
        Replace('@RECOVERY_OPTIONS@', $recoveryOptions).
        Replace('@CHAIN_OPTIONS@', $chainOptions)
    $flowPath = Join-Path $output 'run_fast_single_seed.tcl'
    [IO.File]::WriteAllText($flowPath, $flow + "`n", [Text.UTF8Encoding]::new($false))
    $state.flow_sha256 = (Get-FileHash -LiteralPath $flowPath -Algorithm SHA256).Hash
    $state.status = 'running'
    $state.process_started_at = (Get-Date).ToString('o')
    $state | ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding utf8
    $logPath = Join-Path $output 'build.log'
    $timer = [Diagnostics.Stopwatch]::StartNew()
    & $shellPath -file $flowPath -work_dir $output -project_name video_dma_hdmi_fast *> $logPath
    $timer.Stop()
    $state.process_elapsed_seconds = $timer.Elapsed.TotalSeconds
    $state.process_finished_at = (Get-Date).ToString('o')
    $state.exit_code = $LASTEXITCODE
    $log = [IO.File]::ReadAllText($logPath)
    if ($state.exit_code -ne 0 -or
        -not $log.Contains('FAST_SINGLE_SEED_BUILD_COMPLETE') -or
        [regex]::IsMatch($log, '(?m)^E:')) {
        throw "PDS build failed; inspect $logPath (exit $($state.exit_code))"
    }
    $bitstream = Join-Path $output 'generate_bitstream/video_dma_hdmi_top.sbit'
    if (-not (Test-Path -LiteralPath $bitstream -PathType Leaf) -or
        (Get-Item -LiteralPath $bitstream).Length -eq 0) {
        throw 'PDS completed without a nonempty expected bitstream'
    }
    $state.bitstream = $bitstream
    $state.bitstream_sha256 = (Get-FileHash -LiteralPath $bitstream -Algorithm SHA256).Hash
    $state.bitstream_written_at = (Get-Item -LiteralPath $bitstream).LastWriteTime.ToString('o')
    $state.status = 'completed'
}
catch {
    $state.status = 'failed'
    $state.error = $_.Exception.Message
    throw
}
finally {
    $state.finished_at = (Get-Date).ToString('o')
    $state | ConvertTo-Json | Set-Content -LiteralPath $statePath -Encoding utf8
}
