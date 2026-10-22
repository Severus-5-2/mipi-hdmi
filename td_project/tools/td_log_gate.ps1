param(
    [Parameter(Mandatory = $true, Position = 0)]
    [string] $RunLog,

    [Parameter(Mandatory = $false)]
    [string] $TimingReport,

    [Parameter(Mandatory = $false)]
    [int] $BaselineEndpoints = 18180,

    [Parameter(Mandatory = $false)]
    [int] $MaxEndpointDrop = 20
)

$ErrorActionPreference = 'Stop'
$fatalCodes = @('PHY-8077', 'PHY-9005', 'PHY-8010', 'PHY-1036', 'OPT-8004', 'PRJ-1420')
$warningCodes = @('PHY-5010', 'PHY-1039', 'PHY-5088', 'RUN-6002', 'RUN-6004')

if (-not (Test-Path -LiteralPath $RunLog -PathType Leaf)) {
    Write-Error "TD run log not found: $RunLog"
    exit 2
}
$logLines = Get-Content -LiteralPath $RunLog
$blockers = [System.Collections.Generic.List[string]]::new()
$suppressed = [System.Collections.Generic.HashSet[string]]::new()

$errorRecords = @($logLines | Where-Object { $_ -match '\bERROR\s*:' })
if ($errorRecords.Count -gt 0) {
    $blockers.Add("TD error records=$($errorRecords.Count)")
}

foreach ($code in $fatalCodes) {
    $count = @($logLines | Where-Object { $_ -match [regex]::Escape($code) -and $_ -notmatch 'Similar messages will be suppressed' }).Count
    if ($count -gt 0) {
        $blockers.Add("$code visible records=$count")
    }
}

for ($i = 1; $i -lt $logLines.Count; $i++) {
    if ($logLines[$i] -match 'Similar messages will be suppressed' -and
        $logLines[$i - 1] -match '\b((PHY|OPT|RUN|PRJ)-\d{4})\b') {
        [void] $suppressed.Add($Matches[1])
    }
}

$endpointCount = $null
if ($TimingReport) {
    if (-not (Test-Path -LiteralPath $TimingReport -PathType Leaf)) {
        Write-Error "Timing report not found: $TimingReport"
        exit 2
    }
    $timingText = Get-Content -LiteralPath $TimingReport -Raw
    # Check global final timing, not intermediate place/route optimization values.
    foreach ($kind in @('S', 'H')) {
        $status = [regex]::Match($timingText, "\b${kind}WNS:\s*([+-]?\d+(?:\.\d+)?)ns,\s*${kind}TNS:\s*([+-]?\d+(?:\.\d+)?)ns")
        if (-not $status.Success) {
            $blockers.Add("${kind}WNS/${kind}TNS missing from final timing report")
        } else {
            $wns = [double]::Parse($status.Groups[1].Value, [cultureinfo]::InvariantCulture)
            $tns = [double]::Parse($status.Groups[2].Value, [cultureinfo]::InvariantCulture)
            if ($wns -lt 0 -or $tns -lt 0) {
                $blockers.Add("${kind}WNS=$wns ns ${kind}TNS=$tns ns")
            }
        }
    }
    $violations = [regex]::Matches($timingText, '\b(\d+)\s+Viol Endpoints\b')
    if ($violations.Count -eq 0) {
        $blockers.Add('Viol Endpoints missing from final timing report')
    } elseif (@($violations | Where-Object { [int]$_.Groups[1].Value -gt 0 }).Count -gt 0) {
        $blockers.Add('Nonzero violating endpoints in final timing report')
    }
    $endpointMatch = [regex]::Match($timingText, '\b(\d+)\s+Total Endpoints\b')
    if ($endpointMatch.Success) {
        $endpointCount = [int] $endpointMatch.Groups[1].Value
        if (($BaselineEndpoints - $endpointCount) -gt $MaxEndpointDrop) {
            $blockers.Add("Total Endpoints=$endpointCount baseline=$BaselineEndpoints allowed_drop=$MaxEndpointDrop")
        }
    } else {
        $blockers.Add('Total Endpoints missing from timing report')
    }
}

if ($blockers.Count -gt 0) {
    Write-Output 'TD release gate: BLOCK'
    Write-Output "Log: $((Resolve-Path -LiteralPath $RunLog).Path)"
    foreach ($item in $blockers) { Write-Output "  $item" }
} else {
    Write-Output 'TD release gate: PASS (requested log/timing checks passed)'
    Write-Output "Log: $((Resolve-Path -LiteralPath $RunLog).Path)"
}

foreach ($code in $suppressed) {
    Write-Output "  ${code}: similar messages were suppressed; actual count is unknown"
}
foreach ($code in $warningCodes) {
    $count = @($logLines | Where-Object { $_ -match [regex]::Escape($code) -and $_ -notmatch 'Similar messages will be suppressed' }).Count
    if ($count -gt 0) { Write-Output "  review $code visible records=$count" }
}
if ($null -ne $endpointCount) { Write-Output "Total Endpoints: $endpointCount" }

if ($blockers.Count -gt 0) { exit 1 }
exit 0
