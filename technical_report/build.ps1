$ErrorActionPreference = 'Stop'

$reportRoot = $PSScriptRoot
$repoRoot = Split-Path -Parent $reportRoot
$buildRoot = Join-Path $repoRoot 'tmp\pdfs\technical-report-build'
$outputRoot = Join-Path $repoRoot 'output\pdf'
$outputPdf = Join-Path $outputRoot 'jiangnan_bosch_flying_life_technical_report_framework.pdf'

New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null

Push-Location $reportRoot
try {
    latexmk -xelatex -interaction=nonstopmode -halt-on-error -file-line-error `
        -outdir="$buildRoot" main.tex
}
finally {
    Pop-Location
}

$builtPdf = Join-Path $buildRoot 'main.pdf'
if (-not (Test-Path -LiteralPath $builtPdf)) {
    throw "LaTeX build completed without producing $builtPdf"
}

Copy-Item -LiteralPath $builtPdf -Destination $outputPdf -Force
Write-Host "Created: $outputPdf"
