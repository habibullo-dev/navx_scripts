# Creates a venv, installs demo deps, starts the mock server, and opens the dashboard
$ErrorActionPreference = "Stop"

Write-Host "[1/4] Creating virtual environment (.venv) if missing..."
if (!(Test-Path ".venv")) {
  python -m venv .venv
}

Write-Host "[2/4] Activating venv and installing dependencies..."
.\.venv\Scripts\Activate.ps1
pip install --upgrade pip
pip install -r requirements-demo.txt

Write-Host "[3/4] Starting mock WebSocket server in a new window..."
$serverCmd = ". .\.venv\Scripts\Activate.ps1; python mock_server.py"
Start-Process powershell -ArgumentList "-NoExit", "-Command", $serverCmd | Out-Null

Write-Host "[4/4] Opening dashboard (camera.html)..."
Start-Process "$PSScriptRoot\camera.html"

Write-Host "Demo started. If the page shows DISCONNECTED, wait 1-2 seconds."