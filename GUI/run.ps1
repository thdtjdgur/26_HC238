$env:PYTHONUTF8 = "1"
$env:PYTHONIOENCODING = "utf-8"
& "$PSScriptRoot\.venv\Scripts\python.exe" "$PSScriptRoot\guitest.py"
exit $LASTEXITCODE
