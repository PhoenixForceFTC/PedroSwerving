param(
    [string]$RobotRepo = (Join-Path $PSScriptRoot '../../Phoenix-Force-10100-Differential-Swerve'),
    [string]$BuildJdk = $env:JAVA_HOME
)
$ErrorActionPreference = 'Stop'
$pedroRepo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$robotPath = (Resolve-Path -LiteralPath $RobotRepo).Path
$jdkArgs = @()
if ($BuildJdk) { $jdkArgs = @("-Dorg.gradle.java.home=$BuildJdk") }
function Invoke-ProjectGradle([string]$Project, [string[]]$Tasks) {
    Push-Location -LiteralPath $Project
    try {
        & ./gradlew.bat @jdkArgs @Tasks --console=plain
        if ($LASTEXITCODE -ne 0) { throw "Gradle verification failed in $Project" }
    } finally { Pop-Location }
}
Invoke-ProjectGradle $pedroRepo @(':core:test', ':core:jar', ':core:spotlessCheck', ':revhub:assembleRelease')
Invoke-ProjectGradle $robotPath @(':TeamCode:testDebugUnitTest', ':TeamCode:assembleDebug', "-PpedroLocalArtifacts=$pedroRepo")
# Restore the default published dependency and leave its APK and reports as the final outputs.
Invoke-ProjectGradle $robotPath @(':TeamCode:testDebugUnitTest', ':TeamCode:assembleDebug')
Write-Output 'Both repositories verified; local Pedro artifacts and published 3.0.1 both pass the robot build/tests.'
