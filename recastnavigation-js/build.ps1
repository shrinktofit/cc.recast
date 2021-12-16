<#
.SYNOPSIS
    Build recast-navigation.js.
.DESCRIPTION
    Build recast-navigation into WebAssembly/asm.js format.
.PARAMETER Wasm
    To proceduce WebAssembly code.
.Notes
    Before you run this script, you should have setup the Emscripten SDK environment.
    It can be typically done by:
    PS > <path-to-emsdk-repos>/emsdk_env.ps1
#>
[CmdletBinding()]
param (
    [Parameter(HelpMessage="To proceduce WebAssembly code.")]
    [Switch] $Wasm
)

function asserts([bool]$Value, [string]$Message = "") {
    if ($true -ne $Value) {
        throw "Assertion failed: $($Message)";
    }
}

$emconfigPath = $env:EM_CONFIG
if ($null -eq $emconfigPath) {
    Write-Error "Enviroment variable 'EM_CONFIG' is not set. Please check if EMSDK is correctly setup."
    exit -1
}

$emconfigText = (Get-Content $emconfigPath).Split('\n')
$emconfig = @{}
foreach ($emconfigLine in $emconfigText) {
    $iEqual = $emconfigLine.IndexOf('=')
    if ($iEqual -lt 0) {
        continue
    }
    $key = $emconfigLine.Substring(0, $iEqual).Trim()
    $value = $emconfigLine.Substring($iEqual + 1).Trim()
    if ($value -match "^\'.*\'$") {
        $value = $value.Substring(1, $value.Length - 2)
    }
    $emconfig.Add($key, $value)
    Write-Host "$($key): $($value)"
}

$emscriptenRoot = $emconfig.EMSCRIPTEN_ROOT
asserts($null -ne $emscriptenRoot)

Push-Location
try {
    $buildDirName = "build"
    if ((Test-Path $buildDirName) -eq $false) {
        New-Item -ItemType Directory -Path $buildDirName
    }

    Set-Location $buildDirName

    $cMakeArgs = @()
    if ($Wasm) {
        $cMakeArgs += @("-DBUILD_WASM=ON")
    }

    & emcmake cmake .. $cMakeArgs

    ## & emmake nmake

    & cmake --build .
} finally {
    Pop-Location
}

$content = (Get-Content -Path "./build/recastnavigation-js.mjs").Replace(
    "throwBindingError('Cannot convert argument of type ' + handle.`$`$.ptrType.name + ' to parameter type ' + this.name)",
    "// throwBindingError('Cannot convert argument of type ' + handle.`$`$.ptrType.name + ' to parameter type ' + this.name)"
).Replace(
    "throwBindingError('Cannot convert argument of type ' + (handle.`$`$.smartPtrType ? handle.`$`$.smartPtrType.name : handle.`$`$.ptrType.name) + ' to parameter type ' + this.name)",
    "// throwBindingError('Cannot convert argument of type ' + (handle.`$`$.smartPtrType ? handle.`$`$.smartPtrType.name : handle.`$`$.ptrType.name) + ' to parameter type ' + this.name)"
)
# .Replace(
#     "throw new BindingError(`"Use 'new' to construct `" + name);",
#     "// throw new BindingError(`"Use 'new' to construct `" + name);"
# )

# Write-Host $content

$content | Out-File -Force "X:\Dev\Repos\Leslie\XLStory\Client\Lib\recastnavigation-js\recastnavigation-js.mjs"
Copy-Item ./build/recastnavigation-js.wasm "X:\Dev\Repos\Leslie\XLStory\Client\assets\Source\recastnavigation-js.wasm.bin" -Force
(gci "X:\Dev\Repos\Leslie\XLStory\Client\assets\Source\Navigation\NavMeshAgent.ts").LastWriteTime = Get-Date
Invoke-WebRequest http://localhost:7456/asset-db/refresh