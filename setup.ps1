# setup.ps1
echo "Configuring vcpkg to install packages locally in the project root."

# Force vcpkg to install packages into the project directory
$projectRoot = $PSScriptRoot
echo "Project Root: $projectRoot"
echo "vcpkg will install packages to: $projectRoot\vcpkg_installed"

# Clear any binary cache overrides to use default
[System.Environment]::SetEnvironmentVariable('VCPKG_BINARY_SOURCES', $null, [System.EnvironmentVariableTarget]::User)

echo "Configuration complete. Run 'cmake --preset=default' or your build command to start installation."
