#!/bin/bash

# Host system setup script for autonomy_stack_vega Docker containers
# This script configures the host system for optimal Docker container performance

echo "=========================================="
echo "Autonomy Stack Vega - Host Setup"
echo "=========================================="
echo ""

# Set kernel receive buffer size for CycloneDDS (required for large LiDAR messages)
echo "[1/3] Setting kernel receive buffer size..."
if sudo sysctl -w net.core.rmem_max=2147483647; then
    echo "✓ Kernel receive buffer size set successfully"
    echo "  To make this permanent, add 'net.core.rmem_max=2147483647' to /etc/sysctl.conf"
else
    echo "✗ Failed to set kernel receive buffer size"
    echo "  Try running: sudo sysctl -w net.core.rmem_max=2147483647"
fi
echo ""

# Enable X11 forwarding for GUI applications (RViz)
echo "[2/3] Enabling X11 forwarding for Docker..."
if xhost +local:docker >/dev/null 2>&1; then
    echo "✓ X11 forwarding enabled"
else
    echo "✗ Failed to enable X11 forwarding (xhost may not be available)"
    echo "  GUI applications (RViz) may not work"
fi
echo ""

# Check Docker and Docker Compose installation
echo "[3/3] Checking Docker installation..."
if command -v docker >/dev/null 2>&1; then
    echo "✓ Docker is installed: $(docker --version)"
else
    echo "✗ Docker is not installed"
    echo "  Please install Docker: https://docs.docker.com/engine/install/"
fi

if docker compose version >/dev/null 2>&1; then
    echo "✓ Docker Compose is installed: $(docker compose version)"
elif command -v docker-compose >/dev/null 2>&1; then
    echo "✓ Docker Compose (legacy) is installed: $(docker-compose --version)"
    echo "  Note: Consider upgrading to 'docker compose' (without hyphen)"
else
    echo "✗ Docker Compose is not installed"
    echo "  Please install Docker: https://docs.docker.com/engine/install/"
fi
echo ""

echo "=========================================="
echo "Setup complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  cd docker"
echo "  docker compose --profile amd64 up -d   # For AMD64 systems"
echo "  docker compose --profile arm64 up -d   # For ARM64 systems"
echo ""
