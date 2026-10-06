#!/bin/bash

## Update and upgrade system packages
echo "Updating system packages..."
export GIT_TERMINAL_PROMPT=0   # fail fast if auth needed

# Install dependencies for some of the Python packages

echo "Installing ..."
pip install -e .

echo "Setup completed successfully!"