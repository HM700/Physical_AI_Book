# Quickstart: Fix JavaScript Build Error

## Overview
Steps to resolve the "Unexpected token '<'" JavaScript error by regenerating the build.

## Prerequisites
- Node.js v20.x installed
- npm package manager
- Docusaurus project dependencies installed

## Setup

1. Navigate to the frontend directory:
   ```bash
   cd frontend
   ```

2. Install dependencies (if not already installed):
   ```bash
   npm install
   ```

## Build Process

1. Clean the existing build:
   ```bash
   rm -rf build
   ```

2. Generate a fresh build:
   ```bash
   npm run build
   ```

## Verification

1. Check that new JavaScript files are generated with fresh hashes:
   ```bash
   ls -la build/assets/js/
   ```

2. Verify HTML references the correct JavaScript files:
   ```bash
   grep -n "main\." build/index.html
   ```

3. Serve the build locally to test:
   ```bash
   npx serve build
   ```

## Expected Outcome
- No "Unexpected token '<'" errors in browser console
- All JavaScript files load with proper content
- Interactive components function correctly
- HTML properly references JavaScript bundles with correct paths

## Troubleshooting
- If errors persist, clear browser cache completely
- Ensure web server is configured to serve static assets correctly
- Verify that all required dependencies are installed