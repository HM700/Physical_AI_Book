# Multi-stage Dockerfile for Physical AI Book
FROM python:3.11-slim AS backend-base
WORKDIR /app/backend

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    git \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY backend/pyproject.toml backend/README.md ./
RUN pip install --upgrade pip && \
    pip install build && \
    python -m build && \
    pip install dist/*.whl

# Backend runtime stage
FROM python:3.11-slim AS backend-runtime
WORKDIR /app/backend

# Install runtime system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    && rm -rf /var/lib/apt/lists/*

# Copy installed Python packages from base stage
COPY --from=backend-base /usr/local/lib/python3.*/site-packages /usr/local/lib/python3.*/site-packages
COPY --from=backend-base /usr/local/bin /usr/local/bin

# Copy application code
COPY backend/src ./src
COPY backend/README.md ./

EXPOSE 8000
CMD ["uvicorn", "src.api.main:app", "--host", "0.0.0.0", "--port", "8000"]

# Frontend build stage
FROM node:18 AS frontend-builder
WORKDIR /app/frontend

# Copy package files
COPY frontend/package*.json ./

# Install dependencies
RUN npm ci

# Copy Docusaurus config and source files
COPY frontend/docusaurus.config.js ./
COPY frontend/src ./src/
COPY frontend/static ./static/
COPY frontend/docs ./docs/
COPY frontend/blog ./blog/

# Build the site
RUN npm run build

# Frontend runtime stage
FROM nginx:alpine AS frontend-runtime

# Copy built site from builder stage
COPY --from=frontend-builder /app/frontend/build /usr/share/nginx/html

# Copy custom nginx configuration
COPY frontend/nginx.conf /etc/nginx/nginx.conf

EXPOSE 3000
CMD ["nginx", "-g", "daemon off;"]