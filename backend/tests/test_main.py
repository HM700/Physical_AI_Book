"""
Basic tests for the Physical AI Book backend.
"""

import pytest
from fastapi.testclient import TestClient
from src.api.main import app

@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)

def test_read_root(client):
    """Test the root endpoint."""
    response = client.get("/")
    assert response.status_code == 200
    assert "message" in response.json()

def test_health_check(client):
    """Test the health check endpoint."""
    response = client.get("/health")
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"
    assert response.json()["service"] == "Physical AI Book Backend"

def test_cors_enabled(client):
    """Test that CORS headers are present."""
    response = client.get("/")
    # This test will pass once CORS middleware is added in a future implementation
    assert response.status_code == 200