"""
Tests for the authentication functionality of the Physical AI Book backend.
"""

import pytest
from fastapi.testclient import TestClient
from src.api.main import app

@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)

def test_auth_routes_exist(client):
    """Test that authentication routes are registered."""
    # Test that the token endpoint exists (though it will fail without proper credentials)
    response = client.post("/auth/token", data={"username": "test", "password": "test"})
    # Should return 401 for invalid credentials rather than 404 for not found
    assert response.status_code in [401, 422]  # 422 if validation fails

def test_register_route_exists(client):
    """Test that the registration endpoint exists."""
    response = client.post("/auth/register", json={
        "username": "testuser",
        "email": "test@example.com",
        "password": "testpass"
    })
    # This should eventually return 200 when implemented, but may return 501 for not implemented
    assert response.status_code in [501, 422]  # 501 if not implemented, 422 if validation issues

def test_profile_route_requires_auth(client):
    """Test that profile route requires authentication."""
    response = client.get("/auth/profile")
    # Should return 401 for unauthorized access
    assert response.status_code == 401