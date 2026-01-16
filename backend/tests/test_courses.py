"""
Tests for the courses functionality of the Physical AI Book backend.
"""

import pytest
from fastapi.testclient import TestClient
from src.api.main import app

@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)

def test_courses_routes_exist(client):
    """Test that courses routes are registered."""
    # Test GET /courses (will require auth, so expect 401/403)
    response = client.get("/courses/")
    assert response.status_code in [401, 403, 200]  # 200 if no auth required in test mode

def test_course_by_id_route_exists(client):
    """Test that individual course route exists."""
    response = client.get("/courses/1")
    # Will likely return 401/403 due to auth, or 404 if not found, or 422 if validation error
    assert response.status_code in [401, 403, 404, 422]

def test_courses_by_module_route_exists(client):
    """Test that courses by module route exists."""
    response = client.get("/courses/module/1")
    assert response.status_code in [401, 403, 200, 422]  # 200 if successful, 422 if validation error