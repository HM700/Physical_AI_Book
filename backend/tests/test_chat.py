"""
Tests for the chat functionality of the Physical AI Book backend.
"""

import pytest
from fastapi.testclient import TestClient
from src.api.main import app

@pytest.fixture
def client():
    """Create a test client for the API."""
    return TestClient(app)

def test_chat_endpoint_exists(client):
    """Test that the chat endpoint exists."""
    # This test will initially fail since we don't have authentication set up
    # but it verifies the route is registered
    response = client.get("/chat/sessions")
    # We expect a 401 or 403 since authentication is required
    assert response.status_code in [401, 403, 200]  # Allow 200 if auth not enforced in test mode

def test_chat_post_endpoint(client):
    """Test the chat message posting endpoint."""
    # This test will check if the endpoint accepts requests
    response = client.post("/chat/chat", json={"message": "Hello"})
    # Expect either 422 (validation error due to missing session_id) or 401/403 (auth error)
    assert response.status_code in [422, 401, 403]