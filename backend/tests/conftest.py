"""
Configuration for pytest in the Physical AI Book backend.
"""

import pytest
from unittest.mock import patch
from fastapi.testclient import TestClient
from src.api.main import app
from src.database import engine
from sqlalchemy import event
from sqlalchemy.engine import Engine

# Enable foreign key constraints for SQLite in tests
@event.listens_for(Engine, "connect")
def set_sqlite_pragma(dbapi_connection, connection_record):
    if dbapi_connection.__class__.__module__ == 'sqlite3':
        cursor = dbapi_connection.cursor()
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.close()

@pytest.fixture(scope="session")
def test_app():
    """Create a test app with overridden dependencies."""
    # In a real implementation, you might want to override certain dependencies
    # for testing purposes, such as using an in-memory database
    yield app

@pytest.fixture
def client(test_app):
    """Create a test client for the API."""
    with TestClient(test_app) as client:
        yield client