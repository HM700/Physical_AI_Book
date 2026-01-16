"""
API initialization for the Physical AI Book backend.
"""

from .main import app
from . import auth, courses, progress, chat

__all__ = ["app", "auth", "courses", "progress", "chat"]