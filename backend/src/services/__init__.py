"""
Services initialization for the Physical AI Book backend.
"""

from .auth import (
    verify_password,
    get_password_hash,
    authenticate_user,
    create_access_token,
    get_current_user
)
from .course import (
    get_course_by_id,
    get_all_courses,
    get_courses_by_module
)
from .progress import (
    get_user_progress,
    get_course_progress,
    create_progress_entry,
    update_progress
)
from .chat import (
    get_chat_sessions,
    create_chat_session,
    get_chat_session,
    delete_chat_session,
    get_messages_from_session,
    add_message_to_session,
    get_chat_response
)

__all__ = [
    # Auth services
    "verify_password",
    "get_password_hash",
    "authenticate_user",
    "create_access_token",
    "get_current_user",

    # Course services
    "get_course_by_id",
    "get_all_courses",
    "get_courses_by_module",

    # Progress services
    "get_user_progress",
    "get_course_progress",
    "create_progress_entry",
    "update_progress",

    # Chat services
    "get_chat_sessions",
    "create_chat_session",
    "get_chat_session",
    "delete_chat_session",
    "get_messages_from_session",
    "add_message_to_session",
    "get_chat_response"
]