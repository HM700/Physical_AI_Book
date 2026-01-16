"""
Chat API endpoints for the Physical AI Book backend.
"""

from fastapi import APIRouter, Depends, HTTPException
from typing import List
from ..models.chat import ChatSession, ChatMessage
from ..models.user import User
from ..services.chat import (
    create_chat_session,
    get_chat_session,
    get_chat_sessions,
    add_message_to_session,
    get_messages_from_session,
    delete_chat_session,
    get_chat_response
)
from .auth import get_current_user
from pydantic import BaseModel

router = APIRouter()

class ChatSessionCreate(BaseModel):
    title: str

class ChatSessionResponse(BaseModel):
    id: int
    user_id: int
    title: str
    created_at: str

    class Config:
        from_attributes = True

class ChatMessageCreate(BaseModel):
    role: str  # 'user' or 'assistant'
    content: str

class ChatMessageResponse(BaseModel):
    id: int
    session_id: int
    role: str
    content: str
    timestamp: str

    class Config:
        from_attributes = True

class ChatRequest(BaseModel):
    message: str
    session_id: int = None  # If not provided, creates a new session

class ChatResponse(BaseModel):
    session_id: int
    message: str
    timestamp: str

@router.get("/sessions", response_model=List[ChatSessionResponse])
async def read_chat_sessions(current_user: User = Depends(get_current_user)):
    """Get all chat sessions for the current user."""
    sessions = await get_chat_sessions(current_user.id)
    return sessions

@router.post("/sessions", response_model=ChatSessionResponse)
async def create_new_session(session_create: ChatSessionCreate, current_user: User = Depends(get_current_user)):
    """Create a new chat session."""
    session = await create_chat_session(user_id=current_user.id, title=session_create.title)
    return session

@router.get("/sessions/{session_id}", response_model=ChatSessionResponse)
async def read_chat_session(session_id: int, current_user: User = Depends(get_current_user)):
    """Get a specific chat session."""
    session = await get_chat_session(session_id, current_user.id)
    if not session:
        raise HTTPException(status_code=404, detail="Chat session not found")
    return session

@router.delete("/sessions/{session_id}")
async def delete_chat_session_endpoint(session_id: int, current_user: User = Depends(get_current_user)):
    """Delete a chat session."""
    success = await delete_chat_session(session_id, current_user.id)
    if not success:
        raise HTTPException(status_code=404, detail="Chat session not found")
    return {"message": "Chat session deleted successfully"}

@router.get("/sessions/{session_id}/messages", response_model=List[ChatMessageResponse])
async def read_session_messages(session_id: int, current_user: User = Depends(get_current_user)):
    """Get all messages from a specific chat session."""
    # Verify that the user owns this session
    session = await get_chat_session(session_id, current_user.id)
    if not session:
        raise HTTPException(status_code=404, detail="Chat session not found")

    messages = await get_messages_from_session(session_id)
    return messages

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(chat_request: ChatRequest, current_user: User = Depends(get_current_user)):
    """Send a message to the chatbot and get a response."""
    # If no session ID provided, create a new session
    if not chat_request.session_id:
        session = await create_chat_session(
            user_id=current_user.id,
            title=chat_request.message[:50] + "..." if len(chat_request.message) > 50 else chat_request.message
        )
        session_id = session.id
    else:
        session_id = chat_request.session_id
        # Verify that the user owns this session
        session = await get_chat_session(session_id, current_user.id)
        if not session:
            raise HTTPException(status_code=404, detail="Chat session not found")

    # Add user message to session
    user_message = await add_message_to_session(
        session_id=session_id,
        role="user",
        content=chat_request.message
    )

    # Get response from RAG service
    bot_response = await get_chat_response(chat_request.message)

    # Add bot response to session
    bot_message = await add_message_to_session(
        session_id=session_id,
        role="assistant",
        content=bot_response
    )

    return ChatResponse(
        session_id=session_id,
        message=bot_response,
        timestamp=str(bot_message.timestamp)
    )