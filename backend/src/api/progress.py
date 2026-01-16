"""
Progress API endpoints for the Physical AI Book backend.
"""

from fastapi import APIRouter, Depends, HTTPException
from typing import List
from ..models.progress import Progress
from ..models.user import User
from ..services.progress import (
    get_user_progress,
    get_course_progress,
    update_progress,
    create_progress_entry
)
from .auth import get_current_user
from pydantic import BaseModel

router = APIRouter()

class ProgressCreate(BaseModel):
    course_id: int
    completion_percentage: float
    time_spent_minutes: int
    notes: str = None

class ProgressUpdate(BaseModel):
    completion_percentage: float = None
    time_spent_minutes: int = None
    notes: str = None

class ProgressResponse(BaseModel):
    id: int
    user_id: int
    course_id: int
    completion_percentage: float
    time_spent_minutes: int
    notes: str = None

    class Config:
        from_attributes = True

@router.get("/", response_model=List[ProgressResponse])
async def read_user_progress(current_user: User = Depends(get_current_user)):
    """Get all progress records for the current user."""
    progress_records = await get_user_progress(current_user.id)
    return progress_records

@router.get("/{course_id}", response_model=ProgressResponse)
async def read_course_progress(course_id: int, current_user: User = Depends(get_current_user)):
    """Get progress for a specific course for the current user."""
    progress_record = await get_course_progress(current_user.id, course_id)
    if not progress_record:
        raise HTTPException(status_code=404, detail="Progress record not found")
    return progress_record

@router.post("/", response_model=ProgressResponse)
async def create_progress(progress_create: ProgressCreate, current_user: User = Depends(get_current_user)):
    """Create a new progress record."""
    progress_record = await create_progress_entry(
        user_id=current_user.id,
        course_id=progress_create.course_id,
        completion_percentage=progress_create.completion_percentage,
        time_spent_minutes=progress_create.time_spent_minutes,
        notes=progress_create.notes
    )
    return progress_record

@router.put("/{course_id}", response_model=ProgressResponse)
async def update_course_progress(
    course_id: int,
    progress_update: ProgressUpdate,
    current_user: User = Depends(get_current_user)
):
    """Update progress for a specific course."""
    progress_record = await update_progress(
        user_id=current_user.id,
        course_id=course_id,
        completion_percentage=progress_update.completion_percentage,
        time_spent_minutes=progress_update.time_spent_minutes,
        notes=progress_update.notes
    )
    if not progress_record:
        raise HTTPException(status_code=404, detail="Progress record not found")
    return progress_record