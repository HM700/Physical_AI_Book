"""
Courses API endpoints for the Physical AI Book backend.
"""

from fastapi import APIRouter, Depends, HTTPException
from typing import List
from ..models.course import Course
from ..models.user import User
from ..services.course import get_course_by_id, get_all_courses, get_courses_by_module
from .auth import get_current_user
from pydantic import BaseModel

router = APIRouter()

class CourseResponse(BaseModel):
    id: int
    title: str
    description: str
    module_number: int
    duration_hours: int

    class Config:
        from_attributes = True

@router.get("/", response_model=List[CourseResponse])
async def read_courses(skip: int = 0, limit: int = 100):
    """Get all courses."""
    courses = await get_all_courses(skip=skip, limit=limit)
    return courses

@router.get("/{course_id}", response_model=CourseResponse)
async def read_course(course_id: int):
    """Get a specific course by ID."""
    course = await get_course_by_id(course_id)
    if not course:
        raise HTTPException(status_code=404, detail="Course not found")
    return course

@router.get("/module/{module_number}", response_model=List[CourseResponse])
async def read_courses_by_module(module_number: int):
    """Get courses for a specific module."""
    if module_number < 1 or module_number > 4:
        raise HTTPException(status_code=400, detail="Module number must be between 1 and 4")

    courses = await get_courses_by_module(module_number)
    return courses