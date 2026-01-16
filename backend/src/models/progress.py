"""
Progress model for the Physical AI Book backend.
"""

from sqlalchemy import Column, Integer, ForeignKey, DateTime, Float, Text
from sqlalchemy.sql import func
from .base import Base


class Progress(Base):
    __tablename__ = "progress"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(Integer, ForeignKey("users.id"), nullable=False)
    course_id = Column(Integer, ForeignKey("courses.id"), nullable=False)
    completion_percentage = Column(Float, default=0.0)  # 0.0 to 100.0
    time_spent_minutes = Column(Integer, default=0)  # Time spent in minutes
    notes = Column(Text, nullable=True)
    last_accessed = Column(DateTime(timezone=True), server_default=func.now())
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<Progress(user_id={self.user_id}, course_id={self.course_id}, completion={self.completion_percentage}%)>"