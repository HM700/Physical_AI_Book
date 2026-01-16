"""
Course model for the Physical AI Book backend.
"""

from sqlalchemy import Column, Integer, String, Text, DateTime, Boolean
from sqlalchemy.sql import func
from .base import Base


class Course(Base):
    __tablename__ = "courses"

    id = Column(Integer, primary_key=True, index=True)
    title = Column(String, nullable=False)
    description = Column(Text, nullable=True)
    module_number = Column(Integer, nullable=False)  # 1-4 for the four modules
    duration_hours = Column(Integer, nullable=True)  # Estimated duration in hours
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())

    def __repr__(self):
        return f"<Course(id={self.id}, title='{self.title}', module={self.module_number})>"