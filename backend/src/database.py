"""
Database module for the Physical AI Book backend.
"""

from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool
import os
from .models.base import Base

# Get database URL from environment variable, with a default for testing
DATABASE_URL = os.getenv("DATABASE_URL", "sqlite+aiosqlite:///./physical_ai_book.db")

# Create async engine
engine = create_async_engine(
    DATABASE_URL,
    # For SQLite, we need to connect with check_same_thread=False
    connect_args={"check_same_thread": False} if DATABASE_URL.startswith("sqlite") else {},
    poolclass=StaticPool if DATABASE_URL.startswith("sqlite") else None
)

# Create async session maker
AsyncSessionLocal = sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False
)

async def get_db():
    """
    Dependency function that yields a database session.
    """
    async with AsyncSessionLocal() as session:
        yield session

async def init_db():
    """
    Initialize the database by creating all tables.
    """
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)