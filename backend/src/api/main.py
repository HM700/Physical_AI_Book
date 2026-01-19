"""
Main FastAPI application for the Physical AI Book backend.
"""

from fastapi import FastAPI
from src.services.ai.rag_service import rag_service
from . import auth, courses, progress, chat

app = FastAPI(
    title="Physical AI Book API",
    description="Backend API for the Physical AI Book educational platform",
    version="0.1.0",
    contact={
        "name": "Physical AI Book Team",
        "email": "team@physical-ai-book.org",
    },
)

# Include API routes
app.include_router(auth.router, prefix="/auth", tags=["authentication"])
app.include_router(courses.router, prefix="/courses", tags=["courses"])
app.include_router(progress.router, prefix="/progress", tags=["progress"])
app.include_router(chat.router, prefix="/chat", tags=["chat"])

import asyncio

@app.on_event("startup")
async def startup_event():
    # Initialize RAG service in the background to not block startup
    asyncio.create_task(initialize_rag_service())

async def initialize_rag_service():
    try:
        await rag_service.initialize()
        print("RAG service initialized successfully")
    except Exception as e:
        print(f"Warning: Could not initialize RAG service: {e}")
        print("Some features may be unavailable until dependencies are configured")

@app.get("/")
def read_root():
    return {"message": "Welcome to the Physical AI Book API"}

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "Physical AI Book Backend"}
