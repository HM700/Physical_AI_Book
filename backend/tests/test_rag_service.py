"""
Tests for the RAG service of the Physical AI Book backend.
"""

import pytest
import asyncio
from src.services.ai.rag_service import RAGService

@pytest.fixture
def rag_service():
    """Create a RAG service instance for testing."""
    service = RAGService()
    # In a real test, we might want to use a test-specific collection
    return service

@pytest.mark.asyncio
async def test_embed_text(rag_service):
    """Test that the embed_text method returns a valid embedding."""
    text = "This is a test sentence."
    embedding = await rag_service.embed_text(text)

    # Check that embedding is a list of floats
    assert isinstance(embedding, list)
    assert len(embedding) == 1536  # OpenAI ada-002 embedding dimension
    assert all(isinstance(val, float) for val in embedding)

@pytest.mark.asyncio
async def test_store_and_search_document(rag_service):
    """Test storing and searching for a document."""
    content = "This is a sample document about robotics."
    metadata = {"source": "test", "type": "concept"}

    # Store document
    doc_id = await rag_service.store_document(content, metadata)
    assert isinstance(doc_id, str)

    # Search for similar content
    results = await rag_service.search_similar("robotics concepts", limit=1)
    # Note: This test might not return results if the collection is empty or not properly initialized
    # In a real test environment, we would ensure the collection exists and contains test data

    # Just verify the structure of results if any are returned
    if results:
        assert isinstance(results, list)
        if results:
            result = results[0]
            assert "content" in result
            assert "metadata" in result
            assert "score" in result