"""
RAG (Retrieval Augmented Generation) service for the Physical AI Book backend.
"""

import os
import logging
from typing import List, Dict, Any, Optional
from hashlib import md5
import asyncio

from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from openai import AsyncOpenAI

logger = logging.getLogger(__name__)


class RAGService:
    def __init__(self):
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.qdrant_client = QdrantClient(url=qdrant_url)

        openai_api_key = os.getenv("OPENAI_API_KEY")
        if not openai_api_key:
            logger.warning("OPENAI_API_KEY environment variable not set")
            self.openai_client = None
        else:
            self.openai_client = AsyncOpenAI(api_key=openai_api_key)

        self.collection_name = os.getenv(
            "QDRANT_COLLECTION_NAME",
            "physical_ai_book_docs"
        )

    # ✅ THIS METHOD MUST EXIST
    async def initialize(self):
        await self._initialize_collection()

    async def _initialize_collection(self):
        # Run synchronous Qdrant calls in a thread to avoid blocking
        collections = await asyncio.to_thread(self.qdrant_client.get_collections)
        names = [c.name for c in collections.collections]

        if self.collection_name not in names:
            await asyncio.to_thread(
                self.qdrant_client.create_collection,
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=1536,
                    distance=Distance.COSINE
                ),
            )
            logger.info(f"Created Qdrant collection: {self.collection_name}")
        else:
            logger.info(f"Using existing Qdrant collection: {self.collection_name}")

    async def embed_text(self, text: str) -> List[float]:
        if self.openai_client is None:
            return [0.0] * 1536

        response = await self.openai_client.embeddings.create(
            input=text,
            model="text-embedding-ada-002",
        )
        return response.data[0].embedding

    async def store_document(
        self,
        content: str,
        metadata: Dict[str, Any],
        doc_id: Optional[str] = None,
    ) -> str:
        embedding = await self.embed_text(content)

        if not doc_id:
            doc_id = md5(content.encode()).hexdigest()

        point = PointStruct(
            id=doc_id,
            vector=embedding,
            payload={"content": content, "metadata": metadata},
        )

        # Run synchronous upsert in a thread
        await asyncio.to_thread(
            self.qdrant_client.upsert,
            collection_name=self.collection_name,
            points=[point],
        )

        return doc_id


# ✅ GLOBAL INSTANCE
rag_service = RAGService()

