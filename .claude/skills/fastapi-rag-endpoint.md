---
description: Generate a complete FastAPI RAG endpoint with Gemini embeddings, Qdrant search, and OpenAI ChatKit integration
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Overview

Generate a production-ready FastAPI endpoint that implements a complete RAG (Retrieval-Augmented Generation) pipeline:
1. Takes a user question as input
2. Generates embeddings using Google Gemini
3. Searches Qdrant vector database for similar chunks
4. Passes retrieved context to OpenAI ChatKit
5. Returns answer with source citations
6. Includes all Pydantic models, error handling, and validation

## Implementation Steps

### 1. Analyze User Requirements

Extract from user input (if provided):
- Endpoint path (default: `/api/chat/query`)
- Collection name (default: `documents`)
- Top-k results (default: 5)
- Similarity threshold (default: 0.7)
- Custom response fields
- Authentication requirements
- Rate limiting needs

### 2. Generate Complete FastAPI Endpoint Code

Create a fully functional endpoint with the following structure:

```python
from fastapi import APIRouter, HTTPException, status, Depends
from pydantic import BaseModel, Field, validator
from typing import List, Optional, Dict, Any
import os
from datetime import datetime
import logging

# Import necessary services (adjust paths based on project structure)
from backend.services.embedding_service import EmbeddingService
from backend.services.qdrant_service import QdrantService
from backend.services.chatkit_service import ChatKitService
from backend.config.settings import settings

# Configure logging
logger = logging.getLogger(__name__)

# Initialize services
embedding_service = EmbeddingService()
qdrant_service = QdrantService()
chatkit_service = ChatKitService()

router = APIRouter(prefix="/api/chat", tags=["chat"])


# ============================================================================
# PYDANTIC MODELS
# ============================================================================

class QuestionRequest(BaseModel):
    """Request model for asking a question"""
    question: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The question to ask the RAG system"
    )
    collection_name: Optional[str] = Field(
        default="documents",
        description="Qdrant collection to search"
    )
    top_k: Optional[int] = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of similar chunks to retrieve"
    )
    similarity_threshold: Optional[float] = Field(
        default=0.7,
        ge=0.0,
        le=1.0,
        description="Minimum similarity score for retrieved chunks"
    )
    include_metadata: Optional[bool] = Field(
        default=True,
        description="Include metadata in the response"
    )

    @validator('question')
    def validate_question(cls, v):
        """Validate question is not empty or just whitespace"""
        if not v.strip():
            raise ValueError("Question cannot be empty or whitespace only")
        return v.strip()


class SourceChunk(BaseModel):
    """Model for a retrieved source chunk"""
    chunk_id: str = Field(..., description="Unique chunk identifier")
    content: str = Field(..., description="Chunk text content")
    similarity_score: float = Field(..., description="Similarity score (0-1)")
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Chunk metadata (file path, section, etc.)"
    )


class QuestionResponse(BaseModel):
    """Response model for the RAG query"""
    question: str = Field(..., description="Original question")
    answer: str = Field(..., description="Generated answer from ChatKit")
    sources: List[SourceChunk] = Field(
        default=[],
        description="Retrieved source chunks used for the answer"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Additional response metadata"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Response timestamp"
    )


class ErrorResponse(BaseModel):
    """Standard error response model"""
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Error message")
    details: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Additional error details"
    )
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="Error timestamp"
    )


# ============================================================================
# ENDPOINT IMPLEMENTATION
# ============================================================================

@router.post(
    "/query",
    response_model=QuestionResponse,
    status_code=status.HTTP_200_OK,
    responses={
        400: {"model": ErrorResponse, "description": "Invalid request"},
        500: {"model": ErrorResponse, "description": "Internal server error"},
        503: {"model": ErrorResponse, "description": "Service unavailable"}
    },
    summary="Ask a question using RAG",
    description="Submit a question to the RAG system. Returns an AI-generated answer with source citations."
)
async def query_rag(
    request: QuestionRequest,
    # Add authentication dependency if needed:
    # current_user: User = Depends(get_current_user)
) -> QuestionResponse:
    """
    RAG Query Endpoint

    Process flow:
    1. Validate input question
    2. Generate embedding using Gemini
    3. Search Qdrant for similar chunks
    4. Filter by similarity threshold
    5. Pass context to ChatKit for answer generation
    6. Return answer with source citations

    Args:
        request: QuestionRequest with question and search parameters

    Returns:
        QuestionResponse with answer and sources

    Raises:
        HTTPException: For validation errors, service failures, or rate limits
    """
    try:
        logger.info(f"Processing RAG query: {request.question[:100]}...")

        # ====================================================================
        # STEP 1: Generate embedding for the question
        # ====================================================================
        try:
            question_embedding = await embedding_service.generate_embedding(
                text=request.question
            )
            logger.debug(f"Generated embedding with dimension: {len(question_embedding)}")
        except Exception as e:
            logger.error(f"Embedding generation failed: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail={
                    "error": "EmbeddingServiceError",
                    "message": "Failed to generate question embedding",
                    "details": {"reason": str(e)}
                }
            )

        # ====================================================================
        # STEP 2: Search Qdrant for similar chunks
        # ====================================================================
        try:
            search_results = await qdrant_service.search(
                collection_name=request.collection_name,
                query_vector=question_embedding,
                limit=request.top_k
            )
            logger.debug(f"Retrieved {len(search_results)} results from Qdrant")
        except Exception as e:
            logger.error(f"Qdrant search failed: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail={
                    "error": "VectorSearchError",
                    "message": "Failed to search vector database",
                    "details": {"reason": str(e)}
                }
            )

        # ====================================================================
        # STEP 3: Filter by similarity threshold
        # ====================================================================
        filtered_results = [
            result for result in search_results
            if result.score >= request.similarity_threshold
        ]

        if not filtered_results:
            logger.warning(
                f"No chunks found above threshold {request.similarity_threshold}"
            )
            # Return a response indicating no relevant context found
            return QuestionResponse(
                question=request.question,
                answer="I couldn't find relevant information to answer your question. Please try rephrasing or ask a different question.",
                sources=[],
                metadata={
                    "total_retrieved": len(search_results),
                    "threshold": request.similarity_threshold,
                    "filtered_count": 0
                }
            )

        # ====================================================================
        # STEP 4: Prepare context for ChatKit
        # ====================================================================
        source_chunks = []
        context_texts = []

        for idx, result in enumerate(filtered_results, 1):
            chunk_content = result.payload.get("text", "")
            chunk_metadata = {
                "file_path": result.payload.get("file_path"),
                "section": result.payload.get("section"),
                "chunk_index": result.payload.get("chunk_index")
            }

            source_chunks.append(
                SourceChunk(
                    chunk_id=str(result.id),
                    content=chunk_content,
                    similarity_score=round(result.score, 4),
                    metadata=chunk_metadata if request.include_metadata else None
                )
            )

            # Format context for ChatKit
            context_texts.append(
                f"[Source {idx}]\n{chunk_content}\n"
            )

        # Combine all context
        combined_context = "\n".join(context_texts)

        # ====================================================================
        # STEP 5: Generate answer using ChatKit
        # ====================================================================
        try:
            chatkit_prompt = f"""You are a helpful assistant answering questions based on the provided context.

Context:
{combined_context}

Question: {request.question}

Instructions:
- Answer the question using ONLY the information from the provided context
- If the context doesn't contain enough information, say so clearly
- Cite specific sources when possible using [Source N] notation
- Be concise and accurate
- If multiple sources support your answer, mention that

Answer:"""

            answer = await chatkit_service.generate_response(
                prompt=chatkit_prompt,
                max_tokens=500,
                temperature=0.7
            )

            logger.info(f"Generated answer for question: {request.question[:50]}...")

        except Exception as e:
            logger.error(f"ChatKit generation failed: {str(e)}")
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail={
                    "error": "LLMGenerationError",
                    "message": "Failed to generate answer",
                    "details": {"reason": str(e)}
                }
            )

        # ====================================================================
        # STEP 6: Build and return response
        # ====================================================================
        response = QuestionResponse(
            question=request.question,
            answer=answer.strip(),
            sources=source_chunks,
            metadata={
                "total_retrieved": len(search_results),
                "filtered_count": len(filtered_results),
                "threshold": request.similarity_threshold,
                "model": "chatkit",
                "embedding_model": "gemini-text-embedding"
            }
        )

        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise

    except Exception as e:
        # Catch-all for unexpected errors
        logger.exception(f"Unexpected error in RAG query: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "InternalServerError",
                "message": "An unexpected error occurred",
                "details": {"reason": str(e)}
            }
        )


# ============================================================================
# HEALTH CHECK ENDPOINT
# ============================================================================

@router.get(
    "/health",
    status_code=status.HTTP_200_OK,
    summary="Check RAG system health",
    description="Verify all RAG components are operational"
)
async def health_check() -> Dict[str, Any]:
    """
    Check the health of all RAG system components

    Returns:
        Dictionary with service statuses
    """
    health_status = {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "services": {}
    }

    # Check Embedding Service
    try:
        await embedding_service.health_check()
        health_status["services"]["embedding"] = "healthy"
    except Exception as e:
        health_status["services"]["embedding"] = f"unhealthy: {str(e)}"
        health_status["status"] = "degraded"

    # Check Qdrant Service
    try:
        await qdrant_service.health_check()
        health_status["services"]["qdrant"] = "healthy"
    except Exception as e:
        health_status["services"]["qdrant"] = f"unhealthy: {str(e)}"
        health_status["status"] = "degraded"

    # Check ChatKit Service
    try:
        await chatkit_service.health_check()
        health_status["services"]["chatkit"] = "healthy"
    except Exception as e:
        health_status["services"]["chatkit"] = f"unhealthy: {str(e)}"
        health_status["status"] = "degraded"

    return health_status
```

### 3. Generate Service Layer Components

Create supporting service classes if they don't exist:

#### EmbeddingService

```python
# backend/services/embedding_service.py

import os
from typing import List
import google.generativeai as genai
from backend.config.settings import settings

class EmbeddingService:
    """Service for generating embeddings using Google Gemini"""

    def __init__(self):
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model = "models/text-embedding-004"

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using Gemini

        Args:
            text: Input text to embed

        Returns:
            List of float values representing the embedding
        """
        try:
            result = genai.embed_content(
                model=self.model,
                content=text,
                task_type="retrieval_query"
            )
            return result['embedding']
        except Exception as e:
            raise Exception(f"Gemini embedding failed: {str(e)}")

    async def health_check(self) -> bool:
        """Check if embedding service is available"""
        try:
            await self.generate_embedding("health check")
            return True
        except:
            return False
```

#### QdrantService

```python
# backend/services/qdrant_service.py

from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import ScoredPoint
from backend.config.settings import settings

class QdrantService:
    """Service for vector search using Qdrant"""

    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY
        )

    async def search(
        self,
        collection_name: str,
        query_vector: List[float],
        limit: int = 5
    ) -> List[ScoredPoint]:
        """
        Search for similar vectors in Qdrant

        Args:
            collection_name: Name of the collection to search
            query_vector: Query embedding vector
            limit: Maximum number of results

        Returns:
            List of scored points (search results)
        """
        try:
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_vector,
                limit=limit
            )
            return results
        except Exception as e:
            raise Exception(f"Qdrant search failed: {str(e)}")

    async def health_check(self) -> bool:
        """Check if Qdrant is available"""
        try:
            collections = self.client.get_collections()
            return True
        except:
            return False
```

#### ChatKitService

```python
# backend/services/chatkit_service.py

import openai
from backend.config.settings import settings

class ChatKitService:
    """Service for generating answers using OpenAI ChatKit"""

    def __init__(self):
        openai.api_key = settings.OPENAI_API_KEY
        self.model = "gpt-4"  # or "gpt-3.5-turbo"

    async def generate_response(
        self,
        prompt: str,
        max_tokens: int = 500,
        temperature: float = 0.7
    ) -> str:
        """
        Generate response using ChatKit

        Args:
            prompt: The prompt including context and question
            max_tokens: Maximum tokens in response
            temperature: Sampling temperature

        Returns:
            Generated text response
        """
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful assistant."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=max_tokens,
                temperature=temperature
            )
            return response.choices[0].message.content
        except Exception as e:
            raise Exception(f"ChatKit generation failed: {str(e)}")

    async def health_check(self) -> bool:
        """Check if ChatKit is available"""
        try:
            await self.generate_response("test", max_tokens=10)
            return True
        except:
            return False
```

### 4. Configuration Settings

Add to `backend/config/settings.py`:

```python
from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    # API Keys
    GEMINI_API_KEY: str
    QDRANT_API_KEY: str
    QDRANT_URL: str
    OPENAI_API_KEY: str

    # Collection Settings
    DEFAULT_COLLECTION: str = "documents"
    DEFAULT_TOP_K: int = 5
    DEFAULT_SIMILARITY_THRESHOLD: float = 0.7

    # Server Settings
    API_HOST: str = "0.0.0.0"
    API_PORT: int = 8000

    class Config:
        env_file = ".env"
        case_sensitive = True

settings = Settings()
```

### 5. Requirements

Add to `requirements.txt`:

```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
pydantic==2.5.0
pydantic-settings==2.1.0
qdrant-client>=1.9.0
google-generativeai==0.3.1
openai==1.3.0
python-dotenv==1.0.0
```

### 6. Example Usage

Create a test client example:

```python
import requests

# Example 1: Basic query
response = requests.post(
    "http://localhost:8000/api/chat/query",
    json={
        "question": "What is the main topic of the documentation?"
    }
)
print(response.json())

# Example 2: Custom parameters
response = requests.post(
    "http://localhost:8000/api/chat/query",
    json={
        "question": "How do I install the library?",
        "top_k": 10,
        "similarity_threshold": 0.8,
        "collection_name": "technical_docs"
    }
)
print(response.json())

# Example 3: Health check
response = requests.get("http://localhost:8000/api/chat/health")
print(response.json())
```

### 7. Testing

Create comprehensive tests:

```python
# tests/test_rag_endpoint.py

import pytest
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_query_endpoint_success():
    """Test successful RAG query"""
    response = client.post(
        "/api/chat/query",
        json={"question": "What is FastAPI?"}
    )
    assert response.status_code == 200
    data = response.json()
    assert "answer" in data
    assert "sources" in data
    assert "question" in data

def test_query_endpoint_validation():
    """Test input validation"""
    # Empty question
    response = client.post(
        "/api/chat/query",
        json={"question": ""}
    )
    assert response.status_code == 422

    # Invalid top_k
    response = client.post(
        "/api/chat/query",
        json={"question": "test", "top_k": 0}
    )
    assert response.status_code == 422

def test_health_endpoint():
    """Test health check endpoint"""
    response = client.get("/api/chat/health")
    assert response.status_code == 200
    data = response.json()
    assert "status" in data
    assert "services" in data
```

## Deliverables

When complete, provide:

1. **Complete endpoint file** with all models and error handling
2. **Service layer files** (embedding, qdrant, chatkit)
3. **Configuration file** with settings
4. **Requirements.txt** with dependencies
5. **Example usage code** for testing
6. **Unit tests** for the endpoint
7. **README section** documenting the API

## Notes

- All code includes comprehensive error handling
- Pydantic models provide automatic validation
- Logging is integrated throughout
- Health check endpoint verifies all services
- Response includes metadata for debugging
- Similarity threshold prevents irrelevant results
- Source citations included in response
- Ready for production deployment
