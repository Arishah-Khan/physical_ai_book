from pydantic import BaseModel, Field
from typing import List, Optional
from enum import Enum


class AskRequest(BaseModel):
    """
    Represents a user's question and optional parameters for the chatbot API
    """
    question: str = Field(..., description="The user's question text", min_length=1, max_length=2000)
    temperature: Optional[float] = Field(0.7, description="Controls randomness of responses", ge=0.0, le=1.0)
    max_tokens: Optional[int] = Field(500, description="Maximum tokens in response", ge=100, le=2000)


class AskSelectedRequest(BaseModel):
    """
    Represents a user's question with selected text context
    """
    question: str = Field(..., description="The user's question text", min_length=1, max_length=2000)
    selected_text: str = Field(..., description="Text context for the question", min_length=1)
    temperature: Optional[float] = Field(0.7, description="Controls randomness of responses", ge=0.0, le=1.0)
    max_tokens: Optional[int] = Field(500, description="Maximum tokens in response", ge=100, le=2000)


class Source(BaseModel):
    """
    Represents a documentation chunk used to generate the answer
    """
    title: str = Field(..., description="Title or heading of the source document")
    path: str = Field(..., description="Relative path to the source document")
    text: Optional[str] = Field(None, description="Excerpt from the source document")
    score: Optional[float] = Field(None, description="Relevance score from vector search", ge=0.0, le=1.0)


class AskResponse(BaseModel):
    """
    Contains the chatbot's answer and metadata
    """
    answer: str = Field(..., description="The AI-generated answer to the question")
    sources: Optional[List[Source]] = Field(None, description="List of source citations")
    error: Optional[str] = Field(None, description="Error message if the query failed")


class HealthStatus(str, Enum):
    """
    Enum for health status values
    """
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"


class ServiceHealthStatus(BaseModel):
    """
    Health status of individual services
    """
    qdrant: str = Field(..., description="Qdrant service status")
    gemini: str = Field(..., description="Gemini service status")
    openai_agents: str = Field(..., description="OpenAI Agents service status")


class HealthResponse(BaseModel):
    """
    Contains system health status information
    """
    status: HealthStatus = Field(..., description="Overall system status")
    services: ServiceHealthStatus = Field(..., description="Status of individual services")
    timestamp: str = Field(..., description="ISO 8601 timestamp of the health check")


class ErrorResponse(BaseModel):
    """
    Error response model
    """
    error: str = Field(..., description="Error message")


class ChatRequest(BaseModel):
    """
    Request model for chat endpoint
    """
    message: str = Field(..., description="User's message", min_length=1)
    thread_id: Optional[str] = Field(None, description="Thread identifier for conversation continuity")


class ChatResponse(BaseModel):
    """
    Response model for chat endpoint
    """
    answer: str = Field(..., description="AI-generated response")
    thread_id: Optional[str] = Field(None, description="Thread identifier for conversation continuity")
    sources: List[Source] = Field(default_factory=list, description="List of sources used in response")


class SelectedTextChatRequest(BaseModel):
    """
    Request model for chat with selected text context
    """
    message: str = Field(..., description="User's message", min_length=1)
    selected_text: str = Field(..., description="Selected text to provide context", min_length=1)
    thread_id: Optional[str] = Field(None, description="Thread identifier for conversation continuity")