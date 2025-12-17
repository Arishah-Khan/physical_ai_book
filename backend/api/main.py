from fastapi import APIRouter
from typing import Dict, Any
import logging

logger = logging.getLogger(__name__)

# Create router for main API endpoints (root only)
router = APIRouter()

@router.get("/", response_model=Dict[str, Any])
async def root_info():
    """
    API information and documentation links
    """
    return {
        "message": "RAG Chatbot API",
        "version": "1.0.0",
        "endpoints": {
            "health": "/api/v1/health",
            "chat": "/api/v1/chat",
            "chat_selected_text": "/api/v1/chat/selected-text"
        },
        "docs": "/docs",
        "redoc": "/redoc"
    }