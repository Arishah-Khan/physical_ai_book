from fastapi import APIRouter
from app.api.v1.endpoints import chat, health

# Create API router for v1
api_router = APIRouter()

# Include chat endpoints
api_router.include_router(chat.router, tags=["chat"])

# Include health endpoints
api_router.include_router(health.router, tags=["health"])