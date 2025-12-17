import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.v1.router import api_router
from api.main import router as main_router  # Import the main API router with root endpoint
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Create FastAPI app instance
app = FastAPI(
    title="RAG Chatbot API",
    description="""
# RAG Chatbot API

This API provides a RAG (Retrieval-Augmented Generation) powered chatbot using OpenAI Agents SDK, FastAPI, and Qdrant vector database.

## Features

- **Documentation Q&A**: Ask questions about documentation and get accurate answers with citations
- **Greeting Handling**: Friendly greeting responses for initial interactions
- **Context-Limited Querying**: Answer questions based on provided selected text only
- **Health Monitoring**: Comprehensive health checks for all services

## Endpoints

- `GET /` - API information and documentation links
- `GET /api/v1/health` - Health check for the service
- `POST /api/v1/chat` - Process user queries using RAG functionality
- `POST /api/v1/chat/selected-text` - Process queries based only on provided selected text

## Quick Start

1. Set up environment variables in `.env` file
2. Start the Qdrant vector database
3. Run the application: `uvicorn main:app --reload`
4. Visit `/docs` for interactive API documentation

## Authentication

This API uses API keys configured in environment variables for access to external services.
    """,
    version="1.0.0",
    contact={
        "name": "API Support",
        "email": "support@example.com",
    },
    license_info={
        "name": "MIT License",
        "url": "https://opensource.org/licenses/MIT",
    },
)

# Add CORS middleware for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include root endpoint at base level
app.include_router(main_router)

# Include API v1 routes (this includes health and chat endpoints)
app.include_router(api_router, prefix="/api/v1")

if __name__ == "__main__":
    host = os.getenv("HOST", "0.0.0.0")
    port = int(os.getenv("PORT", 8080))
    debug = os.getenv("DEBUG", "False").lower() == "true"

    uvicorn.run(
        app,
        host=host,
        port=port,
        reload=debug,
        log_level="info" if debug else "info"
    )