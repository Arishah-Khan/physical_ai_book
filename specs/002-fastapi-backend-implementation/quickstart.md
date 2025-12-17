# Quickstart Guide: FastAPI Backend for Physical AI & Humanoid Robotics Textbook

**Feature**: FastAPI Backend Implementation
**Date**: 2025-12-14

## Overview

This guide provides step-by-step instructions to set up and run the FastAPI backend for the Physical AI & Humanoid Robotics textbook with RAG chatbot functionality.

## Prerequisites

- Python 3.11 or higher
- Docker and Docker Compose (for local development)
- OpenAI API key
- Qdrant Cloud account and API key
- Neon Postgres account and connection string

## Environment Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Set up Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 4. Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
# OpenAI Configuration
OPENAI_API_KEY=your_openai_api_key_here

# Qdrant Configuration
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=textbook_chunks

# Database Configuration
DATABASE_URL=your_neon_postgres_connection_string

# Application Configuration
APP_ENV=development
APP_DEBUG=true
SECRET_KEY=your_secret_key_for_sessions

# Rate Limiting
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=3600  # in seconds
```

## Running the Application

### 1. Start External Services (Optional - using Docker)

```bash
# If running local Qdrant for development
docker run -d --name qdrant-local -p 6333:6333 qdrant/qdrant
```

### 2. Run Database Migrations

```bash
# If using Alembic for migrations
alembic upgrade head
```

### 3. Start the FastAPI Server

```bash
# Using uvicorn directly
uvicorn src.main:app --host 0.0.0.0 --port 8000 --reload

# Or using the start script if available
python -m src.main
```

The API will be available at `http://localhost:8000`

### 4. Access API Documentation

- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`

## Key API Endpoints

### RAG Chat Endpoint
```
POST /api/v1/chat
```
Send questions about the textbook content and receive AI-generated responses.

**Example request:**
```json
{
  "question": "Explain the principles of humanoid robotics locomotion",
  "session_id": "optional-session-uuid",
  "language": "en"
}
```

### Document Ingestion
```
POST /api/v1/documents/upload
```
Upload new textbook content for RAG processing.

**Example request:**
```json
{
  "title": "Chapter 5: Humanoid Locomotion",
  "content": "Full text content of the chapter...",
  "source_type": "text"
}
```

### Session Management
```
POST /api/v1/sessions
```
Create a new user session for maintaining conversation context.

## Testing

### Run Unit Tests

```bash
pytest tests/unit/
```

### Run Integration Tests

```bash
pytest tests/integration/
```

### Run All Tests

```bash
pytest tests/
```

## Configuration Options

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `APP_ENV` | Application environment | `development` |
| `APP_DEBUG` | Enable debug mode | `false` |
| `QDRANT_COLLECTION_NAME` | Name of the Qdrant collection | `textbook_chunks` |
| `RATE_LIMIT_REQUESTS` | Requests allowed per window | `100` |
| `RATE_LIMIT_WINDOW` | Time window for rate limiting (seconds) | `3600` |

## Development Workflow

### Adding New Endpoints

1. Create the endpoint in the appropriate file under `src/api/v1/`
2. Define Pydantic models in `src/models/`
3. Implement business logic in `src/services/`
4. Write unit tests in `tests/unit/`
5. Write integration tests in `tests/integration/`

### Running with Hot Reload

```bash
uvicorn src.main:app --reload
```

## Troubleshooting

### Common Issues

1. **Import Errors**: Ensure virtual environment is activated and dependencies are installed
2. **Database Connection**: Verify `DATABASE_URL` is correctly set
3. **Qdrant Connection**: Check `QDRANT_URL` and `QDRANT_API_KEY` environment variables
4. **OpenAI API**: Verify `OPENAI_API_KEY` is valid and has sufficient quota

### Enable Debug Logging

Set `APP_DEBUG=true` in your environment variables to enable detailed logging.

## Next Steps

1. Implement authentication with Better-Auth
2. Add more sophisticated document processing capabilities
3. Implement Urdu translation features
4. Add comprehensive monitoring and observability
5. Set up CI/CD pipeline for automated testing and deployment