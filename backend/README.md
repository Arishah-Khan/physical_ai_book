# RAG Chatbot Backend

This is the backend for the RAG (Retrieval-Augmented Generation) chatbot project that uses OpenAI Agents SDK, FastAPI, and Qdrant vector database to provide intelligent responses to user queries about documentation.

## Features

- **Documentation Q&A**: Ask questions about documentation and get accurate answers with citations
- **Greeting Handling**: Friendly greeting responses for initial interactions
- **Context-Limited Querying**: Answer questions based on provided selected text only
- **Health Monitoring**: Comprehensive health checks for all services
- **Thread-based Conversations**: Maintain conversation context using thread IDs

## Prerequisites

- Python 3.9+
- Qdrant vector database (running locally or accessible via URL)
- OpenAI API key
- Google Gemini API key

## Setup

### 1. Clone the repository and navigate to the backend directory

```bash
cd backend
```

### 2. Create a virtual environment and install dependencies

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Create a `.env` file with the following environment variables:

```env
# API Configuration
HOST=0.0.0.0
PORT=8000
DEBUG=true

# Qdrant Configuration
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COLLECTION_NAME=documentation_chunks

# API Keys
OPENAI_API_KEY=your_openai_api_key
GEMINI_API_KEY=your_gemini_api_key

# Model Configuration
CHAT_MODEL=gpt-4-turbo-preview
EMBEDDING_MODEL=text-embedding-004
```

### 4. Start the Qdrant vector database

You can run Qdrant locally using Docker:

```bash
docker run -d --name qdrant -p 6333:6333 -p 6334:6334 qdrant/qdrant
```

Or use a cloud-hosted Qdrant instance.

### 5. Populate the vector database with documentation

Run the ingestion script to populate your Qdrant collection with documentation:

```bash
python ingest.py
```

## Usage

### Starting the Server

```bash
python main.py
```

Or using uvicorn:

```bash
uvicorn main:app --reload
```

The server will start on `http://localhost:8000` by default.

### API Endpoints

- `GET /` - API information and documentation links
- `GET /api/v1/health` - Health check for the service
- `POST /api/v1/chat` - Process user queries using RAG functionality
- `POST /api/v1/chat/selected-text` - Process queries based only on provided selected text

### Example API Calls

#### Chat with RAG functionality:

```bash
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is the main concept of this book?",
    "thread_id": "my_thread_123"
  }'
```

#### Chat with selected text only:

```bash
curl -X POST http://localhost:8000/api/v1/chat/selected-text \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What does this text say about AI?",
    "selected_text": "Artificial Intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.",
    "thread_id": "my_thread_123"
  }'
```

#### Health check:

```bash
curl http://localhost:8000/api/v1/health
```

## Testing

Run the integration tests:

```bash
python -m pytest tests/test_api_integration.py -v
```

## Development

- The project follows a modular structure with separate directories for agents, api, config, and services
- API models are defined in `api/models.py`
- Agent logic is in the `agents/` directory
- Service implementations are in the `services/` directory
- Configuration is handled via `config/settings.py`

## API Documentation

Interactive API documentation is available at:
- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`