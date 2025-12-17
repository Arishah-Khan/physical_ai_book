# Quickstart: Phase 2 - OpenAI Agents SDK Implementation

## Overview
This guide provides setup instructions for the RAG chatbot implementation using OpenAI Agents SDK.

## Prerequisites
- Python 3.11+
- Access to OpenAI API key
- Access to Google Gemini API key
- Access to Qdrant Cloud (from Phase 1)
- Phase 1 data pipeline completed (documentation indexed)

## Setup Instructions

### 1. Environment Setup
```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r backend/requirements.txt
```

### 2. Environment Variables
Create a `.env` file in the project root with the following variables:

```env
OPENAI_API_KEY=your-openai-api-key
QDRANT_URL=your-qdrant-cloud-url
QDRANT_API_KEY=your-qdrant-api-key
GEMINI_API_KEY=your-gemini-api-key
COLLECTION_NAME=documentation_chunks
DEBUG=True
HOST=0.0.0.0
PORT=8000
CHAT_MODEL=gpt-4-turbo-preview
```

### 3. Verify Phase 1 Integration
Before starting Phase 2, ensure Phase 1 components are working:
- Verify Qdrant collection exists with documentation chunks
- Confirm embedding model is accessible
- Test that Phase 1 indexing completed successfully

### 4. Start the Service
```bash
cd backend
python main.py
```

The service will start on `http://localhost:8000` by default.

## API Endpoints

### Health Check
```
GET /
```
Returns API information and links to documentation.

### Health Status
```
GET /api/v1/health
```
Returns health status of the service.

### RAG Chat
```
POST /api/v1/chat
Content-Type: application/json

{
  "message": "Your question here",
  "thread_id": "optional-thread-id"
}
```
Processes user query using RAG functionality.

### Selected Text Chat
```
POST /api/v1/chat/selected-text
Content-Type: application/json

{
  "message": "Your question about the text",
  "selected_text": "The text you want to ask about",
  "thread_id": "optional-thread-id"
}
```
Processes user query based only on provided selected text.

## Testing the Implementation
```bash
# Test health endpoint
curl http://localhost:8000/api/v1/health

# Test basic chat
curl -X POST http://localhost:8000/api/v1/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "Hello"}'

# Test selected text chat
curl -X POST http://localhost:8000/api/v1/chat/selected-text \
  -H "Content-Type: application/json" \
  -d '{"message": "Explain this", "selected_text": "Sample text to explain"}'
```

## Next Steps
1. Integrate with frontend application
2. Add authentication if needed
3. Implement monitoring and logging
4. Performance testing and optimization