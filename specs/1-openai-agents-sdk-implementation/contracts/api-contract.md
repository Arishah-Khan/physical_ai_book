# API Contract: RAG Chatbot Service

## Overview
REST API for the RAG (Retrieval-Augmented Generation) chatbot service. This API enables users to ask questions about documentation and receive intelligent responses with proper citations.

## Base URL
`http://localhost:8000` (or configured host/port)

## Endpoints

### GET /
**Description**: API information and documentation links
**Authentication**: None required

**Response**:
- Status: 200 OK
- Content-Type: application/json
- Body: API info with links to docs and health

### GET /api/v1/health
**Description**: Health check for the service
**Authentication**: None required

**Response**:
- Status: 200 OK
- Content-Type: application/json
- Body:
```json
{
  "status": "healthy",
  "message": "Service is running normally"
}
```

### POST /api/v1/chat
**Description**: Process user query using RAG functionality
**Authentication**: None required (for this implementation)

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "message": "string (required) - User's question",
  "thread_id": "string (optional) - Thread identifier for conversation context"
}
```

**Response**:
- Status: 200 OK
- Content-Type: application/json
- Body:
```json
{
  "answer": "string - Agent's response to the user's query",
  "thread_id": "string (optional) - Thread identifier",
  "sources": "array of objects (optional) - List of source documents cited, default: []"
}
```

**Error Responses**:
- 400 Bad Request: Invalid request format
- 500 Internal Server Error: Processing error

### POST /api/v1/chat/selected-text
**Description**: Process user query based only on provided selected text
**Authentication**: None required (for this implementation)

**Request**:
- Content-Type: application/json
- Body:
```json
{
  "message": "string (required) - User's question about selected text",
  "selected_text": "string (required) - The text the user has selected",
  "thread_id": "string (optional) - Thread identifier for conversation context"
}
```

**Response**:
- Status: 200 OK
- Content-Type: application/json
- Body:
```json
{
  "answer": "string - Agent's response based on selected text",
  "thread_id": "string (optional) - Thread identifier",
  "sources": "array (optional) - Empty array (no external sources used)"
}
```

**Error Responses**:
- 400 Bad Request: Invalid request format
- 500 Internal Server Error: Processing error

## Data Models

### ChatRequest
```json
{
  "message": "string (required)",
  "thread_id": "string (optional)"
}
```

### SelectedTextChatRequest
```json
{
  "message": "string (required)",
  "selected_text": "string (required)",
  "thread_id": "string (optional)"
}
```

### ChatResponse
```json
{
  "answer": "string (required)",
  "thread_id": "string (optional)",
  "sources": "array (optional, default: [])"
}
```

### HealthResponse
```json
{
  "status": "string (required)",
  "message": "string (required)"
}
```

## Headers
- Content-Type: application/json for POST requests

## Error Handling
All error responses follow the format:
```json
{
  "detail": "Error message describing the issue"
}
```

## Rate Limiting
No rate limiting implemented in this version.

## Versioning
API version is included in the path (e.g., /api/v1/).