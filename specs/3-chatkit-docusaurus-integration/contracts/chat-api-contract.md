# API Contract: Chat Endpoints

## Overview
This document describes the API contracts for the chat endpoints that the frontend will consume. These endpoints already exist and no modifications are required.

## POST /api/v1/chat

### Description
Endpoint for submitting general chat queries to the backend chat service.

### Request
- **Method**: POST
- **Path**: /api/v1/chat
- **Content-Type**: application/json

#### Request Body
```json
{
  "message": "string, the user's query message",
  "sessionId": "string, optional session identifier",
  "pageContext": {
    "url": "string, current page URL",
    "title": "string, current page title",
    "path": "string, current page path"
  }
}
```

#### Validation
- `message` is required and must be 1-1000 characters
- `sessionId` is optional, if provided must be a valid string
- `pageContext` is optional but if provided, must include all fields

### Response
#### Success (200 OK)
```json
{
  "message": "string, the chatbot's response",
  "sources": ["string, list of source documents used"],
  "sessionId": "string, session identifier",
  "timestamp": "ISO 8601 date string"
}
```

#### Error (400 Bad Request)
```json
{
  "error": "string, error message",
  "code": "string, error code"
}
```

#### Error (500 Internal Server Error)
```json
{
  "error": "string, error message",
  "code": "string, error code"
}
```

## POST /api/v1/chat/selected-text

### Description
Endpoint for submitting queries about selected text from documentation pages.

### Request
- **Method**: POST
- **Path**: /api/v1/chat/selected-text
- **Content-Type**: application/json

#### Request Body
```json
{
  "message": "string, the user's query about the selected text",
  "selectedText": "string, the text that was selected on the page",
  "sessionId": "string, optional session identifier",
  "pageContext": {
    "url": "string, current page URL",
    "title": "string, current page title",
    "path": "string, current page path"
  }
}
```

#### Validation
- `message` is required and must be 1-1000 characters
- `selectedText` is required and must be 1-500 characters
- `sessionId` is optional, if provided must be a valid string
- `pageContext` is optional but if provided, must include all fields

### Response
#### Success (200 OK)
```json
{
  "message": "string, the chatbot's response",
  "sources": ["string, list of source documents used"],
  "sessionId": "string, session identifier",
  "timestamp": "ISO 8601 date string"
}
```

#### Error (400 Bad Request)
```json
{
  "error": "string, error message",
  "code": "string, error code"
}
```

#### Error (500 Internal Server Error)
```json
{
  "error": "string, error message",
  "code": "string, error code"
}
```

## Error Handling

### Common Error Responses
All endpoints may return the following common errors:

#### 429 Too Many Requests
```json
{
  "error": "Rate limit exceeded",
  "code": "RATE_LIMIT_EXCEEDED",
  "retryAfter": "number of seconds to wait before retrying"
}
```

#### 503 Service Unavailable
```json
{
  "error": "Service temporarily unavailable",
  "code": "SERVICE_UNAVAILABLE"
}
```