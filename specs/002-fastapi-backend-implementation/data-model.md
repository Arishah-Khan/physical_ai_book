# Data Model: FastAPI Backend Implementation

**Feature**: FastAPI Backend for Physical AI & Humanoid Robotics Textbook
**Date**: 2025-12-14
**Status**: Completed

## Overview

This document defines the data entities and relationships for the FastAPI backend, focusing on the RAG system, document management, and user session functionality.

## Core Entities

### 1. Document

**Purpose**: Represents textbook content that has been ingested and processed for RAG retrieval

**Attributes**:
- `id` (UUID): Unique identifier for the document
- `title` (string): Title of the document/chapter
- `content` (text): Full text content of the document
- `source_type` (enum): Type of source (PDF, DOCX, TXT, HTML, etc.)
- `source_path` (string): Path or URL where the original document is stored
- `version` (string): Version identifier for content tracking
- `created_at` (timestamp): When the document was first ingested
- `updated_at` (timestamp): When the document was last updated
- `metadata` (JSON): Additional metadata like author, page count, etc.
- `status` (enum): Processing status (pending, processing, completed, failed)

**Relationships**:
- One Document → Many DocumentChunks
- One Document → Many DocumentTags (many-to-many through association)

### 2. DocumentChunk

**Purpose**: Represents a processed chunk of a document that has been embedded for vector search

**Attributes**:
- `id` (UUID): Unique identifier for the chunk
- `document_id` (UUID): Reference to the parent document
- `content` (text): The chunked text content (typically 500-1000 tokens)
- `chunk_index` (integer): Sequential index within the document
- `embedding` (vector): The embedding vector stored in Qdrant
- `metadata` (JSON): Additional metadata like page number, section
- `created_at` (timestamp): When the chunk was created

**Relationships**:
- One DocumentChunk → One Document (parent)
- One DocumentChunk → Many ChunkTags (many-to-many through association)

### 3. UserSession

**Purpose**: Represents a user's conversation session with the RAG system

**Attributes**:
- `id` (UUID): Unique identifier for the session
- `user_id` (UUID, optional): Reference to authenticated user (null for anonymous)
- `session_token` (string): Unique token for session identification
- `created_at` (timestamp): When the session started
- `updated_at` (timestamp): When the session was last active
- `expires_at` (timestamp): When the session expires
- `metadata` (JSON): Additional session metadata

**Relationships**:
- One UserSession → Many Queries (in the session)
- One UserSession → Many Conversations (session history)

### 4. Query

**Purpose**: Represents a user's question/query to the RAG system

**Attributes**:
- `id` (UUID): Unique identifier for the query
- `session_id` (UUID): Reference to the user session
- `question` (text): The original question asked by the user
- `question_embedding` (vector): The embedding of the question (for potential analysis)
- `query_type` (enum): Type of query (factual, conceptual, translation, etc.)
- `created_at` (timestamp): When the query was submitted
- `language` (string): Language of the query (for translation features)

**Relationships**:
- One Query → One UserSession (parent)
- One Query → Many RetrievedChunks (many-to-many through QueryRetrieval)
- One Query → One Response (one-to-one)

### 5. Response

**Purpose**: Represents the AI-generated response to a user's query

**Attributes**:
- `id` (UUID): Unique identifier for the response
- `query_id` (UUID): Reference to the original query
- `answer` (text): The AI-generated answer
- `answer_urdu` (text, optional): Urdu translation of the answer
- `confidence_score` (float): Confidence score of the response (0.0-1.0)
- `sources` (JSON): List of document chunks used to generate the response
- `created_at` (timestamp): When the response was generated
- `token_usage` (JSON): Token usage statistics for the generation

**Relationships**:
- One Response → One Query (parent)

### 6. QueryRetrieval

**Purpose**: Junction table to track which document chunks were retrieved for a specific query

**Attributes**:
- `id` (UUID): Unique identifier
- `query_id` (UUID): Reference to the query
- `chunk_id` (UUID): Reference to the retrieved document chunk
- `similarity_score` (float): Similarity score from vector search (0.0-1.0)
- `retrieval_rank` (integer): Rank order of retrieval

**Relationships**:
- One QueryRetrieval → One Query
- One QueryRetrieval → One DocumentChunk

### 7. User

**Purpose**: Represents an authenticated user (if authentication is implemented)

**Attributes**:
- `id` (UUID): Unique identifier for the user
- `email` (string): User's email address
- `name` (string): User's display name
- `created_at` (timestamp): When the user account was created
- `updated_at` (timestamp): When the user profile was last updated
- `preferences` (JSON): User preferences including language settings

**Relationships**:
- One User → Many UserSessions
- One User → Many Queries (through sessions)

## Value Objects

### 1. EmbeddingVector

**Purpose**: Represents a vector embedding for semantic search

**Attributes**:
- `vector_data` (array of floats): The actual embedding values
- `model` (string): The model used to generate the embedding
- `dimension` (integer): Number of dimensions in the vector

### 2. TokenUsage

**Purpose**: Tracks token usage for API cost management

**Attributes**:
- `prompt_tokens` (integer): Number of tokens in the prompt
- `completion_tokens` (integer): Number of tokens in the response
- `total_tokens` (integer): Total tokens used

## Validation Rules

### Document Entity
- Title must not be empty
- Content must be at least 100 characters for meaningful RAG
- Source path must be a valid URL or file path
- Status must be one of the defined enum values

### Query Entity
- Question must not be empty
- Question must be less than 1000 characters
- Session must be active (not expired)

### DocumentChunk Entity
- Content must not be empty
- Chunk size should be between 100-2000 characters
- Must belong to a valid document

## State Transitions

### Document Status Transitions
```
pending → processing → completed
pending → processing → failed
completed → processing (on update) → completed
```

### Session Lifecycle
```
created → active → expired
```

## Indexing Strategy

### Database Indexes
- Document: index on (title, created_at, status)
- DocumentChunk: index on (document_id, chunk_index)
- UserSession: index on (session_token, expires_at)
- Query: index on (session_id, created_at)
- User: index on (email)

### Vector Database Indexes (Qdrant)
- DocumentChunk embeddings: vector index for similarity search
- Metadata filtering: indexes on document_id, chunk_index for efficient retrieval

## Data Integrity Constraints

1. Referential Integrity: Foreign keys enforce parent-child relationships
2. Unique Constraints: Session tokens must be unique
3. Check Constraints: Confidence scores between 0.0 and 1.0
4. Cascade Deletes: When a document is deleted, its chunks are also deleted

## Performance Considerations

1. Document chunks should be sized for optimal vector search performance
2. Session data should be archived after expiration to maintain performance
3. Query history may need pagination for long-running sessions
4. Embedding vectors are stored in Qdrant for efficient similarity search