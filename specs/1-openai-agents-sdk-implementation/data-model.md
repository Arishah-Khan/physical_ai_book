# Data Model: Phase 2 - OpenAI Agents SDK Implementation

## Overview
This document defines the data models for the RAG chatbot implementation, based on the entities identified in the feature specification.

## Entity: Query
**Description**: User's question or input text that requires processing by the system

**Fields**:
- `message` (string): The user's question or input text
- `thread_id` (string, optional): Identifier for conversation context
- `selected_text` (string, optional): Specific text provided by user for context-limited queries

**Validation Rules**:
- `message` must not be empty
- `thread_id` must follow UUID format if provided
- `selected_text` should have reasonable length limits

## Entity: Documentation Chunk
**Description**: Individual pieces of documentation stored in vector database with metadata and embeddings

**Fields**:
- `id` (string): Unique identifier for the chunk
- `text` (string): The actual content of the documentation chunk
- `metadata` (object): Additional information including source_file, relative_path, full_path
- `vector` (array of numbers): Embedding vector representation
- `size` (number): Size of the chunk

**Validation Rules**:
- `text` must not be empty
- `vector` must have consistent dimensions (768 for Gemini embeddings)
- `metadata` must contain required source information

## Entity: Chat Thread
**Description**: Conversation context identified by thread ID to maintain context across multiple exchanges

**Fields**:
- `thread_id` (string): Unique identifier for the conversation thread
- `messages` (array of objects): History of messages in the conversation
- `created_at` (datetime): Timestamp when thread was created
- `updated_at` (datetime): Timestamp when thread was last updated

**Validation Rules**:
- `thread_id` must be unique
- `messages` must have reasonable size limits
- Thread should have TTL for cleanup

## Entity: Agent Response
**Description**: System's answer to user query, potentially including citations to source documents

**Fields**:
- `answer` (string): The agent's response to the user's query
- `thread_id` (string, optional): Thread identifier if applicable
- `sources` (array of objects, optional): List of source documents cited
- `timestamp` (datetime): When the response was generated

**Validation Rules**:
- `answer` must not be empty
- `sources` should contain valid document references when present
- Response should be under reasonable character limits

## API Request/Response Models

### ChatRequest Model
**Purpose**: Request model for standard chat interactions

**Fields**:
- `message` (string): User's question
- `thread_id` (string, optional): Thread identifier for conversation context

### SelectedTextChatRequest Model
**Purpose**: Request model for queries based on selected text

**Fields**:
- `message` (string): User's question about selected text
- `selected_text` (string): The text the user has selected
- `thread_id` (string, optional): Thread identifier for conversation context

### ChatResponse Model
**Purpose**: Response model for chat interactions

**Fields**:
- `answer` (string): Agent's response
- `thread_id` (string, optional): Thread identifier
- `sources` (array, optional): List of source documents (default: empty list)

### HealthResponse Model
**Purpose**: Response model for health check endpoint

**Fields**:
- `status` (string): Health status (e.g., "healthy")
- `message` (string): Status message