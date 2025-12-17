# Research: Phase 2 - OpenAI Agents SDK Implementation

## Overview
This document captures research findings and technical decisions for implementing the RAG chatbot using OpenAI Agents SDK.

## Decision: OpenAI Agents SDK Integration Pattern
**Rationale**: Based on the phase2-workflow.md document, we need to implement the OpenAI Agents SDK with specific patterns:
- Use `@function_tool` decorator for custom tools
- Implement `RunContextWrapper[None]` as first parameter in function tools
- Create agents as module-level variables using `Agent[None]` constructor
- Use `Runner.run()` for agent execution with `trace()` context manager

**Alternatives considered**:
- Direct OpenAI API calls without Agents SDK
- Custom agent framework
- LangChain agents

## Decision: Service Layer Architecture
**Rationale**: Reuse and adapt Phase 1 services for real-time operations:
- `QdrantService` with `search_similar()` method for vector search
- `GeminiEmbeddingService` with `generate_single_embedding()` for query embeddings
- Proper error handling and connection management optimized for real-time use

**Alternatives considered**:
- Building from scratch instead of adapting Phase 1
- Using different embedding models
- Different vector database solutions

## Decision: Agent Architecture Pattern
**Rationale**: Four specialized agents as specified in workflow:
- Greeting Agent: Handles greetings only, uses greet_user tool
- RAG Agent: Documentation Q&A, uses search_documentation tool
- Selected Text Agent: Answers from provided text only, no tools
- Main Router Agent: Routes queries to appropriate agents

**Alternatives considered**:
- Single monolithic agent
- Different number of specialized agents
- Rule-based routing vs agent-based routing

## Decision: API Endpoint Design
**Rationale**: REST API endpoints following the specification:
- Health check endpoint: GET /api/v1/health
- Full RAG chat: POST /api/v1/chat
- Selected text chat: POST /api/v1/chat/selected-text
- Root endpoint: GET / for API info

**Alternatives considered**:
- GraphQL instead of REST
- Different endpoint naming conventions
- WebSocket for real-time communication

## Decision: Data Model and Pydantic Models
**Rationale**: Following the specified models:
- `ChatRequest`: message, thread_id (optional)
- `SelectedTextChatRequest`: message, selected_text, thread_id (optional)
- `ChatResponse`: answer, thread_id (optional), sources (optional list)
- `HealthResponse`: status, message

**Alternatives considered**:
- Different field names or structures
- Additional metadata fields
- Different validation approaches

## Decision: Vector Database Integration
**Rationale**: Integration with existing Phase 1 Qdrant setup:
- Reuse same collection (`documentation_chunks`)
- Same embedding model (Gemini text-embedding-004)
- Same metadata structure for consistency
- Leverage existing indexed data

**Alternatives considered**:
- Different vector database
- Different embedding model
- Separate collection for Phase 2

## Decision: Dependency Management
**Rationale**: Based on workflow document requirements:
- FastAPI for web framework
- OpenAI library for Agents SDK
- Qdrant-client for vector database access
- Google Generative AI for embeddings
- Pydantic for data validation

**Alternatives considered**:
- Different web frameworks (Flask, Django)
- Different agent frameworks
- Different vector databases