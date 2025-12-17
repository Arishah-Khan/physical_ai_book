# Phase 2: OpenAI Agents SDK Implementation - Workflow Documentation

## Section 1: Overview

Phase 2 focuses on implementing the RAG (Retrieval-Augmented Generation) chatbot using FastAPI as the backend framework and the OpenAI Agents SDK for intelligent query processing. This phase builds upon the data pipeline and Qdrant vector database established in Phase 1.

### Phase 2 Goals
- Implement a FastAPI backend that integrates with OpenAI Agents SDK
- Create specialized agents for different chatbot functionalities (greeting, full RAG, selected text, main orchestrator)
- Enable real-time document search and retrieval using the vector database from Phase 1
- Provide a clean API interface for chatbot interactions
- Support both full documentation search and context-limited responses based on user-selected text

### What Gets Built
- Service layer wrappers for real-time Qdrant and embedding operations
- Custom function tools for vector search and greeting
- Four specialized OpenAI agents with defined handoff patterns
- Pydantic models for API request/response handling
- FastAPI application with health check and chat endpoints
- Complete integration with Phase 1 vector database

### Connection to Phase 1
Phase 2 directly leverages the vector database and embedding infrastructure created in Phase 1:
- Uses the same Qdrant collection (`documentation_chunks`) populated during Phase 1 data ingestion
- Reuses the same Gemini embedding model for consistency between indexing and query time
- Adapts existing service classes for real-time rather than batch operations
- Maintains the same data format and metadata structure established in Phase 1

---

## Section 2: Project Structure

```
backend/
├── services/
│   ├── __init__.py
│   ├── qdrant_service.py          # [PHASE 2] Real-time Qdrant operations
│   └── embedding_service.py       # [PHASE 2] Real-time embedding operations
├── agents/
│   ├── __init__.py
│   ├── tools.py                   # [PHASE 2] Function tools for agents
│   └── rag_agent.py               # [PHASE 2] Agent definitions as variables
├── api/
│   ├── __init__.py
│   ├── models.py                  # [PHASE 2] Pydantic API models
│   └── main.py                    # [PHASE 2] FastAPI application
├── config/
│   ├── __init__.py
│   └── settings.py                # [PHASE 1/2] Extended configuration
├── requirements.txt               # [PHASE 1/2] Updated dependencies
└── main.py                        # [PHASE 2] Application entry point
```

### Folder/Component Purposes:
- **services/**: Contains service layer wrappers that adapt Phase 1 functionality for real-time use
- **agents/**: Houses OpenAI Agents SDK components including tools and agent definitions
- **api/**: Contains FastAPI application, models, and endpoint definitions
- **config/**: Configuration management extended from Phase 1 with additional agent settings
- **main.py**: Application entry point that initializes and runs the FastAPI application

---

## Section 3: Implementation Workflow (Step-by-Step)

### Step 1: Service Layer Adaptation (Phase 1 reuse)
**What to create**: Adapt existing Qdrant and embedding services from Phase 1 for real-time usage

**Dependencies**: Phase 1 data ingestion and vector database setup

**Key implementation requirements**:
- Create `QdrantService` class with `search_similar()` method
- Create `GeminiEmbeddingService` class with `generate_single_embedding()` method
- Add proper error handling and connection management
- Ensure services are optimized for real-time rather than batch operations
- Initialize Qdrant client with connection settings from Phase 1
- Initialize Gemini embedding model with same configuration as Phase 1

**Integration points**: Direct import of existing service classes from Phase 1

---

### Step 2: Function Tools Creation (@function_tool pattern)
**What to create**: Custom function tools using OpenAI Agents SDK `@function_tool` decorator

**Dependencies**: Services from Step 1

**Key implementation requirements**:
- Import: `from agents import function_tool, RunContextWrapper`
- Create `search_documentation` tool:
  - First parameter: `wrapper: RunContextWrapper[None]`
  - Additional parameters: `query: str`, `top_k: int = 5`
  - Return type: `str` (formatted search results)
  - Uses `GeminiEmbeddingService.generate_single_embedding()` to create query embedding
  - Uses `QdrantService.search_similar()` to find relevant chunks
  - Formats results with source file, relevance score, and content
  - Includes error handling and logging

- Create `greet_user` tool:
  - First parameter: `wrapper: RunContextWrapper[None]`
  - No additional parameters
  - Return type: `str` (greeting message)
  - Returns a friendly greeting introducing the assistant

- Initialize services at module level before defining tools

**Integration points**: Used by agent definitions in Step 3

---

### Step 3: Agent Definitions (4 agents as module-level variables)
**What to create**: 4 different agents with specific instructions and handoff patterns

**Dependencies**: Tools from Step 2

**Key implementation requirements**:
- Import: `from agents import Agent`
- Import tools: `from .tools import search_documentation, greet_user`

**Agent 1 - Greeting Agent**:
- Variable name: `greeting_agent`
- Type: `Agent[None]`
- Name: "Greeting Assistant"
- Instructions: Handle greetings only (hello, hi, hey, good morning, etc.), call greet_user tool, do not answer documentation questions
- Tools: `[greet_user]`
- Handoffs: `[]`

**Agent 2 - RAG Agent**:
- Variable name: `rag_agent`
- Type: `Agent[None]`
- Name: "Documentation RAG Agent"
- Instructions: Answer questions using search_documentation tool, analyze retrieved content, provide accurate answers with citations, always search before answering
- Tools: `[search_documentation]`
- Handoffs: `[]`

**Agent 3 - Selected Text Agent**:
- Variable name: `selected_text_agent`
- Type: `Agent[None]`
- Name: "Selected Text Assistant"
- Instructions: Answer ONLY from provided text, do not use any tools, do not use external knowledge, be precise and quote relevant parts
- Tools: `[]` (no tools)
- Handoffs: `[]`

**Agent 4 - Main Router Agent**:
- Variable name: `main_agent`
- Type: `Agent[None]`
- Name: "Main Documentation Assistant"
- Instructions: Route queries to appropriate agents (greetings → greeting_agent, documentation questions → rag_agent), do not answer questions yourself
- Tools: `[]` (no tools)
- Handoffs: `[greeting_agent, rag_agent]`

**Integration points**: Connected via handoff mechanism, imported by FastAPI application

---

### Step 4: Pydantic API Models
**What to create**: Pydantic models for API request/response handling

**Dependencies**: None (standalone models)

**Key implementation requirements**:
- Import: `from pydantic import BaseModel, Field`
- Import: `from typing import Optional, List`

**Models to create**:

1. **ChatRequest**:
   - `message: str` - User's question
   - `thread_id: Optional[str]` - Thread ID for conversation history

2. **SelectedTextChatRequest**:
   - `message: str` - User's question
   - `selected_text: str` - Text selected by user
   - `thread_id: Optional[str]` - Thread ID

3. **ChatResponse**:
   - `answer: str` - Agent's response
   - `thread_id: Optional[str]` - Thread ID
   - `sources: Optional[list]` - Source files (default empty list)

4. **HealthResponse**:
   - `status: str` - Health status
   - `message: str` - Status message

**Integration points**: Used by FastAPI endpoints in Step 6

---

### Step 5: FastAPI Application Setup
**What to create**: Initialize FastAPI application with proper configuration

**Dependencies**: All other components

**Key implementation requirements**:
- Create FastAPI app instance with title, version, description
- Add CORS middleware (allow all origins for development)
- Import agents: `from agents.rag_agent import main_agent, selected_text_agent`
- Import Runner and trace: `from agents import Runner, trace`
- Import models: `from .models import ChatRequest, SelectedTextChatRequest, ChatResponse, HealthResponse`
- Set up logging configuration

**Integration points**: Serves as the main API gateway

---

### Step 6: API Endpoints Implementation
**What to create**: API endpoints for health check and chat functionality

**Dependencies**: All previous steps

**Key implementation requirements**:

**Endpoint 1 - Root endpoint**:
- Route: `GET /`
- Returns: API info with links to docs and health

**Endpoint 2 - Health check**:
- Route: `GET /api/v1/health`
- Response model: `HealthResponse`
- Returns: Status "healthy" with message

**Endpoint 3 - Full RAG chat**:
- Route: `POST /api/v1/chat`
- Request model: `ChatRequest`
- Response model: `ChatResponse`
- Implementation:
  - Log incoming request
  - Use `with trace("RAG Chat"):`
  - Execute: `result = await Runner.run(main_agent, request.message, context=None)`
  - Extract: `final_output = result.final_output`
  - Handle different output types (dict, str)
  - Return ChatResponse with answer, thread_id, sources
  - Include error handling with try-except

**Endpoint 4 - Selected text chat**:
- Route: `POST /api/v1/chat/selected-text`
- Request model: `SelectedTextChatRequest`
- Response model: `ChatResponse`
- Implementation:
  - Format message with selected text context
  - Use `with trace("Selected Text Chat"):`
  - Execute: `result = await Runner.run(selected_text_agent, formatted_message, context=None)`
  - Extract and process final_output
  - Return ChatResponse (no sources)
  - Include error handling

**Endpoint 5 - Main execution**:
- Add `if __name__ == "__main__":` block
- Import uvicorn
- Run app with settings from config (host, port, reload)

**Integration points**: Connects agents to external world via HTTP

---

### Step 7: Testing
**What to create**: Unit and integration tests for the complete system

**Dependencies**: All implemented components

**Key implementation requirements**:
- Test individual tools (search_documentation, greet_user)
- Test agent responses for different query types
- Verify both RAG and non-RAG chat endpoints
- Test error handling and edge cases
- Test CORS and middleware functionality

---

## Section 4: Component Specifications (Structure Only)

### 1. services/qdrant_service.py
**File path**: `backend/services/qdrant_service.py`

**Purpose**: Provides vector search operations for RAG

**Class**: `QdrantService`

**Methods to implement**:
- `__init__()`: Initialize Qdrant client with settings
- `search_similar(query_embedding: List[float], limit: int = 5) -> List[Dict]`: Search vector database and return results with score, text, metadata, size

**Required imports**: `qdrant_client`, `config.settings`

---

### 2. services/embedding_service.py
**File path**: `backend/services/embedding_service.py`

**Purpose**: Provides embedding generation for queries

**Class**: `GeminiEmbeddingService`

**Methods to implement**:
- `__init__()`: Configure Gemini API and load embedding model
- `generate_single_embedding(text: str) -> List[float]`: Generate embedding vector for text, handle errors with zero vector fallback

**Required imports**: `google.generativeai`, `config.settings`

---

### 3. agents/tools.py
**File path**: `backend/agents/tools.py`

**Purpose**: Function tools for OpenAI agents

**Tools to implement**:
1. `search_documentation(wrapper, query, top_k)`:
   - Search Qdrant using embeddings
   - Format results with source info
   - Return formatted string

2. `greet_user(wrapper)`:
   - Return greeting message

**Required imports**: `agents` (function_tool, RunContextWrapper), services, typing, logging

**Module-level initialization**: Create service instances before defining tools

---

### 4. agents/rag_agent.py
**File path**: `backend/agents/rag_agent.py`

**Purpose**: Agent definitions with handoff patterns

**Agents to define** (as module-level variables):
1. `greeting_agent`: Handles greetings, uses greet_user tool
2. `rag_agent`: Documentation Q&A, uses search_documentation tool
3. `selected_text_agent`: Answers from provided text only, no tools
4. `main_agent`: Router with handoffs to greeting_agent and rag_agent

**Required imports**: `agents` (Agent), `.tools` (search_documentation, greet_user), logging

---

### 5. api/models.py
**File path**: `backend/api/models.py`

**Purpose**: Pydantic models for API

**Models to define**:
1. `ChatRequest`: message, thread_id (optional)
2. `SelectedTextChatRequest`: message, selected_text, thread_id (optional)
3. `ChatResponse`: answer, thread_id (optional), sources (optional list)
4. `HealthResponse`: status, message

**Required imports**: `pydantic`, `typing`

---

### 6. api/main.py
**File path**: `backend/api/main.py`

**Purpose**: FastAPI application with endpoints

**Components to implement**:
- FastAPI app instance with metadata
- CORS middleware
- Root endpoint (GET /)
- Health check endpoint (GET /api/v1/health)
- Chat endpoint (POST /api/v1/chat) using Runner.run with main_agent
- Selected text endpoint (POST /api/v1/chat/selected-text) using Runner.run with selected_text_agent
- Main execution block with uvicorn

**Required imports**: FastAPI, CORS, agents (Runner, trace), agents.rag_agent, models, settings, logging

---

## Section 5: OpenAI Agents SDK Patterns Reference

### Pattern 1: Correct Imports
```
from agents import Agent, Runner, function_tool, RunContextWrapper, trace
```

### Pattern 2: Function Tool Signature
```
@function_tool
async def tool_name(wrapper: RunContextWrapper[None], param1: str) -> ReturnType:
    # Implementation
```

**Key requirements**:
- `@function_tool` decorator
- First parameter always: `wrapper: RunContextWrapper[None]`
- Simple Python types for other parameters
- Proper return type annotation

### Pattern 3: Agent Definition
```
agent_name = Agent[None](
    name="Agent Name",
    instructions="Detailed instructions...",
    tools=[tool1, tool2],
    handoffs=[other_agent]
)
```

**Key requirements**:
- Stored as module-level variable (not inside a class)
- Use `Agent[None]()` constructor
- `tools` parameter is a list of function tool references
- `handoffs` parameter is a list of other agent variable references

### Pattern 4: Agent Execution
```
with trace("Workflow Name"):
    result = await Runner.run(agent_variable, message, context=None)
    answer = result.final_output
```

**Key requirements**:
- Import `Runner` and `trace` from `agents`
- Use `await Runner.run()`
- Pass agent variable (not class)
- Extract answer with `result.final_output`

---

## Section 6: API Endpoints Flow Diagrams

### Endpoint 1: GET /api/v1/health
```
Request → FastAPI → Return HealthResponse → Response
```
**No agents involved, direct response**

---

### Endpoint 2: POST /api/v1/chat (Full RAG)
```
Request with message
    ↓
FastAPI receives ChatRequest
    ↓
Runner.run(main_agent, message)
    ↓
main_agent analyzes message
    ↓
    ├→ If greeting → HANDOFF to greeting_agent → greet_user tool
    │
    └→ If question → HANDOFF to rag_agent → search_documentation tool
                                                 ↓
                                          Embedding generated
                                                 ↓
                                          Qdrant search
                                                 ↓
                                          Results formatted
                                                 ↓
                                          Agent synthesizes answer
    ↓
result.final_output extracted
    ↓
ChatResponse returned
```

---

### Endpoint 3: POST /api/v1/chat/selected-text (No RAG)
```
Request with message + selected_text
    ↓
FastAPI receives SelectedTextChatRequest
    ↓
Format message with selected text context
    ↓
Runner.run(selected_text_agent, formatted_message)
    ↓
selected_text_agent analyzes only provided text
    ↓
Generates answer WITHOUT using any tools
    ↓
result.final_output extracted
    ↓
ChatResponse returned (sources = [])
```

---

## Section 7: Dependencies & Requirements

### From Phase 1 (already installed):
```
fastapi==0.104.1
uvicorn==0.24.0
qdrant-client>=1.9.0
google-generativeai==0.4.1
python-dotenv==1.0.0
beautifulsoup4==4.12.2
numpy==1.24.3
pydantic==2.5.0
tiktoken==0.5.2
```

### New for Phase 2:
```
openai==1.3.5              # OpenAI base library
agents==0.1.0              # OpenAI Agents SDK (check actual package name)
```

### Complete requirements.txt:
```
fastapi==0.104.1
uvicorn==0.24.0
qdrant-client>=1.9.0
google-generativeai==0.4.1
python-dotenv==1.0.0
beautifulsoup4==4.12.2
openai==1.3.5
agents==0.1.0
numpy==1.24.3
pydantic==2.5.0
tiktoken==0.5.2
```

**Note**: Verify the exact package name for OpenAI Agents SDK from official documentation or GitHub repository.

---

## Section 8: Environment Variables

### New for Phase 2:
- `OPENAI_API_KEY` - Required for OpenAI Agents SDK

### From Phase 1 (required):
- `QDRANT_URL` - Qdrant cloud URL
- `QDRANT_API_KEY` - Qdrant API key
- `GEMINI_API_KEY` - Google Gemini API key for embeddings
- `COLLECTION_NAME` - Qdrant collection name (e.g., "documentation_chunks")

### Application settings (from Phase 1):
- `DEBUG` - Debug mode (True/False)
- `HOST` - Server host (default: "0.0.0.0")
- `PORT` - Server port (default: 8000)
- `CHAT_MODEL` - OpenAI model for agents (e.g., "gpt-4-turbo-preview")

**Example .env file**:
```
OPENAI_API_KEY=sk-...
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key
GEMINI_API_KEY=your-key
COLLECTION_NAME=documentation_chunks
DEBUG=True
HOST=0.0.0.0
PORT=8000
CHAT_MODEL=gpt-4-turbo-preview
```

---

## Section 9: Integration with Phase 1

### Qdrant Data Access:
- Phase 2 uses `QdrantService.search_similar()` to query the collection created in Phase 1
- Same collection name (`documentation_chunks`)
- Same vector dimensions (768 from Gemini text-embedding-004)
- Same metadata structure (text, metadata, size)

### Embedding Consistency:
- Phase 2 uses `GeminiEmbeddingService.generate_single_embedding()` with same model as Phase 1
- Same embedding model: "models/text-embedding-004"
- Query embeddings match indexed document embeddings
- Ensures accurate similarity search

### Service Adaptation:
- Phase 1 services designed for batch processing
- Phase 2 adapts them for real-time single-query processing
- Same error handling patterns
- Same configuration management

### Data Format:
- Search results include: score, text, metadata (source_file, relative_path, full_path), size
- Metadata preserved from Phase 1 ingestion
- Enables proper source attribution in responses

---

## Section 10: Implementation Order & Dependencies

### Implementation Sequence:

**Stage 1 - Foundation (Parallel):**
1. `services/qdrant_service.py` - Qdrant operations
2. `services/embedding_service.py` - Embedding operations
3. `api/models.py` - Pydantic models (independent)

**Stage 2 - Agent Layer (Sequential):**
4. `agents/tools.py` - Depends on Stage 1 (services)
5. `agents/rag_agent.py` - Depends on Step 4 (tools)

**Stage 3 - API Layer (Sequential):**
6. `api/main.py` - Depends on Stage 1 (models) and Stage 2 (agents)
7. `main.py` - Entry point, depends on Step 6

**Stage 4 - Validation:**
8. Testing - Depends on all previous stages

### Dependency Graph:
```
Phase 1 (Qdrant data + services)
         ↓
    ┌────┴────┬─────────────┐
    ↓         ↓             ↓
qdrant_   embedding_    models.py
service.py service.py    (independent)
    ↓         ↓
    └────┬────┘
         ↓
     tools.py
         ↓
    rag_agent.py
         ↓         ↓
         └────┬────┘
              ↓
          main.py
              ↓
          Testing
```

### Critical Dependencies:
- **tools.py** MUST have services ready
- **rag_agent.py** MUST have tools.py ready
- **api/main.py** MUST have both agents and models ready
- All components MUST have Phase 1 setup complete (Qdrant data ingested)

### Parallel Opportunities:
- Services and models can be built in parallel
- Testing can begin as soon as individual components are ready

---

## Section 11: Ready for Implementation

This workflow document provides the complete architecture and requirements for Phase 2.

**Next steps**:
1. Use this document to create detailed SpecKit Plus specifications
2. Implement each component following the patterns described
3. Test each stage before moving to the next
4. Verify integration with Phase 1 components

**Key success criteria**:
- All imports use `from agents import ...` (not `openai_agents`)
- Function tools have `RunContextWrapper[None]` as first parameter
- Agents are module-level variables (not classes)
- Runner.run() is used for execution
- Both RAG and non-RAG endpoints work correctly