# Data Model: Data Pipeline and Qdrant Setup

## Entities

### Documentation Chunk
- **Description**: Represents a segment of processed documentation content with associated embedding vector and metadata
- **Fields**:
  - `id` (string): Unique identifier for the chunk
  - `text` (string): The actual text content of the chunk
  - `embedding` (list[float]): Vector representation of the text content
  - `metadata` (dict):
    - `source_file` (string): Original filename
    - `relative_path` (string): Path relative to docs directory
    - `full_path` (string): Full path to the source file
  - `size` (int): Size of the text content in characters

### Embedding Vector
- **Description**: Numerical representation of text content for semantic similarity search
- **Fields**:
  - `values` (list[float]): Array of float values representing the embedding (768 dimensions for Google Gemini)
  - `model` (string): The model used to generate the embedding (text-embedding-004)

### Qdrant Collection
- **Description**: Vector database collection storing document chunks with their embeddings and metadata
- **Fields**:
  - `name` (string): Name of the collection (documentation_chunks)
  - `vector_size` (int): Dimension of the vectors (768 for Google Gemini embeddings)
  - `distance` (string): Distance metric used (cosine)
  - `points_count` (int): Number of vectors stored in the collection

### Processing Configuration
- **Description**: Configuration parameters for the data processing pipeline
- **Fields**:
  - `max_chunk_size` (int): Maximum size of each chunk in characters (default: 1000)
  - `overlap` (int): Overlap between chunks in characters (default: 200)
  - `docs_path` (string): Path to the documentation directory (default: docusaurus-project/docs)
  - `batch_size` (int): Number of texts to process in each embedding batch (default: 10)

## Relationships

- One Qdrant Collection contains many Documentation Chunks
- Each Documentation Chunk has one Embedding Vector
- Each Documentation Chunk has metadata that links back to the source document

## Validation Rules

- Documentation Chunk text must not be empty
- Embedding vector must have exactly 768 dimensions (for Google Gemini)
- Metadata must include source_file and relative_path
- Chunk size must be within configured limits
- Relative path must be a valid path within the documentation directory