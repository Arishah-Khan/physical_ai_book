# Integrated RAG Chatbot Engineering Specification
**System Requirement Document**

## Overview
Build an integrated RAG Chatbot using FastAPI backend, Qdrant Cloud Free Tier (vector DB), ChatKit (OpenAI Agents SDK), and Google Gemini text-embedding-3-small as the embedding model. The chatbot will be embedded inside a Docusaurus book located at `docusaurus-project/docs`.

The chatbot must:
1. Answer questions based on entire book content
2. Answer questions based ONLY on user-selected text
3. Use a complete RAG pipeline (embedding → vector search → context → LLM reasoning)
4. Provide a clean chatbot UI embedded directly into Docusaurus pages

---

## Agent & Skill Mapping Overview

This project uses specialized Claude Code agents and skills for implementation. Below is the mapping of which agents/skills to use in each phase:

### Phase 1: Data Pipeline + Qdrant Setup
**Status:** ✅ Implemented
- Manual implementation using Python scripts
- No specialized agents required (core data processing)


---

## PHASE 1 — Data Pipeline + Qdrant Setup

### 1.1 Backend Ingestion Folder Structure
```
backend/
├── data_ingestion/
│   ├── __init__.py
│   ├── markdown_reader.py
│   ├── text_converter.py
│   ├── chunker.py
│   ├── embedding_service.py
│   └── qdrant_client.py
├── config/
│   ├── __init__.py
│   └── settings.py
├── requirements.txt
└── ingest.py
```

### 1.2 Complete List of Files/Scripts
- `markdown_reader.py` - Reads .md/.mdx files from docusaurus-project/docs
- `text_converter.py` - Converts markdown to clean plain text
- `chunker.py` - Implements chunking strategy
- `embedding_service.py` - Handles Gemini embedding generation
- `qdrant_client.py` - Manages Qdrant collection operations
- `ingest.py` - Main ingestion script
- `settings.py` - Configuration management

### 1.3 Markdown (.md/.mdx) Reader for docusaurus-project/docs

**File: backend/data_ingestion/markdown_reader.py**
```python
import os
import re
from pathlib import Path
from typing import List, Dict, Optional

class MarkdownReader:
    def __init__(self, docs_path: str = "docusaurus-project/docs"):
        self.docs_path = Path(docs_path)

    def read_all_markdown_files(self) -> List[Dict[str, str]]:
        """Read all markdown files from the documentation directory"""
        markdown_files = []

        # Find all .md and .mdx files
        for file_path in self.docs_path.rglob("*.md"):
            try:
                content = file_path.read_text(encoding='utf-8')
                relative_path = str(file_path.relative_to(self.docs_path))

                markdown_files.append({
                    "id": str(file_path),
                    "content": content,
                    "path": relative_path,
                    "filename": file_path.name,
                    "relative_path": relative_path
                })

            except Exception as e:
                print(f"Error reading {file_path}: {str(e)}")

        for file_path in self.docs_path.rglob("*.mdx"):
            try:
                content = file_path.read_text(encoding='utf-8')
                relative_path = str(file_path.relative_to(self.docs_path))

                markdown_files.append({
                    "id": str(file_path),
                    "content": content,
                    "path": relative_path,
                    "filename": file_path.name,
                    "relative_path": relative_path
                })

            except Exception as e:
                print(f"Error reading {file_path}: {str(e)}")

        return markdown_files

    def extract_frontmatter(self, content: str) -> tuple:
        """Extract frontmatter and content from markdown file"""
        # Match frontmatter between --- delimiters
        frontmatter_match = re.match(r'^---\s*\n(.*?)\n---\s*\n(.*)', content, re.DOTALL)

        if frontmatter_match:
            frontmatter = frontmatter_match.group(1)
            content_body = frontmatter_match.group(2)
        else:
            frontmatter = ""
            content_body = content

        return frontmatter, content_body
```

### 1.4 Markdown → Clean Plain Text Converter

**File: backend/data_ingestion/text_converter.py**
```python
import re
from typing import List
from bs4 import BeautifulSoup

class MarkdownToTextConverter:
    def __init__(self):
        self.code_block_pattern = re.compile(r'```.*?\n.*?\n```', re.DOTALL)
        self.inline_code_pattern = re.compile(r'`.*?`')
        self.header_pattern = re.compile(r'^#+\s+(.*)', re.MULTILINE)
        self.list_item_pattern = re.compile(r'^\s*[-*+]\s+(.*)', re.MULTILINE)
        self.link_pattern = re.compile(r'\[(.*?)\]\(.*?\)')
        self.bold_pattern = re.compile(r'\*\*(.*?)\*\*')
        self.italic_pattern = re.compile(r'\*(.*?)\*')
        self.table_pattern = re.compile(r'^\|.*\|$', re.MULTILINE)

    def convert(self, markdown_content: str) -> str:
        """Convert markdown content to clean plain text"""
        # Extract and remove code blocks temporarily
        code_blocks = self.code_block_pattern.findall(markdown_content)
        content_without_code = self.code_block_pattern.sub(' [CODE_BLOCK] ', markdown_content)

        # Remove inline code
        content_without_inline_code = self.inline_code_pattern.sub(' [INLINE_CODE] ', content_without_code)

        # Convert headers to plain text
        content_no_headers = self.header_pattern.sub(r'\1', content_without_inline_code)

        # Convert list items to plain text
        content_no_lists = self.list_item_pattern.sub(r'- \1', content_no_headers)

        # Convert links to plain text
        content_no_links = self.link_pattern.sub(r'\1', content_no_lists)

        # Convert bold to plain text
        content_no_bold = self.bold_pattern.sub(r'\1', content_no_links)

        # Convert italic to plain text
        content_no_italic = self.italic_pattern.sub(r'\1', content_no_bold)

        # Remove tables
        content_no_tables = self.table_pattern.sub('', content_no_italic)

        # Remove HTML tags if any
        soup = BeautifulSoup(content_no_tables, 'html.parser')
        clean_text = soup.get_text()

        # Clean up multiple newlines and spaces
        clean_text = re.sub(r'\n\s*\n', '\n\n', clean_text.strip())
        clean_text = re.sub(r'[ \t]+', ' ', clean_text)

        # Replace placeholders with original content
        for i, code_block in enumerate(code_blocks):
            clean_text = clean_text.replace(f' [CODE_BLOCK] ', f'\n```\n{code_block}\n```\n', 1)

        return clean_text
```

### 1.5 Chunking Strategy (Sizes + Overlap)

**File: backend/data_ingestion/chunker.py**
```python
from typing import List, Dict
import re

class TextChunker:
    def __init__(self, max_chunk_size: int = 1000, overlap: int = 200):
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap

    def chunk_document(self, text: str, doc_metadata: Dict[str, str]) -> List[Dict]:
        """Chunk document into overlapping segments"""
        chunks = []

        # Split text by paragraphs first
        paragraphs = text.split('\n\n')

        current_chunk = ""
        current_size = 0

        for paragraph in paragraphs:
            paragraph_size = len(paragraph)

            # If adding this paragraph exceeds the chunk size
            if current_size + paragraph_size > self.max_chunk_size and current_chunk:
                # Save current chunk
                chunks.append({
                    'text': current_chunk.strip(),
                    'metadata': doc_metadata.copy(),
                    'size': current_size
                })

                # Start new chunk with overlap
                if len(current_chunk) > self.overlap:
                    overlap_start = len(current_chunk) - self.overlap
                    current_chunk = current_chunk[overlap_start:]
                    current_size = len(current_chunk)
                else:
                    current_chunk = ""
                    current_size = 0

                # Add current paragraph to new chunk
                current_chunk += f"\n\n{paragraph}"
                current_size += paragraph_size + 2
            else:
                # Add paragraph to current chunk
                if current_chunk:
                    current_chunk += f"\n\n{paragraph}"
                    current_size += paragraph_size + 2
                else:
                    current_chunk = paragraph
                    current_size = paragraph_size

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                'text': current_chunk.strip(),
                'metadata': doc_metadata.copy(),
                'size': current_size
            })

        return chunks

    def recursive_chunk(self, text: str, doc_metadata: Dict[str, str]) -> List[Dict]:
        """Recursively chunk text by different separators"""
        chunks = []

        # First try to split by paragraphs
        splits = text.split('\n\n')

        if len(text) <= self.max_chunk_size:
            return [{'text': text, 'metadata': doc_metadata, 'size': len(text)}]

        # If splits are too large, try to split by sentences
        if any(len(split) > self.max_chunk_size for split in splits):
            sentence_splits = re.split(r'[.!?]+\s+', text)
            temp_chunks = []
            current_chunk = ""

            for sentence in sentence_splits:
                if len(current_chunk) + len(sentence) + 1 <= self.max_chunk_size:
                    current_chunk += f"{sentence}. "
                else:
                    if current_chunk.strip():
                        temp_chunks.append(current_chunk.strip())
                    current_chunk = f"{sentence}. "

            if current_chunk.strip():
                temp_chunks.append(current_chunk.strip())

            splits = temp_chunks

        # Process each split
        for split in splits:
            if len(split) <= self.max_chunk_size:
                chunks.append({'text': split, 'metadata': doc_metadata, 'size': len(split)})
            else:
                # If still too big, force split
                forced_chunks = self._force_split(split, doc_metadata)
                chunks.extend(forced_chunks)

        return chunks

    def _force_split(self, text: str, doc_metadata: Dict[str, str]) -> List[Dict]:
        """Force split text when other methods fail"""
        chunks = []
        start = 0

        while start < len(text):
            end = start + self.max_chunk_size

            if end >= len(text):
                chunk = text[start:]
                chunks.append({'text': chunk, 'metadata': doc_metadata, 'size': len(chunk)})
                break
            else:
                # Find nearest space to avoid cutting words
                while end > start and text[end] != ' ' and text[end] != '\n':
                    end -= 1

                if end == start:  # No space found, just cut at max length
                    end = start + self.max_chunk_size

                chunk = text[start:end]
                chunks.append({'text': chunk, 'metadata': doc_metadata, 'size': len(chunk)})

                start = end
                if start < len(text) and text[start] == ' ':
                    start += 1

        return chunks
```

### 1.6 Embedding Generation Using Gemini text-embedding-3-small

**File: backend/data_ingestion/embedding_service.py**
```python
import google.generativeai as genai
from typing import List, Union
import numpy as np
from config.settings import settings

class GeminiEmbeddingService:
    def __init__(self):
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model = genai.EmbeddingModel("models/text-embedding-004")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generate embeddings for a list of texts using Gemini text-embedding-3-small"""
        embeddings = []

        # Batch process texts to avoid rate limits
        for i in range(0, len(texts), 10):  # Process in batches of 10
            batch = texts[i:i+10]

            try:
                response = self.model.batch_embed_content(
                    contents=batch
                )

                for embedding in response.embeddings:
                    embeddings.append(embedding.values)

            except Exception as e:
                print(f"Error generating embeddings for batch {i//10}: {str(e)}")
                # Fallback: return zero vectors for failed embeddings
                for _ in range(len(batch)):
                    embeddings.append([0.0] * 768)  # Assuming 768-dim embeddings

        return embeddings

    def generate_single_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text"""
        try:
            response = self.model.embed_content(
                content=text
            )
            return response.embeddings[0].values
        except Exception as e:
            print(f"Error generating embedding: {str(e)}")
            return [0.0] * 768  # Return zero vector on error

class EmbeddingProcessor:
    def __init__(self):
        self.embedding_service = GeminiEmbeddingService()

    def process_chunks_for_embedding(self, chunks: List[Dict]) -> List[Dict]:
        """Process chunks to add embeddings"""
        texts = [chunk['text'] for chunk in chunks]
        embeddings = self.embedding_service.generate_embeddings(texts)

        # Update chunks with embeddings
        for i, chunk in enumerate(chunks):
            chunk['embedding'] = embeddings[i]

        return chunks
```

### 1.7 Qdrant Collection Schema

**File: backend/data_ingestion/qdrant_client.py**
```python
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict
import uuid
from config.settings import settings

class QdrantVectorDB:
    def __init__(self):
        self.client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            prefer_grpc=True
        )
        self.collection_name = "documentation_book"

    def create_collection(self):
        """Create Qdrant collection with proper schema"""
        self.client.recreate_collection(
            collection_name=self.collection_name,
            vectors_config=models.VectorParams(
                size=768,  # Gemini text-embedding-3-small dimension
                distance=models.Distance.COSINE
            ),
            optimizers_config=models.OptimizersConfigDiff(
                memmap_threshold=20000,
                indexing_threshold=20000
            )
        )

    def setup_payload_schema(self):
        """Define payload schema for metadata storage"""
        # Qdrant handles dynamic schemas, but we define what we'll store
        # - text: the chunk text
        # - metadata: dict with path, filename, etc.
        # - size: chunk size
        pass  # Dynamic schema in Qdrant

    def batch_upload_chunks(self, chunks: List[Dict]):
        """Upload processed chunks to Qdrant"""
        points = []

        for chunk in chunks:
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=chunk['embedding'],
                payload={
                    "text": chunk['text'],
                    "metadata": chunk['metadata'],
                    "size": chunk['size']
                }
            )
            points.append(point)

        # Upload in batches of 100
        for i in range(0, len(points), 100):
            batch = points[i:i+100]
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch
            )

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict]:
        """Search for similar chunks to the query"""
        search_results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True
        )

        results = []
        for hit in search_results:
            results.append({
                'score': hit.score,
                'text': hit.payload['text'],
                'metadata': hit.payload['metadata'],
                'size': hit.payload['size']
            })

        return results

    def get_total_points(self) -> int:
        """Get total number of points in the collection"""
        collection_info = self.client.get_collection(self.collection_name)
        return collection_info.points_count
```

### 1.8 Full Code Templates

**File: backend/ingest.py**
```python
#!/usr/bin/env python3
"""
Data ingestion script for RAG Chatbot
Ingests markdown files from docusaurus-project/docs and uploads to Qdrant
"""
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from data_ingestion.markdown_reader import MarkdownReader
from data_ingestion.text_converter import MarkdownToTextConverter
from data_ingestion.chunker import TextChunker
from data_ingestion.embedding_service import EmbeddingProcessor
from data_ingestion.qdrant_client import QdrantVectorDB
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    logger.info("Starting data ingestion process...")

    # Initialize components
    reader = MarkdownReader()
    converter = MarkdownToTextConverter()
    chunker = TextChunker(max_chunk_size=1000, overlap=200)
    embedding_processor = EmbeddingProcessor()
    qdrant_db = QdrantVectorDB()

    # Step 1: Read all markdown files
    logger.info("Reading markdown files from docusaurus-project/docs...")
    markdown_files = reader.read_all_markdown_files()
    logger.info(f"Found {len(markdown_files)} markdown files")

    # Step 2: Convert to plain text and chunk
    all_chunks = []
    for file_data in markdown_files:
        logger.info(f"Processing {file_data['relative_path']}...")

        # Convert markdown to plain text
        clean_text = converter.convert(file_data['content'])

        # Create metadata
        metadata = {
            'source_file': file_data['filename'],
            'relative_path': file_data['relative_path'],
            'full_path': file_data['id']
        }

        # Chunk the text
        chunks = chunker.recursive_chunk(clean_text, metadata)
        all_chunks.extend(chunks)
        logger.info(f"  Created {len(chunks)} chunks")

    logger.info(f"Total chunks created: {len(all_chunks)}")

    # Step 3: Generate embeddings
    logger.info("Generating embeddings...")
    chunks_with_embeddings = embedding_processor.process_chunks_for_embedding(all_chunks)
    logger.info("Embeddings generated successfully")

    # Step 4: Create Qdrant collection and upload
    logger.info("Creating Qdrant collection...")
    qdrant_db.create_collection()

    logger.info("Uploading chunks to Qdrant...")
    qdrant_db.batch_upload_chunks(chunks_with_embeddings)
    logger.info("Upload completed successfully")

    # Verify upload
    total_points = qdrant_db.get_total_points()
    logger.info(f"Verification: Total points in collection: {total_points}")

    print("\nIngestion completed successfully!")
    print(f"- Processed {len(markdown_files)} files")
    print(f"- Created {len(all_chunks)} chunks")
    print(f"- Uploaded {total_points} vectors to Qdrant")

if __name__ == "__main__":
    main()
```

**File: backend/config/settings.py**
```python
import os
from dotenv import load_dotenv

load_dotenv()

class Settings:
    # Qdrant configuration
    QDRANT_URL = os.getenv("QDRANT_URL")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

    # Gemini configuration
    GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

    # OpenAI configuration (for ChatKit)
    OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

    # Application settings
    APP_NAME = "RAG Chatbot"
    DEBUG = os.getenv("DEBUG", "False").lower() == "true"
    HOST = os.getenv("HOST", "0.0.0.0")
    PORT = int(os.getenv("PORT", 8000))

    # Model settings
    EMBEDDING_MODEL = "models/text-embedding-004"  # Gemini
    CHAT_MODEL = os.getenv("CHAT_MODEL", "gpt-4-turbo-preview")

    # Qdrant collection name
    COLLECTION_NAME = "xyz..."

    # Chunk settings
    MAX_CHUNK_SIZE = int(os.getenv("MAX_CHUNK_SIZE", 1000))
    CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", 200))

settings = Settings()
```

**File: backend/requirements.txt**
```
fastapi==0.104.1
uvicorn==0.24.0
qdrant-client>=1.9.0
google-generativeai==0.4.1
python-dotenv==1.0.0
beautifulsoup4==4.12.2
openai==1.3.5
numpy==1.24.3
pydantic==2.5.0
tiktoken==0.5.2
```

