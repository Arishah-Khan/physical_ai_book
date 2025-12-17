#!/usr/bin/env python3
"""
Usage example for the enhanced search_similar functionality
"""

# Example usage of the enhanced search_similar methods

# 1. Using QdrantService with enhanced features:

from services.qdrant_service import QdrantService
from qdrant_client.http.models import Filter, FieldCondition, MatchValue

# Initialize the service
# qdrant_service = QdrantService()  # Requires proper configuration

# Example 1: Basic search with score threshold
"""
results = qdrant_service.search_similar(
    query_vector=[0.1, 0.2, 0.3, 0.4, 0.5],
    limit=10,
    score_threshold=0.7  # Only return results with similarity score >= 0.7
)
"""

# Example 2: Search with metadata filters
"""
# Filter for documents from a specific source file
source_filter = Filter(
    must=[
        FieldCondition(
            key="metadata.source_file",
            match=MatchValue(value="document.pdf")
        )
    ]
)

results = qdrant_service.search_similar(
    query_vector=[0.1, 0.2, 0.3, 0.4, 0.5],
    limit=5,
    filters=source_filter,
    score_threshold=0.5
)
"""

# Example 3: Search that includes vector embeddings in results
"""
results = qdrant_service.search_similar(
    query_vector=[0.1, 0.2, 0.3, 0.4, 0.5],
    limit=3,
    with_vectors=True  # Include the vector embeddings in the results
)

# Each result will now have a 'vector' field containing the embedding
for result in results:
    print(f"Text: {result['text']}")
    print(f"Score: {result['score']}")
    print(f"Vector: {result['vector']}")  # Available when with_vectors=True
"""

# 2. Using QdrantVectorDB (data ingestion module) with enhanced features:

from data_ingestion.qdrant_client import QdrantVectorDB
from qdrant_client.http import models

# Example 4: Using QdrantVectorDB with filters
"""
qdrant_db = QdrantVectorDB()  # Requires proper configuration

# Create a filter for specific metadata
metadata_filter = models.Filter(
    must=[
        models.FieldCondition(
            key="source_file",
            match=models.MatchValue(value="important_document.txt")
        )
    ]
)

results = qdrant_db.search_similar(
    query_embedding=[0.1, 0.2, 0.3, 0.4, 0.5],
    limit=5,
    score_threshold=0.6,
    query_filter=metadata_filter
)
"""

# Example 5: Complex filter with multiple conditions
"""
complex_filter = models.Filter(
    must=[
        models.FieldCondition(
            key="source_file",
            match=models.MatchValue(value="document.pdf")
        )
    ],
    should=[
        models.FieldCondition(
            key="full_path",
            match=models.MatchText(text="important")
        )
    ]
)

results = qdrant_service.search_similar(
    query_vector=[0.1, 0.2, 0.3, 0.4, 0.5],
    limit=10,
    filters=complex_filter,
    score_threshold=0.4
)
"""

print("Enhanced search_similar functionality examples:")
print("1. Score threshold filtering - Only return results above a similarity threshold")
print("2. Metadata filtering - Filter results by specific metadata values")
print("3. Vector inclusion - Option to return vector embeddings with results")
print("4. Improved error handling - Better logging and error propagation")
print("5. Comprehensive metadata - Better extraction and organization of metadata fields")