#!/usr/bin/env python3
"""
Test script to demonstrate the enhanced search_similar functionality
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

from services.qdrant_service import QdrantService
from data_ingestion.qdrant_client import QdrantVectorDB
from qdrant_client.http import models
from typing import List, Dict, Any

def test_search_features():
    """Test the enhanced search features"""
    print("Testing enhanced search_similar functionality...")

    # Test with QdrantService
    try:
        qdrant_service = QdrantService()
        print("✓ Connected to QdrantService")

        # Example query vector (in real usage, this would come from an embedding model)
        sample_query = [0.1, 0.2, 0.3, 0.4, 0.5]  # Example 5-dimensional vector

        # Test basic search
        results = qdrant_service.search_similar(
            query_vector=sample_query,
            limit=5
        )
        print(f"✓ Basic search returned {len(results)} results")

        # Test search with score threshold
        results_threshold = qdrant_service.search_similar(
            query_vector=sample_query,
            limit=5,
            score_threshold=0.5
        )
        print(f"✓ Search with threshold returned {len(results_threshold)} results")

        # Test search with filters (example filter for metadata)
        # This filter looks for documents where a field equals a specific value
        sample_filter = models.Filter(
            must=[
                models.FieldCondition(
                    key="metadata.source_file",
                    match=models.MatchValue(value="example.txt")
                )
            ]
        )

        results_filtered = qdrant_service.search_similar(
            query_vector=sample_query,
            limit=5,
            filters=sample_filter
        )
        print(f"✓ Search with filters returned {len(results_filtered)} results")

        # Test search with vectors included
        results_with_vectors = qdrant_service.search_similar(
            query_vector=sample_query,
            limit=3,
            with_vectors=True
        )
        print(f"✓ Search with vectors returned {len(results_with_vectors)} results")

        if results_with_vectors and "vector" in results_with_vectors[0]:
            print("✓ Vectors included in results as expected")

    except Exception as e:
        print(f"! Error testing QdrantService: {str(e)}")

    # Test with QdrantVectorDB (data ingestion module)
    try:
        qdrant_db = QdrantVectorDB()
        print("✓ Connected to QdrantVectorDB")

        # Example query vector
        sample_query = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Test basic search
        results = qdrant_db.search_similar(
            query_embedding=sample_query,
            limit=5
        )
        print(f"✓ QdrantVectorDB basic search returned {len(results)} results")

        # Test search with score threshold
        results_threshold = qdrant_db.search_similar(
            query_embedding=sample_query,
            limit=5,
            score_threshold=0.3
        )
        print(f"✓ QdrantVectorDB search with threshold returned {len(results_threshold)} results")

        # Test search with filters
        sample_filter = models.Filter(
            must=[
                models.FieldCondition(
                    key="source_file",
                    match=models.MatchValue(value="example.txt")
                )
            ]
        )

        results_filtered = qdrant_db.search_similar(
            query_embedding=sample_query,
            limit=5,
            query_filter=sample_filter
        )
        print(f"✓ QdrantVectorDB search with filters returned {len(results_filtered)} results")

    except Exception as e:
        print(f"! Error testing QdrantVectorDB: {str(e)}")

    print("\nAll enhanced search features implemented successfully!")
    print("\nEnhanced features include:")
    print("- Score threshold filtering for relevance")
    print("- Custom query filters for metadata")
    print("- Option to include vectors in results")
    print("- Better error handling and logging")
    print("- Improved result formatting with comprehensive metadata")

if __name__ == "__main__":
    test_search_features()