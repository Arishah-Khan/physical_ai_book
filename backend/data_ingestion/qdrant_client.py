from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from config.settings import settings
import uuid
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QdrantVectorDB:
    """
    Handles interaction with Qdrant vector database.
    """

    def __init__(self):
        if settings.QDRANT_URL:
            # Connect to Qdrant Cloud
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=30  # Add timeout for requests
            )
        else:
            # Connect to local Qdrant (for development)
            self.client = QdrantClient(host="localhost", port=6333)

        self.collection_name = settings.COLLECTION_NAME

        # Initialize async client for async operations
        if settings.QDRANT_URL:
            self.async_client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=30,
                prefer_grpc=True  # Use gRPC for better async performance
            )
        else:
            self.async_client = QdrantClient(host="localhost", port=6333, prefer_grpc=True)

    def create_collection(self):
        """
        Create a collection in Qdrant with proper schema for documentation chunks.
        """
        try:
            # Check if collection already exists
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists with {collection_info.points_count} points")
            return
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(
                    size=settings.EMBEDDING_DIMENSION,
                    distance=models.Distance.COSINE
                )
            )
            logger.info(f"Created collection {self.collection_name} with {settings.EMBEDDING_DIMENSION}-dimension vectors")

    def batch_upload_chunks(self, chunks: List[Dict[str, Any]]):
        """
        Upload chunks to Qdrant in batches.

        Args:
            chunks: List of chunk dictionaries with text, metadata, and embeddings
        """
        if not chunks:
            logger.warning("No chunks to upload to Qdrant")
            return

        logger.info(f"Uploading {len(chunks)} chunks to Qdrant collection {self.collection_name}")

        # Prepare points for upload
        points = []
        for chunk in chunks:
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=chunk["embedding"],
                payload={
                    "text": chunk["text"],
                    "source_file": chunk["metadata"]["source_file"],
                    "relative_path": chunk["metadata"]["relative_path"],
                    "full_path": chunk["metadata"]["full_path"],
                    # Include any additional metadata from the original file
                    **{k: v for k, v in chunk["metadata"].items()
                       if k not in ["source_file", "relative_path", "full_path"]}
                }
            )
            points.append(point)

        # Upload in batches using the modern approach
        batch_size = settings.BATCH_SIZE
        total_uploaded = 0
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            # Use upsert with modern parameters
            self.client.upsert(
                collection_name=self.collection_name,
                points=batch,
                # Enable wait for consistency in production
                wait=True  # Ensures operation completes before returning
            )
            total_uploaded += len(batch)
            logger.info(f"Uploaded batch of {len(batch)} points ({total_uploaded}/{len(points)} total)")

        logger.info(f"Successfully uploaded all {len(chunks)} chunks to Qdrant")

    async def batch_upload_chunks_async(self, chunks: List[Dict[str, Any]]):
        """
        Asynchronously upload chunks to Qdrant in batches.

        Args:
            chunks: List of chunk dictionaries with text, metadata, and embeddings
        """
        if not chunks:
            logger.warning("No chunks to upload to Qdrant")
            return

        logger.info(f"Uploading {len(chunks)} chunks to Qdrant collection {self.collection_name} (async)")

        # Prepare points for upload
        points = []
        for chunk in chunks:
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=chunk["embedding"],
                payload={
                    "text": chunk["text"],
                    "source_file": chunk["metadata"]["source_file"],
                    "relative_path": chunk["metadata"]["relative_path"],
                    "full_path": chunk["metadata"]["full_path"],
                    # Include any additional metadata from the original file
                    **{k: v for k, v in chunk["metadata"].items()
                       if k not in ["source_file", "relative_path", "full_path"]}
                }
            )
            points.append(point)

        # Upload in batches using async client
        batch_size = settings.BATCH_SIZE
        total_uploaded = 0
        for i in range(0, len(points), batch_size):
            batch = points[i:i + batch_size]
            # Use async upsert with modern parameters
            await self.async_client.upsert(
                collection_name=self.collection_name,
                points=batch,
                wait=True  # Ensures operation completes before returning
            )
            total_uploaded += len(batch)
            logger.info(f"Uploaded batch of {len(batch)} points ({total_uploaded}/{len(points)} total) (async)")

        logger.info(f"Successfully uploaded all {len(chunks)} chunks to Qdrant (async)")

    def search_similar(self, query_embedding: List[float], limit: int = 5, score_threshold: float = 0.0,
                       query_filter: Optional[models.Filter] = None, with_vectors: bool = False) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in the vector database.

        Args:
            query_embedding: Embedding vector to search for
            limit: Number of similar chunks to return
            score_threshold: Minimum similarity score threshold (0.0 to 1.0 for cosine similarity)
            query_filter: Optional filter conditions to apply to the search
            with_vectors: Whether to return the vector embeddings with results

        Returns:
            List of similar chunks with text, metadata, and similarity scores
        """
        try:
            # Prepare search parameters using the modern approach
            search_params = {
                "collection_name": self.collection_name,
                "query": query_embedding,
                "limit": limit,
                "with_payload": True,
                "with_vectors": with_vectors
            }

            # Add score threshold if provided
            if score_threshold > 0:
                search_params["score_threshold"] = score_threshold

            # Add filters if provided
            if query_filter is not None:
                search_params["query_filter"] = query_filter

            # Use the modern query_points method (compatible with newer versions)
            search_result = self.client.query_points(**search_params)
            results = search_result.points

            return [
                {
                    "id": result.id,
                    "text": result.payload["text"],
                    "metadata": {
                        "source_file": result.payload.get("source_file", ""),
                        "relative_path": result.payload.get("relative_path", ""),
                        "full_path": result.payload.get("full_path", ""),
                        # Include any additional metadata from the payload
                        **{k: v for k, v in result.payload.items()
                           if k not in ["text", "source_file", "relative_path", "full_path"]}
                    },
                    "score": result.score,
                    **({"vector": result.vector} if with_vectors and result.vector else {})
                }
                for result in results
            ]
        except Exception as e:
            logger.error(f"Error performing similarity search: {str(e)}")
            return []

    async def search_similar_async(self, query_embedding: List[float], limit: int = 5, score_threshold: float = 0.0,
                                   query_filter: Optional[models.Filter] = None, with_vectors: bool = False) -> List[Dict[str, Any]]:
        """
        Asynchronously search for similar chunks in the vector database.

        Args:
            query_embedding: Embedding vector to search for
            limit: Number of similar chunks to return
            score_threshold: Minimum similarity score threshold (0.0 to 1.0 for cosine similarity)
            query_filter: Optional filter conditions to apply to the search
            with_vectors: Whether to return the vector embeddings with results

        Returns:
            List of similar chunks with text, metadata, and similarity scores
        """
        try:
            # Prepare search parameters using the modern approach
            search_params = {
                "collection_name": self.collection_name,
                "query": query_embedding,
                "limit": limit,
                "with_payload": True,
                "with_vectors": with_vectors
            }

            # Add score threshold if provided
            if score_threshold > 0:
                search_params["score_threshold"] = score_threshold

            # Add filters if provided
            if query_filter is not None:
                search_params["query_filter"] = query_filter

            # Use the async client with query_points method for better performance
            search_result = await self.async_client.query_points(**search_params)
            results = search_result.points

            return [
                {
                    "id": result.id,
                    "text": result.payload["text"],
                    "metadata": {
                        "source_file": result.payload.get("source_file", ""),
                        "relative_path": result.payload.get("relative_path", ""),
                        "full_path": result.payload.get("full_path", ""),
                        # Include any additional metadata from the payload
                        **{k: v for k, v in result.payload.items()
                           if k not in ["text", "source_file", "relative_path", "full_path"]}
                    },
                    "score": result.score,
                    **({"vector": result.vector} if with_vectors and result.vector else {})
                }
                for result in results
            ]
        except Exception as e:
            logger.error(f"Error performing async similarity search: {str(e)}")
            return []

    def get_total_points(self) -> int:
        """
        Get the total number of points (vectors) in the collection.

        Returns:
            Number of points in the collection
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return collection_info.points_count
        except Exception as e:
            logger.error(f"Error getting total points count: {str(e)}")
            return 0

    def test_connection(self) -> bool:
        """
        Test the connection to Qdrant.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            # Try to get collections list to test connection
            self.client.get_collections()
            logger.info("Successfully connected to Qdrant")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to Qdrant: {str(e)}")
            return False