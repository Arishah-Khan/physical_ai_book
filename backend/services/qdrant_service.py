from qdrant_client import QdrantClient
from qdrant_client.http.exceptions import UnexpectedResponse
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
from config.settings import settings
import logging

# Setup logger
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class QdrantService:
    def __init__(self):
        try:
            # Initialize synchronous client
            self.client = QdrantClient(
                url=settings.QDRANT_URL,
                api_key=settings.QDRANT_API_KEY,
                timeout=30
            )
            self.collection_name = settings.COLLECTION_NAME

            # Ensure collection exists
            try:
                self.client.get_collection(self.collection_name)
                logger.info(f"Connected to existing Qdrant collection: {self.collection_name}")
            except UnexpectedResponse:
                logger.info(f"Collection '{self.collection_name}' not found. Creating new collection...")
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=settings.EMBEDDING_DIMENSION,
                        distance=models.Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")

            # Initialize "async" client (pseudo-async)
            self.async_client = self.client  # Same client, await not needed

        except Exception as e:
            logger.error(f"Failed to initialize Qdrant client: {str(e)}")
            raise

    def search_similar(
        self,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: float = 0.0,
        filters: Optional[models.Filter] = None,
        with_vectors: bool = False
    ) -> List[Dict[str, Any]]:
        """Sync search for similar documents."""
        try:
            search_params = {
                "collection_name": self.collection_name,
                "query": query_vector,
                "limit": limit,
                "with_payload": True,
                "with_vectors": with_vectors
            }

            if filters is not None:
                search_params["query_filter"] = filters
            if score_threshold > 0:
                search_params["score_threshold"] = score_threshold

            search_result = self.client.query_points(**search_params)

            results = []
            for point in search_result.points:
                if score_threshold > 0 and (point.score is None or point.score < score_threshold):
                    continue
                results.append({
                    "id": str(point.id),
                    "text": point.payload.get("text", "") if point.payload else "",
                    "metadata": point.payload.get("metadata", {}) if point.payload else {},
                    "score": point.score,
                    "vector": point.vector if with_vectors and hasattr(point, 'vector') else None
                })
            return results

        except Exception as e:
            logger.error(f"Error searching in Qdrant: {str(e)}")
            raise ConnectionError(f"Vector database error: {str(e)}")

    async def search_similar_async(
        self,
        query_vector: List[float],
        limit: int = 5,
        score_threshold: float = 0.0,
        filters: Optional[models.Filter] = None,
        with_vectors: bool = False
    ) -> List[Dict[str, Any]]:
        """
        Async-like search (pseudo-async).
        Note: Qdrant sync client is used, so no real await needed.
        """
        # Just call sync method
        return self.search_similar(
            query_vector=query_vector,
            limit=limit,
            score_threshold=score_threshold,
            filters=filters,
            with_vectors=with_vectors
        )

    def get_document_by_id(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """Retrieve a document by ID"""
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[doc_id]
            )
            if records:
                record = records[0]
                return {
                    "id": str(record.id),
                    "text": record.payload.get("text", ""),
                    "metadata": record.payload.get("metadata", {})
                }
            return None
        except Exception as e:
            logger.error(f"Error retrieving document: {str(e)}")
            return None

    def health_check(self) -> bool:
        """Check Qdrant connection"""
        try:
            self.client.get_collection(self.collection_name)
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {str(e)}")
            return False
