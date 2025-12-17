from typing import Dict, Any
from services.qdrant_service import QdrantService
from services.embedding_service import GeminiEmbeddingService
from config.settings import settings
import openai
import logging

logger = logging.getLogger(__name__)


class HealthService:
    def __init__(self):
        self.qdrant_service = QdrantService()
        self.embedding_service = GeminiEmbeddingService()

    def health_check(self) -> Dict[str, Any]:
        """
        Perform health check for all services
        """
        try:
            # Test Qdrant connection
            qdrant_healthy = self.qdrant_service.health_check()

            # Test Gemini connection
            gemini_healthy = self.embedding_service.health_check()

            # Test OpenAI connection
            try:
                client = openai.OpenAI(api_key=settings.OPENAI_API_KEY)
                # Make a simple API call to test connection
                client.models.list()
                openai_healthy = True
            except Exception as e:
                logger.error(f"OpenAI connection test failed: {str(e)}")
                openai_healthy = False

            # Overall status
            all_healthy = qdrant_healthy and gemini_healthy and openai_healthy
            status = "healthy" if all_healthy else "degraded"

            return {
                "status": status,
                "services": {
                    "qdrant": "healthy" if qdrant_healthy else "unhealthy",
                    "gemini": "healthy" if gemini_healthy else "unhealthy",
                    "openai_agents": "healthy" if openai_healthy else "unhealthy"
                },
                "timestamp": __import__('datetime').datetime.utcnow().isoformat()
            }
        except Exception as e:
            logger.error(f"Health check failed: {str(e)}")
            return {
                "status": "unhealthy",
                "services": {
                    "qdrant": "unknown",
                    "gemini": "unknown",
                    "openai_agents": "unknown"
                },
                "timestamp": __import__('datetime').datetime.utcnow().isoformat(),
                "error": str(e)
            }