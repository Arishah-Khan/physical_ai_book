import google.generativeai as genai
from typing import List
from config.settings import settings
import logging

logger = logging.getLogger(__name__)


class GeminiEmbeddingService:
    def __init__(self):
        try:
            genai.configure(api_key=settings.GEMINI_API_KEY)
            # Using the correct format for Google's embedding models
            # The model name should be in the format "models/text-embedding-004"
            if not settings.EMBEDDING_MODEL.startswith("models/"):
                self.model = f"models/{settings.EMBEDDING_MODEL}"
            else:
                self.model = settings.EMBEDDING_MODEL
            # Test the API connection
            self.generate_single_embedding("test")
            logger.info("Successfully connected to Gemini embedding service")
        except Exception as e:
            logger.error(f"Failed to connect to Gemini embedding service: {str(e)}")
            raise

    def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for the provided text
        """
        if not text or len(text.strip()) == 0:
            raise ValueError("Text cannot be empty for embedding generation")

        try:
            result = genai.embed_content(
                model=self.model,
                content=[text],
                task_type="retrieval_document"
            )
            return result['embedding'][0]
        except Exception as e:
            logger.error(f"Error generating embedding: {str(e)}")
            raise ConnectionError(f"Gemini API error: {str(e)}")

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts
        """
        if not texts:
            raise ValueError("Texts list cannot be empty")

        try:
            result = genai.embed_content(
                model=self.model,
                content=texts,
                task_type="retrieval_document"
            )
            return result['embedding']
        except Exception as e:
            logger.error(f"Error generating embeddings: {str(e)}")
            raise ConnectionError(f"Gemini API error: {str(e)}")

    def health_check(self) -> bool:
        """
        Check if Gemini embedding service is available
        """
        try:
            # Test with a simple text
            self.generate_single_embedding("health check")
            return True
        except Exception as e:
            logger.error(f"Gemini embedding health check failed: {str(e)}")
            return False