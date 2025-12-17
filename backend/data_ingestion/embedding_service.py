import google.generativeai as genai
from typing import List, Dict, Any
from config.settings import settings
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class GeminiEmbeddingService:
    """
    Service for generating embeddings using Google Gemini.
    """

    def __init__(self):
        genai.configure(api_key=settings.GEMINI_API_KEY)
        self.model_name = "models/embedding-001"

    def get_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Google Gemini.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embedding vectors
        """
        embeddings = []

        # Process texts in batches to handle API limits
        for i in range(0, len(texts), settings.BATCH_SIZE):
            batch = texts[i:i + settings.BATCH_SIZE]

            try:
                # Use the embedding generation API
                result = genai.embed_content(
                    model=self.model_name,
                    content=batch,  # Can process multiple texts at once
                    task_type="retrieval_document"
                )

                # Extract embeddings from the result
                batch_embeddings = result['embedding']
                embeddings.extend(batch_embeddings)

                logger.info(f"Generated embeddings for batch of {len(batch)} texts")
            except Exception as e:
                logger.error(f"Error generating embeddings for batch: {str(e)}")
                # For each text in the failed batch, add a zero vector as fallback
                for _ in batch:
                    embeddings.append([0.0] * settings.EMBEDDING_DIMENSION)

        # Validate embedding dimensions
        for i, embedding in enumerate(embeddings):
            if len(embedding) != settings.EMBEDDING_DIMENSION:
                logger.warning(f"Embedding {i} has incorrect dimension: {len(embedding)}, expected {settings.EMBEDDING_DIMENSION}")
                # Pad or truncate to correct size
                if len(embedding) < settings.EMBEDDING_DIMENSION:
                    embedding.extend([0.0] * (settings.EMBEDDING_DIMENSION - len(embedding)))
                else:
                    embedding = embedding[:settings.EMBEDDING_DIMENSION]
                embeddings[i] = embedding

        return embeddings


class EmbeddingProcessor:
    """
    Processes chunks to add embeddings to them.
    """

    def __init__(self):
        self.embedding_service = GeminiEmbeddingService()

    def process_chunks_for_embedding(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Add embeddings to a list of chunks.

        Args:
            chunks: List of chunk dictionaries

        Returns:
            List of chunk dictionaries with embeddings added
        """
        if not chunks:
            logger.warning("No chunks to process for embeddings")
            return chunks

        logger.info(f"Processing {len(chunks)} chunks for embeddings")

        # Extract texts for embedding
        texts = [chunk["text"] for chunk in chunks]

        # Get embeddings
        embeddings = self.embedding_service.get_embeddings(texts)

        # Add embeddings to chunks
        processed_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_with_embedding = chunk.copy()
            chunk_with_embedding["embedding"] = embeddings[i]
            processed_chunks.append(chunk_with_embedding)

        logger.info(f"Successfully added embeddings to {len(processed_chunks)} chunks")
        return processed_chunks