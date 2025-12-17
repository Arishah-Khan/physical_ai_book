import pytest
import unittest.mock as mock
from unittest.mock import Mock, patch
from services.qdrant_service import QdrantService
from services.embedding_service import GeminiEmbeddingService
from services.health_service import HealthService
from config.settings import settings


def test_qdrant_service_initialization():
    """Test QdrantService initialization"""
    # Mock the QdrantClient and its methods
    with patch('services.qdrant_service.QdrantClient') as mock_client:
        mock_instance = Mock()
        mock_client.return_value = mock_instance
        mock_instance.get_collection.return_value = Mock()

        service = QdrantService()

        assert service.client == mock_instance
        assert service.collection_name == settings.collection_name
        mock_client.assert_called_once()
        mock_instance.get_collection.assert_called_once_with(settings.collection_name)


def test_qdrant_service_search_similar():
    """Test QdrantService search_similar method"""
    with patch('services.qdrant_service.QdrantClient') as mock_client:
        mock_instance = Mock()
        mock_client.return_value = mock_instance

        # Mock search result
        mock_search_result = [
            Mock(id="1", payload={"text": "test text", "metadata": {"key": "value"}}, score=0.9),
            Mock(id="2", payload={"text": "another text", "metadata": {"key2": "value2"}}, score=0.8)
        ]
        mock_instance.search.return_value = mock_search_result

        service = QdrantService()
        service.client = mock_instance

        result = service.search_similar([0.1, 0.2, 0.3], limit=2)

        # Verify the result structure
        assert len(result) == 2
        assert result[0]["id"] == "1"
        assert result[0]["text"] == "test text"
        assert result[0]["score"] == 0.9
        assert result[1]["id"] == "2"
        assert result[1]["text"] == "another text"
        assert result[1]["score"] == 0.8

        # Verify the search was called with correct parameters
        mock_instance.search.assert_called_once_with(
            collection_name=service.collection_name,
            query_vector=[0.1, 0.2, 0.3],
            limit=2,
            with_payload=True,
            with_vectors=False
        )


def test_qdrant_service_health_check():
    """Test QdrantService health_check method"""
    with patch('services.qdrant_service.QdrantClient') as mock_client:
        mock_instance = Mock()
        mock_client.return_value = mock_instance
        mock_instance.get_collection.return_value = Mock()

        service = QdrantService()
        service.client = mock_instance

        result = service.health_check()
        assert result is True

        # Test failure case
        mock_instance.get_collection.side_effect = Exception("Connection failed")
        result = service.health_check()
        assert result is False


def test_gemini_embedding_service_initialization():
    """Test GeminiEmbeddingService initialization"""
    with patch('services.embedding_service.genai') as mock_genai:
        mock_genai.configure = Mock()
        mock_genai.embed_content = Mock(return_value={'embedding': [[0.1, 0.2, 0.3]]})

        service = GeminiEmbeddingService()

        mock_genai.configure.assert_called_once_with(api_key=settings.gemini_api_key)


def test_gemini_embedding_service_generate_single_embedding():
    """Test GeminiEmbeddingService generate_single_embedding method"""
    with patch('services.embedding_service.genai') as mock_genai:
        mock_genai.configure = Mock()
        mock_genai.embed_content = Mock(return_value={'embedding': [[0.1, 0.2, 0.3]]})

        service = GeminiEmbeddingService()

        result = service.generate_single_embedding("test text")
        assert result == [0.1, 0.2, 0.3]

        mock_genai.embed_content.assert_called_once_with(
            model=service.model,
            content=["test text"],
            task_type="retrieval_document"
        )


def test_gemini_embedding_service_generate_embeddings():
    """Test GeminiEmbeddingService generate_embeddings method"""
    with patch('services.embedding_service.genai') as mock_genai:
        mock_genai.configure = Mock()
        mock_genai.embed_content = Mock(return_value={'embedding': [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]})

        service = GeminiEmbeddingService()

        result = service.generate_embeddings(["text1", "text2"])
        assert result == [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

        mock_genai.embed_content.assert_called_once_with(
            model=service.model,
            content=["text1", "text2"],
            task_type="retrieval_document"
        )


def test_gemini_embedding_service_health_check():
    """Test GeminiEmbeddingService health_check method"""
    with patch('services.embedding_service.genai') as mock_genai:
        mock_genai.configure = Mock()
        mock_genai.embed_content = Mock(return_value={'embedding': [[0.1, 0.2, 0.3]]})

        service = GeminiEmbeddingService()

        result = service.health_check()
        assert result is True

        # Test failure case
        mock_genai.embed_content.side_effect = Exception("API error")
        result = service.health_check()
        assert result is False


def test_health_service_initialization():
    """Test HealthService initialization"""
    with patch('services.health_service.QdrantService'), \
         patch('services.health_service.GeminiEmbeddingService'):

        service = HealthService()

        assert service.qdrant_service is not None
        assert service.embedding_service is not None


def test_health_service_health_check():
    """Test HealthService health_check method"""
    with patch('services.health_service.QdrantService') as mock_qdrant_service, \
         patch('services.health_service.GeminiEmbeddingService') as mock_embedding_service, \
         patch('services.health_service.openai') as mock_openai:

        # Mock services
        mock_qdrant = Mock()
        mock_qdrant.health_check.return_value = True
        mock_qdrant_service.return_value = mock_qdrant

        mock_embedding = Mock()
        mock_embedding.health_check.return_value = True
        mock_embedding_service.return_value = mock_embedding

        mock_openai.OpenAI.return_value.models.list.return_value = Mock()

        service = HealthService()
        result = service.health_check()

        assert result["status"] in ["healthy", "degraded"]
        assert "qdrant" in result["services"]
        assert "gemini" in result["services"]
        assert "openai_agents" in result["services"]

        # Verify method calls
        mock_qdrant.health_check.assert_called_once()
        mock_embedding.health_check.assert_called_once()


if __name__ == "__main__":
    pytest.main([__file__])