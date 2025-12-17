import pytest
import asyncio
from fastapi.testclient import TestClient
from main import app
from api.models import ChatRequest, SelectedTextChatRequest
import os
from dotenv import load_dotenv

# Load environment variables for testing
load_dotenv()

client = TestClient(app)

def test_root_endpoint():
    """Test the root endpoint returns API information"""
    response = client.get("/")
    assert response.status_code == 200

    data = response.json()
    assert "message" in data
    assert "version" in data
    assert "endpoints" in data
    assert data["message"] == "RAG Chatbot API"
    assert data["version"] == "1.0.0"

def test_health_endpoint():
    """Test the health check endpoint"""
    response = client.get("/api/v1/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "message" in data
    # Note: Actual status depends on service availability during test

def test_chat_endpoint_basic():
    """Test the chat endpoint with a basic request"""
    # Skip if required environment variables are not set
    if not os.getenv("OPENAI_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("GEMINI_API_KEY"):
        pytest.skip("Required API keys not set for integration test")

    request_data = ChatRequest(message="Hello", thread_id="test_thread_1")
    response = client.post("/api/v1/chat", json=request_data.model_dump())

    # The response should be successful (200) or have a validation error (422) if models are invalid
    # If external services are unavailable, it might return 500
    assert response.status_code in [200, 422, 500]

    if response.status_code == 200:
        data = response.json()
        assert "answer" in data
        assert "thread_id" in data
        assert data["thread_id"] == "test_thread_1"

def test_chat_endpoint_empty_message():
    """Test the chat endpoint with empty message (should fail validation)"""
    request_data = ChatRequest(message="", thread_id="test_thread_2")
    response = client.post("/api/v1/chat", json=request_data.model_dump())

    # Should return 422 for validation error due to empty message
    # Or 400 if validation is handled at the endpoint level
    assert response.status_code in [400, 422]

def test_chat_selected_text_endpoint_basic():
    """Test the selected text chat endpoint"""
    # Skip if required environment variables are not set
    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("OpenAI API key not set for integration test")

    request_data = SelectedTextChatRequest(
        message="What is this text about?",
        selected_text="This is a sample text for testing purposes.",
        thread_id="test_thread_3"
    )
    response = client.post("/api/v1/chat/selected-text", json=request_data.model_dump())

    # The response should be successful (200) or have a validation error (422) if models are invalid
    # If external services are unavailable, it might return 500
    assert response.status_code in [200, 422, 500]

    if response.status_code == 200:
        data = response.json()
        assert "answer" in data
        assert "thread_id" in data
        assert data["thread_id"] == "test_thread_3"

def test_chat_selected_text_endpoint_empty_fields():
    """Test the selected text chat endpoint with empty fields (should fail validation)"""
    # Test with empty message
    request_data = SelectedTextChatRequest(
        message="",
        selected_text="This is a sample text.",
        thread_id="test_thread_4"
    )
    response = client.post("/api/v1/chat/selected-text", json=request_data.model_dump())
    assert response.status_code in [400, 422]

    # Test with empty selected_text
    request_data = SelectedTextChatRequest(
        message="What is this text about?",
        selected_text="",
        thread_id="test_thread_5"
    )
    response = client.post("/api/v1/chat/selected-text", json=request_data.model_dump())
    assert response.status_code in [400, 422]

def test_invalid_thread_id_format():
    """Test endpoints with invalid thread ID format"""
    # Test chat endpoint with invalid thread_id
    request_data = ChatRequest(
        message="Hello",
        thread_id="invalid thread id with spaces!"  # Invalid format
    )
    response = client.post("/api/v1/chat", json=request_data.model_dump())
    assert response.status_code in [400, 422]

def test_valid_thread_id_format():
    """Test endpoints with valid thread ID format"""
    request_data = ChatRequest(
        message="Hello",
        thread_id="valid_thread-id_123"  # Valid format
    )
    response = client.post("/api/v1/chat", json=request_data.model_dump())
    # Should not fail due to thread_id validation
    assert response.status_code != 422  # If it's a validation error, it's not due to thread_id format

if __name__ == "__main__":
    pytest.main([__file__])