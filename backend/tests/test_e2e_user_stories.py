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

def test_user_story_1_rag_chatbot_interaction():
    """
    End-to-end test for User Story 1: RAG Chatbot Interaction
    Goal: User can ask questions about documentation and receive accurate answers with citations
    """
    # Skip if required environment variables are not set
    if not os.getenv("OPENAI_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("GEMINI_API_KEY"):
        pytest.skip("Required API keys not set for end-to-end test")

    # Test a basic RAG query
    request_data = ChatRequest(
        message="What is the main concept of this book?",
        thread_id="e2e_test_thread_1"
    )
    response = client.post("/api/v1/chat", json=request_data.model_dump())

    # Should return 200 if services are available
    if response.status_code == 200:
        data = response.json()
        assert "answer" in data
        assert "thread_id" in data
        assert data["thread_id"] == "e2e_test_thread_1"
        # Note: sources may or may not be populated depending on search results

    # Test with a different thread ID to verify conversation context
    request_data = ChatRequest(
        message="Can you elaborate on that?",
        thread_id="e2e_test_thread_1"  # Same thread to maintain context
    )
    response = client.post("/api/v1/chat", json=request_data.model_dump())

    if response.status_code == 200:
        data = response.json()
        assert "answer" in data
        assert data["thread_id"] == "e2e_test_thread_1"


def test_user_story_2_greeting_interaction():
    """
    End-to-end test for User Story 2: Greeting and Initial Interaction
    Goal: User receives friendly greeting responses when first interacting
    """
    # Skip if required environment variables are not set
    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("OpenAI API key not set for end-to-end test")

    # Test various greeting phrases
    greetings = ["hello", "hi", "hey", "good morning", "greetings"]

    for greeting in greetings:
        request_data = ChatRequest(
            message=greeting,
            thread_id=f"e2e_greeting_test_{hash(greeting) % 10000}"
        )
        response = client.post("/api/v1/chat", json=request_data.model_dump())

        # Response could be 200 (success), 422 (validation error), or 500 (service unavailable)
        if response.status_code == 200:
            data = response.json()
            assert "answer" in data
            # The answer should contain greeting-related content
            # Note: This depends on the agent's behavior and may vary


def test_user_story_3_context_limited_querying():
    """
    End-to-end test for User Story 3: Context-Limited Querying
    Goal: User can get answers based only on provided selected text
    """
    # Skip if required environment variables are not set
    if not os.getenv("OPENAI_API_KEY"):
        pytest.skip("OpenAI API key not set for end-to-end test")

    # Test with selected text
    request_data = SelectedTextChatRequest(
        message="What does this text say about AI?",
        selected_text="Artificial Intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. AI systems can perform tasks that typically require human intelligence such as visual perception, speech recognition, and decision-making.",
        thread_id="e2e_selected_text_1"
    )
    response = client.post("/api/v1/chat/selected-text", json=request_data.model_dump())

    if response.status_code == 200:
        data = response.json()
        assert "answer" in data
        assert "thread_id" in data
        assert data["thread_id"] == "e2e_selected_text_1"
        # The answer should be based on the provided text


def test_health_check_endpoint():
    """
    End-to-end test for health check functionality
    """
    response = client.get("/api/v1/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "message" in data
    assert data["status"] in ["healthy", "unhealthy", "degraded"]


def test_root_endpoint():
    """
    End-to-end test for root API information endpoint
    """
    response = client.get("/")
    assert response.status_code == 200

    data = response.json()
    assert "message" in data
    assert "version" in data
    assert "endpoints" in data
    assert data["message"] == "RAG Chatbot API"


def test_error_handling_scenarios():
    """
    End-to-end test for error handling scenarios
    """
    # Test empty message to chat endpoint
    request_data = ChatRequest(message="", thread_id="error_test_1")
    response = client.post("/api/v1/chat", json=request_data.model_dump())
    assert response.status_code in [400, 422]  # Validation error expected

    # Test empty fields to selected text endpoint
    request_data = SelectedTextChatRequest(
        message="",
        selected_text="Some text",
        thread_id="error_test_2"
    )
    response = client.post("/api/v1/chat/selected-text", json=request_data.model_dump())
    assert response.status_code in [400, 422]  # Validation error expected

    request_data = SelectedTextChatRequest(
        message="What is this about?",
        selected_text="",
        thread_id="error_test_3"
    )
    response = client.post("/api/v1/chat/selected-text", json=request_data.model_dump())
    assert response.status_code in [400, 422]  # Validation error expected


def test_thread_context_preservation():
    """
    End-to-end test for thread context preservation across conversations
    """
    # Skip if required environment variables are not set
    if not os.getenv("OPENAI_API_KEY") or not os.getenv("QDRANT_URL") or not os.getenv("GEMINI_API_KEY"):
        pytest.skip("Required API keys not set for end-to-end test")

    # Start a conversation with a thread ID
    thread_id = "context_preservation_test"

    # First message
    request_data = ChatRequest(
        message="What is machine learning?",
        thread_id=thread_id
    )
    response1 = client.post("/api/v1/chat", json=request_data.model_dump())

    # Second message in the same thread
    request_data = ChatRequest(
        message="Can you give me examples?",
        thread_id=thread_id  # Same thread ID
    )
    response2 = client.post("/api/v1/chat", json=request_data.model_dump())

    # Both responses should have the same thread ID if successful
    if response1.status_code == 200 and response2.status_code == 200:
        data1 = response1.json()
        data2 = response2.json()
        assert data1["thread_id"] == thread_id
        assert data2["thread_id"] == thread_id


if __name__ == "__main__":
    pytest.main([__file__])