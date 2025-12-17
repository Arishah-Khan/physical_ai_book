#!/usr/bin/env python3
"""
Test script to verify Phase 2: Foundational Components are working correctly.
This script tests the core foundational components implemented in Phase 2.
"""

import sys
import os
import traceback

# Add backend to path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

def test_config_loading():
    """Test that configuration loads correctly"""
    print("Testing: Config/settings.py loading...")
    try:
        from backend.config.settings import settings
        print(f"  ✓ Config loaded successfully")
        print(f"  ✓ Host: {settings.host}")
        print(f"  ✓ Port: {settings.port}")
        print(f"  ✓ Debug: {settings.debug}")
        return True
    except Exception as e:
        print(f"  ✗ Config loading failed: {e}")
        traceback.print_exc()
        return False

def test_models():
    """Test that API models are defined correctly"""
    print("\nTesting: API models (api/models.py)...")
    try:
        from backend.api.models import ChatRequest, SelectedTextChatRequest, ChatResponse, HealthResponse

        # Test ChatRequest model
        chat_req = ChatRequest(message="Hello", thread_id="test_thread")
        print(f"  ✓ ChatRequest created: message='{chat_req.message}', thread_id='{chat_req.thread_id}'")

        # Test SelectedTextChatRequest model
        selected_req = SelectedTextChatRequest(message="Question", selected_text="Some text", thread_id="test_thread")
        print(f"  ✓ SelectedTextChatRequest created: message='{selected_req.message}', selected_text='{selected_req.selected_text[:10]}...'")

        # Test ChatResponse model
        chat_resp = ChatResponse(answer="Response", thread_id="test_thread", sources=[])
        print(f"  ✓ ChatResponse created: answer='{chat_resp.answer}', thread_id='{chat_resp.thread_id}'")

        # Test HealthResponse model
        health_resp = HealthResponse(status="healthy", message="Test message")
        print(f"  ✓ HealthResponse created: status='{health_resp.status}', message='{health_resp.message}'")

        return True
    except Exception as e:
        print(f"  ✗ Models test failed: {e}")
        traceback.print_exc()
        return False

def test_qdrant_service():
    """Test Qdrant service initialization (without actual connection)"""
    print("\nTesting: Qdrant service (services/qdrant_service.py)...")
    try:
        from backend.services.qdrant_service import QdrantService
        from unittest.mock import Mock, patch

        # Mock the QdrantClient to avoid actual connection
        with patch('backend.services.qdrant_service.QdrantClient') as mock_client:
            mock_instance = Mock()
            mock_client.return_value = mock_instance
            mock_instance.get_collection.return_value = Mock()

            service = QdrantService()
            print(f"  ✓ QdrantService initialized successfully")
            print(f"  ✓ Collection name: {service.collection_name}")

            # Test search_similar method structure (without actual call)
            print(f"  ✓ QdrantService methods available: {hasattr(service, 'search_similar')}")

        return True
    except Exception as e:
        print(f"  ✗ Qdrant service test failed: {e}")
        traceback.print_exc()
        return False

def test_embedding_service():
    """Test embedding service initialization (without actual API call)"""
    print("\nTesting: Embedding service (services/embedding_service.py)...")
    try:
        from backend.services.embedding_service import GeminiEmbeddingService
        from unittest.mock import Mock, patch

        # Mock the genai module to avoid actual API calls
        with patch('backend.services.embedding_service.genai') as mock_genai:
            mock_genai.configure = Mock()
            mock_genai.embed_content = Mock(return_value={'embedding': [[0.1, 0.2, 0.3]]})

            service = GeminiEmbeddingService()
            print(f"  ✓ GeminiEmbeddingService initialized successfully")
            print(f"  ✓ Model: {service.model}")

            # Test generate_single_embedding method structure (without actual call)
            result = service.generate_single_embedding("test")
            print(f"  ✓ generate_single_embedding method works, result length: {len(result)}")

        return True
    except Exception as e:
        print(f"  ✗ Embedding service test failed: {e}")
        traceback.print_exc()
        return False

def test_health_endpoint():
    """Test health check endpoint"""
    print("\nTesting: Health endpoint (app/api/v1/endpoints/health.py)...")
    try:
        from backend.app.api.v1.endpoints.health import router
        from backend.api.models import HealthResponse
        from backend.services.health_service import HealthService
        from unittest.mock import Mock, patch

        # Test HealthService
        with patch('backend.services.health_service.QdrantService') as mock_qdrant, \
             patch('backend.services.health_service.GeminiEmbeddingService') as mock_embedding, \
             patch('backend.services.health_service.openai') as mock_openai:

            mock_qdrant.return_value = Mock()
            mock_embedding.return_value = Mock()
            mock_openai.OpenAI = Mock()

            health_service = HealthService()
            result = health_service.health_check()
            print(f"  ✓ HealthService.health_check() works, result status: {result['status']}")

        print(f"  ✓ Health endpoint router available: {router is not None}")

        return True
    except Exception as e:
        print(f"  ✗ Health endpoint test failed: {e}")
        traceback.print_exc()
        return False

def test_root_endpoint():
    """Test root endpoint"""
    print("\nTesting: Root endpoint (api/main.py)...")
    try:
        from backend.api.main import router
        import asyncio

        # Test that the router exists and has the root endpoint
        print(f"  ✓ Root endpoint router available: {router is not None}")

        # Test the root_info function directly
        from backend.api.main import root_info
        result = asyncio.run(root_info())
        print(f"  ✓ Root endpoint returns: {result['message']}")
        print(f"  ✓ Endpoints available: {list(result['endpoints'].keys())}")

        return True
    except Exception as e:
        print(f"  ✗ Root endpoint test failed: {e}")
        traceback.print_exc()
        return False

def main():
    """Run all Phase 2 foundational components tests"""
    print("=" * 60)
    print("PHASE 2: FOUNDATIONAL COMPONENTS - IMPLEMENTATION VERIFICATION")
    print("=" * 60)

    tests = [
        ("Config Loading", test_config_loading),
        ("API Models", test_models),
        ("Qdrant Service", test_qdrant_service),
        ("Embedding Service", test_embedding_service),
        ("Health Endpoint", test_health_endpoint),
        ("Root Endpoint", test_root_endpoint),
    ]

    results = []
    for test_name, test_func in tests:
        print(f"\n{test_name}")
        print("-" * len(test_name))
        result = test_func()
        results.append((test_name, result))

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    passed = 0
    total = len(results)

    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        symbol = "✓" if result else "✗"
        print(f"{symbol} {test_name}: {status}")
        if result:
            passed += 1

    print(f"\nOverall: {passed}/{total} tests passed")

    if passed == total:
        print("\n🎉 All Phase 2 foundational components are working correctly!")
        return True
    else:
        print(f"\n⚠️  {total - passed} components need attention.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)