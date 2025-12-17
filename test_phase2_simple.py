#!/usr/bin/env python3
"""
Simple test to verify Phase 2 foundational components are defined correctly.
"""

import sys
import os

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
        print(f"  X Config loading failed: {e}")
        import traceback
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
        print(f"  X Models test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run basic Phase 2 foundational components tests"""
    print("=" * 60)
    print("PHASE 2: FOUNDATIONAL COMPONENTS - QUICK VERIFICATION")
    print("=" * 60)

    print("\n1. Checking if required files exist...")
    required_files = [
        "backend/config/settings.py",
        "backend/api/models.py",
        "backend/services/qdrant_service.py",
        "backend/services/embedding_service.py"
    ]

    all_exist = True
    for file_path in required_files:
        if os.path.exists(file_path):
            print(f"  ✓ {file_path} exists")
        else:
            print(f"  X {file_path} missing")
            all_exist = False

    if not all_exist:
        print("\nSome required files are missing!")
        return False

    print("\n2. Testing components that don't require external services...")
    tests_passed = 0
    total_tests = 2

    if test_config_loading():
        tests_passed += 1

    if test_models():
        tests_passed += 1

    print(f"\nQuick verification: {tests_passed}/{total_tests} basic tests passed")

    if tests_passed == total_tests:
        print("\n🎉 Basic Phase 2 foundational components are properly defined!")
        print("\nNote: Full functionality tests require external services (Qdrant, OpenAI, Gemini).")
        return True
    else:
        print(f"\n⚠️  Some basic components need attention.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)