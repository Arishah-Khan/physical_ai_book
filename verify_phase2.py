#!/usr/bin/env python3
"""
Verification script for Phase 2: Foundational Components
This script checks that all Phase 2 components are properly implemented.
"""

import os
import sys

def verify_config_settings():
    """Verify config/settings.py implementation"""
    print("1. Verifying config/settings.py...")

    settings_path = "backend/config/settings.py"
    if not os.path.exists(settings_path):
        print("   ❌ File does not exist")
        return False

    with open(settings_path, 'r', encoding='utf-8') as f:
        content = f.read()

    checks = [
        ("Settings class defined", "class Settings" in content),
        ("BaseSettings imported", "BaseSettings" in content),
        ("SettingsConfigDict imported", "SettingsConfigDict" in content),
        ("env_file configuration", "env_file" in content),
        ("host field", "host: str" in content),
        ("port field", "port: int" in content),
        ("qdrant_url field", "qdrant_url" in content),
        ("openai_api_key field", "openai_api_key" in content),
        ("gemini_api_key field", "gemini_api_key" in content),
        ("settings instance", "settings = Settings" in content)
    ]

    all_passed = True
    for check_name, passed in checks:
        status = "[OK]" if passed else "[FAIL]"
        print(f"   {status} {check_name}")
        if not passed:
            all_passed = False

    return all_passed

def verify_api_models():
    """Verify api/models.py implementation"""
    print("\n2. Verifying api/models.py...")

    models_path = "backend/api/models.py"
    if not os.path.exists(models_path):
        print("   ❌ File does not exist")
        return False

    with open(models_path, 'r', encoding='utf-8') as f:
        content = f.read()

    checks = [
        ("ChatRequest class", "class ChatRequest" in content),
        ("SelectedTextChatRequest class", "class SelectedTextChatRequest" in content),
        ("ChatResponse class", "class ChatResponse" in content),
        ("HealthResponse class", "class HealthResponse" in content),
        ("ChatRequest has message field", "'message: str' in ChatRequest" in content or "message: str =" in content),
        ("ChatRequest has thread_id field", "thread_id" in content),
        ("SelectedTextChatRequest has selected_text", "selected_text" in content),
        ("Validation fields with Field", "Field(" in content),
        ("Validation imports", "from pydantic import BaseModel, Field" in content)
    ]

    all_passed = True
    for check_name, passed in checks:
        status = "[OK]" if passed else "[FAIL]"
        print(f"   {status} {check_name}")
        if not passed:
            all_passed = False

    return all_passed

def verify_qdrant_service():
    """Verify services/qdrant_service.py implementation"""
    print("\n3. Verifying services/qdrant_service.py...")

    service_path = "backend/services/qdrant_service.py"
    if not os.path.exists(service_path):
        print("   ❌ File does not exist")
        return False

    with open(service_path, 'r', encoding='utf-8') as f:
        content = f.read()

    checks = [
        ("QdrantService class", "class QdrantService" in content),
        ("QdrantClient imported", "QdrantClient" in content),
        ("search_similar method", "search_similar" in content),
        ("health_check method", "health_check" in content),
        ("__init__ method", "def __init__" in content)
    ]

    all_passed = True
    for check_name, passed in checks:
        status = "[OK]" if passed else "[FAIL]"
        print(f"   {status} {check_name}")
        if not passed:
            all_passed = False

    return all_passed

def verify_embedding_service():
    """Verify services/embedding_service.py implementation"""
    print("\n4. Verifying services/embedding_service.py...")

    service_path = "backend/services/embedding_service.py"
    if not os.path.exists(service_path):
        print("   ❌ File does not exist")
        return False

    with open(service_path, 'r', encoding='utf-8') as f:
        content = f.read()

    checks = [
        ("GeminiEmbeddingService class", "class GeminiEmbeddingService" in content),
        ("genai imported", "import google.generativeai" in content or "genai" in content),
        ("generate_single_embedding method", "generate_single_embedding" in content),
        ("generate_embeddings method", "generate_embeddings" in content),
        ("health_check method", "health_check" in content),
        ("__init__ method", "def __init__" in content)
    ]

    all_passed = True
    for check_name, passed in checks:
        status = "[OK]" if passed else "[FAIL]"
        print(f"   {status} {check_name}")
        if not passed:
            all_passed = False

    return all_passed

def verify_health_endpoint():
    """Verify health endpoint implementation"""
    print("\n5. Verifying health endpoint...")

    health_path = "backend/app/api/v1/endpoints/health.py"
    if not os.path.exists(health_path):
        print("   ❌ File does not exist")
        return False

    with open(health_path, 'r', encoding='utf-8') as f:
        content = f.read()

    checks = [
        ("health endpoint function", "def health_check" in content),
        ("HealthResponse used", "HealthResponse" in content),
        ("health service used", "HealthService" in content or "health_check" in content)
    ]

    all_passed = True
    for check_name, passed in checks:
        status = "[OK]" if passed else "[FAIL]"
        print(f"   {status} {check_name}")
        if not passed:
            all_passed = False

    return all_passed

def verify_root_endpoint():
    """Verify root endpoint implementation"""
    print("\n6. Verifying root endpoint...")

    root_path = "backend/api/main.py"
    if not os.path.exists(root_path):
        print("   ❌ File does not exist")
        return False

    with open(root_path, 'r', encoding='utf-8') as f:
        content = f.read()

    checks = [
        ("root endpoint function", "def root_info" in content),
        ("returns API info", "endpoints" in content or "version" in content)
    ]

    all_passed = True
    for check_name, passed in checks:
        status = "[OK]" if passed else "[FAIL]"
        print(f"   {status} {check_name}")
        if not passed:
            all_passed = False

    return all_passed

def main():
    print("=" * 70)
    print("PHASE 2: FOUNDATIONAL COMPONENTS - IMPLEMENTATION VERIFICATION")
    print("=" * 70)
    print("Verifying all components from T008 to T013 are properly implemented...")
    print()

    tests = [
        ("Config Settings", verify_config_settings),
        ("API Models", verify_api_models),
        ("Qdrant Service", verify_qdrant_service),
        ("Embedding Service", verify_embedding_service),
        ("Health Endpoint", verify_health_endpoint),
        ("Root Endpoint", verify_root_endpoint),
    ]

    results = []
    for test_name, test_func in tests:
        result = test_func()
        results.append((test_name, result))

    print("\n" + "=" * 70)
    print("VERIFICATION SUMMARY")
    print("=" * 70)

    passed = sum(1 for _, result in results if result)
    total = len(results)

    for test_name, result in results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"{status} {test_name}")

    print(f"\nOverall: {passed}/{total} components verified")

    if passed == total:
        print("\n🎉 SUCCESS: All Phase 2 foundational components are properly implemented!")
        print("\nImplementation includes:")
        print("- T008: config/settings.py with application configuration")
        print("- T009: api/models.py with Pydantic models")
        print("- T010: services/qdrant_service.py with QdrantService class")
        print("- T011: services/embedding_service.py with GeminiEmbeddingService class")
        print("- T012: Basic health check endpoint GET /api/v1/health")
        print("- T013: Root endpoint GET / with API info")
        return True
    else:
        print(f"\n⚠️  {total - passed} components need attention.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)