import google.generativeai as genai
from config.settings import settings

def list_available_models():
    """List all available models to find the correct embedding model"""
    try:
        genai.configure(api_key=settings.gemini_api_key)
        models = genai.list_models()
        print("Available models:")
        for model in models:
            print(f"- {model.name}")
            # Look for embedding models
            if 'embed' in model.name.lower() or 'embedding' in model.name.lower():
                print(f"  * Embedding model: {model.name}")
    except Exception as e:
        print(f"Error listing models: {e}")

if __name__ == "__main__":
    list_available_models()