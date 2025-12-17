from pydantic_settings import SettingsConfigDict, BaseSettings
import os


class Settings(BaseSettings):
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore"
    )

    # API Configuration
    host: str = "0.0.0.0"
    port: int = 8000
    debug: bool = False
    chat_model: str = "gpt-4-turbo-preview"

    # Qdrant Configuration
    QDRANT_URL: str = ""
    QDRANT_API_KEY: str = ""
    COLLECTION_NAME: str = "documentation_chunks"

    # API Keys
    OPENAI_API_KEY: str = ""
    GEMINI_API_KEY: str = ""

    # Service Configuration
    EMBEDDING_MODEL: str = "text-embedding-004"
    EMBEDDING_DIMENSION: int = 1536  # Default for text-embedding-004

    # Processing Configuration
    BATCH_SIZE: int = 10
    DOCS_PATH: str = "./docs"
    MAX_CHUNK_SIZE: int = 1000
    CHUNK_OVERLAP: int = 100


# Create a single instance of settings
settings = Settings()