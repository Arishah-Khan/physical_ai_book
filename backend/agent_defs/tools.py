from agents import function_tool, RunContextWrapper
import logging
from services.qdrant_service import QdrantService

from services.embedding_service import GeminiEmbeddingService

logger = logging.getLogger(__name__)

# Initialize services at module level
qdrant_service = QdrantService()  
embedding_service = GeminiEmbeddingService()

@function_tool
async def search_documentation(
    wrapper: RunContextWrapper[None],
    query: str,
    top_k: int = 5
) -> str:
    """Search for relevant documents in the vector database"""
    try:
        logger.info(f"Searching documentation with query: {query}")

        if not query or len(query.strip()) == 0:
            return "Error: Query cannot be empty"

        # Generate embedding
        query_embedding = embedding_service.generate_single_embedding(query)

        # Async search using updated QdrantService
        results = await qdrant_service.search_similar_async(query_embedding, limit=top_k)

        if not results:
            return "No relevant documents found for your query."

        formatted = []
        for idx, result in enumerate(results, 1):
            metadata_path = result['metadata'].get('relative_path', 'Unknown')
            formatted.append(
                f"[Source {idx}] (Score: {result['score']:.2f})\n"
                f"File: {metadata_path}\n"
                f"Content: {result['text']}\n"
            )

        logger.info(f"Found {len(results)} results for query: {query}")
        return "\n---\n".join(formatted)

    except ValueError as e:
        logger.error(f"Validation error: {e}")
        return f"Validation error: {str(e)}"
    except ConnectionError as e:
        logger.error(f"Connection error: {e}")
        return f"Service unavailable: {str(e)}"
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        return f"Error searching documentation: {str(e)}"

@function_tool
async def greet_user(wrapper: RunContextWrapper[None]) -> str:
    """Greet the user"""
    logger.info("Greeting user")
    return "Hello! I'm your documentation assistant. What would you like to know?"

@function_tool
async def search_qdrant(query: str) -> str:
    """Search relevant book content from Qdrant"""
    try:
        # Step 1: Get embedding
        query_vector = await embedding_service.create_query_embedding(query)

        # Step 2: Search Qdrant
        results = await qdrant_service.search_documentation(query_vector, limit=5)

        # Step 3: Format results
        result_strings = []
        for result in results:
            result_strings.append(
                f"Content: {result.get('text', '')}\nSource: {result.get('metadata', {}).get('source_file', 'Unknown')}"
            )

        return "\n\n".join(result_strings) if result_strings else "No relevant content found."
    except Exception as e:
        return f"Error searching Qdrant: {str(e)}"