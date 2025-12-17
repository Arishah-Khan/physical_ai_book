#!/usr/bin/env python3
"""
Main ingestion script that connects all components of the data pipeline.
Implements the complete workflow from markdown reading to Qdrant storage.
"""

import os
import sys
from typing import List, Dict, Any
import logging

# Add the backend directory to the path so imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from data_ingestion.markdown_reader import MarkdownReader
from data_ingestion.text_converter import MarkdownToTextConverter
from data_ingestion.chunker import TextChunker
from data_ingestion.embedding_service import EmbeddingProcessor
from data_ingestion.qdrant_client import QdrantVectorDB
from config.settings import settings

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def main():
    """
    Main ingestion workflow that connects all components.
    Implements progress feedback as required by FR-010 and continues processing
    when individual files have issues as required by FR-011.
    """
    logger.info("Starting documentation ingestion process...")

    try:
        # Initialize components
        reader = MarkdownReader(docs_path=settings.DOCS_PATH)
        chunker = TextChunker(
            max_chunk_size=settings.MAX_CHUNK_SIZE,
            overlap=settings.CHUNK_OVERLAP
        )
        embedding_processor = EmbeddingProcessor()
        vector_db = QdrantVectorDB()

        # Test Qdrant connection before proceeding
        logger.info("Testing Qdrant connection...")
        if not vector_db.test_connection():
            logger.error("Cannot connect to Qdrant. Please check your configuration.")
            sys.exit(1)

        # Step 1: Read all markdown files
        logger.info(f"Reading markdown files from {settings.DOCS_PATH}...")
        files_data = reader.read_all_markdown_files()
        logger.info(f"Found {len(files_data)} files")

        # Step 2: Process each file with progress feedback (FR-010)
        all_chunks = []
        processed_files = 0
        total_files = len(files_data)

        for i, file_data in enumerate(files_data):
            try:
                logger.info(f"Processing file {i+1}/{total_files}: {file_data['metadata']['source_file']}")

                # Chunk the document
                chunks = chunker.chunk_document(
                    file_data["text"],
                    file_data["metadata"]
                )

                all_chunks.extend(chunks)
                processed_files += 1

                logger.debug(f"  - Created {len(chunks)} chunks from {file_data['metadata']['source_file']}")

            except Exception as e:
                logger.error(f"Error processing file {file_data['metadata']['source_file']}: {str(e)}")
                # Continue with other files as per FR-011 requirement
                continue

        logger.info(f"Successfully processed {processed_files} out of {total_files} files")

        if processed_files == 0:
            logger.error("No files were successfully processed. Exiting.")
            sys.exit(1)

        # Step 3: Add embeddings to chunks
        logger.info(f"Generating embeddings for {len(all_chunks)} chunks...")
        chunks_with_embeddings = embedding_processor.process_chunks_for_embedding(all_chunks)
        logger.info("Embeddings generated successfully")

        # Step 4: Create Qdrant collection
        logger.info("Creating/verifying Qdrant collection...")
        vector_db.create_collection()

        # Step 5: Upload chunks to Qdrant
        logger.info("Uploading chunks to Qdrant...")
        vector_db.batch_upload_chunks(chunks_with_embeddings)
        logger.info("Upload completed successfully")

        # Step 6: Verification step as required by FR-012
        total_points = vector_db.get_total_points()
        logger.info(f"Verification: Total points in collection: {total_points}")

        # Final summary
        logger.info("Ingestion process completed successfully!")
        logger.info(f"Summary: {processed_files} files processed, {len(all_chunks)} chunks created, {total_points} vectors stored")

        # Verify that the number of vectors matches expectations
        if total_points == len(chunks_with_embeddings):
            logger.info("✓ Vector count verification passed")
        else:
            logger.warning(f"⚠ Vector count mismatch: {total_points} vectors stored, {len(chunks_with_embeddings)} chunks processed")

    except Exception as e:
        logger.error(f"Critical error during ingestion process: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()