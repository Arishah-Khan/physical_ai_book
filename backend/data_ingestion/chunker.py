import re
from typing import List, Dict, Any
from .text_converter import MarkdownToTextConverter
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class TextChunker:
    """
    Splits text content into chunks of specified size with overlap.
    Implements paragraph-based chunking with recursive splitting for large content.
    """

    def __init__(self, max_chunk_size: int = 1000, overlap: int = 200):
        self.max_chunk_size = max_chunk_size
        self.overlap = overlap
        self.converter = MarkdownToTextConverter()

    def chunk_document(self, text: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Split a document into chunks with overlap using paragraph-based chunking.
        Handles various content types: headers, lists, code blocks, links.

        Args:
            text: The text content to chunk
            metadata: Metadata to include with each chunk

        Returns:
            List of chunk dictionaries with text and metadata
        """
        chunks = []

        # First clean the text
        clean_text = self.converter.convert(text)

        # Split by paragraphs first (preserving structure)
        paragraphs = self._split_by_paragraphs(clean_text)

        current_chunk = ""
        current_size = 0

        for paragraph in paragraphs:
            paragraph_size = len(paragraph)

            # If adding this paragraph would exceed the chunk size
            if current_size + paragraph_size > self.max_chunk_size and current_chunk:
                # Finalize the current chunk
                chunks.append({
                    "text": current_chunk.strip(),
                    "metadata": metadata.copy(),
                    "size": current_size
                })

                # Start a new chunk with overlap
                if len(paragraph) <= self.max_chunk_size:
                    # Add overlap from the end of the current chunk
                    overlap_text = self._get_overlap_text(current_chunk)
                    current_chunk = overlap_text + paragraph
                    current_size = len(current_chunk)
                else:
                    # The paragraph itself is too large, need to split it recursively
                    sub_chunks = self.recursive_chunk(paragraph, metadata)
                    chunks.extend(sub_chunks)
                    # Start fresh after processing the large paragraph
                    current_chunk = ""
                    current_size = 0
            else:
                # Add paragraph to current chunk
                if current_chunk:
                    current_chunk += "\n\n" + paragraph
                else:
                    current_chunk = paragraph
                current_size += paragraph_size + 2  # +2 for the \n\n

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunks.append({
                "text": current_chunk.strip(),
                "metadata": metadata.copy(),
                "size": current_size
            })

        logger.info(f"Chunked document into {len(chunks)} chunks")
        return chunks

    def recursive_chunk(self, text: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Recursively chunk text that is too large for a single chunk.
        Implements sentence-based chunking as a fallback.

        Args:
            text: Text that needs to be chunked recursively
            metadata: Metadata to include with each chunk

        Returns:
            List of chunk dictionaries
        """
        logger.debug(f"Recursively chunking text of size {len(text)}")

        if len(text) <= self.max_chunk_size:
            return [{
                "text": text,
                "metadata": metadata.copy(),
                "size": len(text)
            }]

        # Try to split by sentences first
        sentences = self._split_by_sentences(text)
        if len(sentences) > 1:
            chunks = []
            current_chunk = ""
            current_size = 0

            for sentence in sentences:
                sentence_size = len(sentence)

                if current_size + sentence_size > self.max_chunk_size and current_chunk:
                    # Finalize current chunk
                    chunks.append({
                        "text": current_chunk.strip(),
                        "metadata": metadata.copy(),
                        "size": current_size
                    })

                    # Start new chunk with overlap
                    overlap_text = self._get_overlap_text(current_chunk)
                    current_chunk = overlap_text + sentence
                    current_size = len(current_chunk)
                else:
                    if current_chunk:
                        current_chunk += " " + sentence
                    else:
                        current_chunk = sentence
                    current_size += sentence_size + 1  # +1 for the space

            # Add final chunk if it has content
            if current_chunk.strip():
                chunks.append({
                    "text": current_chunk.strip(),
                    "metadata": metadata.copy(),
                    "size": current_size
                })

            # If any chunk is still too large, force split it
            final_chunks = []
            for chunk in chunks:
                if len(chunk["text"]) > self.max_chunk_size:
                    forced_chunks = self._force_split(chunk["text"], chunk["metadata"])
                    final_chunks.extend(forced_chunks)
                else:
                    final_chunks.append(chunk)

            return final_chunks
        else:
            # If sentence splitting didn't work, force split
            return self._force_split(text, metadata)

    def _split_by_paragraphs(self, text: str) -> List[str]:
        """
        Split text into paragraphs while preserving structure.

        Args:
            text: Text to split

        Returns:
            List of paragraphs
        """
        # Split by double newlines, but keep empty strings that might be meaningful
        paragraphs = text.split('\n\n')
        # Filter out empty paragraphs but keep those with content
        return [p.strip() for p in paragraphs if p.strip()]

    def _split_by_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences.

        Args:
            text: Text to split

        Returns:
            List of sentences
        """
        # Split by sentence endings (., !, ?) followed by whitespace or end of string
        # This regex handles abbreviations and other edge cases better
        sentences = re.split(r'(?<=[.!?])\s+', text)
        # Filter out empty sentences
        return [s.strip() for s in sentences if s.strip()]

    def _get_overlap_text(self, text: str) -> str:
        """
        Get the overlap text from the end of a chunk, trying to break at word boundaries.

        Args:
            text: Text to extract overlap from

        Returns:
            Overlap text
        """
        if len(text) <= self.overlap:
            return text

        # Get the last 'overlap' characters
        overlap_start = len(text) - self.overlap
        overlap_text = text[overlap_start:]

        # Try to find a space to break at to avoid cutting words
        space_pos = overlap_text.rfind(' ')
        if space_pos != -1 and space_pos > len(overlap_text) // 2:  # Only if space is not too early
            overlap_text = overlap_text[space_pos + 1:]

        return overlap_text

    def _force_split(self, text: str, metadata: Dict[str, Any]) -> List[Dict[str, Any]]:
        """
        Force split text that is too large into fixed-size chunks.

        Args:
            text: Text to force split
            metadata: Metadata to include with each chunk

        Returns:
            List of chunk dictionaries
        """
        logger.warning(f"Force splitting text of size {len(text)} into chunks of max size {self.max_chunk_size}")

        chunks = []
        start = 0

        while start < len(text):
            end = start + self.max_chunk_size
            if end > len(text):
                end = len(text)

            chunk_text = text[start:end]
            chunks.append({
                "text": chunk_text,
                "metadata": metadata.copy(),
                "size": len(chunk_text)
            })

            start = end

        return chunks