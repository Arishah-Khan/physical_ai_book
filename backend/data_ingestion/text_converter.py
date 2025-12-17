import re
from typing import Optional
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MarkdownToTextConverter:
    """
    Converts markdown content to clean plain text by removing markdown formatting
    while preserving the semantic meaning of the content.
    """

    def convert(self, markdown_content: str) -> str:
        """
        Convert markdown content to clean plain text.

        Args:
            markdown_content: Raw markdown content

        Returns:
            Clean plain text
        """
        try:
            text = self._remove_markdown_formatting(markdown_content)
            text = self._clean_text(text)
            logger.debug(f"Converted markdown to text (length: {len(text)})")
            return text
        except Exception as e:
            logger.error(f"Error converting markdown to text: {str(e)}")
            return markdown_content  # Return original if conversion fails

    def _remove_markdown_formatting(self, text: str) -> str:
        """
        Remove common markdown formatting while preserving content structure.

        Args:
            text: Raw markdown text

        Returns:
            Text with markdown formatting removed
        """
        # Remove headers but preserve the text
        text = re.sub(r'^#+\s+', '', text, flags=re.MULTILINE)

        # Remove bold and italic formatting
        text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)
        text = re.sub(r'\*(.*?)\*', r'\1', text)
        text = re.sub(r'__(.*?)__', r'\1', text)
        text = re.sub(r'_(.*?)_', r'\1', text)

        # Remove inline code
        text = re.sub(r'`(.*?)`', r'\1', text)

        # Remove code blocks (including language specifier)
        text = re.sub(r'```.*?\n(.*?)```', r'\1', text, flags=re.DOTALL)

        # Handle code blocks more carefully to preserve content
        text = re.sub(r'```.*?\n(.*?)```', r'\1\n', text, flags=re.DOTALL)
        # Remove remaining code block markers
        text = re.sub(r'```\w*\s*', '', text)

        # Remove links [text](url) -> text
        text = re.sub(r'\[([^\]]+)\]\([^)]+\)', r'\1', text)

        # Remove images ![alt](url) -> alt
        text = re.sub(r'!\[([^\]]*)\]\([^)]+\)', r'\1', text)

        # Remove emphasis markers but keep the text
        text = re.sub(r'\*\s+', ' ', text)
        text = re.sub(r'_\s+', ' ', text)

        # Remove horizontal rules
        text = re.sub(r'^\s*[-*_]{3,}\s*$', '\n', text, flags=re.MULTILINE)

        # Remove reference-style links
        text = re.sub(r'^\[.+\]: .+$', '', text, flags=re.MULTILINE)

        return text

    def _clean_text(self, text: str) -> str:
        """
        Clean up the text by removing extra whitespace and normalizing.

        Args:
            text: Text after markdown formatting removal

        Returns:
            Cleaned text
        """
        # Replace multiple newlines with a single newline
        text = re.sub(r'\n\s*\n', '\n\n', text)

        # Remove leading/trailing whitespace from each line
        lines = [line.strip() for line in text.split('\n')]
        text = '\n'.join(lines)

        # Remove extra blank lines
        text = re.sub(r'\n\n+', '\n\n', text)

        # Strip leading/trailing whitespace from the entire text
        text = text.strip()

        return text