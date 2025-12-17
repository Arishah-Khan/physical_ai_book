import os
import glob
from typing import List, Dict, Any
from pathlib import Path
import frontmatter  # type: ignore
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class MarkdownReader:
    """
    Reads markdown files from a directory and extracts content with metadata.
    """

    def __init__(self, docs_path: str = "docusaurus-project/docs"):
        self.docs_path = docs_path

    def read_all_markdown_files(self) -> List[Dict[str, Any]]:
        """
        Find and read all markdown files in the documentation directory.
        Handles .md and .mdx files as specified in the requirements.

        Returns:
            List of dictionaries containing file content and metadata
        """
        files_data = []

        # Find all .md and .mdx files in the docs directory
        md_pattern = os.path.join(self.docs_path, "**", "*.md")
        mdx_pattern = os.path.join(self.docs_path, "**", "*.mdx")

        md_files = glob.glob(md_pattern, recursive=True)
        mdx_files = glob.glob(mdx_pattern, recursive=True)

        all_files = md_files + mdx_files

        for file_path in all_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract relative path from docs directory
                relative_path = os.path.relpath(file_path, self.docs_path)

                # Extract frontmatter if present
                try:
                    post = frontmatter.loads(content)
                    text_content = post.content
                    frontmatter_data = post.metadata
                except Exception as e:
                    logger.warning(f"Could not parse frontmatter in {file_path}: {str(e)}")
                    # If frontmatter parsing fails, treat as plain markdown
                    text_content = content
                    frontmatter_data = {}

                file_data = {
                    "text": text_content,
                    "metadata": {
                        "source_file": os.path.basename(file_path),
                        "relative_path": relative_path,
                        "full_path": file_path,
                        **frontmatter_data
                    }
                }

                files_data.append(file_data)
                logger.info(f"Successfully processed: {file_path}")

            except UnicodeDecodeError:
                logger.error(f"Could not decode file (encoding issue): {file_path}")
                continue  # Continue with other files as per error handling requirement
            except FileNotFoundError:
                logger.error(f"File not found: {file_path}")
                continue  # Continue with other files
            except PermissionError:
                logger.error(f"Permission denied reading file: {file_path}")
                continue  # Continue with other files
            except Exception as e:
                logger.error(f"Unexpected error reading file {file_path}: {str(e)}")
                continue  # Continue with other files as per FR-011 requirement

        return files_data

    def extract_frontmatter(self, content: str) -> Dict[str, Any]:
        """
        Extract frontmatter from markdown content.

        Args:
            content: Raw markdown content

        Returns:
            Dictionary containing frontmatter data
        """
        try:
            post = frontmatter.loads(content)
            return post.metadata
        except Exception as e:
            logger.warning(f"Could not parse frontmatter: {str(e)}")
            return {}