# Quickstart: Data Pipeline and Qdrant Setup

## Prerequisites

- Python 3.11+
- Qdrant Cloud account with API key
- Google Gemini API key
- Documentation files in `docusaurus-project/docs` directory

## Setup

1. **Install dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

2. **Configure environment variables**:
   ```bash
   # Create .env file in backend/
   QDRANT_URL=https://your-qdrant-cluster.qdrant.tech
   QDRANT_API_KEY=your_qdrant_api_key
   GEMINI_API_KEY=your_gemini_api_key
   OPENAI_API_KEY=your_openai_api_key  # For later phases
   ```

## Running the Data Pipeline

1. **Execute the ingestion script**:
   ```bash
   python ingest.py
   ```

2. **Monitor the process**:
   - The script will read all .md and .mdx files from `docusaurus-project/docs`
   - Convert markdown to clean text
   - Chunk documents with overlap
   - Generate embeddings using Google Gemini
   - Upload to Qdrant vector database

3. **Verify completion**:
   - Check console output for success message
   - Verify number of files processed, chunks created, and vectors uploaded

## Configuration Options

- **Chunk size**: Modify `MAX_CHUNK_SIZE` in `config/settings.py` (default: 1000 characters)
- **Overlap**: Modify `CHUNK_OVERLAP` in `config/settings.py` (default: 200 characters)
- **Documentation path**: Modify in `data_ingestion/markdown_reader.py` (default: `docusaurus-project/docs`)

## Troubleshooting

- **API key errors**: Verify QDRANT_API_KEY and GEMINI_API_KEY are correctly set
- **File access errors**: Ensure the documentation directory exists and contains markdown files
- **Memory issues**: For very large documentation sets, consider processing in smaller batches