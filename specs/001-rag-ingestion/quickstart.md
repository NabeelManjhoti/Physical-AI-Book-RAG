# Quickstart: URL Ingestion & Embedding Pipeline

## Overview
This guide provides step-by-step instructions to set up and run the RAG content ingestion pipeline that crawls Docusaurus documentation, generates embeddings, and stores them in Qdrant.

## Prerequisites
- Python 3.11 or higher
- `uv` package manager installed
- Cohere API key
- Qdrant Cloud account and API key

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Navigate to Backend Directory
```bash
cd backend/
```

### 3. Install Dependencies with uv
```bash
uv venv  # Create virtual environment
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
```

### 4. Configure Environment Variables
Create a `.env` file in the backend directory with the following content:
```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cluster_url
QDRANT_API_KEY=your_qdrant_api_key
SOURCE_URLS=https://example-docusaurus-site.com/docs
COLLECTION_NAME=docs_embeddings
```

### 5. Run the Ingestion Pipeline
```bash
python main.py
```

## Configuration Options

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key for generating embeddings
- `QDRANT_URL`: URL of your Qdrant Cloud cluster
- `QDRANT_API_KEY`: API key for your Qdrant Cloud cluster
- `SOURCE_URLS`: Comma-separated list of Docusaurus documentation URLs to crawl
- `COLLECTION_NAME`: Name of the Qdrant collection to store embeddings in
- `CHUNK_SIZE`: Maximum size of text chunks (default: 500 characters)
- `CHUNK_OVERLAP`: Overlap between chunks in characters (default: 50)

## Running Tests
```bash
pip install pytest
pytest tests/
```

## Troubleshooting

### Common Issues
1. **API Key Issues**: Verify your Cohere and Qdrant API keys are correct and have proper permissions
2. **Network Issues**: Check that the source URLs are accessible and not blocked by firewalls
3. **Memory Issues**: For large documentation sets, consider processing in smaller batches

### Error Messages
- "Connection refused to Qdrant": Verify QDRANT_URL and QDRANT_API_KEY are correct
- "Invalid API key": Check your Cohere API key
- "Failed to fetch URL": Verify the source URL is accessible

## Next Steps
After successful setup, you can:
1. Customize the URL list in environment variables
2. Adjust chunking parameters for your content
3. Monitor the ingestion progress in Qdrant dashboard
4. Test vector search functionality