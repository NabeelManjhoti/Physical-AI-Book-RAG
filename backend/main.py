import requests
from bs4 import BeautifulSoup
import logging
from typing import List, Dict, Optional
from urllib.parse import urljoin, urlparse
import time
import re
from dotenv import load_dotenv
import os
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct


class DocumentCrawler:
    """
    Class to crawl Docusaurus documentation sites and extract clean text content.
    """

    def __init__(self, delay: float = 1.0, timeout: int = 30):
        """
        Initialize the crawler with delay and timeout settings.

        Args:
            delay: Delay between requests in seconds to be respectful to the server
            timeout: Request timeout in seconds
        """
        self.delay = delay
        self.timeout = timeout
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/91.0.4472.124 Safari/537.36'
        })

        # Common selectors for Docusaurus non-content elements
        self.docusaurus_selectors = [
            'nav', 'header', 'footer', '.navbar', '.theme-doc-sidebar-container',
            '.theme-doc-breadcrumb', '.theme-edit-this-page', '.theme-last-updated',
            '.pagination-nav', '.toc', '.table-of-contents', '.sidebar',
            '[role="banner"]', '[role="navigation"]', '.DocSearch-content'
        ]

    def fetch_url(self, url: str) -> Optional[Dict]:
        """
        Fetch content from a URL and extract clean text.

        Args:
            url: The URL to fetch

        Returns:
            Dictionary with content information or None if failed
        """
        # Validate URL format
        parsed_url = urlparse(url)
        if not parsed_url.scheme or not parsed_url.netloc:
            logging.error(f"Invalid URL format: {url}")
            return {
                'url': url,
                'title': '',
                'section_title': '',
                'content': '',
                'status': 'error',
                'error': 'Invalid URL format'
            }

        try:
            logging.info(f"Fetching URL: {url}")
            response = self.session.get(url, timeout=self.timeout)
            response.raise_for_status()

            # Parse the HTML content
            soup = BeautifulSoup(response.content, 'html.parser')

            # Remove common Docusaurus non-content elements
            for selector in self.docusaurus_selectors:
                elements = soup.select(selector)
                for element in elements:
                    element.decompose()

            # Try to find the main content area (common in Docusaurus)
            main_content = soup.find('main') or soup.find('article') or soup.find(class_='main-wrapper') or soup

            # Extract the clean text content
            content = main_content.get_text(separator=' ', strip=True)

            # Clean up the text (remove extra whitespace, etc.)
            content = re.sub(r'\s+', ' ', content).strip()

            # Get the page title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else ''

            # Extract H1 as section title if available
            h1_tag = soup.find('h1')
            section_title = h1_tag.get_text().strip() if h1_tag else title.split('|')[0].strip() if title else ''

            # Sleep to be respectful to the server
            time.sleep(self.delay)

            return {
                'url': url,
                'title': title,
                'section_title': section_title,
                'content': content,
                'status': 'success'
            }

        except requests.exceptions.HTTPError as e:
            logging.error(f"HTTP error fetching {url}: {str(e)}")
            return {
                'url': url,
                'title': '',
                'section_title': '',
                'content': '',
                'status': 'error',
                'error': f"HTTP error: {str(e)}"
            }
        except requests.exceptions.ConnectionError as e:
            logging.error(f"Connection error fetching {url}: {str(e)}")
            return {
                'url': url,
                'title': '',
                'section_title': '',
                'content': '',
                'status': 'error',
                'error': f"Connection error: {str(e)}"
            }
        except requests.exceptions.Timeout as e:
            logging.error(f"Timeout error fetching {url}: {str(e)}")
            return {
                'url': url,
                'title': '',
                'section_title': '',
                'content': '',
                'status': 'error',
                'error': f"Timeout error: {str(e)}"
            }
        except requests.exceptions.RequestException as e:
            logging.error(f"Request error fetching {url}: {str(e)}")
            return {
                'url': url,
                'title': '',
                'section_title': '',
                'content': '',
                'status': 'error',
                'error': f"Request error: {str(e)}"
            }
        except Exception as e:
            logging.error(f"Unexpected error fetching {url}: {str(e)}")
            return {
                'url': url,
                'title': '',
                'section_title': '',
                'content': '',
                'status': 'error',
                'error': f"Unexpected error: {str(e)}"
            }

    def fetch_multiple_urls(self, urls: List[str]) -> List[Dict]:
        """
        Fetch content from multiple URLs.

        Args:
            urls: List of URLs to fetch

        Returns:
            List of dictionaries with content information
        """
        results = []
        for url in urls:
            result = self.fetch_url(url)
            results.append(result)
        return results


def setup_logging():
    """Set up basic logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('crawler.log'),
            logging.StreamHandler()
        ]
    )


def load_config():
    """Load configuration from environment variables."""
    load_dotenv()

    config = {
        'cohere_api_key': os.getenv('COHERE_API_KEY'),
        'qdrant_url': os.getenv('QDRANT_URL'),
        'qdrant_api_key': os.getenv('QDRANT_API_KEY'),
        'source_urls': os.getenv('SOURCE_URLS', '').split(','),
        'collection_name': os.getenv('COLLECTION_NAME', 'docs_embeddings'),
        'chunk_size': int(os.getenv('CHUNK_SIZE', '500')),
        'chunk_overlap': int(os.getenv('CHUNK_OVERLAP', '50'))
    }

    return config


def main():
    """
    Main function to orchestrate the full RAG ingestion pipeline.
    """
    # Set up logging
    setup_logging()
    logging.info("Starting RAG ingestion pipeline")

    try:
        # Load configuration
        config = load_config()
        logging.info(f"Configuration loaded. Target URLs: {config['source_urls']}")

        # Validate configuration
        if not config['cohere_api_key']:
            raise ValueError("COHERE_API_KEY environment variable is required")
        if not config['qdrant_url']:
            raise ValueError("QDRANT_URL environment variable is required")
        if not config['qdrant_api_key']:
            raise ValueError("QDRANT_API_KEY environment variable is required")

        # Initialize components
        logging.info("Initializing pipeline components...")
        crawler = DocumentCrawler()
        chunker = TextChunker(chunk_size=config['chunk_size'], chunk_overlap=config['chunk_overlap'])
        embedder = EmbeddingGenerator(api_key=config['cohere_api_key'])
        vector_store = VectorStore(url=config['qdrant_url'], api_key=config['qdrant_api_key'], collection_name=config['collection_name'])

        # Create Qdrant collection
        logging.info(f"Creating Qdrant collection: {config['collection_name']}")
        vector_store.create_collection(vector_size=1024, distance="Cosine")

        # Step 1: Crawl URLs
        logging.info(f"Starting to crawl {len(config['source_urls'])} URLs...")
        crawled_documents = crawler.fetch_multiple_urls(config['source_urls'])

        successful_crawls = [doc for doc in crawled_documents if doc['status'] == 'success']
        failed_crawls = [doc for doc in crawled_documents if doc['status'] == 'error']

        logging.info(f"Crawling completed. Successful: {len(successful_crawls)}, Failed: {len(failed_crawls)}")

        if not successful_crawls:
            logging.error("No documents were successfully crawled. Exiting.")
            return

        # Step 2: Chunk documents
        logging.info("Starting text chunking...")
        all_chunks = []
        for doc in successful_crawls:
            if doc['content'].strip():  # Only chunk documents with content
                chunks = chunker.chunk_document(doc)
                all_chunks.extend(chunks)

        logging.info(f"Text chunking completed. Generated {len(all_chunks)} chunks.")

        if not all_chunks:
            logging.error("No content was extracted to process. Exiting.")
            return

        # Step 3: Generate embeddings
        logging.info("Starting embedding generation...")
        chunks_with_embeddings = embedder.generate_embedding_for_chunks(all_chunks)
        logging.info(f"Embedding generation completed for {len(chunks_with_embeddings)} chunks.")

        # Step 4: Store in vector database
        logging.info("Storing embeddings in vector database...")
        vector_store.store_embeddings(chunks_with_embeddings)
        logging.info("Embeddings successfully stored in vector database.")

        # Summary
        logging.info(f"Pipeline completed successfully!")
        logging.info(f"- URLs processed: {len(successful_crawls)}/{len(config['source_urls'])}")
        logging.info(f"- Content chunks generated: {len(all_chunks)}")
        logging.info(f"- Embeddings stored: {len(chunks_with_embeddings)}")
        logging.info(f"Embeddings stored in collection: {config['collection_name']}")

    except Exception as e:
        logging.error(f"Pipeline failed with error: {str(e)}")
        raise


class TextChunker:
    """
    Class to chunk text content into smaller, manageable pieces for embedding.
    """

    def __init__(self, chunk_size: int = 500, chunk_overlap: int = 50):
        """
        Initialize the chunker with chunk size and overlap settings.

        Args:
            chunk_size: Maximum size of each text chunk
            chunk_overlap: Number of characters to overlap between chunks
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def chunk_text(self, text: str, source_url: str = "", section_title: str = "") -> List[Dict]:
        """
        Chunk the input text into smaller pieces.

        Args:
            text: The text to chunk
            source_url: URL where the text originated from
            section_title: Title of the section this text belongs to

        Returns:
            List of dictionaries containing chunk information
        """
        if not text:
            return []

        chunks = []
        start = 0
        position = 0

        while start < len(text):
            # Determine the end position for this chunk
            end = start + self.chunk_size

            # If we're at the end of the text, just take the remainder
            if end >= len(text):
                end = len(text)
            else:
                # Try to break at a sentence boundary near the end
                chunk_text = text[start:end]
                last_period = chunk_text.rfind('.', self.chunk_size - 100)  # Look for period in last 100 chars
                last_space = chunk_text.rfind(' ', self.chunk_size - 50)   # Look for space in last 50 chars

                if last_period > self.chunk_size // 2:  # If we found a good sentence break
                    end = start + last_period + 1
                elif last_space > self.chunk_size // 2:  # Otherwise, try to break at a space
                    end = start + last_space

            # Extract the chunk
            chunk_content = text[start:end].strip()

            if chunk_content:  # Only add non-empty chunks
                chunk = {
                    'id': f"chunk_{position}_{start}-{end}",
                    'content': chunk_content,
                    'source_url': source_url,
                    'section_title': section_title,
                    'position': position,
                    'metadata': {
                        'start_pos': start,
                        'end_pos': end,
                        'original_length': len(text)
                    }
                }
                chunks.append(chunk)
                position += 1

            # Determine the next start position
            if start == end:
                # If we couldn't advance, move forward by chunk_size to avoid infinite loop
                start += self.chunk_size
            else:
                # Move start position, accounting for overlap
                start = end - self.chunk_overlap
                if self.chunk_overlap <= 0:
                    # If no overlap, move to the end of the current chunk
                    start = end

        return chunks

    def chunk_document(self, document: Dict) -> List[Dict]:
        """
        Chunk a document dictionary (from DocumentCrawler) into smaller pieces.

        Args:
            document: Document dictionary with 'content', 'url', and 'section_title' keys

        Returns:
            List of chunk dictionaries
        """
        return self.chunk_text(
            text=document.get('content', ''),
            source_url=document.get('url', ''),
            section_title=document.get('section_title', '')
        )


class EmbeddingGenerator:
    """
    Class to generate embeddings using Cohere API.
    """

    def __init__(self, api_key: str, model_name: str = "embed-multilingual-v3.0"):
        """
        Initialize the embedding generator with Cohere API key and model.

        Args:
            api_key: Cohere API key
            model_name: Name of the Cohere embedding model to use
        """
        self.client = cohere.Client(api_key)
        self.model_name = model_name

    def generate_embeddings(self, texts: List[str], batch_size: int = 96) -> List[List[float]]:
        """
        Generate embeddings for a list of texts.

        Args:
            texts: List of text strings to embed
            batch_size: Number of texts to process in each batch (Cohere has limits)

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        all_embeddings = []

        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            try:
                response = self.client.embed(
                    texts=batch,
                    model=self.model_name,
                    input_type="search_document"  # Using search_document for content indexing
                )

                batch_embeddings = [embedding for embedding in response.embeddings]
                all_embeddings.extend(batch_embeddings)

            except cohere.CohereError as e:
                logging.error(f"Cohere API error during embedding generation: {str(e)}")
                raise
            except Exception as e:
                logging.error(f"Unexpected error during embedding generation: {str(e)}")
                raise

        return all_embeddings

    def generate_embedding_for_chunks(self, chunks: List[Dict]) -> List[Dict]:
        """
        Generate embeddings for a list of text chunks.

        Args:
            chunks: List of chunk dictionaries with 'content' field

        Returns:
            List of chunk dictionaries with added 'embedding' field
        """
        if not chunks:
            return []

        # Extract text content from chunks
        texts = [chunk['content'] for chunk in chunks]

        # Generate embeddings
        embeddings = self.generate_embeddings(texts)

        # Add embeddings to chunk dictionaries
        chunks_with_embeddings = []
        for i, chunk in enumerate(chunks):
            chunk_copy = chunk.copy()
            chunk_copy['embedding'] = embeddings[i]
            chunks_with_embeddings.append(chunk_copy)

        return chunks_with_embeddings


class VectorStore:
    """
    Class to store and retrieve embeddings using Qdrant vector database.
    """

    def __init__(self, url: str, api_key: str, collection_name: str = "docs_embeddings"):
        """
        Initialize the vector store with Qdrant connection details.

        Args:
            url: Qdrant server URL
            api_key: Qdrant API key
            collection_name: Name of the collection to store embeddings
        """
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name

    def create_collection(self, vector_size: int = 1024, distance: str = "Cosine"):
        """
        Create a collection in Qdrant for storing embeddings.

        Args:
            vector_size: Dimension of the vectors to be stored
            distance: Distance metric for similarity search (Cosine, Euclidean, Dot)
        """
        try:
            # Check if collection already exists
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                # Create the collection
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=vector_size,
                        distance=models.Distance[distance.upper()]
                    )
                )
                logging.info(f"Created collection '{self.collection_name}' in Qdrant")
            else:
                logging.info(f"Collection '{self.collection_name}' already exists in Qdrant")
        except Exception as e:
            logging.error(f"Error creating Qdrant collection: {str(e)}")
            raise

    def store_embeddings(self, chunks_with_embeddings: List[Dict]):
        """
        Store embeddings with metadata in Qdrant.

        Args:
            chunks_with_embeddings: List of chunk dictionaries with 'embedding' field
        """
        if not chunks_with_embeddings:
            logging.info("No embeddings to store")
            return

        # Prepare points for insertion
        points = []
        for i, chunk in enumerate(chunks_with_embeddings):
            # Create a payload with metadata
            payload = {
                'id': chunk.get('id', f'chunk_{i}'),
                'content': chunk.get('content', ''),
                'source_url': chunk.get('source_url', ''),
                'section_title': chunk.get('section_title', ''),
                'position': chunk.get('position', 0),
                'metadata': chunk.get('metadata', {})
            }

            # Create a PointStruct for Qdrant
            point = PointStruct(
                id=i,  # Using sequential IDs, in production use UUIDs
                vector=chunk['embedding'],
                payload=payload
            )
            points.append(point)

        try:
            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            logging.info(f"Successfully stored {len(points)} embeddings in Qdrant collection '{self.collection_name}'")
        except Exception as e:
            logging.error(f"Error storing embeddings in Qdrant: {str(e)}")
            raise

    def search(self, query_embedding: List[float], top_k: int = 5, min_score: float = 0.0) -> List[Dict]:
        """
        Search for similar embeddings in Qdrant.

        Args:
            query_embedding: The embedding vector to search for similar items
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold

        Returns:
            List of dictionaries containing search results with content and metadata
        """
        try:
            # Perform the search
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k,
                score_threshold=min_score
            )

            # Format results
            results = []
            for hit in search_results:
                result = {
                    'id': hit.payload.get('id', ''),
                    'content': hit.payload.get('content', ''),
                    'source_url': hit.payload.get('source_url', ''),
                    'section_title': hit.payload.get('section_title', ''),
                    'score': hit.score,
                    'position': hit.payload.get('position', 0)
                }
                results.append(result)

            return results
        except Exception as e:
            logging.error(f"Error searching in Qdrant: {str(e)}")
            raise

    def delete_collection(self):
        """
        Delete the collection from Qdrant (useful for testing/resetting).
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logging.info(f"Deleted collection '{self.collection_name}' from Qdrant")
        except Exception as e:
            logging.error(f"Error deleting Qdrant collection: {str(e)}")
            raise


if __name__ == "__main__":
    main()
