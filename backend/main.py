"""
URL Ingestion & Embedding Pipeline

This script implements a complete RAG content ingestion pipeline that:
1. Crawls Docusaurus documentation URLs
2. Extracts and cleans text content
3. Chunks the content appropriately
4. Generates embeddings using Cohere models
5. Stores embeddings and metadata in Qdrant Cloud
"""
import os
import re
import time
import requests
from bs4 import BeautifulSoup
from typing import List, Dict, Any, Optional
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


class DocumentCrawler:
    """Crawls Docusaurus documentation sites and extracts clean text content."""

    def __init__(self, urls: List[str]):
        self.urls = urls
        self.session = requests.Session()
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36'
        })

    def fetch_page(self, url: str) -> Optional[str]:
        """Fetches a single page and returns its HTML content."""
        try:
            response = self.session.get(url, timeout=30)
            response.raise_for_status()
            return response.text
        except requests.RequestException as e:
            print(f"Error fetching {url}: {e}")
            return None

    def extract_content(self, html: str, url: str) -> Dict[str, Any]:
        """Extracts clean text content from Docusaurus page HTML."""
        soup = BeautifulSoup(html, 'html.parser')

        # Remove navigation, headers, footers, and other non-content elements
        for element in soup.find_all(['nav', 'header', 'footer', 'aside']):
            element.decompose()

        # Remove common Docusaurus-specific elements
        for element in soup.find_all(class_=re.compile(r'(navbar|toc|pagination|footer|sidebar|menu|nav)')):
            element.decompose()

        # Try to find the main content area (Docusaurus specific selectors)
        content_selectors = [
            'article',  # Common for blog posts and docs
            '.main-wrapper',  # Docusaurus main content wrapper
            '.container.padding-vert--lg',  # Docusaurus container
            '.theme-doc-markdown',  # Docusaurus markdown content
            '.markdown',  # Markdown content
            'main',  # Main content area
            '.docItemContainer',  # Docusaurus doc item container
            '.theme-doc-article',  # Docusaurus article container
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        if not content_element:
            # If no specific content area found, use body
            content_element = soup.find('main') or soup.find('body') or soup

        # Extract clean text
        clean_text = content_element.get_text(separator=' ', strip=True)

        # Extract title
        title = soup.find('title')
        title_text = title.get_text().strip() if title else url.split('/')[-1]

        return {
            'url': url,
            'title': title_text,
            'content': clean_text,
            'html': str(content_element)
        }

    def crawl_all(self) -> List[Dict[str, Any]]:
        """Crawls all URLs and returns list of extracted content."""
        results = []
        for url in self.urls:
            print(f"Crawling: {url}")
            html = self.fetch_page(url)
            if html:
                content = self.extract_content(html, url)
                results.append(content)
            time.sleep(0.5)  # Be respectful to servers
        return results


class TextChunker:
    """Chunks text content into appropriate sizes for embedding."""

    def __init__(self, chunk_size: int = 500, chunk_overlap: int = 50):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

    def chunk_text(self, text: str, source_url: str, title: str) -> List[Dict[str, Any]]:
        """Chunks text into smaller pieces with overlap."""
        if not text:
            return []

        # Split text into sentences to avoid breaking them
        sentences = re.split(r'(?<=[.!?]) +', text)

        chunks = []
        current_chunk = ""
        chunk_position = 0

        for sentence in sentences:
            # Check if adding this sentence would exceed chunk size
            if len(current_chunk) + len(sentence) <= self.chunk_size:
                current_chunk += sentence + " "
            else:
                # If current chunk is too small, add more content
                if len(current_chunk) < 100 and current_chunk:
                    current_chunk += sentence + " "
                else:
                    # Save current chunk
                    if current_chunk.strip():
                        chunks.append({
                            'id': f"{source_url}#{chunk_position}",
                            'content': current_chunk.strip(),
                            'source_url': source_url,
                            'section_title': title,
                            'position': chunk_position
                        })
                        chunk_position += 1

                    # Start new chunk with overlap
                    if len(sentence) > self.chunk_size:
                        # If sentence is too long, split it
                        for i in range(0, len(sentence), self.chunk_size - self.chunk_overlap):
                            sub_sentence = sentence[i:i + self.chunk_size - self.chunk_overlap]
                            chunks.append({
                                'id': f"{source_url}#{chunk_position}",
                                'content': sub_sentence.strip(),
                                'source_url': source_url,
                                'section_title': title,
                                'position': chunk_position
                            })
                            chunk_position += 1
                        current_chunk = ""
                    else:
                        current_chunk = sentence + " "

        # Add the last chunk if it exists
        if current_chunk.strip():
            chunks.append({
                'id': f"{source_url}#{chunk_position}",
                'content': current_chunk.strip(),
                'source_url': source_url,
                'section_title': title,
                'position': chunk_position
            })

        return chunks


class EmbeddingGenerator:
    """Generates embeddings using Cohere models."""

    def __init__(self, api_key: str, model: str = "embed-multilingual-v3.0"):
        self.client = cohere.Client(api_key)
        self.model = model

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Generates embeddings for a list of texts."""
        try:
            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type="search_document"
            )
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            print(f"Error generating embeddings: {e}")
            return [[] for _ in texts]


class VectorStore:
    """Manages storage and retrieval of embeddings in Qdrant."""

    def __init__(self, url: str, api_key: str, collection_name: str = "docs_embeddings"):
        self.client = QdrantClient(url=url, api_key=api_key, timeout=10)
        self.collection_name = collection_name
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Creates the collection if it doesn't exist."""
        try:
            collections = self.client.get_collections()
            if not any(col.name == self.collection_name for col in collections.collections):
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=1024,  # Cohere multilingual model returns 1024-dim vectors
                        distance=models.Distance.COSINE
                    )
                )
                print(f"Created collection: {self.collection_name}")
        except Exception as e:
            print(f"Error ensuring collection exists: {e}")

    def store_embeddings(self, chunks: List[Dict[str, Any]], embeddings: List[List[float]]):
        """Stores document chunks with their embeddings in Qdrant."""
        if not chunks or not embeddings or len(chunks) != len(embeddings):
            print("Chunks and embeddings count mismatch or empty")
            return

        points = []
        for chunk, embedding in zip(chunks, embeddings):
            if not embedding:  # Skip empty embeddings
                continue

            point = models.PointStruct(
                id=chunk['id'],
                vector=embedding,
                payload={
                    'content': chunk['content'],
                    'source_url': chunk['source_url'],
                    'section_title': chunk['section_title'],
                    'position': chunk['position']
                }
            )
            points.append(point)

        if points:
            try:
                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points
                )
                print(f"Stored {len(points)} embeddings in Qdrant")
            except Exception as e:
                print(f"Error storing embeddings: {e}")

    def search(self, query_embedding: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """Searches for similar content using the query embedding."""
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            return [
                {
                    'id': result.id,
                    'content': result.payload.get('content', ''),
                    'source_url': result.payload.get('source_url', ''),
                    'section_title': result.payload.get('section_title', ''),
                    'score': result.score
                }
                for result in results
            ]
        except Exception as e:
            print(f"Error searching: {e}")
            return []


def main():
    """Main function to run the full ingestion pipeline."""
    # Load environment variables
    load_dotenv()

    # Configuration
    cohere_api_key = os.getenv('COHERE_API_KEY')
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')
    source_urls_str = os.getenv('SOURCE_URLS', '')
    collection_name = os.getenv('COLLECTION_NAME', 'docs_embeddings')
    chunk_size = int(os.getenv('CHUNK_SIZE', '500'))
    chunk_overlap = int(os.getenv('CHUNK_OVERLAP', '50'))

    # Validate required environment variables
    if not all([cohere_api_key, qdrant_url, qdrant_api_key, source_urls_str]):
        print("Missing required environment variables. Please set:")
        print("- COHERE_API_KEY")
        print("- QDRANT_URL")
        print("- QDRANT_API_KEY")
        print("- SOURCE_URLS")
        return

    # Parse source URLs
    source_urls = [url.strip() for url in source_urls_str.split(',')]

    print("Starting RAG Content Ingestion Pipeline...")
    print(f"Processing {len(source_urls)} URLs")

    # Step 1: Crawl URLs
    print("\n1. Crawling documentation URLs...")
    crawler = DocumentCrawler(source_urls)
    documents = crawler.crawl_all()
    print(f"Crawled {len(documents)} documents")

    if not documents:
        print("No documents were successfully crawled. Exiting.")
        return

    # Step 2: Chunk content
    print("\n2. Chunking content...")
    chunker = TextChunker(chunk_size=chunk_size, chunk_overlap=chunk_overlap)
    all_chunks = []
    for doc in documents:
        chunks = chunker.chunk_text(doc['content'], doc['url'], doc['title'])
        all_chunks.extend(chunks)
    print(f"Created {len(all_chunks)} content chunks")

    if not all_chunks:
        print("No content chunks were created. Exiting.")
        return

    # Step 3: Generate embeddings
    print("\n3. Generating embeddings...")
    embedder = EmbeddingGenerator(cohere_api_key)

    # Process in batches to avoid API limits
    batch_size = 96  # Cohere's batch limit
    all_embeddings = []

    for i in range(0, len(all_chunks), batch_size):
        batch_chunks = all_chunks[i:i + batch_size]
        batch_texts = [chunk['content'] for chunk in batch_chunks]

        print(f"Processing batch {i//batch_size + 1}/{(len(all_chunks) - 1)//batch_size + 1}")
        batch_embeddings = embedder.generate_embeddings(batch_texts)
        all_embeddings.extend(batch_embeddings)

    # Filter out chunks that didn't get embeddings
    valid_pairs = [
        (chunk, embedding)
        for chunk, embedding in zip(all_chunks, all_embeddings)
        if embedding
    ]

    if not valid_pairs:
        print("No valid embeddings were generated. Exiting.")
        return

    # Separate valid chunks and embeddings
    valid_chunks, valid_embeddings = zip(*valid_pairs)
    valid_chunks = list(valid_chunks)
    valid_embeddings = list(valid_embeddings)

    print(f"Generated embeddings for {len(valid_chunks)} chunks")

    # Step 4: Store in vector database
    print("\n4. Storing embeddings in Qdrant...")
    vector_store = VectorStore(qdrant_url, qdrant_api_key, collection_name)
    vector_store.store_embeddings(valid_chunks, valid_embeddings)

    print("\nPipeline completed successfully!")
    print(f"Stored {len(valid_chunks)} embeddings in collection '{collection_name}'")


if __name__ == "__main__":
    main()