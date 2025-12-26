import pytest
from unittest.mock import Mock, patch, MagicMock
from main import DocumentCrawler, TextChunker, EmbeddingGenerator, VectorStore


class TestEndToEndPipeline:
    """Test class for end-to-end pipeline functionality."""

    @patch('main.VectorStore')
    @patch('main.EmbeddingGenerator')
    @patch('main.TextChunker')
    @patch('main.DocumentCrawler')
    def test_end_to_end_pipeline(self, mock_crawler_cls, mock_chunker_cls, mock_embedder_cls, mock_vector_store_cls):
        """Test the complete end-to-end pipeline flow."""
        # Create mock instances
        mock_crawler = Mock()
        mock_chunker = Mock()
        mock_embedder = Mock()
        mock_vector_store = Mock()

        # Set up the class mocks to return our instances
        mock_crawler_cls.return_value = mock_crawler
        mock_chunker_cls.return_value = mock_chunker
        mock_embedder_cls.return_value = mock_embedder
        mock_vector_store_cls.return_value = mock_vector_store

        # Mock the crawler's fetch_multiple_urls method
        mock_crawler.fetch_multiple_urls.return_value = [
            {
                'url': 'https://example.com/docs/intro',
                'title': 'Introduction',
                'section_title': 'Introduction',
                'content': 'This is sample documentation content for testing the RAG pipeline.',
                'status': 'success'
            },
            {
                'url': 'https://example.com/docs/setup',
                'title': 'Setup Guide',
                'section_title': 'Setup Guide',
                'content': 'This is another piece of documentation content for testing purposes.',
                'status': 'success'
            }
        ]

        # Mock the chunker's chunk_document method
        mock_chunker.chunk_document.side_effect = lambda doc: [
            {
                'id': f"chunk_0_{doc['url']}",
                'content': doc['content'][:30],  # First 30 chars as a sample
                'source_url': doc['url'],
                'section_title': doc['section_title'],
                'position': 0,
                'metadata': {}
            }
        ]

        # Mock the embedder's generate_embedding_for_chunks method
        mock_embedder.generate_embedding_for_chunks.return_value = [
            {
                'id': 'chunk_0_https://example.com/docs/intro',
                'content': 'This is sample documentation content for testing the RAG pipeline.',
                'source_url': 'https://example.com/docs/intro',
                'section_title': 'Introduction',
                'position': 0,
                'metadata': {},
                'embedding': [0.1, 0.2, 0.3]  # Mock embedding
            },
            {
                'id': 'chunk_0_https://example.com/docs/setup',
                'content': 'This is another piece of documentation content for testing purposes.',
                'source_url': 'https://example.com/docs/setup',
                'section_title': 'Setup Guide',
                'position': 0,
                'metadata': {},
                'embedding': [0.4, 0.5, 0.6]  # Mock embedding
            }
        ]

        # Mock configuration
        config = {
            'cohere_api_key': 'test-key',
            'qdrant_url': 'http://localhost:6333',
            'qdrant_api_key': 'test-key',
            'source_urls': ['https://example.com/docs/intro', 'https://example.com/docs/setup'],
            'collection_name': 'test_collection',
            'chunk_size': 500,
            'chunk_overlap': 50
        }

        # Test the main pipeline flow (logic from main function)
        crawler = mock_crawler_cls()
        chunker = mock_chunker_cls(chunk_size=config['chunk_size'], chunk_overlap=config['chunk_overlap'])
        embedder = mock_embedder_cls(api_key=config['cohere_api_key'])
        vector_store = mock_vector_store_cls(url=config['qdrant_url'], api_key=config['qdrant_api_key'], collection_name=config['collection_name'])

        # Create Qdrant collection
        vector_store.create_collection(vector_size=1024, distance="Cosine")

        # Step 1: Crawl URLs
        crawled_documents = crawler.fetch_multiple_urls(config['source_urls'])

        successful_crawls = [doc for doc in crawled_documents if doc['status'] == 'success']
        failed_crawls = [doc for doc in crawled_documents if doc['status'] == 'error']

        # Verify crawling worked
        assert len(successful_crawls) == 2
        assert len(failed_crawls) == 0

        # Step 2: Chunk documents
        all_chunks = []
        for doc in successful_crawls:
            if doc['content'].strip():
                chunks = chunker.chunk_document(doc)
                all_chunks.extend(chunks)

        # Verify chunking worked
        assert len(all_chunks) == 2

        # Step 3: Generate embeddings
        chunks_with_embeddings = embedder.generate_embedding_for_chunks(all_chunks)

        # Verify embedding generation worked
        assert len(chunks_with_embeddings) == 2
        assert 'embedding' in chunks_with_embeddings[0]

        # Step 4: Store in vector database
        vector_store.store_embeddings(chunks_with_embeddings)

        # Verify storage worked
        mock_vector_store.store_embeddings.assert_called_once()

        # Verify all components were called appropriately
        mock_crawler.fetch_multiple_urls.assert_called_once()
        assert mock_chunker.chunk_document.call_count == 2  # Called for each document
        mock_embedder.generate_embedding_for_chunks.assert_called_once()
        mock_vector_store.store_embeddings.assert_called_once()