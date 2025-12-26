import pytest
from unittest.mock import Mock, patch
from main import VectorStore


class TestVectorStore:
    """Test class for VectorStore functionality."""

    @patch('main.QdrantClient')
    def test_create_collection(self, mock_qdrant_client):
        """Test creating a Qdrant collection."""
        # Mock the Qdrant client
        mock_client_instance = Mock()
        mock_client_instance.get_collections.return_value = Mock()
        mock_client_instance.get_collections.return_value.collections = []
        mock_qdrant_client.return_value = mock_client_instance

        vector_store = VectorStore(url='http://localhost:6333', api_key='test-key')

        # Test creating a collection
        vector_store.create_collection(vector_size=1024, distance="Cosine")

        # Verify the collection was created
        mock_client_instance.create_collection.assert_called_once()
        call_args = mock_client_instance.create_collection.call_args
        assert call_args[1]['collection_name'] == 'docs_embeddings'
        assert call_args[1]['vectors_config'].size == 1024

    @patch('main.QdrantClient')
    def test_store_embeddings(self, mock_qdrant_client):
        """Test storing embeddings in Qdrant."""
        # Mock the Qdrant client
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance

        vector_store = VectorStore(url='http://localhost:6333', api_key='test-key')

        # Sample chunks with embeddings
        chunks_with_embeddings = [
            {
                'id': 'chunk_0_0-100',
                'content': 'Sample content 1',
                'source_url': 'https://example.com',
                'section_title': 'Test Section',
                'position': 0,
                'embedding': [0.1, 0.2, 0.3]
            },
            {
                'id': 'chunk_1_100-200',
                'content': 'Sample content 2',
                'source_url': 'https://example.com',
                'section_title': 'Test Section',
                'position': 1,
                'embedding': [0.4, 0.5, 0.6]
            }
        ]

        # Store embeddings
        vector_store.store_embeddings(chunks_with_embeddings)

        # Verify upsert was called
        mock_client_instance.upsert.assert_called_once()
        call_args = mock_client_instance.upsert.call_args
        assert call_args[1]['collection_name'] == 'docs_embeddings'
        assert len(call_args[1]['points']) == 2

    @patch('main.QdrantClient')
    def test_search(self, mock_qdrant_client):
        """Test searching for similar embeddings in Qdrant."""
        # Mock the Qdrant client
        mock_client_instance = Mock()

        # Mock search results
        mock_hit1 = Mock()
        mock_hit1.payload = {
            'id': 'chunk_0_0-100',
            'content': 'Sample content 1',
            'source_url': 'https://example.com',
            'section_title': 'Test Section',
            'position': 0
        }
        mock_hit1.score = 0.95

        mock_hit2 = Mock()
        mock_hit2.payload = {
            'id': 'chunk_1_100-200',
            'content': 'Sample content 2',
            'source_url': 'https://example.com',
            'section_title': 'Test Section',
            'position': 1
        }
        mock_hit2.score = 0.87

        mock_client_instance.search.return_value = [mock_hit1, mock_hit2]
        mock_qdrant_client.return_value = mock_client_instance

        vector_store = VectorStore(url='http://localhost:6333', api_key='test-key')

        # Perform search
        query_embedding = [0.1, 0.2, 0.3]
        results = vector_store.search(query_embedding, top_k=5, min_score=0.5)

        # Verify search was called
        mock_client_instance.search.assert_called_once()
        call_args = mock_client_instance.search.call_args
        assert call_args[1]['collection_name'] == 'docs_embeddings'
        assert call_args[1]['query_vector'] == query_embedding
        assert call_args[1]['limit'] == 5
        assert call_args[1]['score_threshold'] == 0.5

        # Verify results format
        assert len(results) == 2
        assert results[0]['content'] == 'Sample content 1'
        assert results[0]['score'] == 0.95
        assert results[1]['content'] == 'Sample content 2'
        assert results[1]['score'] == 0.87

    @patch('main.QdrantClient')
    def test_delete_collection(self, mock_qdrant_client):
        """Test deleting a Qdrant collection."""
        # Mock the Qdrant client
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance

        vector_store = VectorStore(url='http://localhost:6333', api_key='test-key')

        # Delete collection
        vector_store.delete_collection()

        # Verify delete was called
        mock_client_instance.delete_collection.assert_called_once_with(collection_name='docs_embeddings')

    @patch('main.QdrantClient')
    def test_vector_search_relevance(self, mock_qdrant_client):
        """Test that vector search returns relevant content based on semantic similarity."""
        # Mock the Qdrant client
        mock_client_instance = Mock()

        # Mock search results with different scores to simulate relevance
        # Higher scores should indicate more relevant results
        relevant_hit = Mock()
        relevant_hit.payload = {
            'id': 'chunk_0_0-100',
            'content': 'Artificial Intelligence and Machine Learning are advanced technologies',
            'source_url': 'https://example.com/ai',
            'section_title': 'AI Concepts',
            'position': 0
        }
        relevant_hit.score = 0.95  # High relevance

        less_relevant_hit = Mock()
        less_relevant_hit.payload = {
            'id': 'chunk_1_100-200',
            'content': 'Cooking recipes and kitchen tips',
            'source_url': 'https://example.com/cooking',
            'section_title': 'Cooking Guide',
            'position': 1
        }
        less_relevant_hit.score = 0.30  # Lower relevance

        mock_client_instance.search.return_value = [relevant_hit, less_relevant_hit]
        mock_qdrant_client.return_value = mock_client_instance

        vector_store = VectorStore(url='http://localhost:6333', api_key='test-key')

        # Search for AI-related content
        query_embedding = [0.8, 0.1, 0.9]  # Hypothetical AI-related embedding
        results = vector_store.search(query_embedding, top_k=5, min_score=0.0)

        # Verify that results are returned in order of relevance (highest score first)
        assert len(results) == 2
        assert results[0]['score'] >= results[1]['score']  # Higher score first
        assert results[0]['content'] == 'Artificial Intelligence and Machine Learning are advanced technologies'
        assert results[1]['content'] == 'Cooking recipes and kitchen tips'

        # Verify that high-relevance content has higher score
        assert results[0]['score'] == 0.95
        assert results[1]['score'] == 0.30