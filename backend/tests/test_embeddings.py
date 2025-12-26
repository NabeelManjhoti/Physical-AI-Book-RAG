import pytest
from unittest.mock import Mock, patch
from main import EmbeddingGenerator


class TestEmbeddingGenerator:
    """Test class for EmbeddingGenerator functionality."""

    @patch('main.cohere.Client')
    def test_generate_embeddings_success(self, mock_cohere_client):
        """Test successful embedding generation."""
        # Mock the Cohere client and its embed method
        mock_client_instance = Mock()
        mock_client_instance.embed.return_value = Mock()
        mock_client_instance.embed.return_value.embeddings = [
            [0.1, 0.2, 0.3],
            [0.4, 0.5, 0.6]
        ]
        mock_cohere_client.return_value = mock_client_instance

        generator = EmbeddingGenerator(api_key='test-key')

        texts = ["Hello world", "Test text"]
        embeddings = generator.generate_embeddings(texts)

        # Verify the embeddings were returned correctly
        assert len(embeddings) == 2
        assert embeddings[0] == [0.1, 0.2, 0.3]
        assert embeddings[1] == [0.4, 0.5, 0.6]

        # Verify the embed method was called with correct parameters
        mock_client_instance.embed.assert_called_once_with(
            texts=texts,
            model="embed-multilingual-v3.0",
            input_type="search_document"
        )

    @patch('main.cohere.Client')
    def test_generate_embeddings_batching(self, mock_cohere_client):
        """Test embedding generation with batching."""
        # Mock the Cohere client
        mock_client_instance = Mock()
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2]]  # Single embedding per call
        mock_client_instance.embed.return_value = mock_response
        mock_cohere_client.return_value = mock_client_instance

        generator = EmbeddingGenerator(api_key='test-key')

        # Create 5 texts to test batching (with batch_size=2, should create 3 calls)
        texts = ["text1", "text2", "text3", "text4", "text5"]
        embeddings = generator.generate_embeddings(texts, batch_size=2)

        # Should have 5 embeddings in the result
        assert len(embeddings) == 5

        # Should have been called 3 times: (0-1), (2-3), (4)
        assert mock_client_instance.embed.call_count == 3

    @patch('main.cohere.Client')
    def test_generate_embedding_for_chunks(self, mock_cohere_client):
        """Test embedding generation for text chunks."""
        # Mock the Cohere client
        mock_client_instance = Mock()
        mock_response = Mock()
        mock_response.embeddings = [
            [0.1, 0.2, 0.3],
            [0.4, 0.5, 0.6]
        ]
        mock_client_instance.embed.return_value = mock_response
        mock_cohere_client.return_value = mock_client_instance

        generator = EmbeddingGenerator(api_key='test-key')

        chunks = [
            {
                'id': 'chunk_0_0-100',
                'content': 'Hello world',
                'source_url': 'https://example.com',
                'section_title': 'Test',
                'position': 0
            },
            {
                'id': 'chunk_1_100-200',
                'content': 'Test content',
                'source_url': 'https://example.com',
                'section_title': 'Test',
                'position': 1
            }
        ]

        chunks_with_embeddings = generator.generate_embedding_for_chunks(chunks)

        # Verify that embeddings were added to the chunks
        assert len(chunks_with_embeddings) == 2
        assert 'embedding' in chunks_with_embeddings[0]
        assert 'embedding' in chunks_with_embeddings[1]
        assert chunks_with_embeddings[0]['embedding'] == [0.1, 0.2, 0.3]
        assert chunks_with_embeddings[1]['embedding'] == [0.4, 0.5, 0.6]

        # Verify that original fields are preserved
        assert chunks_with_embeddings[0]['id'] == 'chunk_0_0-100'
        assert chunks_with_embeddings[0]['content'] == 'Hello world'
        assert chunks_with_embeddings[0]['source_url'] == 'https://example.com'

    @patch('main.cohere.Client')
    def test_consistent_embedding_dimensions(self, mock_cohere_client):
        """Test that all embeddings have consistent dimensions."""
        # Mock the Cohere client to return embeddings of the same dimension
        mock_client_instance = Mock()
        mock_response = Mock()
        # Return embeddings with 1024 dimensions (common for Cohere models)
        mock_response.embeddings = [
            [0.1] * 1024,  # 1024-dimensional vector
            [0.2] * 1024,  # 1024-dimensional vector
            [0.3] * 1024   # 1024-dimensional vector
        ]
        mock_client_instance.embed.return_value = mock_response
        mock_cohere_client.return_value = mock_client_instance

        generator = EmbeddingGenerator(api_key='test-key')

        texts = ["text1", "text2", "text3"]
        embeddings = generator.generate_embeddings(texts)

        # Verify all embeddings have the same dimensionality
        assert len(embeddings) == 3
        for embedding in embeddings:
            assert len(embedding) == 1024  # Standard Cohere embedding dimension

        # Verify that dimensions are consistent across all embeddings
        first_embedding_length = len(embeddings[0])
        for embedding in embeddings:
            assert len(embedding) == first_embedding_length