import pytest
from unittest.mock import Mock, patch
from main import TextChunker


class TestTextChunker:
    """Test class for TextChunker functionality."""

    def test_chunk_text_basic(self):
        """Test basic text chunking functionality."""
        chunker = TextChunker(chunk_size=50, chunk_overlap=10)

        text = "This is a sample text that will be chunked into smaller pieces. " * 3  # Make it longer
        chunks = chunker.chunk_text(text, source_url="https://example.com", section_title="Test Section")

        # Verify we got chunks
        assert len(chunks) > 0

        # Verify each chunk has required fields
        for chunk in chunks:
            assert 'id' in chunk
            assert 'content' in chunk
            assert 'source_url' in chunk
            assert 'section_title' in chunk
            assert 'position' in chunk
            assert 'metadata' in chunk

            # Verify content is not empty
            assert len(chunk['content']) > 0

            # Verify source info is preserved
            assert chunk['source_url'] == "https://example.com"
            assert chunk['section_title'] == "Test Section"

    def test_chunk_text_with_small_content(self):
        """Test chunking text that's smaller than chunk size."""
        chunker = TextChunker(chunk_size=1000, chunk_overlap=10)

        text = "This is a short text."
        chunks = chunker.chunk_text(text)

        # Should have one chunk with the entire text
        assert len(chunks) == 1
        assert chunks[0]['content'] == text

    def test_chunk_text_empty(self):
        """Test chunking empty text."""
        chunker = TextChunker()

        chunks = chunker.chunk_text("")
        assert len(chunks) == 0

    def test_chunk_document(self):
        """Test chunking a document dictionary."""
        chunker = TextChunker(chunk_size=30, chunk_overlap=5)

        document = {
            'content': "This is a sample document content that will be chunked. " * 2,
            'url': 'https://example.com/doc',
            'section_title': 'Sample Section'
        }

        chunks = chunker.chunk_document(document)

        # Verify we got chunks
        assert len(chunks) > 0

        # Verify that document metadata is preserved in each chunk
        for chunk in chunks:
            assert chunk['source_url'] == document['url']
            assert chunk['section_title'] == document['section_title']

    def test_chunk_with_overlap(self):
        """Test that chunks have proper overlap."""
        chunker = TextChunker(chunk_size=20, chunk_overlap=5)

        text = "A" * 50  # 50 A's
        chunks = chunker.chunk_text(text)

        # If we have multiple chunks, verify overlap
        if len(chunks) > 1:
            first_chunk_end = chunks[0]['content'][-chunker.chunk_overlap:]
            second_chunk_start = chunks[1]['content'][:chunker.chunk_overlap]

            # The overlap parts should match
            assert first_chunk_end == second_chunk_start