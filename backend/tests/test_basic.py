"""
Basic tests for the RAG ingestion pipeline components.
"""
import pytest
from main import TextChunker, DocumentCrawler


def test_text_chunker_basic():
    """Test basic text chunking functionality."""
    chunker = TextChunker(chunk_size=100, chunk_overlap=10)
    text = "This is a test sentence. " * 10  # Create text longer than chunk size
    chunks = chunker.chunk_text(text, "https://example.com", "Test Title")

    # Should have multiple chunks
    assert len(chunks) > 1

    # Each chunk should have required fields
    for chunk in chunks:
        assert 'id' in chunk
        assert 'content' in chunk
        assert 'source_url' in chunk
        assert 'section_title' in chunk
        assert 'position' in chunk


def test_text_chunker_empty():
    """Test chunking with empty text."""
    chunker = TextChunker()
    chunks = chunker.chunk_text("", "https://example.com", "Test Title")
    assert len(chunks) == 0


def test_document_crawler_init():
    """Test DocumentCrawler initialization."""
    urls = ["https://example.com"]
    crawler = DocumentCrawler(urls)
    assert crawler.urls == urls


if __name__ == "__main__":
    pytest.main([__file__])