import pytest
import os
from unittest.mock import Mock, patch
from main import DocumentCrawler


class TestDocumentCrawler:
    """Test class for DocumentCrawler functionality."""

    def test_fetch_url_success(self):
        """Test successful URL fetching."""
        crawler = DocumentCrawler(delay=0)  # No delay for tests

        # Mock response
        mock_response = Mock()
        mock_response.content = b'<html><head><title>Test Page</title></head><body><main><p>This is test content.</p></main></body></html>'
        mock_response.raise_for_status.return_value = None

        with patch.object(crawler.session, 'get', return_value=mock_response):
            result = crawler.fetch_url('https://example.com/test')

            assert result['status'] == 'success'
            assert 'test content' in result['content'].lower()
            assert result['title'] == 'Test Page'

    def test_fetch_url_invalid_format(self):
        """Test URL validation with invalid format."""
        crawler = DocumentCrawler(delay=0)

        result = crawler.fetch_url('invalid-url')

        assert result['status'] == 'error'
        assert 'Invalid URL format' in result['error']

    def test_fetch_multiple_urls(self):
        """Test fetching multiple URLs."""
        crawler = DocumentCrawler(delay=0)

        # Mock response
        mock_response = Mock()
        mock_response.content = b'<html><head><title>Test Page</title></head><body><main><p>This is test content.</p></main></body></html>'
        mock_response.raise_for_status.return_value = None

        with patch.object(crawler.session, 'get', return_value=mock_response):
            urls = ['https://example.com/test1', 'https://example.com/test2']
            results = crawler.fetch_multiple_urls(urls)

            assert len(results) == 2
            for result in results:
                assert result['status'] == 'success'

    def test_init_with_custom_parameters(self):
        """Test crawler initialization with custom parameters."""
        crawler = DocumentCrawler(delay=2.0, timeout=60)

        assert crawler.delay == 2.0
        assert crawler.timeout == 60