import pytest
from unittest.mock import Mock, patch
from main import DocumentCrawler


class TestTextExtraction:
    """Test class for verifying clean text extraction."""

    def test_clean_text_extraction(self):
        """Test that HTML formatting is removed from extracted content."""
        crawler = DocumentCrawler(delay=0)  # No delay for tests

        # HTML content with various formatting
        html_content = b'''
        <html>
            <head><title>Test Page</title></head>
            <body>
                <nav>This should be removed</nav>
                <header>Header content</header>
                <main>
                    <h1>Main Title</h1>
                    <p>This is a <strong>bold</strong> paragraph with <em>emphasis</em>.</p>
                    <p>Another paragraph with <a href="#">links</a> and other formatting.</p>
                    <ul>
                        <li>List item 1</li>
                        <li>List item 2</li>
                    </ul>
                </main>
                <footer>Footer content</footer>
            </body>
        </html>
        '''

        # Mock response
        mock_response = Mock()
        mock_response.content = html_content
        mock_response.raise_for_status.return_value = None

        with patch.object(crawler.session, 'get', return_value=mock_response):
            result = crawler.fetch_url('https://example.com/test')

            # Verify that the result is successful
            assert result['status'] == 'success'

            # Verify that HTML tags are not present in the content
            content = result['content']
            assert '<p>' not in content
            assert '<strong>' not in content
            assert '<em>' not in content
            assert '<a' not in content
            assert '<ul>' not in content
            assert '<li>' not in content

            # Verify that actual text content is present
            assert 'bold' in content
            assert 'emphasis' in content
            assert 'paragraph' in content
            assert 'List item' in content

            # Verify that non-content elements are removed
            assert 'This should be removed' not in content  # nav content
            assert 'Header content' not in content  # header content
            assert 'Footer content' not in content  # footer content

    def test_special_characters_handling(self):
        """Test that special characters are properly handled."""
        crawler = DocumentCrawler(delay=0)

        html_content = b'''
        <html>
            <body>
                <main>
                    <p>This is text with &amp; special &lt;characters&gt; and quotes &quot;here&quot;.</p>
                    <p>Multiple    spaces    should    be    normalized.</p>
                </main>
            </body>
        </html>
        '''

        # Mock response
        mock_response = Mock()
        mock_response.content = html_content
        mock_response.raise_for_status.return_value = None

        with patch.object(crawler.session, 'get', return_value=mock_response):
            result = crawler.fetch_url('https://example.com/test')

            assert result['status'] == 'success'
            content = result['content']

            # Verify special HTML entities are converted
            assert '&' in content  # &amp; should become &
            assert '<' in content  # &lt; should become <
            assert '>' in content  # &gt; should become >
            assert '"' in content  # &quot; should become "

            # Verify multiple spaces are normalized
            assert 'spaces should be normalized' in content  # Multiple spaces collapsed to single space