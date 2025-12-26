# API Contracts: URL Ingestion & Embedding Pipeline

## Overview
This document defines the API contracts for the RAG content ingestion system.

## Ingestion Pipeline API

### 1. Ingest Documentation URLs
**Endpoint**: `POST /api/ingest`
**Description**: Starts the ingestion process for a list of documentation URLs

**Request**:
```json
{
  "urls": ["https://example.com/docs/intro", "https://example.com/docs/setup"],
  "options": {
    "chunk_size": 500,
    "chunk_overlap": 50,
    "embedding_model": "embed-multilingual-v3.0"
  }
}
```

**Response**:
```json
{
  "job_id": "ingestion-job-123",
  "status": "started",
  "total_urls": 2,
  "estimated_completion": "2025-12-26T15:30:00Z"
}
```

### 2. Check Ingestion Status
**Endpoint**: `GET /api/ingest/{job_id}`
**Description**: Checks the status of an ingestion job

**Response**:
```json
{
  "job_id": "ingestion-job-123",
  "status": "completed",
  "processed_urls": 2,
  "successful": 2,
  "failed": 0,
  "processed_chunks": 45,
  "stored_embeddings": 45,
  "started_at": "2025-12-26T14:30:00Z",
  "completed_at": "2025-12-26T14:35:00Z"
}
```

### 3. Search Documentation
**Endpoint**: `POST /api/search`
**Description**: Performs semantic search on the ingested documentation

**Request**:
```json
{
  "query": "How to set up the development environment?",
  "top_k": 5,
  "min_score": 0.7
}
```

**Response**:
```json
{
  "query": "How to set up the development environment?",
  "results": [
    {
      "id": "chunk-123",
      "content": "To set up the development environment...",
      "source_url": "https://example.com/docs/setup",
      "section_title": "Development Setup",
      "score": 0.89
    }
  ]
}
```

## Data Models

### Document Chunk
```json
{
  "id": "string",
  "content": "string",
  "source_url": "string",
  "section_title": "string",
  "position": "integer",
  "metadata": {}
}
```

### Embedding Vector
```json
{
  "vector_id": "string",
  "vector": "array of floats",
  "chunk_id": "string",
  "model_name": "string",
  "created_at": "datetime"
}
```

### Ingestion Job
```json
{
  "job_id": "string",
  "status": "string (pending|running|completed|failed)",
  "total_urls": "integer",
  "processed_urls": "integer",
  "successful": "integer",
  "failed": "integer",
  "processed_chunks": "integer",
  "stored_embeddings": "integer",
  "started_at": "datetime",
  "completed_at": "datetime"
}
```