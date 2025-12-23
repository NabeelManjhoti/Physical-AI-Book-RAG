# API Contracts: Docusaurus Book - Physical AI & Humanoid Robotics

## Overview
This document defines the interface contracts for the Docusaurus-based textbook platform. Since this is primarily a static site, most interactions are client-side with minimal API requirements.

## Static Content API
The site uses Docusaurus' built-in static generation, but the following endpoints may be used for dynamic features:

### Content Retrieval
```
GET /api/content/{module}/{section}
```
**Description**: Retrieve specific content sections for dynamic loading
**Parameters**:
- module: The module identifier (e.g., "module-1", "module-2")
- section: The specific section within the module
**Response**:
```json
{
  "id": "string",
  "title": "string",
  "content": "string",
  "type": "text|code|diagram|interactive",
  "metadata": {}
}
```

### Search API
```
GET /api/search
```
**Description**: Search across all textbook content
**Parameters**:
- query: Search term
- limit: Number of results (default: 10)
- module: Optional module filter
**Response**:
```json
{
  "results": [
    {
      "id": "string",
      "title": "string",
      "url": "string",
      "module": "string",
      "contentPreview": "string",
      "relevance": "number"
    }
  ],
  "total": "number"
}
```

### Progress Tracking API
```
POST /api/progress
```
**Description**: Save user progress through the textbook
**Request**:
```json
{
  "userId": "string",
  "moduleId": "string",
  "sectionId": "string",
  "completed": "boolean",
  "timeSpent": "number"
}
```
**Response**:
```json
{
  "success": "boolean",
  "progress": {
    "moduleId": "string",
    "completionPercentage": "number"
  }
}
```

### User Session API
```
POST /api/session
```
**Description**: Initialize or update user session
**Request**:
```json
{
  "userId": "string",
  "sessionId": "string",
  "currentPage": "string",
  "referrer": "string"
}
```
**Response**:
```json
{
  "sessionId": "string",
  "sessionStart": "timestamp",
  "progress": {}
}
```

## Client-Side Components API
For interactive elements, the following component interfaces are defined:

### Interactive Code Component
```typescript
interface InteractiveCodeProps {
  code: string;
  language: 'typescript' | 'python' | 'bash';
  title?: string;
  description?: string;
  onRun?: (code: string) => void;
  onExecute?: (result: any) => void;
}
```

### Robotics Animation Component
```typescript
interface RoboticsAnimationProps {
  type: 'navigation' | 'manipulation' | 'perception';
  width?: number;
  height?: number;
  autoplay?: boolean;
  controls?: boolean;
}
```

## Error Handling
All API endpoints follow standard HTTP status codes:
- 200: Success
- 400: Bad request (invalid parameters)
- 404: Content not found
- 500: Server error