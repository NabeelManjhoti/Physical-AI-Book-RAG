# Data Model: Docusaurus Book - Physical AI & Humanoid Robotics

## Overview
This document defines the data structures and relationships for the Docusaurus-based textbook platform on Physical AI & Humanoid Robotics.

## Content Entities

### Textbook Module
**Description**: A structured section of educational content covering a specific robotics topic
**Fields**:
- id: Unique identifier for the module
- title: Display name of the module
- slug: URL-friendly identifier
- description: Brief overview of module content
- order: Sequential position in the curriculum
- prerequisites: List of required modules or concepts
- learningObjectives: Array of learning objectives
- duration: Estimated completion time in minutes
- status: Draft, Review, Published, Archived

### Course Section
**Description**: Hierarchical organization of content (e.g., quarter overview, individual modules, capstone project)
**Fields**:
- id: Unique identifier for the section
- title: Display name of the section
- slug: URL-friendly identifier
- type: Overview, Module, Capstone, Reference
- parentSection: Reference to parent section (for hierarchy)
- childSections: Array of child sections
- contentPath: File path to the content
- order: Position in the sequence

### Navigation Item
**Description**: Menu elements that allow users to access different parts of the textbook
**Fields**:
- id: Unique identifier for the navigation item
- title: Display text for the navigation
- path: URL path to the content
- type: Home, Textbook, Module, Section, External
- icon: Optional icon identifier
- order: Position in navigation menu
- visible: Whether the item is visible to users
- children: Array of sub-navigation items

### Content Block
**Description**: Individual units of educational content (text, code, diagrams, etc.)
**Fields**:
- id: Unique identifier for the content block
- type: Text, Code, Diagram, Video, Image, Interactive
- content: The actual content (Markdown, code, etc.)
- title: Optional title for the block
- metadata: Additional information about the content
- language: For code blocks (TypeScript, Python, etc.)
- interactive: Whether the block supports interaction
- dependencies: Other blocks this block depends on

### Code Example
**Description**: Executable or illustrative code snippets in TypeScript demonstrating concepts
**Fields**:
- id: Unique identifier for the code example
- title: Descriptive title of the example
- code: The actual code content
- language: Programming language (TypeScript, Python, etc.)
- description: Explanation of what the code demonstrates
- complexity: Beginner, Intermediate, Advanced
- category: ROS2, Gazebo, Isaac, VLA, etc.
- runnable: Whether the code can be executed in the browser
- dependencies: Libraries or modules required
- output: Expected output or behavior

## User Interaction Entities

### User Session
**Description**: The interaction context for a user accessing the textbook content
**Fields**:
- userId: Identifier for the user (anonymous if not logged in)
- sessionId: Unique session identifier
- startTime: When the session started
- currentPage: Current page being viewed
- progress: Learning progress data
- interactions: List of user interactions during the session
- endTime: When the session ended (null if active)
- duration: Total session duration

### Learning Progress
**Description**: Track user progress through the textbook modules
**Fields**:
- userId: Identifier for the user
- moduleId: Module the progress applies to
- sectionsCompleted: List of completed sections
- completionPercentage: Overall completion percentage
- timeSpent: Total time spent on the module
- lastAccessed: When the user last accessed the module
- notes: User's personal notes on the content
- bookmarks: Marked important sections

## Relationship Diagram

```
[Course Section] 1--* [Textbook Module]
[Course Section] 1--* [Navigation Item]
[Textbook Module] 1--* [Content Block]
[Content Block] 1--* [Code Example]
[User Session] 1--* [Learning Progress]
[Textbook Module] 1--* [Learning Progress]
```

## Validation Rules

### Textbook Module
- Title must be 1-100 characters
- Slug must be unique and URL-friendly
- Order must be a positive integer
- Status must be one of the defined values

### Course Section
- Type must be one of: Overview, Module, Capstone, Reference
- Parent section must exist if specified
- Content path must point to a valid file

### Navigation Item
- Path must be a valid URL path
- Order must be a positive integer
- Type must be one of the defined values

### Content Block
- Type must be one of the defined values
- Content must not be empty for published blocks
- Language is required for code blocks

### Code Example
- Language must be a supported programming language
- Complexity must be Beginner, Intermediate, or Advanced
- Category must match the textbook modules